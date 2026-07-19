"""Drive avl.exe in batch mode over stdin and parse its .st stability output.

AVL is a 1980s-Fortran program: filename buffers are short and the ST command
asks for overwrite confirmation if the file exists. So we always run with
cwd = Scripts/model_import and RELATIVE paths, and delete stale outputs first.

Run cases per vehicle (all at Mach 0):
  a0.st      alpha = 0                      -> cl0 (= CLtot at body alpha 0)
  cruise.st  CL constrained to cruise CL    -> all stability derivatives
  elev.st    alpha = 0, elevator = +5 deg   -> control-derivative UNIT CHECK:
             AVL versions differ on per-degree vs per-radian control
             derivatives; comparing dCm from this case against Cmd*5deg
             settles it empirically instead of trusting documentation.
"""
from __future__ import annotations

import re
import subprocess
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent
AVL_EXE = MODULE_DIR / "bin" / "avl.exe"
ELEV_CHECK_DEG = 5.0


def run_avl(name: str, avl_text: str, cl_cruise: float | None,
            controls: list[str]) -> dict:
    """Write runs/<name>/model.avl, execute the three cases, return parsed dicts."""
    run_dir = MODULE_DIR / "runs" / name
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "model.avl").write_text(avl_text, encoding="ascii", errors="replace")

    outs = {k: run_dir / f"{k}.st" for k in ("a0", "cruise", "elev")}
    for p in outs.values():
        p.unlink(missing_ok=True)

    rel = f"runs/{name}"
    cmds = [f"LOAD {rel}/model.avl", "OPER",
            "A A 0", "X", f"ST {rel}/a0.st"]
    if cl_cruise is not None:
        cmds += [f"A C {cl_cruise:.4f}", "X", f"ST {rel}/cruise.st"]
    if "elevator" in controls:
        di = controls.index("elevator") + 1
        cmds += ["A A 0", f"D{di} D{di} {ELEV_CHECK_DEG}", "X", f"ST {rel}/elev.st"]
    cmds += ["", "QUIT"]

    proc = subprocess.run([str(AVL_EXE)], input="\n".join(cmds) + "\n",
                          text=True, capture_output=True, timeout=300,
                          cwd=str(MODULE_DIR))
    (run_dir / "avl_stdout.log").write_text(proc.stdout or "", encoding="utf-8",
                                            errors="replace")
    result = {"stdout_tail": (proc.stdout or "")[-2000:], "cases": {}}
    for key, path in outs.items():
        if path.exists():
            result["cases"][key] = parse_st(path)
    if "cruise" not in result["cases"] and "a0" not in result["cases"]:
        raise RuntimeError(
            f"AVL produced no .st output — see {run_dir/'avl_stdout.log'}\n"
            f"stderr: {(proc.stderr or '')[-500:]}")
    return result


_KV = re.compile(r"([A-Za-z]\w*)\s*=\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)")


def parse_st(path: Path) -> dict:
    """Parse an AVL .st file into {key: float}. Control derivatives are
    renamed from CLd1/CLd01-style keys to plain 'CLd1'..'CLdn'."""
    vals: dict[str, float] = {}
    for key, num in _KV.findall(path.read_text(encoding="utf-8", errors="replace")):
        m = re.match(r"^(C[LlYmnD][A-Za-z]*?)d0*(\d+)$", key)
        if m:
            key = f"{m.group(1)}d{m.group(2)}"
        vals.setdefault(key, float(num))     # keep FIRST occurrence
    return vals


def control_deriv_scale(cases: dict, controls: list[str]) -> tuple[float, str]:
    """Empirically decide if AVL control derivatives are per-radian or
    per-degree. Returns (multiplier to convert to per-radian, note)."""
    import math
    if "elevator" not in controls or "elev" not in cases or "a0" not in cases:
        return 1.0, "unit check skipped (no elevator case) — assumed per-radian"
    di = controls.index("elevator") + 1
    cmd = cases["cruise" if "cruise" in cases else "a0"].get(f"Cmd{di}")
    dcm = cases["elev"].get("Cmtot", 0.0) - cases["a0"].get("Cmtot", 0.0)
    if cmd is None or abs(cmd) < 1e-9 or abs(dcm) < 1e-9:
        return 1.0, "unit check inconclusive — assumed per-radian"
    per_rad_err = abs(dcm - cmd * math.radians(ELEV_CHECK_DEG)) / abs(dcm)
    per_deg_err = abs(dcm - cmd * ELEV_CHECK_DEG) / abs(dcm)
    if per_rad_err <= per_deg_err:
        return 1.0, f"unit check: per-RADIAN (err {per_rad_err:.1%} vs per-deg {per_deg_err:.1%})"
    return 180.0 / math.pi, f"unit check: per-DEGREE (err {per_deg_err:.1%}) — converted to per-radian"

"""Build an AVL input file (.avl) from a geometry-spec dict.

Coordinate system (AVL convention): x aft, y right, z up.
Origin: wing ROOT LEADING EDGE — the same datum the data-request files use for
CG ("CG aft of wing root leading edge"), so Xref = cg_aft_of_root_le_m directly.

Modelling choices (documented limits):
 - Wing + horizontal tail + fin only. No fuselage body: a fuselage adds a
   destabilizing (nose-up) Cm_alpha contribution AVL's slender-body model
   captures poorly anyway; expect the computed static margin to be slightly
   OPTIMISTIC (~1-2% MAC for this size class).
 - Inviscid: no cd0, no stall. Those come from flight data / XFOIL.
 - Control declaration order fixed: d1=aileron, d2=elevator, d3=rudder.
"""
from __future__ import annotations

import math
import re
from pathlib import Path

AIRFOIL_DIR = Path(__file__).resolve().parent / "airfoils"


class Geometry:
    """Derived geometry + the .avl text."""
    def __init__(self):
        self.S = self.b = self.mac = self.ar = self.taper = 0.0
        self.x_le_mac = self.y_mac = self.x_c4_mac = 0.0
        self.xref = self.zref = 0.0
        self.cl_cruise = None
        self.controls: list[str] = []      # declaration order -> d1, d2, d3
        self.notes: list[str] = []
        self.avl_text = ""


def wing_planform(wing: dict) -> tuple[float, float, float, float]:
    """Return (S, mac, y_mac, x_le_mac) for a linearly tapered wing."""
    b, cr, ct = wing["span_m"], wing["root_chord_m"], wing["tip_chord_m"]
    lam = ct / cr
    S = 0.5 * (cr + ct) * b
    mac = (2.0 / 3.0) * cr * (1 + lam + lam * lam) / (1 + lam)
    y_mac = (b / 6.0) * (1 + 2 * lam) / (1 + lam)
    x_le_mac = math.tan(math.radians(wing.get("le_sweep_deg") or 0.0)) * y_mac
    return S, mac, y_mac, x_le_mac


_airfoil_warnings: list[str] = []


def _airfoil_lines(airfoil: str | None) -> list[str]:
    """Map an airfoil spec string to AVL section cards. Names are resolved
    against airfoils/*.dat by normalized stem ("PSU 94-097" -> psu94097.dat).
    Paths are cwd-relative: avl_runner runs avl.exe with cwd=Scripts/model_import."""
    if not airfoil:
        return []
    a = airfoil.strip()
    if a.upper().startswith("NACA"):
        digits = a.upper().replace("NACA", "").strip()
        if digits.isdigit() and len(digits) == 4:
            return ["NACA", digits]
    if a.lower().endswith(".dat"):
        return ["AFILE", f"airfoils/{Path(a).name}"]
    want = re.sub(r"[^a-z0-9]", "", a.lower())
    for f in sorted(AIRFOIL_DIR.glob("*.dat")):
        if re.sub(r"[^a-z0-9]", "", f.stem.lower()) == want:
            return ["AFILE", f"airfoils/{f.name}"]
    _airfoil_warnings.append(f"airfoil {a!r} not found in airfoils/ — modelled as FLAT PLATE "
                             "(cl0/cm0 will be wrong; drop the .dat into airfoils/)")
    return []


def _section(x: float, y: float, z: float, chord: float, ainc: float,
             airfoil: str | None, control: str | None = None) -> list[str]:
    lines = ["SECTION",
             "#Xle    Yle     Zle     Chord   Ainc",
             f"{x:.4f} {y:.4f} {z:.4f} {chord:.4f} {ainc:.3f}"]
    lines += _airfoil_lines(airfoil)
    if control:
        lines += ["CONTROL", control]
    return lines


def build(spec: dict) -> Geometry:
    g = Geometry()
    _airfoil_warnings.clear()
    wing = spec["wing"]
    ht, vt = spec["htail"], spec["vtail"]
    ctl = spec.get("controls", {})

    b = wing["span_m"]
    cr, ct_c = wing["root_chord_m"], wing["tip_chord_m"]
    g.S, g.mac, g.y_mac, g.x_le_mac = wing_planform(wing)
    g.b, g.ar, g.taper = b, b * b / g.S, ct_c / cr
    g.x_c4_mac = g.x_le_mac + 0.25 * g.mac
    stated = wing.get("area_m2")
    if stated and abs(stated - g.S) / g.S > 0.02:
        g.notes.append(f"stated wing area {stated} differs from trapezoid area {g.S:.4f} by >2% — using computed")

    # ---- CG / reference point -------------------------------------------
    cg_mode = spec.get("cg_mode", {})
    cg_x = spec.get("mass", {}).get("cg_aft_of_root_le_m")
    if cg_mode.get("static_margin_frac") is not None:
        # placeholder Xref; orchestrator does a second pass with Xnp from run 1
        g.xref = cg_mode.get("_xref_override", g.x_c4_mac)
        g.notes.append(f"CG mode: static margin {cg_mode['static_margin_frac']:.0%} of MAC (two-pass)")
    elif cg_x is not None:
        g.xref = cg_x
    else:
        g.xref = g.x_c4_mac
        g.notes.append("CG unknown — Xref placed at quarter-chord of MAC")
    g.zref = -(spec.get("mass", {}).get("cg_below_chord_m") or 0.0)   # +down -> -z

    sweep_tan = math.tan(math.radians(wing.get("le_sweep_deg") or 0.0))
    dihed_tan = math.tan(math.radians(wing.get("dihedral_deg") or 0.0))
    incidence = wing.get("incidence_deg") or 0.0
    twist = wing.get("twist_deg") or 0.0
    airfoil = wing.get("airfoil")

    def chord_at(y: float) -> float:
        return cr + (ct_c - cr) * (2 * y / b)

    L: list[str] = []
    L += [spec.get("name", "vehicle"),
          "#Mach", "0.0",
          "#IYsym IZsym Zsym", "0 0 0.0",
          "#Sref   Cref    Bref",
          f"{g.S:.4f} {g.mac:.4f} {b:.4f}",
          "#Xref   Yref    Zref",
          f"{g.xref:.4f} 0.0 {g.zref:.4f}",
          "#CDp", "0.0", ""]

    # ---- WING ------------------------------------------------------------
    L += ["SURFACE", "Wing",
          "#Nchord Cspace Nspan Sspace", "10 1.0 24 1.0",
          "COMPONENT", "1",
          "YDUPLICATE", "0.0",
          "ANGLE", f"{incidence:.3f}", ""]
    ail = ctl.get("aileron")
    ail_card = None
    if ail and ail.get("y_in_m") is not None:
        g.controls.append("aileron")
        hinge = 1.0 - (ail.get("chord_frac") or 0.25)
        ail_card = f"aileron 1.0 {hinge:.3f} 0. 0. 0. -1.0"
    ys = [0.0]
    if ail_card:
        y_in = min(ail["y_in_m"], b / 2)
        y_out = min(ail.get("y_out_m") or b / 2, b / 2)
        if y_in > 0.0:
            ys.append(y_in)
        if y_out < b / 2 - 1e-6:
            ys.append(y_out)
    ys.append(b / 2)
    for y in ys:
        on_ail = ail_card and (ail["y_in_m"] - 1e-6) <= y <= (ail.get("y_out_m", b / 2) + 1e-6)
        ainc = twist * (2 * y / b)
        L += _section(sweep_tan * y, y, dihed_tan * y, chord_at(y), ainc,
                      airfoil, ail_card if on_ail else None)
    L += [""]

    # ---- HORIZONTAL TAIL ---------------------------------------------------
    if ht.get("area_m2"):
        Sh, bh = ht["area_m2"], ht["span_m"]
        ch = Sh / bh                                   # rectangular assumption
        is_t_tail = "t" in (ht.get("config") or spec.get("layout", {}).get("tail_type") or "").lower().replace("conventional", "")
        zh = (vt.get("height_m") or 0.0) if is_t_tail else 0.0
        x_c4_h = g.x_c4_mac + ht["arm_c4_to_c4_m"]
        xle_h = x_c4_h - 0.25 * ch
        g.controls.append("elevator")
        hinge = 1.0 - (ctl.get("elevator", {}).get("chord_frac") or 0.35)
        elev_card = f"elevator 1.0 {hinge:.3f} 0. 0. 0. 1.0"
        L += ["SURFACE", "Horizontal tail",
              "#Nchord Cspace Nspan Sspace", "8 1.0 12 1.0",
              "COMPONENT", "1",
              "YDUPLICATE", "0.0",
              "ANGLE", f"{ht.get('incidence_deg') or 0.0:.3f}", ""]
        for y in (0.0, bh / 2):
            L += _section(xle_h, y, zh, ch, 0.0, ht.get("airfoil"), elev_card)
        L += [""]
        if is_t_tail:
            g.notes.append(f"T-tail: H-tail placed at fin tip z={zh:.2f} m")

    # ---- FIN (single, on centreline) ----------------------------------------
    if vt.get("area_m2"):
        Sv, hv = vt["area_m2"], vt["height_m"]
        cv = Sv / hv                                   # rectangular assumption
        cg_ref = g.xref
        if vt.get("arm_to_cg_m"):
            x_c4_v = cg_ref + vt["arm_to_cg_m"]
        else:
            x_c4_v = g.x_c4_mac + (ht.get("arm_c4_to_c4_m") or 0.0)
            g.notes.append("fin arm unknown — placed at H-tail arm")
        xle_v = x_c4_v - 0.25 * cv
        g.controls.append("rudder")
        rud = ctl.get("rudder", {})
        hinge = 1.0 - (rud.get("chord_frac") or 0.30)
        rud_top = min(rud.get("height_m") or hv, hv)
        rud_card = f"rudder 1.0 {hinge:.3f} 0. 0. 0. 1.0"
        L += ["SURFACE", "Fin",
              "#Nchord Cspace Nspan Sspace", "8 1.0 10 1.0",
              "COMPONENT", "1",
              "ANGLE", "0.0", ""]
        zs = [0.0]
        if rud_top < hv - 1e-6:
            zs += [rud_top]
        zs += [hv]
        for z in zs:
            on_rud = z <= rud_top + 1e-6
            L += _section(xle_v, 0.0, z, cv, 0.0, vt.get("airfoil") or "NACA 0009",
                          rud_card if on_rud else None)
        L += [""]

    g.notes.extend(dict.fromkeys(_airfoil_warnings))     # dedupe, keep order
    g.avl_text = "\n".join(L) + "\n"

    # cruise CL for the derivative run point: level flight at cruise speed
    m = spec.get("mass", {}).get("takeoff_kg")
    v = (spec.get("envelope", {}) or {}).get("cruise_ms") or (spec.get("cruise", {}) or {}).get("speed_ms")
    rho = (spec.get("environment", {}) or {}).get("air_density") or 1.225
    if m and v:
        g.cl_cruise = 2.0 * m * 9.81 / (rho * v * v * g.S)
    return g

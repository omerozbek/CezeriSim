"""CezeriSim vehicle model import toolchain.

Pipeline: filled data-request .txt  ->  geometry spec .json  ->  AVL run
          ->  stability derivatives ->  Vehicles/<name>/mechanical.json etc.

Usage (run from anywhere; paths resolved relative to this file):
  python model_import.py parse <MECHANICAL.txt> [--electrical <ELECTRICAL.txt>]
                               [--name pasifik] [-o geometry/pasifik.json]
      Parse filled request files into a reviewable geometry spec (+ warnings
      + a gaps report of everything the team still owes us).

  python model_import.py avl <geometry/spec.json>
      Build the AVL model, run the three cases, write runs/<name>/derivs.json
      and print the derivative table.

  python model_import.py compare <geometry/spec.json> --vehicle gokce_flight16
      Run AVL and print computed derivatives next to the vehicle's current
      mechanical.json values (validation mode — writes nothing).

  python model_import.py write <geometry/spec.json> --vehicle pasifik_vtol
                               [--base vtol_default]
      Run AVL and write/update Vehicles/<vehicle>/ (mechanical + electrical +
      model.json + placeholder params.parm from --base).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(MODULE_DIR))

import avl_builder
import avl_runner
import datasheet_parser
import mech_writer


def _read(p: str | Path) -> str:
    return Path(p).read_text(encoding="utf-8", errors="replace")


def cmd_parse(args) -> dict:
    res = datasheet_parser.parse_mechanical(_read(args.mechanical))
    spec = res.spec
    spec["name"] = args.name
    spec["source"] = Path(args.mechanical).name
    warnings, gaps = list(res.warnings), list(res.gaps)
    if args.electrical:
        res_e = datasheet_parser.parse_electrical(_read(args.electrical))
        spec["electrical"] = res_e.spec
        spec["source"] += f" + {Path(args.electrical).name}"
        warnings += res_e.warnings
        gaps += res_e.gaps
    spec["_warnings"] = warnings
    spec["_gaps"] = gaps

    out = Path(args.output) if args.output else MODULE_DIR / "geometry" / f"{args.name}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(spec, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"spec -> {out}")
    if warnings:
        print("\nPARSE WARNINGS (review the spec!):")
        for w in warnings:
            print(f"  ! {w}")
    if gaps:
        print("\nDATA GAPS (still owed by the team):")
        for g in gaps:
            print(f"  - {g}")
    return spec


def run_avl_for_spec(spec: dict) -> tuple:
    """Build + run AVL, honouring the two-pass static-margin CG mode."""
    geom = avl_builder.build(spec)
    result = avl_runner.run_avl(spec["name"], geom.avl_text, geom.cl_cruise, geom.controls)

    sm = (spec.get("cg_mode") or {}).get("static_margin_frac")
    if sm is not None:
        cr = result["cases"].get("cruise") or result["cases"]["a0"]
        xnp = cr.get("Xnp")
        if xnp is not None:
            spec.setdefault("cg_mode", {})["_xref_override"] = xnp - sm * geom.mac
            geom = avl_builder.build(spec)
            result = avl_runner.run_avl(spec["name"], geom.avl_text, geom.cl_cruise, geom.controls)

    scale, unit_note = avl_runner.control_deriv_scale(result["cases"], geom.controls)
    geom.notes.append("control derivatives: " + unit_note)
    return geom, result, scale


def _deriv_table(spec, geom, result, scale) -> dict:
    vals, notes = mech_writer.derive_mechanical(spec, geom, result, scale)
    return {"values": vals, "notes": notes}


def cmd_avl(args):
    spec = json.loads(_read(args.spec))
    geom, result, scale = run_avl_for_spec(spec)
    d = _deriv_table(spec, geom, result, scale)
    out = MODULE_DIR / "runs" / spec["name"] / "derivs.json"
    out.write_text(json.dumps(d, indent=2), encoding="utf-8")
    print(f"AVL run OK -> {out}\n")
    print(f"{'field':28s} {'AVL/import':>12s}")
    for k, v in d["values"].items():
        print(f"{k:28s} {v!s:>12s}")
    print("\nNOTES:")
    for n in d["notes"]:
        print(f"  - {n}")


def cmd_compare(args):
    spec = json.loads(_read(args.spec))
    geom, result, scale = run_avl_for_spec(spec)
    d = _deriv_table(spec, geom, result, scale)
    mech = json.loads(_read(mech_writer.VEHICLES / args.vehicle / "mechanical.json"))
    print(f"\n{'field':28s} {'AVL/import':>12s} {'current':>12s}   (Vehicles/{args.vehicle})")
    for k, v in d["values"].items():
        cur = mech.get(k, "—")
        try:
            flag = " <-- differs" if isinstance(cur, (int, float)) and v and \
                abs(cur - v) > 0.25 * max(abs(cur), abs(v), 1e-9) else ""
        except TypeError:
            flag = ""
        print(f"{k:28s} {v!s:>12s} {cur!s:>12s}{flag}")
    print("\nNOTES:")
    for n in d["notes"]:
        print(f"  - {n}")


def cmd_write(args):
    spec = json.loads(_read(args.spec))
    geom, result, scale = run_avl_for_spec(spec)
    mech_vals, mech_notes = mech_writer.derive_mechanical(spec, geom, result, scale)
    elec_vals = elec_notes = None
    if spec.get("electrical"):
        elec_vals, elec_notes = mech_writer.derive_electrical(spec["electrical"], spec)
    vdir = mech_writer.write_vehicle(args.vehicle, mech_vals, mech_notes,
                                     elec_vals, elec_notes or [], spec,
                                     base=args.base, display_name=args.display_name)
    print(f"vehicle written -> {vdir}")
    for n in mech_notes + (elec_notes or []):
        print(f"  - {n}")
    if spec.get("_gaps"):
        print("\nSTILL OWED BY THE TEAM:")
        for g in spec["_gaps"]:
            print(f"  - {g}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("parse")
    p.add_argument("mechanical")
    p.add_argument("--electrical")
    p.add_argument("--name", default="vehicle")
    p.add_argument("-o", "--output")
    p.set_defaults(fn=cmd_parse)

    p = sub.add_parser("avl")
    p.add_argument("spec")
    p.set_defaults(fn=cmd_avl)

    p = sub.add_parser("compare")
    p.add_argument("spec")
    p.add_argument("--vehicle", required=True)
    p.set_defaults(fn=cmd_compare)

    p = sub.add_parser("write")
    p.add_argument("spec")
    p.add_argument("--vehicle", required=True)
    p.add_argument("--base", default="vtol_default")
    p.add_argument("--display-name")
    p.set_defaults(fn=cmd_write)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()

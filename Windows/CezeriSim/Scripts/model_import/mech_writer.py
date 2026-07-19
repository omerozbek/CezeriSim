"""Map parsed datasheet spec + AVL derivatives to Vehicles/<name>/*.json.

Sign conventions follow the existing mechanical.json plant convention
(see Vehicles/gokce_flight16/mechanical.json _doc):
  cm_aileron  POSITIVE   cm_elevator NEGATIVE   cm_rudder POSITIVE
AVL hinge-sign choices vary with how the CONTROL cards are written, so the
magnitudes are taken from AVL and the plant's documented signs are imposed.

Every value that is estimated rather than measured/computed gets a note in
the generated "_import_doc" block — nothing silent.
"""
from __future__ import annotations

import json
import math
from datetime import date
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
VEHICLES = REPO / "Vehicles"


def _r(v: float | None, nd: int = 4) -> float | None:
    return None if v is None else round(v, nd)


def estimate_inertia(spec: dict, geom) -> tuple[dict, str]:
    """Point-mass + rod estimate when the team returned no inertia data.
    Empty-mass split: 40% at the four motor-rectangle corners (motors, ESCs,
    booms), 22% wing (rod, full span), 38% fuselage+tail (rod, full length).
    Battery assumed at the CG (contributes ~nothing)."""
    m_e = spec["mass"].get("empty_kg") or (spec["mass"]["takeoff_kg"] - (spec["mass"].get("battery_kg") or 0))
    w = spec["motors"].get("rect_width_m") or 2 * 0.7
    l = spec["motors"].get("rect_length_m") or w
    b = geom.b
    L = spec.get("layout", {}).get("length_m") or spec.get("fuselage", {}).get("length_m") or 0.8 * b
    m_c, m_w, m_f = 0.40 * m_e, 0.22 * m_e, 0.38 * m_e
    ixx = m_c * (w / 2) ** 2 + m_w * b * b / 12
    iyy = m_c * (l / 2) ** 2 + m_f * L * L / 12 + m_w * geom.mac ** 2 / 12
    izz = m_c * ((w / 2) ** 2 + (l / 2) ** 2) + m_w * b * b / 12 + m_f * L * L / 12
    note = (f"ESTIMATED (team returned '?'): point-mass model, empty {m_e} kg split "
            f"40% motor-rect corners / 22% wing rod / 38% fuselage rod {L} m, battery at CG. "
            f"Replace with CAD mass-properties report or pendulum test.")
    return {"ixx": _r(ixx, 3), "iyy": _r(iyy, 3), "izz": _r(izz, 3)}, note


def derive_mechanical(spec: dict, geom, avl: dict, ctrl_scale: float) -> tuple[dict, list[str]]:
    """Return (mechanical.json value updates, provenance notes)."""
    notes: list[str] = list(geom.notes)
    cases = avl["cases"]
    cr = cases.get("cruise") or cases["a0"]
    a0 = cases.get("a0", {})
    controls = geom.controls

    def cd(prefix: str, name: str) -> float | None:
        if name not in controls:
            return None
        v = cr.get(f"{prefix}d{controls.index(name) + 1}")
        return None if v is None else v * ctrl_scale

    vals: dict = {}
    m = spec["mass"]
    vals["mass_kg"] = m["takeoff_kg"]
    if m.get("battery_kg") is not None:
        vals["battery_mass_kg"] = m["battery_kg"]

    if spec["inertia"].get("ixx") is not None:
        ine = spec["inertia"]
        notes.append("inertia: from team-provided CAD/pendulum data")
    else:
        ine, note = estimate_inertia(spec, geom)
        notes.append("inertia: " + note)
    vals["inertia_roll_kg_m2"] = ine["ixx"]
    vals["inertia_pitch_kg_m2"] = ine["iyy"]
    vals["inertia_yaw_kg_m2"] = ine["izz"]

    mot = spec["motors"]
    if mot.get("rect_width_m") and mot.get("rect_length_m"):
        w, l = mot["rect_width_m"], mot["rect_length_m"]
        vals["arm_length_m"] = _r(0.5 * math.hypot(w, l))
        vals["motor_rect_width_m"] = w
        vals["motor_rect_length_m"] = l

    vals["wing_area_m2"] = _r(geom.S)
    vals["aspect_ratio"] = _r(geom.ar, 3)
    vals["mac_m"] = _r(geom.mac)
    vals["wingspan_m"] = geom.b

    # ---- AVL derivatives (stability axes, at the cruise-CL run point) -----
    vals["cl0"] = _r(a0.get("CLtot"))
    vals["cl_alpha"] = _r(cr.get("CLa"))
    vals["cy_beta"] = _r(cr.get("CYb"))
    vals["cm_alpha"] = _r(cr.get("Cma"))
    vals["cm_q"] = _r(cr.get("Cmq"), 3)
    vals["cl_p"] = _r(cr.get("Clp"))
    vals["cn_r"] = _r(cr.get("Cnr"))
    vals["cn_beta"] = _r(cr.get("Cnb"))
    if cr.get("e"):
        vals["oswald_e"] = _r(cr["e"], 3)
        notes.append("oswald_e: AVL inviscid span efficiency (slightly optimistic vs true Oswald)")

    ail, elv, rud = cd("Cl", "aileron"), cd("Cm", "elevator"), cd("Cn", "rudder")
    if ail is not None:
        vals["cm_aileron"] = _r(abs(ail))
    if elv is not None:
        vals["cm_elevator"] = _r(-abs(elv))
    if rud is not None:
        vals["cm_rudder"] = _r(abs(rud))

    xnp = cr.get("Xnp")
    if xnp is not None:
        sm = (xnp - geom.xref) / geom.mac
        notes.append(f"AVL neutral point Xnp={xnp:.4f} m, CG at {geom.xref:.4f} m -> "
                     f"static margin {sm:.1%} MAC (no-fuselage model, real margin slightly lower)")

    # ---- envelope-driven values -------------------------------------------
    env = spec.get("envelope", {})
    if env.get("stall_speed_ms"):
        vals["stall_speed_ms"] = env["stall_speed_ms"]
        vals["min_airspeed_ms"] = round(0.7 * env["stall_speed_ms"], 1)
    if env.get("cruise_ms"):
        vals["cruise_airspeed_ms"] = env["cruise_ms"]
    if env.get("stall_aoa_deg"):
        vals["stall_aoa_deg"] = env["stall_aoa_deg"]
    elif env.get("clmax") and vals.get("cl0") is not None and vals.get("cl_alpha"):
        vals["stall_aoa_deg"] = _r(math.degrees((env["clmax"] - vals["cl0"]) / vals["cl_alpha"]), 1)
        notes.append(f"stall_aoa_deg: from team CLmax {env['clmax']} via (CLmax-cl0)/cl_alpha")

    mdef = spec.get("controls", {}).get("max_deflection_deg")
    vals["max_control_deflection_deg"] = mdef or 20
    if not mdef:
        notes.append("max_control_deflection_deg: not measured, default 20")

    # cd0: NOT computable by AVL (inviscid). Initial estimate; calibrate against
    # the real closed-loop cruise point once a real .BIN log exists (gokce method).
    if env.get("cruise_ms") and vals.get("oswald_e"):
        vals["cd0"] = 0.04
        notes.append("cd0: INITIAL ESTIMATE 0.04 — no glide/polar data returned; "
                     "calibrate to the real cruise operating point like gokce (ROADMAP 4.3)")

    grd = spec.get("ground", {})
    vals["collision_radius_m"] = grd.get("half_dimension_m") or _r(geom.b / 2, 2)
    vals["cg_offset_x_m"] = spec["mass"].get("cg_vs_motor_center_m") or 0
    vals["cg_offset_y_m"] = spec["mass"].get("cg_lateral_m") or 0
    vals["cg_offset_z_m"] = 0
    vals["gravity_ms2"] = 9.81
    vals["air_density"] = 1.225
    return vals, notes


def derive_electrical(spec_e: dict, spec_m: dict) -> tuple[dict, list[str]]:
    """Electrical values: identity fields copied, coefficients estimated and
    LOUDLY flagged — they need the real .param + thrust-stand data."""
    notes: list[str] = []
    vals: dict = {}
    lm, pm, bat = spec_e["lift_motor"], spec_e["pusher_motor"], spec_e["battery"]
    m_kg = spec_m["mass"]["takeoff_kg"]

    if lm.get("pwm_min"):
        vals["pwm_min"], vals["pwm_max"] = lm["pwm_min"], lm["pwm_max"]
    for k, v in (("motor_kv", lm.get("kv")), ("motor_max_current_a", lm.get("max_current_a")),
                 ("prop_diameter_in", lm.get("prop_diameter_in")),
                 ("prop_pitch_in", lm.get("prop_pitch_in")),
                 ("prop_blade_count", lm.get("prop_blades")),
                 ("pusher_motor_kv", pm.get("kv")),
                 ("pusher_prop_diameter_in", pm.get("prop_diameter_in")),
                 ("pusher_prop_pitch_in", pm.get("prop_pitch_in")),
                 ("pusher_prop_blade_count", pm.get("prop_blades"))):
        if v is not None:
            vals[k] = v

    if bat.get("cells_s"):
        s = bat["cells_s"]
        vals["battery_cells_s"] = s
        vals["battery_voltage_full_v"] = bat.get("voltage_full_v") or round(4.2 * s, 1)
        vals["battery_voltage_nominal_v"] = round(3.7 * s, 1)
    if bat.get("capacity_mah"):
        vals["battery_capacity_mah"] = bat["capacity_mah"]

    sv = spec_e.get("servos", {}).get("speed_deg_s")
    if sv:
        vals["servo_speed_deg_s"] = sv
        notes.append(f"servo_speed_deg_s {sv}: slowest listed servo (rated s/60deg)")

    # k_thrust: same P60 + 22x6.6 powertrain as gokce, RPM-equivalent electrically
    # (6S x KV340 = 12S x KV170) -> reuse gokce's AP-output-path-derived value.
    if (lm.get("model") or "").upper().find("P60") >= 0 and lm.get("prop_diameter_in") == 22:
        vals["k_thrust"] = 78.49
        vals["k_torque"] = 2.0
        notes.append("k_thrust 78.49 / k_torque 2.0: reused from gokce — identical "
                     "P60 + 22x6.6 powertrain at the same RPM ceiling (6Sx340KV = 12Sx170KV). "
                     f"Predicted hover throttle sqrt({m_kg}*9.81/4/78.49) = "
                     f"{math.sqrt(m_kg * 9.81 / 4 / 78.49):.2f}. PLACEHOLDER until the real "
                     ".param (Q_M_THST_HOVER) or a thrust-stand sweep arrives.")
    else:
        u_hover = 0.5
        vals["k_thrust"] = _r(m_kg * 9.81 / 4 / u_hover ** 2, 1)
        vals["k_torque"] = _r(vals["k_thrust"] * 0.025, 2)
        notes.append(f"k_thrust {vals['k_thrust']}: assumed hover throttle {u_hover} — "
                     "PLACEHOLDER, needs thrust-stand data or the real .param")

    # pusher: 16x8 prop at ~7600 rpm (0.9 * KV380 * 22.2 V) -> ~27 N static (APC-type data)
    vals["k_thrust_pusher"] = 27.0
    notes.append("k_thrust_pusher 27: APC-style static estimate for 16x8 at ~7600 rpm on 6S — "
                 "PLACEHOLDER, needs stand data / prop table (electrical file sec. 3)")

    vals["motor_time_constant_s"] = 0.05
    notes.append("motor_time_constant_s 0.05: default, spin-up lag not measured")

    # heave damping ~ rho*A_disk*v_i per rotor at hover thrust
    t_hover = m_kg * 9.81 / 4
    a_disk = math.pi * (0.0254 * (lm.get("prop_diameter_in") or 22) / 2) ** 2
    v_i = math.sqrt(t_hover / (2 * 1.225 * a_disk))
    vals["drag_coeff"] = _r(4 * 1.225 * a_disk * v_i, 1)
    notes.append(f"drag_coeff {vals['drag_coeff']}: rotor-inflow heave damping, "
                 f"4 x rho*A_disk*v_i (v_i {v_i:.1f} m/s at {t_hover:.1f} N hover thrust)")
    vals["drag_coeff_hover_xy"] = 2
    vals["drag_coeff_forward"] = 0.2
    vals["hover_drag_fade_lo_ms"] = 6
    vals["hover_drag_fade_hi_ms"] = 12
    vals["rotational_damping"] = 1.5
    notes.append("hover damping family + rotational_damping: copied from gokce (same rotor class)")

    if bat.get("hover_current_a"):
        vals["hover_current_a"] = bat["hover_current_a"]
    else:
        p_hover = 4 * t_hover * v_i / 0.75
        vals["hover_current_a"] = round(p_hover / (3.7 * (bat.get("cells_s") or 6)))
        notes.append(f"hover_current_a {vals['hover_current_a']}: momentum-theory estimate "
                     "(75% prop+motor efficiency) — team returned N/A")
    return vals, notes


def _load_json(p: Path) -> dict:
    return json.loads(p.read_text(encoding="utf-8"))


def write_vehicle(name: str, mech_vals: dict, mech_notes: list[str],
                  elec_vals: dict | None, elec_notes: list[str],
                  spec: dict, base: str = "vtol_default",
                  display_name: str | None = None) -> Path:
    """Create/update Vehicles/<name>/ from Vehicles/<base>/, applying values.
    _doc blocks from the base are preserved; an _import_doc block records
    source, date and every estimation note."""
    vdir = VEHICLES / name
    bdir = VEHICLES / base
    vdir.mkdir(exist_ok=True)
    today = date.today().isoformat()

    stamp = {
        "_import_doc": {
            "source": spec.get("source", "team data files"),
            "date": today,
            "tool": "Scripts/model_import (AVL " + spec.get("_avl_version", "3.40") + ")",
            "notes": mech_notes,
        }
    }
    mech = _load_json((vdir / "mechanical.json") if (vdir / "mechanical.json").exists()
                      else bdir / "mechanical.json")
    mech["backend"] = "ue_physics"
    mech.update({k: v for k, v in mech_vals.items() if v is not None})
    mech.update(stamp)
    (vdir / "mechanical.json").write_text(json.dumps(mech, indent=4), encoding="utf-8")

    if elec_vals is not None:
        elec = _load_json((vdir / "electrical.json") if (vdir / "electrical.json").exists()
                          else bdir / "electrical.json")
        elec.update({k: v for k, v in elec_vals.items() if v is not None})
        elec["_import_doc"] = {"source": spec.get("source", ""), "date": today, "notes": elec_notes}
        (vdir / "electrical.json").write_text(json.dumps(elec, indent=4), encoding="utf-8")

    if not (vdir / "params.parm").exists() and (bdir / "params.parm").exists():
        base_parm = (bdir / "params.parm").read_text(encoding="utf-8")
        (vdir / "params.parm").write_text(
            f"# PLACEHOLDER copied from {base} {today} — replace with the real "
            f"controller's full .param export (data request sec. 1)\n" + base_parm,
            encoding="utf-8")

    model = {"display_name": display_name or name, "type": "vtol",
             "protected": False, "color": "original"}
    (vdir / "model.json").write_text(json.dumps(model, indent=4), encoding="utf-8")
    return vdir

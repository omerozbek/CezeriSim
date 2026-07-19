"""Parse filled *_VTOL_MECHANICAL_DATA.txt / *_VTOL_ELECTRICAL_DATA.txt request
files into a clean geometry-spec dict (JSON-serializable).

The request templates are ours, so parsing is anchor-based: for each field we
locate the first line containing a known label substring (optionally after a
section marker) and take everything right of '='.

Tolerances (all emit warnings so a human can review the generated spec):
 - decimal commas ("1,73" -> 1.73) — the team used them despite instructions
 - fill-in underscores around values ("__7,5___")
 - "N/A", "?", "" -> None
 - metres written into mm-labelled fields: if an mm field holds a non-integer
   value < 5 it is almost certainly metres (0.125 "mm" CG offset) -> treat as m
"""
from __future__ import annotations

import re


class ParseResult:
    def __init__(self):
        self.spec: dict = {}
        self.warnings: list[str] = []
        self.gaps: list[str] = []

    def warn(self, msg: str):
        self.warnings.append(msg)

    def gap(self, msg: str):
        self.gaps.append(msg)


def _clean(raw: str | None) -> str | None:
    if raw is None:
        return None
    s = raw.strip().strip("_").strip()
    s = re.sub(r"_{2,}", " ", s).strip("_ \t")
    if s == "" or s.lower() in ("n/a", "na", "?", "unknown", "-"):
        return None
    return s

def _num(raw: str | None) -> float | None:
    s = _clean(raw)
    if s is None:
        return None
    s = re.sub(r"(\d),(\d)", r"\1.\2", s)          # decimal comma
    m = re.search(r"[-+]?\d*\.?\d+", s)
    return float(m.group(0)) if m else None


def _find_line(lines: list[str], anchor: str, start: int = 0) -> int:
    for i in range(start, len(lines)):
        if anchor.lower() in lines[i].lower():
            return i
    return -1


class _Doc:
    """Anchor-based field grabber over the raw text."""

    def __init__(self, text: str, res: ParseResult):
        self.lines = text.splitlines()
        self.res = res

    def raw(self, anchor: str, after: str | None = None) -> str | None:
        start = 0
        if after is not None:
            i = _find_line(self.lines, after)
            if i < 0:
                self.res.warn(f"section anchor not found: {after!r}")
                return None
            start = i
        i = _find_line(self.lines, anchor, start)
        if i < 0:
            self.res.warn(f"field anchor not found: {anchor!r}")
            return None
        line = self.lines[i]
        if "=" in line:
            return line.split("=", 1)[1]
        # template lines whose answer wraps to the next line ("= value")
        if i + 1 < len(self.lines) and self.lines[i + 1].lstrip().startswith("="):
            return self.lines[i + 1].split("=", 1)[1]
        # lines with no '=' at all (e.g. "AILERON: span extent from _x_ m to _y_ m")
        return line

    def num(self, anchor: str, after: str | None = None) -> float | None:
        return _num(self.raw(anchor, after))

    def text(self, anchor: str, after: str | None = None) -> str | None:
        return _clean(self.raw(anchor, after))

    def mm(self, anchor: str, after: str | None = None) -> float | None:
        """mm-labelled field -> metres, with the metres-in-mm heuristic."""
        v = self.num(anchor, after)
        if v is None:
            return None
        if abs(v) < 5 and v != int(v):
            self.res.warn(f"{anchor!r}: value {v} in an mm field looks like metres — using {v} m")
            return v
        return v / 1000.0


def parse_mechanical(text: str) -> ParseResult:
    res = ParseResult()
    d = _Doc(text, res)

    res.spec["layout"] = {
        "vtol_layout": d.text("VTOL layout"),
        "lift_motors": d.num("Number of lift motors"),
        "wing_config": d.text("Wing configuration"),
        "tail_type": d.text("Tail type"),
    }
    dims = d.raw("Overall length")
    if dims:
        parts = [_num(p) for p in re.split(r"[×x]", dims)]
        if len(parts) == 3:
            res.spec["layout"]["length_m"], res.spec["layout"]["span_check_m"], \
                res.spec["layout"]["height_m"] = parts

    res.spec["mass"] = {
        "takeoff_kg": d.num("Takeoff mass"),
        "empty_kg": d.num("Empty mass"),
        "battery_kg": d.num("Battery mass alone"),
        "payload_kg": d.num("Typical payload mass"),
        "cg_aft_of_root_le_m": d.mm("CG aft of wing root leading edge"),
        "cg_lateral_m": d.mm("CG lateral offset"),
        "cg_below_chord_m": d.mm("CG below/above wing chord plane"),
        "cg_vs_motor_center_m": d.mm("forward/aft offset (mm, +aft)"),
    }
    if res.spec["mass"]["takeoff_kg"] is None:
        res.gap("takeoff mass missing")

    res.spec["inertia"] = {
        "ixx": d.num("Ixx (roll"),
        "iyy": d.num("Iyy (pitch"),
        "izz": d.num("Izz (yaw"),
        "ixz": d.num("Ixz (if reported"),
    }
    if res.spec["inertia"]["ixx"] is None:
        res.gap("moments of inertia not provided (CAD report or pendulum test) — will be ESTIMATED")

    res.spec["motors"] = {
        "rect_width_m": d.num("FRONT pair of lift motors"),
        "rect_length_m": d.num("LEFT pair of lift motors"),
        "plane_above_cg_m": d.mm("Lift-motor plane above/below CG"),
        "spin_directions": d.text("front-right CCW"),
    }
    if res.spec["motors"]["plane_above_cg_m"] is None:
        res.gap("lift-motor plane height above CG not provided")

    res.spec["wing"] = {
        "span_m": d.num("Span (tip to tip"),
        "root_chord_m": d.num("Root chord (m)"),
        "tip_chord_m": d.num("Tip chord (m)"),
        "area_m2": d.num("Wing area if known"),
        "le_sweep_deg": d.num("Leading-edge sweep"),
        "dihedral_deg": d.num("Dihedral (deg)"),
        "incidence_deg": d.num("Incidence at root"),
        "twist_deg": d.num("Twist / washout"),
        "airfoil": d.text("Airfoil name (or .dat"),
    }

    res.spec["htail"] = {
        "config": d.text("Configuration (conventional / V / inv-V"),
        "area_m2": d.num("Area (m", after="HORIZONTAL TAIL"),
        "span_m": d.num("Span (m)", after="HORIZONTAL TAIL"),
        "arm_c4_to_c4_m": d.num("tail 1/4-chord (m)"),
        "incidence_deg": d.num("Incidence (deg)", after="HORIZONTAL TAIL"),
        "airfoil": d.text("Airfoil", after="Incidence (deg)"),
    }
    res.spec["vtail"] = {
        "area_m2": d.num("Area (m", after="VERTICAL TAIL"),
        "height_m": d.num("Height (m)", after="VERTICAL TAIL"),
        "arm_to_cg_m": d.num("Arm to CG (m)", after="VERTICAL TAIL"),
    }
    res.spec["fuselage"] = {
        "length_m": d.num("Fuselage length"),
        "width_m": None,
        "height_m": None,
    }
    wh = d.raw("Max width")
    if wh:
        parts = [_num(p) for p in re.split(r"[×x]", wh)]
        if len(parts) == 2:
            res.spec["fuselage"]["width_m"], res.spec["fuselage"]["height_m"] = parts

    # -- control surfaces ------------------------------------------------
    controls: dict = {}
    ail_line = d.raw("span extent from")
    if ail_line:
        m = re.search(r"from\s*_*\s*([\d.,]+)\s*_*\s*m\s*to\s*_*\s*([\d.,]+)\s*_*\s*m", ail_line)
        if m:
            controls["aileron"] = {"y_in_m": _num(m.group(1)), "y_out_m": _num(m.group(2)),
                                   "chord_frac": None}
    cf = d.num("% of local wing chord")
    if "aileron" in controls and cf is not None:
        controls["aileron"]["chord_frac"] = cf / 100.0
    el = d.raw("ELEVATOR: span")
    if el:
        nums = [_num(p) for p in el.split("chord fraction")]
        if len(nums) == 2 and None not in nums:
            controls["elevator"] = {"span_m": nums[0], "chord_frac": nums[1] / 100.0}
    ru = d.raw("RUDDER:")
    if ru:
        nums = [_num(p) for p in ru.split("chord fraction")]
        if len(nums) == 2 and None not in nums:
            controls["rudder"] = {"height_m": nums[0], "chord_frac": nums[1] / 100.0}
    defl = d.num("max deflection measured: up")
    controls["max_deflection_deg"] = defl
    if defl is None:
        res.gap("control surface max deflections not measured — keeping default 20 deg")
    res.spec["controls"] = controls

    # -- drag polar / envelope / environment ------------------------------
    p1 = d.raw("Point 1:")
    if p1 is None or _num(p1) is None:
        res.gap("drag polar / glide data not provided — cd0 stays an estimate "
                "to be calibrated against the real cruise point (gokce method)")

    res.spec["envelope"] = {
        "stall_speed_ms": d.num("Stall speed, level flight"),
        "stall_test_mass_kg": d.num("Weight at that test"),
        "stall_aoa_deg": d.num("Stall AoA if known"),
        "clmax": d.num("CLmax if known"),
        "cruise_ms": d.num("Normal cruise airspeed"),
        "max_ms": d.num("Max airspeed flown"),
        "transition_ms": d.num("Transition speed used"),
    }

    env_line = d.raw("LAT / LON")
    lat = lon = None
    if env_line:
        m = re.search(r"([-+]?\d+\.\d+)\s*,\s*([-+]?\d+\.\d+)", env_line)
        if m:
            lat, lon = float(m.group(1)), float(m.group(2))
    res.spec["environment"] = {
        "site": _clean(env_line.split(str(lat))[0]) if env_line and lat else _clean(env_line),
        "lat": lat, "lon": lon,
        "field_elevation_m": d.num("Field elevation"),
    }

    res.spec["ground"] = {
        "cg_height_mm": d.num("CG height above ground"),
        "half_dimension_m": d.num("Largest half-dimension"),
    }
    if res.spec["ground"]["half_dimension_m"] is None:
        res.gap("ground stance section empty — collision radius defaults to span/2")

    return res


def parse_electrical(text: str) -> ParseResult:
    """Electrical file: identity + battery fields are extracted; the derived
    coefficients (k_thrust family) need the real .param file / thrust-stand
    data, so everything here is report-and-flag rather than authoritative."""
    res = ParseResult()
    d = _Doc(text, res)

    res.spec["autopilot"] = {
        "hardware": d.text("Flight controller hardware"),
        "firmware": d.text("Firmware + exact version"),
        "q_frame": d.text("Q_FRAME_CLASS"),
        "airspeed_sensor": d.text("Airspeed sensor fitted"),
        "gps": d.text("GPS make/model"),
    }
    res.gap("full .param export not attached — params.parm stays a placeholder "
            "(SITL boots with template params, not the real controller's)")
    res.gap("dataflash .BIN logs not attached — no real-vs-sim validation flight yet")

    res.spec["lift_motor"] = {
        "model": d.text("Lift motor make/model"),
        "kv": d.num("Motor KV rating", after="Lift motor make/model"),
        "max_current_a": d.num("Motor max continuous current"),
        "prop": d.text("Prop make/model"),
        "prop_diameter_in": None, "prop_pitch_in": None,
        "prop_blades": d.num("Prop blade count"),
        "esc": d.text("ESC make/model + current rating"),
        "pwm_min": None, "pwm_max": None,
    }
    dp = d.raw("Prop diameter × pitch")
    if dp:
        parts = [_num(p) for p in re.split(r"[×x]", dp)]
        if len(parts) == 2:
            res.spec["lift_motor"]["prop_diameter_in"], res.spec["lift_motor"]["prop_pitch_in"] = parts
    ep = d.raw("ESC PWM endpoints")
    if ep:
        nums = re.findall(r"\d{3,4}", ep)
        if len(nums) >= 2:
            res.spec["lift_motor"]["pwm_min"], res.spec["lift_motor"]["pwm_max"] = int(nums[0]), int(nums[1])

    # thrust sweep table: any data rows filled?
    sweep_rows = re.findall(r"^\s*\d{4}\s*\|\s*\d+\s*\|\s*([^\s|_]+)", text, re.M)
    if not sweep_rows:
        res.gap("lift-motor thrust-stand sweep empty — k_thrust/k_torque derived from "
                "datasheet + hover-throttle assumption (PLACEHOLDER, needs stand data or .param)")

    res.spec["pusher_motor"] = {
        "model": d.text("Forward motor make/model"),
        "kv": d.num("Motor KV rating", after="Forward motor make/model"),
        "prop": d.text("Propeller make/model"),
        "prop_diameter_in": None, "prop_pitch_in": None,
        "prop_blades": d.num("Blade count", after="Forward motor make/model"),
        "esc": d.text("ESC make/model + rating", after="Forward motor make/model"),
    }
    dp = d.raw("Diameter × pitch", after="Forward motor make/model")
    if dp:
        parts = [_num(p) for p in re.split(r"[×x]", dp)]
        if len(parts) == 2:
            res.spec["pusher_motor"]["prop_diameter_in"], res.spec["pusher_motor"]["prop_pitch_in"] = parts
    if "thrust vs airspeed" not in text or not re.search(r"at throttle\s*_*\s*\d", text):
        res.gap("pusher thrust-vs-airspeed data empty — static coefficient only, "
                "prop unloading stays inside effective cd0 (gokce-style)")

    lag = d.raw("Lift motor: ", after="MOTOR RESPONSE TIME")
    if _num(lag) is None:
        res.gap("motor spin-up lag not measured — motor_time_constant_s keeps default 0.05 s")

    res.spec["battery"] = {
        "chemistry": d.text("Chemistry"),
        "config": d.text("Cell count / config"),
        "capacity_mah": d.num("Capacity (mAh)"),
        "voltage_full_v": None,
        "internal_resistance_mohm": d.num("Pack internal resistance"),
        "hover_current_a": d.num("Hover current draw"),
        "cruise_current_a": d.num("Cruise current draw"),
    }
    tv = d.raw("Typical voltage")
    if tv:
        m = re.search(r"full\s*([\d.,]+)\s*V", tv)
        if m:
            res.spec["battery"]["voltage_full_v"] = _num(m.group(1))
    cfg = res.spec["battery"]["config"]
    if cfg:
        m = re.match(r"(\d+)S", cfg, re.I)
        res.spec["battery"]["cells_s"] = int(m.group(1)) if m else None
    if res.spec["battery"]["hover_current_a"] is None:
        res.gap("hover/cruise current draw N/A — battery endurance model unanchored")

    res.spec["servos"] = {
        "models": d.text("Servo make/model"),
        "rated_speed": d.text("Rated speed"),
    }
    # slowest servo wins for the sim's single servo_speed_deg_s
    sp = d.raw("Rated speed")
    if sp:
        secs = [float(x) for x in re.findall(r"(0[.,]\d+)\s*s\s*/\s*60", re.sub(r"(\d),(\d)", r"\1.\2", sp))]
        if secs:
            res.spec["servos"]["speed_deg_s"] = round(60.0 / max(secs))

    return res

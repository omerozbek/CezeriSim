#!/usr/bin/env python3
"""
compare_logs.py — Compare a real flight log (Mission Planner / ArduPilot) with a
simulation log and generate a single-file interactive HTML report.

Accepted log formats (both sides, any mix):
  .bin / .BIN / .log   ArduPilot dataflash (Mission Planner "Download DataFlash Log",
                       or the sim's  Scripts/control/logs/gokce_mission_*.bin)
  .tlog                Mission Planner / GCS telemetry log
  .csv                 fly_mission.py 4 Hz telemetry CSV
                       (t,wp,mode,lat,lon,alt_rel,airspeed,groundspeed,climb,...)

Usage:
    python compare_logs.py REAL_LOG SIM_LOG [options]
    python compare_logs.py                  (no args -> file-picker dialogs)

Options:
    -o OUT.html          output report path (default: next to REAL_LOG)
    --real-seg N         armed segment of the real log to use, 1-based
    --sim-seg N          armed segment of the sim log to use, 1-based
                         (default: the LONGEST armed segment of each log)
    --align takeoff|arm  t=0 at first climb through --takeoff-alt (default),
                         or at arming
    --takeoff-alt M      altitude threshold for takeoff alignment (default 2 m)
    --shift-sim SEC      extra time shift added to the sim trace (+ = later)
    --labels REAL SIM    display names (default: Real, Sim)
    --list-segments      print the armed segments of both files and exit
    --no-map             skip the OpenStreetMap panel (offline reports)
    --no-open            do not open the report in the browser afterwards

The report contains: stat tiles, track overlay (local East/North + map),
synced time series (altitude, airspeed, groundspeed, climb, roll, pitch,
throttle), mode timelines, a sim-minus-real difference table, and a 1 Hz
aligned data table.
"""
import argparse
import math
import os
import sys
import webbrowser
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# palette / chrome tokens (dataviz reference palette, light mode — validated)
# ---------------------------------------------------------------------------
C_REAL = "#2a78d6"          # categorical slot 1 (blue)
C_SIM = "#1baf7a"           # categorical slot 2 (aqua) — sub-3:1: relief via
                            # direct labels + table view (both present)
MODE_SLOTS = ["#eda100", "#008300", "#4a3aa7", "#e34948", "#e87ba4", "#eb6834"]
C_OTHER = "#898781"         # fold-over for >6 distinct modes
SURFACE = "#fcfcfb"
PAGE = "#f9f9f7"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
BORDER = "rgba(11,11,11,0.10)"
FONT = 'system-ui, -apple-system, "Segoe UI", sans-serif'

GRID_DT = 0.2               # s — common resample grid (5 Hz)

PLANE_MODES = {0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING",
               4: "ACRO", 5: "FBWA", 6: "FBWB", 7: "CRUISE", 8: "AUTOTUNE",
               10: "AUTO", 11: "RTL", 12: "LOITER", 13: "TAKEOFF",
               14: "AVOID_ADSB", 15: "GUIDED", 17: "QSTABILIZE", 18: "QHOVER",
               19: "QLOITER", 20: "QLAND", 21: "QRTL", 22: "QAUTOTUNE",
               23: "QACRO", 24: "THERMAL", 25: "LOITER2QLAND"}

CHANNELS = ["alt_rel", "airspeed", "groundspeed", "climb",
            "roll", "pitch", "throttle", "heading", "voltage", "current"]
CH_META = {  # channel -> (panel title, unit)
    "alt_rel": ("Altitude (relative)", "m"),
    "airspeed": ("Airspeed", "m/s"),
    "groundspeed": ("Groundspeed", "m/s"),
    "climb": ("Climb rate", "m/s"),
    "roll": ("Roll", "deg"),
    "pitch": ("Pitch", "deg"),
    "throttle": ("Throttle", "%"),
    "heading": ("Heading", "deg"),
    "voltage": ("Battery voltage", "V"),
    "current": ("Battery current", "A"),
}
TS_PANELS = ["alt_rel", "airspeed", "groundspeed", "climb",
             "roll", "pitch", "throttle"]      # heading: table only (wraps)
                                               # voltage/current: appended when
                                               # either log carries data

# ---- VTOL regime segmentation thresholds ----------------------------------
# Mode timelines can't mark transitions (the whole mission is AUTO), so the
# regimes come from smoothed airspeed + altitude instead.
CRUISE_AIRSPEED_MS = 12.0   # m/s — wing-borne above this (same as tune_params)
HOVER_AIRSPEED_MS = 7.0     # m/s — rotor-borne below this while airborne
AIRBORNE_ALT_M = 1.0        # m   — below this the vehicle is on the ground


# ---------------------------------------------------------------------------
# raw parsing -> RawLog
# ---------------------------------------------------------------------------
@dataclass
class RawLog:
    path: str
    kind: str                                   # 'dataflash' | 'tlog' | 'csv'
    series: dict = field(default_factory=dict)  # name -> (t[], v[]) np arrays
    mode_events: list = field(default_factory=list)   # [(t, name)]
    segments: list = field(default_factory=list)      # [(t0, t1)] armed spans
    waypoints: list = field(default_factory=list)     # [(seq, lat, lon)]


def _np(pairs):
    if not pairs:
        return np.empty(0), np.empty(0)
    a = np.asarray(pairs, dtype=float)
    return a[:, 0], a[:, 1]


def parse_dataflash(path):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    want = ['ATT', 'GPS', 'POS', 'CTUN', 'QTUN', 'BARO', 'MODE', 'EV', 'CMD',
            'BAT', 'CURR']
    col = {k: [] for k in ['lat', 'lon', 'alt_rel', 'gspd', 'climb_baro',
                           'climb_gps', 'roll', 'pitch', 'heading',
                           'thr_ct', 'thr_qt', 'aspd', 'volt', 'curr']}
    raw = RawLog(path=path, kind='dataflash')
    ev = []
    t_min, t_max = None, None
    while True:
        msg = m.recv_match(type=want)
        if msg is None:
            break
        t = msg.TimeUS / 1e6
        t_min = t if t_min is None else t_min
        t_max = t
        typ = msg.get_type()
        if typ == 'ATT':
            col['roll'].append((t, msg.Roll))
            col['pitch'].append((t, msg.Pitch))
            col['heading'].append((t, msg.Yaw))
        elif typ == 'POS':
            col['lat'].append((t, msg.Lat))
            col['lon'].append((t, msg.Lng))
            if hasattr(msg, 'RelHomeAlt'):
                col['alt_rel'].append((t, msg.RelHomeAlt))
        elif typ == 'GPS':
            if getattr(msg, 'I', 0) != 0:
                continue
            col['gspd'].append((t, msg.Spd))
            if hasattr(msg, 'VZ'):
                col['climb_gps'].append((t, -msg.VZ))
            if not col['lat'] or col['lat'][-1][0] < t - 5:
                col['lat'].append((t, msg.Lat))
                col['lon'].append((t, msg.Lng))
        elif typ == 'CTUN':
            if hasattr(msg, 'ThO'):
                col['thr_ct'].append((t, msg.ThO))
            for f in ('As', 'SAs', 'Aspd'):
                if hasattr(msg, f):
                    col['aspd'].append((t, getattr(msg, f)))
                    break
        elif typ == 'QTUN':
            if hasattr(msg, 'ThO'):
                col['thr_qt'].append((t, msg.ThO))
        elif typ == 'BARO':
            if getattr(msg, 'I', 0) == 0 and hasattr(msg, 'CRt'):
                col['climb_baro'].append((t, msg.CRt))
        elif typ in ('BAT', 'CURR'):            # BAT = modern, CURR = legacy
            if getattr(msg, 'Inst', getattr(msg, 'Instance', 0)) != 0:
                continue                        # first battery only
            if hasattr(msg, 'Volt'):
                col['volt'].append((t, msg.Volt))
            if hasattr(msg, 'Curr'):
                col['curr'].append((t, msg.Curr))
        elif typ == 'MODE':
            num = getattr(msg, 'ModeNum', getattr(msg, 'Mode', None))
            raw.mode_events.append((t, PLANE_MODES.get(num, f"MODE{num}")))
        elif typ == 'EV':
            if msg.Id in (10, 11):              # 10 armed / 11 disarmed
                ev.append((t, msg.Id))
        elif typ == 'CMD':
            if msg.Lat != 0 or msg.Lng != 0:
                raw.waypoints.append((int(msg.CNum), msg.Lat, msg.Lng))

    if t_min is None:
        raise ValueError(f"{path}: no usable messages found")

    for k, v in col.items():
        raw.series[k] = _np(v)

    # armed segments from EV 10/11; log start counts as armed if the first
    # event is a disarm (AP normally only logs while armed)
    segs, start = [], None
    for t, i in ev:
        if i == 10 and start is None:
            start = t
        elif i == 11:
            segs.append((start if start is not None else t_min, t))
            start = None
    if start is not None:
        segs.append((start, t_max))
    if not segs:
        segs = [(t_min, t_max)]
    raw.segments = segs
    return raw


def parse_tlog(path):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    col = {k: [] for k in ['lat', 'lon', 'alt_rel', 'gspd', 'climb_baro',
                           'roll', 'pitch', 'heading', 'thr_ct', 'aspd',
                           'volt', 'curr']}
    raw = RawLog(path=path, kind='tlog')
    ev = []
    armed = None
    t_min, t_max = None, None
    while True:
        msg = m.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD',
                                 'ATTITUDE', 'HEARTBEAT', 'SYS_STATUS'])
        if msg is None:
            break
        t = getattr(msg, '_timestamp', None)
        if t is None:
            continue
        t_min = t if t_min is None else t_min
        t_max = t
        typ = msg.get_type()
        if typ == 'GLOBAL_POSITION_INT':
            col['lat'].append((t, msg.lat / 1e7))
            col['lon'].append((t, msg.lon / 1e7))
            col['alt_rel'].append((t, msg.relative_alt / 1000.0))
            col['heading'].append((t, msg.hdg / 100.0))
        elif typ == 'VFR_HUD':
            col['aspd'].append((t, msg.airspeed))
            col['gspd'].append((t, msg.groundspeed))
            col['climb_baro'].append((t, msg.climb))
            col['thr_ct'].append((t, msg.throttle))
        elif typ == 'ATTITUDE':
            col['roll'].append((t, math.degrees(msg.roll)))
            col['pitch'].append((t, math.degrees(msg.pitch)))
        elif typ == 'SYS_STATUS':
            if msg.voltage_battery not in (0, 65535):     # mV, 65535 = n/a
                col['volt'].append((t, msg.voltage_battery / 1000.0))
            if msg.current_battery != -1:                 # cA, -1 = n/a
                col['curr'].append((t, msg.current_battery / 100.0))
        elif typ == 'HEARTBEAT':
            if msg.get_srcComponent() not in (0, 1) or msg.type == 6:
                continue                        # ignore GCS heartbeats
            a = bool(msg.base_mode & 128)
            if a != armed:
                ev.append((t, 10 if a else 11))
                armed = a
            name = PLANE_MODES.get(msg.custom_mode, f"MODE{msg.custom_mode}")
            if not raw.mode_events or raw.mode_events[-1][1] != name:
                raw.mode_events.append((t, name))

    if t_min is None:
        raise ValueError(f"{path}: no usable messages found")
    for k, v in col.items():
        raw.series[k] = _np(v)

    segs, start = [], None
    for t, i in ev:
        if i == 10 and start is None:
            start = t
        elif i == 11 and start is not None:
            segs.append((start, t))
            start = None
    if start is not None:
        segs.append((start, t_max))
    if not segs:
        segs = [(t_min, t_max)]
    raw.segments = segs
    return raw


def parse_csv(path):
    df = pd.read_csv(path)
    need = {'t', 'lat', 'lon', 'alt_rel'}
    if not need.issubset(df.columns):
        raise ValueError(f"{path}: not a fly_mission telemetry CSV "
                         f"(columns {list(df.columns)})")
    df = df.dropna(subset=['lat', 'lon'])
    t = df['t'].to_numpy(dtype=float)
    raw = RawLog(path=path, kind='csv')

    def put(name, colname, scale=1.0):
        if colname in df.columns:
            v = pd.to_numeric(df[colname], errors='coerce').to_numpy()
            ok = ~np.isnan(v)
            raw.series[name] = (t[ok], v[ok] * scale)
        else:
            raw.series[name] = (np.empty(0), np.empty(0))

    put('lat', 'lat'); put('lon', 'lon'); put('alt_rel', 'alt_rel')
    put('aspd', 'airspeed'); put('gspd', 'groundspeed')
    put('climb_baro', 'climb'); put('roll', 'roll'); put('pitch', 'pitch')
    put('heading', 'yaw_hdg'); put('thr_ct', 'throttle')
    put('volt', 'voltage'); put('curr', 'current')
    if 'mode' in df.columns:
        last = None
        for tt, mm in zip(t, df['mode'].astype(str)):
            if mm != last:
                raw.mode_events.append((float(tt), mm))
                last = mm
    raw.segments = [(float(t[0]), float(t[-1]))]
    return raw


def parse_any(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == '.csv':
        return parse_csv(path)
    if ext == '.tlog':
        return parse_tlog(path)
    return parse_dataflash(path)                # .bin / .log / anything else


# ---------------------------------------------------------------------------
# segment extraction + alignment -> Flight
# ---------------------------------------------------------------------------
@dataclass
class Flight:
    label: str
    color: str
    raw: RawLog
    seg_index: int                              # 0-based
    df: pd.DataFrame = None                     # aligned, resampled channels
    modes: list = None                          # [(t_start, t_end, name)]
    home: tuple = None                          # (lat, lon)
    align_note: str = ""


def _interp(grid_abs, series, wrap=None):
    t, v = series
    if len(t) < 2:
        return np.full(len(grid_abs), np.nan)
    if wrap:                                    # unwrap circular channel
        v = np.degrees(np.unwrap(np.radians(v)))
    out = np.interp(grid_abs, t, v)
    if wrap:
        out = np.mod(out, 360.0)
    return out


def build_flight(raw, seg_index, label, color, align, takeoff_alt, shift):
    t0, t1 = raw.segments[seg_index]
    grid_abs = np.arange(t0, t1, GRID_DT)
    d = {}
    d['lat'] = _interp(grid_abs, raw.series['lat'])
    d['lon'] = _interp(grid_abs, raw.series['lon'])
    d['alt_rel'] = _interp(grid_abs, raw.series['alt_rel'])
    d['airspeed'] = _interp(grid_abs, raw.series.get('aspd', ((), ())))
    d['groundspeed'] = _interp(grid_abs, raw.series.get('gspd', ((), ())))
    d['roll'] = _interp(grid_abs, raw.series.get('roll', ((), ())))
    d['pitch'] = _interp(grid_abs, raw.series.get('pitch', ((), ())))
    d['heading'] = _interp(grid_abs, raw.series.get('heading', ((), ())),
                           wrap=True)

    cb = raw.series.get('climb_baro', (np.empty(0),) * 2)
    cg = raw.series.get('climb_gps', (np.empty(0),) * 2)
    d['climb'] = _interp(grid_abs, cb if len(cb[0]) > 1 else cg)
    if np.all(np.isnan(d['climb'])) and not np.all(np.isnan(d['alt_rel'])):
        d['climb'] = np.gradient(d['alt_rel'], GRID_DT)   # last resort

    # throttle: fixed-wing CTUN.ThO (%), overlaid with QTUN.ThO (0-1) where
    # the quad motors are active (VTOL phases)
    thr = _interp(grid_abs, raw.series.get('thr_ct', ((), ())))
    qt_t, qt_v = raw.series.get('thr_qt', (np.empty(0),) * 2)
    if len(qt_t) > 1:
        if np.nanmax(qt_v) <= 1.5:
            qt_v = qt_v * 100.0
        q = np.interp(grid_abs, qt_t, qt_v)
        idx = np.searchsorted(qt_t, grid_abs).clip(1, len(qt_t) - 1)
        near = np.minimum(np.abs(qt_t[idx] - grid_abs),
                          np.abs(qt_t[idx - 1] - grid_abs))
        valid = near < 1.0                      # quad throttle being logged
        thr = np.where(valid & (q > thr), q, thr)
    d['throttle'] = thr

    d['voltage'] = _interp(grid_abs, raw.series.get('volt', ((), ())))
    d['current'] = _interp(grid_abs, raw.series.get('curr', ((), ())))

    # t = 0 reference
    t_rel = grid_abs - t0
    align_note = "t=0 at arming"
    if align == 'takeoff':
        alt = d['alt_rel']
        ok = np.where(~np.isnan(alt) & (alt >= takeoff_alt))[0]
        if len(ok):
            t_rel = t_rel - t_rel[ok[0]]
            align_note = f"t=0 at climb through {takeoff_alt:g} m"
        else:
            align_note = (f"t=0 at arming (never reached {takeoff_alt:g} m — "
                          "takeoff alignment skipped)")
    t_rel = t_rel + shift
    if shift:
        align_note += f", shifted {shift:+g} s"

    df = pd.DataFrame(d)
    df.insert(0, 't', t_rel)

    # home + local East/North track (removes the sim-vs-real home offset)
    ok = np.where(~np.isnan(d['lat']))[0]
    home = (d['lat'][ok[0]], d['lon'][ok[0]]) if len(ok) else (np.nan, np.nan)
    if not math.isnan(home[0]):
        m_lat = 111132.95
        m_lon = 111319.49 * math.cos(math.radians(home[0]))
        df['east'] = (df['lon'] - home[1]) * m_lon
        df['north'] = (df['lat'] - home[0]) * m_lat
    else:
        df['east'] = np.nan
        df['north'] = np.nan

    # modes clipped to the segment, rebased to aligned time
    off = t_rel[0] - (grid_abs[0] - t0)         # abs -> aligned offset
    events = [(t - t0 + off, name) for t, name in raw.mode_events]
    pre = [e for e in events if e[0] <= t_rel[0]]
    events = ([(t_rel[0], pre[-1][1])] if pre else []) + \
             [e for e in events if t_rel[0] < e[0] < t_rel[-1]]
    modes = []
    for i, (tt, name) in enumerate(events):
        te = events[i + 1][0] if i + 1 < len(events) else t_rel[-1]
        if modes and modes[-1][2] == name:
            modes[-1] = (modes[-1][0], te, name)
        else:
            modes.append((tt, te, name))

    fl = Flight(label=label, color=color, raw=raw, seg_index=seg_index,
                df=df, modes=modes, home=home, align_note=align_note)
    return fl


def pick_segment(raw, requested):
    if requested is not None:
        if not (1 <= requested <= len(raw.segments)):
            raise SystemExit(
                f"{os.path.basename(raw.path)}: segment {requested} out of "
                f"range (log has {len(raw.segments)})")
        return requested - 1
    durs = [t1 - t0 for t0, t1 in raw.segments]
    return int(np.argmax(durs))


# ---------------------------------------------------------------------------
# metrics
# ---------------------------------------------------------------------------
def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def flight_summary(fl):
    df = fl.df
    dur = df['t'].iloc[-1] - df['t'].iloc[0]
    en = df[['east', 'north']].to_numpy()
    step = np.linalg.norm(np.diff(en, axis=0), axis=1)
    dist = float(np.nansum(step))
    aspd = df['airspeed'].to_numpy()
    cruise = aspd > 12.0                        # fixed-wing phase
    return {
        'duration': dur,
        'distance': dist,
        'max_alt': float(np.nanmax(df['alt_rel'])),
        'cruise_aspd': float(np.nanmean(aspd[cruise])) if cruise.any() else float('nan'),
        'cruise_thr': float(np.nanmean(df['throttle'][cruise])) if cruise.any() else float('nan'),
        'max_roll': float(np.nanmax(np.abs(df['roll']))),
        'max_pitch': float(np.nanmax(np.abs(df['pitch']))),
    }


def aligned_deltas(real, sim):
    """Interpolate sim onto real's grid over the overlap; sim - real stats."""
    r, s = real.df, sim.df
    lo = max(r['t'].iloc[0], s['t'].iloc[0])
    hi = min(r['t'].iloc[-1], s['t'].iloc[-1])
    if hi - lo < 5:
        return None, (lo, hi)
    rw = r[(r['t'] >= lo) & (r['t'] <= hi)]
    out = {}
    for ch in CHANNELS:
        rv = rw[ch].to_numpy()
        sv = np.interp(rw['t'], s['t'], s[ch])
        dd = sv - rv
        if ch == 'heading':
            dd = wrap180(dd)
        ok = ~np.isnan(dd)
        if not ok.any():
            continue
        dd = dd[ok]
        out[ch] = {'bias': float(np.mean(dd)),
                   'rms': float(np.sqrt(np.mean(dd ** 2))),
                   'maxabs': float(np.max(np.abs(dd)))}
    return out, (lo, hi)


# ---------------------------------------------------------------------------
# VTOL regime segmentation + per-regime metrics (the digital-twin scorecard)
# ---------------------------------------------------------------------------
def _spans_of(mask, t, min_dur):
    """Contiguous True runs of mask -> [(t0, t1)], runs shorter than min_dur
    seconds dropped (sensor blips must not split a regime)."""
    out, start = [], None
    m = np.asarray(mask, dtype=bool)
    for i, v in enumerate(m):
        if v and start is None:
            start = i
        elif not v and start is not None:
            if t[i - 1] - t[start] >= min_dur:
                out.append((float(t[start]), float(t[i - 1])))
            start = None
    if start is not None and t[-1] - t[start] >= min_dur:
        out.append((float(t[start]), float(t[-1])))
    return out


def segment_regimes(fl):
    """Split one flight into VTOL regimes from smoothed airspeed + altitude:
    hover (airborne, rotor-borne), cruise (wing-borne), transition_out (gap
    between the last hover before the first cruise and that cruise) and
    transition_in (the reverse at the end). Mode timelines can't provide this
    — a whole mission is one AUTO block — so airspeed defines it."""
    df = fl.df
    t = df['t'].to_numpy()
    alt = df['alt_rel'].to_numpy()
    aspd = pd.Series(df['airspeed'].to_numpy()).rolling(
        11, center=True, min_periods=3).median().to_numpy()   # ~2 s median
    airborne = ~np.isnan(alt) & (alt > AIRBORNE_ALT_M)
    ok = ~np.isnan(aspd)
    cruise = _spans_of(airborne & ok & (aspd >= CRUISE_AIRSPEED_MS), t, 5.0)
    hover = _spans_of(airborne & ok & (aspd <= HOVER_AIRSPEED_MS), t, 3.0)
    spans = {'hover': hover, 'cruise': cruise,
             'transition_out': [], 'transition_in': []}
    if cruise:
        c0, c1 = cruise[0][0], cruise[-1][1]
        pre = [h for h in hover if h[1] <= c0]
        post = [h for h in hover if h[0] >= c1]
        if pre and 0.0 < c0 - pre[-1][1] < 90.0:
            spans['transition_out'] = [(pre[-1][1], c0)]
        if post and 0.0 < post[0][0] - c1 < 90.0:
            spans['transition_in'] = [(c1, post[0][0])]
    return spans


def _mask_of(t, spans):
    m = np.zeros(len(t), dtype=bool)
    for t0, t1 in spans:
        m |= (t >= t0) & (t <= t1)
    return m


def _nanmean(v):
    v = v[~np.isnan(v)]
    return float(v.mean()) if len(v) else None


def regime_metrics(fl):
    """Per-regime scalar metrics for one flight — the numbers the digital-twin
    acceptance gates are defined on (docs/ROADMAP.md): hover throttle,
    transition duration / altitude loss / peak pitch, cruise airspeed /
    throttle / pitch trim / climb+sink rates, and battery power draw."""
    df = fl.df
    t = df['t'].to_numpy()
    spans = segment_regimes(fl)
    out = {'spans': {k: [[round(a, 1), round(b, 1)] for a, b in v]
                     for k, v in spans.items()}}

    hm = _mask_of(t, spans['hover'])
    if hm.any():
        out['hover'] = {
            'duration_s': round(float(hm.sum()) * GRID_DT, 1),
            'throttle_mean': _nanmean(df['throttle'].to_numpy()[hm]),
            'voltage_mean': _nanmean(df['voltage'].to_numpy()[hm]),
            'current_mean': _nanmean(df['current'].to_numpy()[hm]),
        }

    cm = _mask_of(t, spans['cruise'])
    if cm.any():
        climb = df['climb'].to_numpy()[cm]
        climb = climb[~np.isnan(climb)]
        # detrended altitude: TECS oscillation without the commanded profile
        alt = pd.Series(df['alt_rel'].to_numpy())
        trend = alt.rolling(101, center=True, min_periods=25).median()
        osc = (alt - trend).to_numpy()[cm]
        osc = osc[~np.isnan(osc)]
        out['cruise'] = {
            'duration_s': round(float(cm.sum()) * GRID_DT, 1),
            'airspeed_mean': _nanmean(df['airspeed'].to_numpy()[cm]),
            'throttle_mean': _nanmean(df['throttle'].to_numpy()[cm]),
            'pitch_mean': _nanmean(df['pitch'].to_numpy()[cm]),
            'climb_rate_mean': (float(climb[climb > 0.5].mean())
                                if (climb > 0.5).any() else None),
            'sink_rate_mean': (float(-climb[climb < -0.5].mean())
                               if (climb < -0.5).any() else None),
            'alt_osc_rms': (float(np.sqrt(np.mean(osc ** 2)))
                            if len(osc) else None),
            'voltage_mean': _nanmean(df['voltage'].to_numpy()[cm]),
            'current_mean': _nanmean(df['current'].to_numpy()[cm]),
        }

    for key in ('transition_out', 'transition_in'):
        if not spans[key]:
            continue
        t0, t1 = spans[key][0]
        m = (t >= t0) & (t <= t1)
        alt = df['alt_rel'].to_numpy()[m]
        alt = alt[~np.isnan(alt)]
        pitch = df['pitch'].to_numpy()[m]
        entry = {'duration_s': round(t1 - t0, 1)}
        if len(alt):
            entry['alt_loss_m'] = round(float(alt[0] - alt.min()), 1)
        if not np.all(np.isnan(pitch)):
            entry['peak_pitch_deg'] = round(float(np.nanmax(np.abs(pitch))), 1)
        out[key] = entry
    return out


def regime_deltas(mr, ms):
    """sim − real for every numeric metric both flights have, per regime."""
    out = {}
    for reg in ('hover', 'transition_out', 'cruise', 'transition_in'):
        a, b = mr.get(reg), ms.get(reg)
        if not a or not b:
            continue
        d = {}
        for k, va in a.items():
            vb = b.get(k)
            if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
                d[k] = round(vb - va, 3)
        if d:
            out[reg] = d
    return out


def en_from(df, origin):
    """East/North (m) of a flight's lat/lon relative to a common origin."""
    m_lat = 111132.95
    m_lon = 111319.49 * math.cos(math.radians(origin[0]))
    return ((df['lon'] - origin[1]) * m_lon).to_numpy(), \
           ((df['lat'] - origin[0]) * m_lat).to_numpy()


def track_separation(real, sim):
    """Nearest-neighbour distance from each sim point to the real track,
    in the SHARED frame (real home origin) — time-independent geometry
    comparison that keeps any real-vs-sim home offset visible."""
    if math.isnan(real.home[0]) or math.isnan(sim.home[0]):
        return None
    ae, an = en_from(sim.df.dropna(subset=['lat', 'lon']), real.home)
    be, bn = en_from(real.df.dropna(subset=['lat', 'lon']), real.home)
    a = np.stack([ae, an], axis=1)
    b = np.stack([be, bn], axis=1)
    if len(a) < 2 or len(b) < 2:
        return None
    mins = np.empty(len(a))
    for i in range(0, len(a), 512):
        chunk = a[i:i + 512]
        dd = np.linalg.norm(chunk[:, None, :] - b[None, :, :], axis=2)
        mins[i:i + 512] = dd.min(axis=1)
    return {'mean': float(mins.mean()),
            'p95': float(np.percentile(mins, 95)),
            'max': float(mins.max())}


# ---------------------------------------------------------------------------
# report
# ---------------------------------------------------------------------------
def _fig_layout(fig, height, title=None):
    fig.update_layout(
        height=height, template=None,
        paper_bgcolor=SURFACE, plot_bgcolor=SURFACE,
        font=dict(family=FONT, size=12, color=INK2),
        title=(dict(text=title, font=dict(size=14, color=INK), x=0)
               if title else None),
        margin=dict(l=56, r=90, t=(44 if title else 24), b=44),
        legend=dict(orientation='h', yanchor='bottom', y=1.0, xanchor='right',
                    x=1.0, font=dict(color=INK2)),
        hoverlabel=dict(bgcolor='#ffffff', bordercolor=AXIS,
                        font=dict(family=FONT, size=12, color=INK)),
    )
    fig.update_xaxes(gridcolor=GRID, gridwidth=1, zeroline=False,
                     linecolor=AXIS, tickcolor=AXIS,
                     tickfont=dict(color=MUTED),
                     title_font=dict(color=MUTED))
    fig.update_yaxes(gridcolor=GRID, gridwidth=1, zeroline=False,
                     linecolor=AXIS, tickcolor=AXIS,
                     tickfont=dict(color=MUTED),
                     title_font=dict(color=MUTED))
    return fig


def fig_track(real, sim, waypoints, frame='shared'):
    import plotly.graph_objects as go
    fig = go.Figure()
    if frame == 'shared':
        origin_note = f'from {real.label} home'
    else:
        origin_note = 'from own home — pattern overlay, home offset removed'
        waypoints = None                        # ambiguous in own-home frames
    if waypoints:
        wlat = np.array([w[1] for w in waypoints])
        wlon = np.array([w[2] for w in waypoints])
        m_lat = 111132.95
        m_lon = 111319.49 * math.cos(math.radians(real.home[0]))
        we = (wlon - real.home[1]) * m_lon
        wn = (wlat - real.home[0]) * m_lat
        fig.add_trace(go.Scatter(
            x=we, y=wn, mode='markers+text', name='Mission WPs',
            marker=dict(symbol='diamond-open', size=9, color=MUTED,
                        line=dict(width=1.5)),
            text=[str(w[0]) for w in waypoints], textposition='top center',
            textfont=dict(size=10, color=MUTED),
            hovertemplate='WP %{text}<extra></extra>'))
    for fl in (real, sim):
        df = fl.df
        if frame == 'shared':
            xe, yn = en_from(df, real.home)
        else:
            xe, yn = df['east'].to_numpy(), df['north'].to_numpy()
        fig.add_trace(go.Scatter(
            x=xe, y=yn, mode='lines', name=fl.label,
            line=dict(color=fl.color, width=2),
            customdata=np.stack([df['t'], df['alt_rel']], axis=1),
            hovertemplate=(fl.label + '  t=%{customdata[0]:.1f}s  '
                           'alt=%{customdata[1]:.1f}m<extra></extra>')))
        # takeoff marker with surface ring
        fig.add_trace(go.Scatter(
            x=[xe[0]], y=[yn[0]], mode='markers',
            marker=dict(size=10, color=fl.color,
                        line=dict(width=2, color=SURFACE)),
            showlegend=False, hovertemplate=fl.label + ' start<extra></extra>'))
    _fig_layout(fig, 560)
    fig.update_xaxes(title_text=f'East (m, {origin_note})')
    fig.update_yaxes(title_text='North (m)', scaleanchor='x', scaleratio=1)
    return fig


def fig_map(real, sim, waypoints):
    import plotly.graph_objects as go
    fig = go.Figure()
    for fl in (real, sim):
        df = fl.df.dropna(subset=['lat', 'lon'])
        fig.add_trace(go.Scattermap(
            lat=df['lat'], lon=df['lon'], mode='lines', name=fl.label,
            line=dict(color=fl.color, width=2),
            hovertemplate=fl.label + '<extra></extra>'))
    if waypoints:
        fig.add_trace(go.Scattermap(
            lat=[w[1] for w in waypoints], lon=[w[2] for w in waypoints],
            mode='markers', name='Mission WPs',
            marker=dict(size=8, color=MUTED),
            text=[f"WP {w[0]}" for w in waypoints],
            hovertemplate='%{text}<extra></extra>'))
    lat0 = np.nanmean(real.df['lat'])
    lon0 = np.nanmean(real.df['lon'])
    fig.update_layout(
        map=dict(style='open-street-map',
                 center=dict(lat=float(lat0), lon=float(lon0)), zoom=14),
        height=560, paper_bgcolor=SURFACE,
        font=dict(family=FONT, size=12, color=INK2),
        margin=dict(l=8, r=8, t=24, b=8),
        legend=dict(orientation='h', yanchor='bottom', y=1.0, xanchor='right',
                    x=1.0, bgcolor='rgba(252,252,251,0.85)'))
    return fig


def fig_timeseries(real, sim, panels=None):
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    panels = panels or TS_PANELS
    n = len(panels)
    fig = make_subplots(
        rows=n, cols=1, shared_xaxes=True, vertical_spacing=0.028,
        subplot_titles=[f"{CH_META[c][0]} ({CH_META[c][1]})"
                        for c in panels])
    for i, ch in enumerate(panels, start=1):
        for fl in (real, sim):
            fig.add_trace(go.Scatter(
                x=fl.df['t'], y=fl.df[ch], mode='lines', name=fl.label,
                legendgroup=fl.label, showlegend=(i == 1),
                line=dict(color=fl.color, width=2),
                hovertemplate='%{y:.2f}<extra>' + fl.label + '</extra>'),
                row=i, col=1)
    _fig_layout(fig, 190 * n)
    fig.update_layout(hovermode='x unified', hoversubplots='axis',
                      legend=dict(y=1.005))
    fig.update_xaxes(showspikes=True, spikemode='across', spikethickness=1,
                     spikedash='solid', spikecolor=AXIS)
    fig.update_xaxes(title_text='t (s)', row=n, col=1)
    for a in fig.layout.annotations:            # subplot titles -> quiet ink
        if a.text in [f"{CH_META[c][0]} ({CH_META[c][1]})" for c in panels]:
            a.update(font=dict(size=12, color=INK), x=0, xanchor='left')
    return fig


def fig_modes(real, sim):
    import plotly.graph_objects as go
    names = []
    for fl in (real, sim):
        for _, _, name in fl.modes:
            if name not in names:
                names.append(name)
    color_of = {n: (MODE_SLOTS[i] if i < len(MODE_SLOTS) else C_OTHER)
                for i, n in enumerate(names)}
    fig = go.Figure()
    seen = set()
    for fl in (real, sim):
        for t0, t1, name in fl.modes:
            fig.add_trace(go.Bar(
                x=[t1 - t0], base=[t0], y=[fl.label], orientation='h',
                width=0.55, name=name, legendgroup=name,
                showlegend=name not in seen,
                marker=dict(color=color_of[name],
                            line=dict(color=SURFACE, width=2)),
                text=name, textposition='inside', insidetextanchor='middle',
                constraintext='inside',
                textfont=dict(family=FONT, size=11),
                hovertemplate=(f"{name}  {t0:.1f} - {t1:.1f} s "
                               f"({t1 - t0:.1f} s)<extra>{fl.label}</extra>")))
            seen.add(name)
    _fig_layout(fig, 220)
    fig.update_layout(barmode='overlay', bargap=0.3,
                      margin=dict(l=120),
                      legend=dict(y=1.02, font=dict(size=11)))
    fig.update_xaxes(title_text='t (s)')
    fig.update_yaxes(categoryorder='array',
                     categoryarray=[sim.label, real.label])
    return fig


def fmt(v, nd=1, unit=""):
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return "–"
    return f"{v:,.{nd}f}{unit}"


def stat_tiles(real, sim, sum_r, sum_s, sep):
    def tile(label, rv, sv, unit, nd=1):
        return f"""
        <div class="tile">
          <div class="tlabel">{label}</div>
          <div class="tpair">
            <span><i class="dot" style="background:{C_REAL}"></i>
                  <b>{fmt(rv, nd)}</b><small> {unit}</small></span>
            <span><i class="dot" style="background:{C_SIM}"></i>
                  <b>{fmt(sv, nd)}</b><small> {unit}</small></span>
          </div>
        </div>"""
    sep_html = f"""
        <div class="tile">
          <div class="tlabel">Mean track separation</div>
          <div class="tvalue">{fmt(sep['mean'] if sep else None, 1)}<small> m</small></div>
          <div class="tsub">p95 {fmt(sep['p95'] if sep else None, 1)} m ·
               max {fmt(sep['max'] if sep else None, 1)} m</div>
        </div>"""
    return f"""
    <div class="tiles">
      {tile("Flight duration", sum_r['duration'], sum_s['duration'], "s", 0)}
      {tile("Max altitude", sum_r['max_alt'], sum_s['max_alt'], "m")}
      {tile("Cruise airspeed (mean)", sum_r['cruise_aspd'], sum_s['cruise_aspd'], "m/s")}
      {sep_html}
    </div>"""


def summary_table(real, sim, sum_r, sum_s):
    rows = [("Duration", 'duration', "s", 0),
            ("Distance flown", 'distance', "m", 0),
            ("Max altitude", 'max_alt', "m", 1),
            ("Cruise airspeed (mean)", 'cruise_aspd', "m/s", 1),
            ("Cruise throttle (mean)", 'cruise_thr', "%", 1),
            ("Max |roll|", 'max_roll', "deg", 1),
            ("Max |pitch|", 'max_pitch', "deg", 1)]
    body = "".join(
        f"<tr><td>{lbl}</td><td>{fmt(sum_r[k], nd)} {u}</td>"
        f"<td>{fmt(sum_s[k], nd)} {u}</td></tr>"
        for lbl, k, u, nd in rows)
    return f"""
    <table class="metrics">
      <thead><tr><th></th>
        <th><i class="dot" style="background:{C_REAL}"></i>{real.label}</th>
        <th><i class="dot" style="background:{C_SIM}"></i>{sim.label}</th>
      </tr></thead><tbody>{body}</tbody>
    </table>"""


REGIME_ROW_META = {  # metric key -> (label, unit, decimals)
    'duration_s': ("Duration", "s", 1),
    'throttle_mean': ("Mean throttle", "%", 1),
    'airspeed_mean': ("Mean airspeed", "m/s", 1),
    'pitch_mean': ("Mean pitch (trim)", "deg", 1),
    'climb_rate_mean': ("Mean climb rate", "m/s", 2),
    'sink_rate_mean': ("Mean sink rate", "m/s", 2),
    'alt_osc_rms': ("Detrended alt RMS", "m", 2),
    'alt_loss_m': ("Altitude loss", "m", 1),
    'peak_pitch_deg': ("Peak |pitch|", "deg", 1),
    'voltage_mean': ("Mean voltage", "V", 2),
    'current_mean': ("Mean current", "A", 1),
}
REGIME_LABELS = {'hover': "Hover", 'transition_out': "Transition out",
                 'cruise': "Cruise", 'transition_in': "Transition in"}


def regime_table(mr, ms, deltas):
    """Per-regime real / sim / Δ table — the scorecard the acceptance gates
    read (same numbers as the summary JSON 'regimes' block)."""
    rows = ""
    for reg in ('hover', 'transition_out', 'cruise', 'transition_in'):
        a, b = mr.get(reg), ms.get(reg)
        if not a and not b:
            continue
        rows += (f"<tr class='rhead'><td colspan='5'>"
                 f"{REGIME_LABELS[reg]}</td></tr>")
        for k, (lbl, unit, nd) in REGIME_ROW_META.items():
            va = (a or {}).get(k)
            vb = (b or {}).get(k)
            if va is None and vb is None:
                continue
            d = deltas.get(reg, {}).get(k)
            dtxt = f"{d:+,.{nd}f}" if d is not None else "–"
            rows += (f"<tr><td>{lbl}</td><td>{unit}</td>"
                     f"<td>{fmt(va, nd)}</td><td>{fmt(vb, nd)}</td>"
                     f"<td>{dtxt}</td></tr>")
    if not rows:
        return ("<p class='note'>No regimes detected — the flight never "
                "reached a sustained hover or cruise phase.</p>")
    return f"""
    <table class="metrics">
      <thead><tr><th>Metric</th><th>Unit</th><th>Real</th><th>Sim</th>
      <th>Δ (sim − real)</th></tr></thead>
      <tbody>{rows}</tbody>
    </table>
    <p class="note">Regimes are segmented from smoothed airspeed + altitude
    (hover &le; {HOVER_AIRSPEED_MS:g} m/s, cruise &ge;
    {CRUISE_AIRSPEED_MS:g} m/s while airborne), each flight on its own
    timeline — so the comparison is regime-to-regime even when timing
    drifts.</p>"""


def delta_table(deltas, overlap):
    if not deltas:
        return ("<p class='note'>Overlap between the two flights is too short "
                "for aligned difference statistics.</p>")
    rows = ""
    for ch in CHANNELS:
        if ch not in deltas:
            continue
        d = deltas[ch]
        name, unit = CH_META[ch]
        rows += (f"<tr><td>{name}</td><td>{unit}</td>"
                 f"<td>{d['bias']:+,.2f}</td><td>{fmt(d['rms'], 2)}</td>"
                 f"<td>{fmt(d['maxabs'], 2)}</td></tr>")
    return f"""
    <table class="metrics">
      <thead><tr><th>Channel</th><th>Unit</th><th>Bias (sim − real)</th>
      <th>RMS</th><th>Max |Δ|</th></tr></thead>
      <tbody>{rows}</tbody>
    </table>
    <p class="note">Computed on the aligned overlap t = {overlap[0]:.1f} …
    {overlap[1]:.1f} s (sim interpolated onto the real timeline).
    Heading difference is wrapped to ±180°.</p>"""


def data_table(real, sim, panels=None):
    """1 Hz aligned table — the table-view twin of the time-series panels."""
    lo = max(real.df['t'].iloc[0], sim.df['t'].iloc[0])
    hi = min(real.df['t'].iloc[-1], sim.df['t'].iloc[-1])
    if hi <= lo:
        return ""
    tt = np.arange(math.ceil(lo), hi, 1.0)
    if len(tt) > 3600:
        tt = tt[:3600]
    cols = panels or TS_PANELS
    head = "<tr><th>t (s)</th>" + "".join(
        f"<th>{CH_META[c][0].split(' (')[0]} R</th><th>S</th>" for c in cols) \
        + "</tr>"
    rows = []
    rv = {c: np.interp(tt, real.df['t'], real.df[c]) for c in cols}
    sv = {c: np.interp(tt, sim.df['t'], sim.df[c]) for c in cols}
    for i, t in enumerate(tt):
        cells = "".join(f"<td>{rv[c][i]:.1f}</td><td>{sv[c][i]:.1f}</td>"
                        for c in cols)
        rows.append(f"<tr><td>{t:.0f}</td>{cells}</tr>")
    return f"""
    <details><summary>Aligned data table (1 Hz, R = real / S = sim)</summary>
    <div class="tscroll"><table class="metrics data">
    <thead>{head}</thead><tbody>{''.join(rows)}</tbody></table></div>
    </details>"""


CSS = """
:root { color-scheme: light; }
* { box-sizing: border-box; }
body { background: %(PAGE)s; color: %(INK)s; margin: 0;
       font-family: %(FONT)s; font-size: 14px; }
.wrap { max-width: 1180px; margin: 0 auto; padding: 28px 20px 60px; }
h1 { font-size: 20px; font-weight: 650; margin: 0 0 4px; }
h2 { font-size: 15px; font-weight: 650; margin: 34px 0 10px; }
.sub { color: %(INK2)s; margin: 0 0 18px; }
.files { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
.fcard { background: %(SURFACE)s; border: 1px solid %(BORDER)s;
         border-radius: 10px; padding: 12px 14px; }
.fcard .flabel { font-weight: 650; margin-bottom: 2px; }
.fcard .fpath { color: %(INK2)s; word-break: break-all; font-size: 12.5px; }
.fcard .fmeta { color: %(MUTED)s; font-size: 12.5px; margin-top: 6px; }
.dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%%;
       margin-right: 7px; vertical-align: baseline; }
.tiles { display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px;
         margin-top: 16px; }
.tile { background: %(SURFACE)s; border: 1px solid %(BORDER)s;
        border-radius: 10px; padding: 12px 14px; }
.tlabel { color: %(INK2)s; font-size: 12.5px; margin-bottom: 8px; }
.tvalue { font-size: 24px; font-weight: 600; }
.tvalue small, .tpair small { color: %(MUTED)s; font-weight: 400; }
.tpair { display: flex; flex-direction: column; gap: 4px; }
.tpair b { font-size: 17px; font-weight: 600; }
.tsub { color: %(MUTED)s; font-size: 12px; margin-top: 4px; }
.card { background: %(SURFACE)s; border: 1px solid %(BORDER)s;
        border-radius: 10px; padding: 10px; margin-top: 10px; }
.grid2 { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
table.metrics { border-collapse: collapse; width: 100%%;
                background: %(SURFACE)s; border: 1px solid %(BORDER)s;
                border-radius: 10px; overflow: hidden; }
table.metrics th, table.metrics td { padding: 7px 12px; text-align: right;
    font-variant-numeric: tabular-nums; border-top: 1px solid %(GRID)s; }
table.metrics th { color: %(INK2)s; font-weight: 600; border-top: none;
                   background: %(SURFACE)s; }
table.metrics td:first-child, table.metrics th:first-child {
    text-align: left; }
table.data { font-size: 12px; }
table.metrics tr.rhead td { text-align: left; font-weight: 650;
    background: %(PAGE)s; color: %(INK)s; }
.tscroll { max-height: 420px; overflow: auto; margin-top: 8px; }
.tscroll thead th { position: sticky; top: 0; }
details summary { cursor: pointer; color: %(INK2)s; margin-top: 18px; }
.note { color: %(MUTED)s; font-size: 12.5px; }
footer { color: %(MUTED)s; font-size: 12px; margin-top: 40px;
         border-top: 1px solid %(GRID)s; padding-top: 10px; }
@media (max-width: 900px) {
  .tiles { grid-template-columns: repeat(2, 1fr); }
  .grid2, .files { grid-template-columns: 1fr; }
}
""" % dict(PAGE=PAGE, INK=INK, INK2=INK2, MUTED=MUTED, SURFACE=SURFACE,
           BORDER=BORDER, GRID=GRID, FONT=FONT)


def file_card(fl, color):
    n_seg = len(fl.raw.segments)
    t0, t1 = fl.raw.segments[fl.seg_index]
    seg = (f"armed segment {fl.seg_index + 1}/{n_seg} "
           f"({t1 - t0:.0f} s)" if n_seg > 1 else f"single segment ({t1 - t0:.0f} s)")
    home = (f"home {fl.home[0]:.7f}, {fl.home[1]:.7f}"
            if not math.isnan(fl.home[0]) else "no position data")
    return f"""
    <div class="fcard">
      <div class="flabel"><i class="dot" style="background:{color}"></i>{fl.label}
        <small style="color:{MUTED}; font-weight: 400;">· {fl.raw.kind}</small></div>
      <div class="fpath">{fl.raw.path}</div>
      <div class="fmeta">{seg} · {home} · {fl.align_note}</div>
    </div>"""


def load_waypoints_file(path):
    """QGC WPL 110 -> [(seq, lat, lon)] for nav items with coordinates."""
    wps = []
    with open(path, encoding='utf-8') as f:
        if 'QGC WPL' not in f.readline():
            raise ValueError(f"{path}: not a QGC WPL waypoints file")
        for line in f:
            p = line.split('\t')
            if len(p) < 12:
                continue
            seq, lat, lon = int(p[0]), float(p[8]), float(p[9])
            if seq > 0 and (lat != 0 or lon != 0):
                wps.append((seq, lat, lon))
    return wps


def build_report(real, sim, out_path, with_map=True, frame='shared',
                 mission_wps=None):
    import plotly.io as pio
    figs = []
    wps = mission_wps or real.raw.waypoints or sim.raw.waypoints
    track_title = ("Track — local East/North (shared frame, origin = "
                   f"{real.label} home)" if frame == 'shared' else
                   "Track — local East/North (each from its own home — "
                   "pattern overlay)")
    # voltage/current panels only when either log actually carries data
    panels = TS_PANELS + [
        c for c in ('voltage', 'current')
        if not (np.all(np.isnan(real.df[c])) and np.all(np.isnan(sim.df[c])))]

    figs.append((track_title, fig_track(real, sim, wps, frame)))
    if with_map and not math.isnan(real.home[0]):
        figs.append(("Track — map (needs internet for tiles)",
                     fig_map(real, sim, wps)))
    figs.append(("Time series", fig_timeseries(real, sim, panels)))
    figs.append(("Flight modes", fig_modes(real, sim)))

    sum_r, sum_s = flight_summary(real), flight_summary(sim)
    deltas, overlap = aligned_deltas(real, sim)
    sep = track_separation(real, sim)
    reg_r, reg_s = regime_metrics(real), regime_metrics(sim)
    reg_d = regime_deltas(reg_r, reg_s)

    parts = []
    first = True
    for title, f in figs:
        html = pio.to_html(f, full_html=False, include_plotlyjs=first,
                           config={'displaylogo': False,
                                   'modeBarButtonsToRemove': ['lasso2d',
                                                              'select2d']})
        first = False
        parts.append((title, html))

    track_html = f"""<div class="card">{parts[0][1]}</div>"""
    map_html = (f"""<h2>{parts[1][0]}</h2><div class="card">{parts[1][1]}</div>"""
                if with_map and len(parts) == 4 else "")
    ts_idx = 2 if (with_map and len(parts) == 4) else 1

    doc = f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Flight comparison — {real.label} vs {sim.label}</title>
<style>{CSS}</style></head>
<body><div class="wrap">
  <h1>Flight log comparison</h1>
  <p class="sub">{real.label} vs {sim.label} ·
     generated {datetime.now().strftime('%Y-%m-%d %H:%M')}</p>
  <div class="files">{file_card(real, C_REAL)}{file_card(sim, C_SIM)}</div>
  {stat_tiles(real, sim, sum_r, sum_s, sep)}
  <h2>{parts[0][0]}</h2>{track_html}
  {map_html}
  <h2>Time series (aligned)</h2>
  <div class="card">{parts[ts_idx][1]}</div>
  <h2>Flight modes</h2>
  <div class="card">{parts[ts_idx + 1][1]}</div>
  <h2>Flight summary</h2>
  {summary_table(real, sim, sum_r, sum_s)}
  <h2>Per-regime metrics (hover / transition / cruise)</h2>
  {regime_table(reg_r, reg_s, reg_d)}
  <h2>Sim − real differences (time-aligned)</h2>
  {delta_table(deltas, overlap)}
  {data_table(real, sim, panels)}
  <footer>compare_logs.py — CezeriSim · real: {os.path.basename(real.raw.path)}
   · sim: {os.path.basename(sim.raw.path)} · alignment: {real.align_note} /
   {sim.align_note}</footer>
</div></body></html>"""
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(doc)

    # Machine-readable closeness stats — consumed by --summary-json (the UE
    # Compare menu reads that file to show results in-engine).
    stats = {
        'real_summary': sum_r,
        'sim_summary': sum_s,
        'track_separation_m': sep,
        'deltas': deltas,
        'overlap_s': (overlap[1] - overlap[0]) if deltas else 0.0,
        'regimes': {'real': reg_r, 'sim': reg_s, 'delta': reg_d},
    }
    return out_path, stats


# ---------------------------------------------------------------------------
# CLI / GUI
# ---------------------------------------------------------------------------
def pick_files_gui():
    import tkinter as tk
    from tkinter import filedialog, messagebox
    root = tk.Tk()
    root.withdraw()
    ft = [("Flight logs", "*.bin *.BIN *.log *.tlog *.csv"),
          ("All files", "*.*")]
    messagebox.showinfo("Compare flight logs",
                        "Select the REAL flight log first (Mission Planner "
                        ".bin/.tlog), then the SIMULATION log (.bin/.csv).")
    real = filedialog.askopenfilename(title="REAL flight log", filetypes=ft)
    if not real:
        sys.exit(0)
    sim = filedialog.askopenfilename(
        title="SIMULATION log", filetypes=ft,
        initialdir=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "control", "logs"))
    if not sim:
        sys.exit(0)
    root.destroy()
    return real, sim


def main():
    ap = argparse.ArgumentParser(
        description="Compare a real flight log with a simulation log.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('real', nargs='?', help="real flight log (.bin/.log/.tlog/.csv)")
    ap.add_argument('sim', nargs='?', help="simulation log (.bin/.log/.tlog/.csv)")
    ap.add_argument('-o', '--out', help="output HTML path")
    ap.add_argument('--real-seg', type=int, help="armed segment # of real log (1-based)")
    ap.add_argument('--sim-seg', type=int, help="armed segment # of sim log (1-based)")
    ap.add_argument('--align', choices=['takeoff', 'arm'], default='takeoff')
    ap.add_argument('--takeoff-alt', type=float, default=2.0)
    ap.add_argument('--shift-sim', type=float, default=0.0)
    ap.add_argument('--labels', nargs=2, default=['Real', 'Sim'],
                    metavar=('REAL', 'SIM'))
    ap.add_argument('--frame', choices=['shared', 'own-home'],
                    default='shared',
                    help="track frame: 'shared' (default) draws both tracks "
                         "and waypoints from the real flight's home — true "
                         "geometry; 'own-home' overlays each track relative "
                         "to its own home (pattern comparison, hides any "
                         "home offset, waypoints omitted)")
    ap.add_argument('--mission', help="QGC WPL .waypoints file to overlay "
                                      "(default: CMD messages from the logs)")
    ap.add_argument('--list-segments', action='store_true')
    ap.add_argument('--no-map', action='store_true')
    ap.add_argument('--no-open', action='store_true')
    ap.add_argument('--summary-json',
                    help="also write the closeness stats (flight summaries, "
                         "track separation, sim-minus-real deltas) as JSON — "
                         "read by the UE Compare menu")
    a = ap.parse_args()

    if not a.real or not a.sim:
        a.real, a.sim = pick_files_gui()

    for p in (a.real, a.sim):
        if not os.path.isfile(p):
            raise SystemExit(f"file not found: {p}")

    print(f"[1/4] parsing real log: {a.real}")
    raw_r = parse_any(a.real)
    print(f"[2/4] parsing sim log:  {a.sim}")
    raw_s = parse_any(a.sim)

    for name, raw in (("real", raw_r), ("sim", raw_s)):
        print(f"  {name}: {raw.kind}, {len(raw.segments)} armed segment(s): " +
              ", ".join(f"#{i + 1} {t1 - t0:.0f}s"
                        for i, (t0, t1) in enumerate(raw.segments)))
    if a.list_segments:
        return

    seg_r = pick_segment(raw_r, a.real_seg)
    seg_s = pick_segment(raw_s, a.sim_seg)
    print(f"[3/4] using real segment #{seg_r + 1}, sim segment #{seg_s + 1}; "
          f"aligning at {a.align}")
    real = build_flight(raw_r, seg_r, a.labels[0], C_REAL,
                        a.align, a.takeoff_alt, 0.0)
    sim = build_flight(raw_s, seg_s, a.labels[1], C_SIM,
                       a.align, a.takeoff_alt, a.shift_sim)

    out = a.out
    if not out:
        stem_r = os.path.splitext(os.path.basename(a.real))[0]
        stem_s = os.path.splitext(os.path.basename(a.sim))[0]
        out = os.path.join(os.path.dirname(os.path.abspath(a.real)),
                           f"compare_{stem_r}__vs__{stem_s}.html")
    mission_wps = load_waypoints_file(a.mission) if a.mission else None
    print(f"[4/4] writing report: {out}")
    _, stats = build_report(real, sim, out, with_map=not a.no_map,
                            frame=a.frame, mission_wps=mission_wps)
    sz = os.path.getsize(out) / 1e6
    print(f"[OK] {out} ({sz:.1f} MB)")

    if a.summary_json:
        import json

        def _clean(v):                    # NaN is not valid strict JSON
            if isinstance(v, dict):
                return {k: _clean(x) for k, x in v.items()}
            if isinstance(v, float) and math.isnan(v):
                return None
            return v

        stats.update({
            'generated': datetime.now().isoformat(timespec='seconds'),
            'real_log': os.path.abspath(a.real),
            'sim_log': os.path.abspath(a.sim),
            'report': os.path.abspath(out),
        })
        os.makedirs(os.path.dirname(os.path.abspath(a.summary_json)),
                    exist_ok=True)
        with open(a.summary_json, 'w', encoding='utf-8') as f:
            json.dump(_clean(stats), f, indent=2)
        print(f"[OK] summary JSON: {a.summary_json}")

    if not a.no_open:
        webbrowser.open('file:///' + os.path.abspath(out).replace('\\', '/'))


if __name__ == '__main__':
    main()

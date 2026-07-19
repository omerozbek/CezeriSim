#!/usr/bin/env python3
"""
tune_params.py — Parameter fine-tuning for VTOL waypoint missions.

Flies the SAME mission repeatedly in the simulator, changing ArduPilot
navigation/TECS/Q parameters between flights, scores every flight from its
dataflash log, and searches for the parameter set with the best score.
Launched by the Compare menu's FINE-TUNE PARAMETERS button; also standalone:

    python tune_params.py --mission plan.waypoints --flights 30
    python tune_params.py --real real_flight.bin --profile accuracy

What "best" means (docs/ROADMAP + the 2026-07-18 design discussion):
  * accuracy  — cross-track distance to the mission PATH (not just waypoint
                proximity: passing near every waypoint while weaving between
                them is bad) + per-waypoint closest approach (corner cutting)
  * smoothness — RMS roll rate + bank reversals per minute (weaving /
                S-turns re-capturing the track after corners)
  * altitude  — RMS of the detrended altitude during cruise (TECS
                oscillation, independent of the commanded profile)
  * time      — mild penalty only when SLOWER than the baseline flight
  * hard constraints (flight scored 0, never traded): mission completes and
    disarms, no emergency/failsafe mode (QLAND/QRTL/RTL) mid-mission
The weighting between accuracy and smoothness is a policy choice — the
--profile flag picks it (balanced / accuracy / smooth).

Search space: nav + TECS + Q_ parameters (all apply live over MAVLink, no
SITL reboot), centered on the model's own params.parm values — the tuner
REFINES the current tune rather than exploring blindly. Rate PIDs are
deliberately excluded: gains tuned against sim physics transfer poorly to
the real vehicle and would triple the dimensionality.

Optimizer: Optuna TPE when installed (pip install optuna), otherwise a
built-in random-then-refine fallback so the UI button never hard-fails.
Flight 1 always flies the unmodified baseline = the reference score.

Progress: --progress-json is rewritten (atomically) after every flight —
the UE Compare menu polls it. The best parameter set is written as a small
.parm file (upload it with the PARAMS box / --params for future replays)
and is re-uploaded to the vehicle at the end of the session.

Caveat (by design): this finds the best params FOR THE SIM. They are only
best for the real aircraft where the compare reports show the sim tracking
reality closely — which is exactly what this menu measures.
"""
import argparse
import json
import math
import os
import random
import sys
import time
from datetime import datetime

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CONTROL_DIR = os.path.normpath(os.path.join(HERE, '..', 'control'))
PROJECT_DIR = os.path.normpath(os.path.join(HERE, '..', '..'))
LOG_DIR = os.path.join(CONTROL_DIR, 'logs')
sys.path.insert(0, CONTROL_DIR)
sys.path.insert(0, HERE)

import fly_mission                            # noqa: E402  (Scripts/control)
import param_manager                          # noqa: E402  (Scripts/control)
import run_mission_compare as rmc             # noqa: E402  (helpers reused)
from mission_from_log import extract_mission  # noqa: E402
from compare_logs import (parse_dataflash, pick_segment, build_flight,
                          load_waypoints_file)  # noqa: E402

try:
    import optuna
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    HAVE_OPTUNA = True
except ImportError:
    HAVE_OPTUNA = False


# ---------------------------------------------------------------------------
# Search space + score profiles
# ---------------------------------------------------------------------------
# name -> (lo, hi). Ranges are deliberately TIGHT around sane quadplane
# values (the baseline params.parm values all sit inside them) — with a
# ~30-flight budget over ~10 dimensions the tuner must refine, not explore.
# Only parameters present in the baseline .parm are searched (missing ones
# are dropped with a note), so a stripped-down model never gets unknown
# params pushed onto it.
SEARCH_SPACE = {
    # fixed-wing path following (dominates track shape)
    'NAVL1_PERIOD':   (10.0, 30.0),   # s     smaller = tighter tracking
    'NAVL1_DAMPING':  (0.55, 0.95),   # -     higher = less overshoot
    'NAVL1_XTRACK_I': (0.0,  0.10),   # -     crosswind offset trim
    'WP_RADIUS':      (20.0, 120.0),  # m     turn anticipation / corner cut
    'ROLL_LIMIT_DEG': (30.0, 55.0),   # deg   max bank in turns
    # TECS (altitude/speed hold quality)
    'TECS_TIME_CONST': (3.0, 8.0),    # s     smaller = tighter alt hold
    'TECS_PTCH_DAMP':  (0.1, 0.7),
    'TECS_THR_DAMP':   (0.3, 0.8),
    # VTOL phases (takeoff/landing legs)
    'Q_WP_SPEED':     (250.0, 800.0),  # cm/s
    'Q_WP_SPEED_UP':  (150.0, 400.0),  # cm/s
}

# score = 100 / (1 + sum(weight * metric/reference)) — 100 = perfect, 50 =
# "reference-level" flight, worse flights keep shrinking but never saturate
# to a flat 0 (a linear 100*(1-penalty) clipped every bad flight to exactly
# 0 and left the optimizer unable to rank them — seen on the wp_tune_cmp
# logs). References make the terms comparable: metric == reference
# contributes its full weight. Weights per profile sum to 1.
#
# excess_dist (2026-07-18): flown track length over the mission path length,
# as a fraction. A tune that swings wide to keep every turn silky-smooth
# flies visibly FARTHER than the mission — this term charges for that, so
# "smooth" can no longer be bought with kilometres of detour.
PROFILES = {
    'balanced': {'xtrack': 0.27, 'wp_miss': 0.13, 'roll_rate': 0.18,
                 'weave': 0.13, 'alt_osc': 0.14, 'time': 0.05,
                 'excess_dist': 0.10},
    'accuracy': {'xtrack': 0.40, 'wp_miss': 0.22, 'roll_rate': 0.09,
                 'weave': 0.09, 'alt_osc': 0.05, 'time': 0.05,
                 'excess_dist': 0.10},
    'smooth':   {'xtrack': 0.13, 'wp_miss': 0.09, 'roll_rate': 0.28,
                 'weave': 0.23, 'alt_osc': 0.14, 'time': 0.05,
                 'excess_dist': 0.08},
}
REFS = {'xtrack': 10.0,     # m    mean cross-track distance to the path
        'wp_miss': 15.0,    # m    mean per-waypoint closest approach
        'roll_rate': 8.0,   # deg/s RMS (cruise)
        'weave': 4.0,       # bank reversals per cruise minute
        'alt_osc': 2.5,     # m    RMS detrended altitude (cruise)
        'time': 0.5,        # fraction slower than the baseline flight
        'excess_dist': 0.30}  # fraction flown beyond the mission path length

CRUISE_AIRSPEED = 12.0      # m/s — fixed-wing phase (same as compare_logs)
EMERGENCY_MODES = {'QLAND', 'QRTL', 'RTL', 'LOITER2QLAND'}


# ---------------------------------------------------------------------------
# Flight scoring
# ---------------------------------------------------------------------------
def en_of(lat, lon, home):
    m_lat = 111132.95
    m_lon = 111319.49 * math.cos(math.radians(home[0]))
    return (lon - home[1]) * m_lon, (lat - home[0]) * m_lat


def dist_to_polyline(pts, poly):
    """Min distance (m) from each point (N,2) to a polyline (M,2)."""
    d = np.full(len(pts), np.inf)
    for a, b in zip(poly[:-1], poly[1:]):
        ab = b - a
        L2 = float(ab @ ab)
        if L2 < 1e-9:
            dd = np.linalg.norm(pts - a, axis=1)
        else:
            t = np.clip((pts - a) @ ab / L2, 0.0, 1.0)
            dd = np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)
        d = np.minimum(d, dd)
    return d


def bank_reversals_per_min(roll, dt, thr=8.0):
    """Count sign flips of banks beyond +-thr deg — weaving / S-turns."""
    s = np.where(roll > thr, 1, np.where(roll < -thr, -1, 0))
    s = s[s != 0]
    flips = int(np.sum(s[1:] != s[:-1])) if len(s) > 1 else 0
    minutes = len(roll) * dt / 60.0
    return flips / minutes if minutes > 0 else 0.0


def score_flight(bin_path, mission_wps, weights, base_dur):
    """Score one pulled dataflash log against the mission path.

    Returns (score 0..100, metrics dict, fail_reason | None)."""
    import pandas as pd
    raw = parse_dataflash(bin_path)
    fl = build_flight(raw, pick_segment(raw, None), 'tune', '#000',
                      align='arm', takeoff_alt=2.0, shift=0.0)
    df = fl.df
    dt = 0.2                                    # compare_logs GRID_DT

    # ---- hard constraints ---------------------------------------------------
    for _, _, name in fl.modes:
        if name in EMERGENCY_MODES:
            return 0.0, {}, f"emergency/failsafe mode {name} during mission"
    if float(np.nanmax(df['alt_rel'])) < 5.0:
        return 0.0, {}, "never became airborne"

    cruise = (df['airspeed'] > CRUISE_AIRSPEED).to_numpy()
    if cruise.sum() * dt < 20.0:
        return 0.0, {}, "no usable fixed-wing cruise phase (>12 m/s)"

    # ---- mission path in the flight's East/North frame ----------------------
    wp_en = np.array([en_of(lat, lon, fl.home) for _, lat, lon in mission_wps])
    if len(wp_en) < 2:
        return 0.0, {}, "mission has fewer than 2 coordinate waypoints"

    track = df[['east', 'north']].to_numpy()
    ok = ~np.isnan(track).any(axis=1)
    track_cruise = track[ok & cruise]
    track_all = track[ok]

    m = {}
    m['xtrack'] = float(np.mean(dist_to_polyline(track_cruise, wp_en)))
    m['wp_miss'] = float(np.mean(
        [np.min(np.linalg.norm(track_all - wp, axis=1)) for wp in wp_en]))

    # Flown distance vs the mission path length: wide smooth turns detour —
    # charge the fraction flown BEYOND the mission polyline so the tuner
    # keeps turns tight in distance, not just smooth in roll.
    mission_len = float(np.sum(np.linalg.norm(np.diff(wp_en, axis=0), axis=1)))
    track_len = float(np.sum(np.linalg.norm(np.diff(track_all, axis=0), axis=1)))
    m['excess_dist'] = max(0.0, track_len / mission_len - 1.0) \
        if mission_len > 1.0 else 0.0

    roll = df['roll'].to_numpy()[ok & cruise]
    roll = roll[~np.isnan(roll)]
    m['roll_rate'] = float(np.sqrt(np.mean(np.gradient(roll, dt) ** 2))) \
        if len(roll) > 2 else 0.0
    m['weave'] = bank_reversals_per_min(roll, dt)

    # detrended altitude: alt minus its 20 s rolling median — slow commanded
    # profile changes pass through, TECS oscillation does not
    alt = pd.Series(df['alt_rel'].to_numpy())
    trend = alt.rolling(101, center=True, min_periods=25).median()
    osc = (alt - trend).to_numpy()[ok & cruise]
    osc = osc[~np.isnan(osc)]
    m['alt_osc'] = float(np.sqrt(np.mean(osc ** 2))) if len(osc) else 0.0

    dur = float(df['t'].iloc[-1] - df['t'].iloc[0])
    m['duration'] = dur
    m['time'] = max(0.0, dur / base_dur - 1.0) if base_dur else 0.0

    penalty = sum(weights[k] * (m[k] / REFS[k]) for k in weights)
    return 100.0 / (1.0 + penalty), m, None


# ---------------------------------------------------------------------------
# Samplers
# ---------------------------------------------------------------------------
class FallbackSampler:
    """No-dependency stand-in for Optuna: random exploration for ~40 % of
    the budget, then gaussian refinement around the best point so far."""

    def __init__(self, space, flights, seed=42):
        self.space = space
        self.explore = max(4, int(flights * 0.4))
        self.rng = random.Random(seed)
        self.best_vals, self.best_score, self.n = None, -1.0, 0

    def ask(self):
        vals = {}
        for name, (lo, hi) in self.space.items():
            if self.n < self.explore or self.best_vals is None:
                vals[name] = self.rng.uniform(lo, hi)
            else:
                sigma = 0.18 * (hi - lo)
                vals[name] = min(hi, max(lo, self.rng.gauss(
                    self.best_vals[name], sigma)))
        return vals

    def tell(self, vals, score):
        self.n += 1
        if score > self.best_score:
            self.best_score, self.best_vals = score, dict(vals)


class OptunaSampler:
    def __init__(self, space, flights, seed=42):
        self.space = space
        self.study = optuna.create_study(
            direction='maximize',
            sampler=optuna.samplers.TPESampler(
                n_startup_trials=max(4, min(8, flights // 3)), seed=seed))
        self._trial = None

    def enqueue(self, vals):
        self.study.enqueue_trial(vals)

    def ask(self):
        self._trial = self.study.ask()
        return {name: self._trial.suggest_float(name, lo, hi)
                for name, (lo, hi) in self.space.items()}

    def tell(self, vals, score):
        self.study.tell(self._trial, score)


# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------
def set_params(mav, values, tries=3):
    """param_set each value with read-back verify — tuning is meaningless
    if a value silently fails to apply. Returns the list that never acked."""
    from pymavlink import mavutil
    missing = []
    for name, val in values.items():
        ok = False
        for _ in range(tries):
            mav.mav.param_set_send(
                mav.target_system, mav.target_component,
                name.encode('ascii')[:16], float(val),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            t0 = time.time()
            while time.time() - t0 < 1.0:
                msg = mav.recv_match(type='PARAM_VALUE', blocking=True,
                                     timeout=0.3)
                if msg and msg.param_id.rstrip('\x00') == name:
                    ok = True
                    break
            if ok:
                break
        if not ok:
            missing.append(name)
    return missing


def wait_ready_for_next(s, timeout):
    """Between flights: vehicle disarmed, settled, EKF still fused."""
    t0 = time.time()
    while time.time() - t0 < 15:                # let the landing settle
        s.pump(0.5)
        if not s.armed:
            break
    return fly_mission.gate(s, timeout=timeout)


# ---------------------------------------------------------------------------
# Progress file (polled by the UE Compare menu)
# ---------------------------------------------------------------------------
class Progress:
    def __init__(self, path):
        self.path = path
        self.doc = {'tuning': 1, 'state': 'starting'}

    def update(self, **kw):
        self.doc.update(kw)
        if not self.path:
            return
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(self.doc, f, indent=1)
        os.replace(tmp, self.path)              # atomic — no half-read JSON


def fmt_vals(vals):
    return '  '.join(f"{k}={v:.3g}" for k, v in vals.items())


def write_parm(path, vals, header_lines):
    with open(path, 'w', encoding='utf-8') as f:
        for ln in header_lines:
            f.write(f"# {ln}\n")
        for k, v in sorted(vals.items()):
            f.write(f"{k:<16}{v:.4f}\n")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Fine-tune ArduPilot params for a waypoint mission by "
                    "flying it repeatedly in the simulator.")
    ap.add_argument('--mission', help="QGC WPL 110 .waypoints file to fly")
    ap.add_argument('--real', help="real flight log to extract the mission "
                                   "from when --mission is not given")
    ap.add_argument('--params', help="optional .parm uploaded before tuning "
                                     "(baseline overrides, like the PARAMS "
                                     "box of a mission replay)")
    ap.add_argument('--flights', type=int, default=30,
                    help="number of tuning flights (default 30)")
    ap.add_argument('--profile', choices=sorted(PROFILES), default='balanced',
                    help="what 'best' optimizes for (default balanced)")
    ap.add_argument('--addr', default='tcp:localhost:5760')
    ap.add_argument('--timeout', type=float, default=900,
                    help="per-flight timeout, seconds (default 900)")
    ap.add_argument('--connect-timeout', type=float, default=600)
    ap.add_argument('--gate-timeout', type=float, default=600)
    ap.add_argument('--progress-json',
                    help="progress file rewritten after every flight "
                         "(read by the UE Compare menu)")
    ap.add_argument('--out', help="tuned .parm output path (default "
                                  "logs/tune_<ts>/best_params.parm)")
    ap.add_argument('--vehicle-name',
                    help="vehicle model name for file naming / baseline parm "
                         "(default: Vehicles/active_vehicle.txt)")
    ap.add_argument('--seed', type=int, default=42)
    a = ap.parse_args()

    if not a.mission and not a.real:
        raise SystemExit("give the mission: --mission plan.waypoints, or "
                         "--real flight.bin to extract it from")

    vehicle = a.vehicle_name or rmc.active_vehicle_name()
    ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    session_dir = os.path.join(LOG_DIR, f"tune_{ts}_{vehicle}")
    os.makedirs(session_dir, exist_ok=True)
    out_parm = os.path.abspath(a.out) if a.out else \
        os.path.join(session_dir, 'best_params.parm')
    prog = Progress(os.path.abspath(a.progress_json)
                    if a.progress_json else
                    os.path.join(session_dir, 'progress.json'))
    prog.update(profile=a.profile, flights_total=a.flights, flights_done=0,
                vehicle=vehicle, session_dir=session_dir,
                optimizer='optuna' if HAVE_OPTUNA else 'builtin',
                message='waiting for the simulator...')

    # ---- mission ------------------------------------------------------------
    if a.mission:
        mission = os.path.abspath(a.mission)
        if not os.path.isfile(mission):
            raise SystemExit(f"waypoints file not found: {mission}")
        with open(mission, encoding='utf-8') as f:
            n_items = len([ln for ln in f.read().splitlines()[1:] if ln.strip()])
    else:
        real = os.path.abspath(a.real)
        if not os.path.isfile(real):
            raise SystemExit(f"real flight log not found: {real}")
        mission = os.path.join(session_dir, 'mission.waypoints')
        print(f"=== MISSION EXTRACT: {real} ===")
        mission, n_items = extract_mission(real, mission)
    mission_wps = load_waypoints_file(mission)
    if len(mission_wps) < 2:
        raise SystemExit(f"{mission}: fewer than 2 coordinate waypoints — "
                         "nothing to score against")
    print(f"=== MISSION: {mission} ({n_items} items, "
          f"{len(mission_wps)} scored waypoints) ===")

    # ---- baseline parameter values (search center + flight 1) --------------
    from pathlib import Path
    base_parm = os.path.join(PROJECT_DIR, 'Vehicles', vehicle, 'params.parm')
    baseline_all = {}
    if os.path.isfile(base_parm):
        baseline_all.update(param_manager.parse_parm_file(Path(base_parm)))
    if a.params:
        if not os.path.isfile(a.params):
            raise SystemExit(f"params file not found: {a.params}")
        baseline_all.update(param_manager.parse_parm_file(Path(a.params)))

    space, baseline = {}, {}
    for name, rng in SEARCH_SPACE.items():
        if name in baseline_all:
            space[name] = rng
            baseline[name] = float(np.clip(baseline_all[name], *rng))
        else:
            print(f"[NOTE] {name} not in {os.path.basename(base_parm)} — "
                  "dropped from the search space")
    if not space:
        raise SystemExit("no searchable parameters found in the baseline "
                         ".parm — nothing to tune")
    weights = PROFILES[a.profile]
    print(f"=== SEARCH: {len(space)} params, {a.flights} flights, "
          f"profile={a.profile}, optimizer="
          f"{'optuna TPE' if HAVE_OPTUNA else 'builtin random+refine'} ===")
    if not HAVE_OPTUNA:
        print("    (pip install optuna for the sample-efficient search)")

    if HAVE_OPTUNA:
        sampler = OptunaSampler(space, a.flights, a.seed)
        sampler.enqueue(baseline)               # flight 1 = the current tune
    else:
        sampler = FallbackSampler(space, a.flights, a.seed)

    # ---- connect once — every flight reuses the session ---------------------
    mav = rmc.connect_with_retry(a.addr, a.connect_timeout)
    s = fly_mission.Telem(mav)
    if not fly_mission.gate(s, timeout=a.gate_timeout):
        prog.update(state='failed', message='EKF gate failed — is the UE '
                    'level running in ue_physics mode?')
        raise SystemExit(1)
    if a.params:
        rmc.upload_params(mav, os.path.abspath(a.params))

    container = rmc.find_sitl_container()
    best = {'flight': 0, 'score': -1.0, 'params': {}}
    baseline_score = None
    fail_streak = 0

    try:
        for k in range(1, a.flights + 1):
            vals = baseline if (k == 1 and not HAVE_OPTUNA) else sampler.ask()
            print(f"\n=== FLIGHT {k}/{a.flights}: {fmt_vals(vals)} ===")
            prog.update(state='running', flight_running=k,
                        message=f"flight {k}/{a.flights} flying...",
                        last_params=vals)

            missing = set_params(mav, vals)
            if missing:
                print(f"[WARN] params never acked: {', '.join(missing)}")

            # fresh upload every flight: clears mission state, restarts at
            # item 0 (MIS_RESTART-independent)
            score, metrics, fail = 0.0, {}, None
            if fly_mission.upload_mission(mav, mission) == 0:
                fail = 'mission upload failed'
            else:
                csv_path = os.path.join(session_dir, f"flight_{k:02d}.csv")
                if not fly_mission.fly(s, n_items, a.timeout, csv_path):
                    fail = 'mission did not complete (timeout / never flew)'
                else:
                    time.sleep(3)               # let AP close the log file
                    bin_path = os.path.join(session_dir, f"flight_{k:02d}.bin")
                    if not container:
                        container = rmc.find_sitl_container()
                    if not (container and
                            rmc.pull_dataflash(container, bin_path)):
                        fail = 'could not pull the dataflash log'
                    else:
                        try:
                            score, metrics, fail = score_flight(
                                bin_path, mission_wps, weights,
                                base_dur=best.get('base_dur'))
                        except Exception as e:
                            fail = f"scoring failed: {e}"

            sampler.tell(vals, score)
            fail_streak = fail_streak + 1 if fail else 0
            if fail:
                print(f"  FLIGHT {k} FAILED: {fail} -> score 0")
            else:
                print(f"  FLIGHT {k} score {score:.1f}  "
                      + '  '.join(f"{n}={metrics[n]:.2f}"
                                  for n in ('xtrack', 'wp_miss', 'roll_rate',
                                            'weave', 'alt_osc', 'excess_dist')))
            if k == 1 and not fail:
                baseline_score = score
                best['base_dur'] = metrics.get('duration')
            if score > best['score']:
                best.update(flight=k, score=score, params=dict(vals),
                            metrics=metrics)
            prog.update(flights_done=k,
                        last={'flight': k, 'score': round(score, 1),
                              'fail': fail,
                              'metrics': {n: round(v, 2) for n, v
                                          in metrics.items()}},
                        baseline_score=(round(baseline_score, 1)
                                        if baseline_score is not None else None),
                        best={'flight': best['flight'],
                              'score': round(best['score'], 1),
                              'params': {n: round(v, 3) for n, v
                                         in best['params'].items()}},
                        message=f"flight {k}/{a.flights} done")

            if fail_streak >= 3:
                prog.update(state='failed',
                            message='3 flights in a row failed '
                                    f'(last: {fail}) — aborting. Check the '
                                    'console / simulator.')
                raise SystemExit(1)

            if k < a.flights and not wait_ready_for_next(
                    s, timeout=a.gate_timeout):
                prog.update(state='failed',
                            message='vehicle never became ready for the next '
                                    'flight (gate failed) — missions must '
                                    'land back near the takeoff point')
                raise SystemExit(1)
    except KeyboardInterrupt:
        print("\n[STOP] interrupted — keeping the best result so far")

    if best['score'] < 0 or not best['params']:
        prog.update(state='failed', message='no flight produced a score')
        raise SystemExit(1)

    # ---- finish: restore best on the vehicle + write the tuned .parm --------
    print(f"\n=== BEST: flight {best['flight']} score {best['score']:.1f} "
          f"(baseline {baseline_score if baseline_score is not None else '?'})"
          f" ===\n    {fmt_vals(best['params'])}")
    set_params(mav, best['params'])             # leave the vehicle on the best
    write_parm(out_parm, best['params'], [
        f"tuned by tune_params.py {ts}",
        f"vehicle: {vehicle}   profile: {a.profile}   "
        f"flights: {a.flights}",
        f"mission: {mission}",
        f"score: {best['score']:.1f} (baseline flight scored "
        f"{baseline_score if baseline_score is not None else 'n/a'})",
        "upload over MAVLink before a flight (Compare menu PARAMS box / "
        "--params) — the model's own params.parm still boots SITL",
    ])
    print(f"[OK] tuned params written: {out_parm}")
    prog.update(state='done', out_parm=out_parm,
                message=f"finished — best score {best['score']:.1f} "
                        f"on flight {best['flight']}"
                        + (f" (baseline {baseline_score:.1f})"
                           if baseline_score is not None else ''))


if __name__ == '__main__':
    main()

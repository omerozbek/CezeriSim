#!/usr/bin/env python3
"""
tune_plant.py — Plant (physics-model) system identification from REAL flights.

Counterpart of tune_params.py with the opposite contract: tune_params refines
ArduPilot NAV/TECS parameters against the sim's physics and deliberately NEVER
touches the plant; this tool fits the PLANT — the vehicle's mechanical /
electrical JSON values — so the simulator reproduces the real flights'
per-regime operating points (docs/ROADMAP.md, Digital Twin phases).

    python tune_plant.py --real A.BIN --trials 20
    python tune_plant.py --real A.BIN --real B.BIN --trials 15
    python tune_plant.py --real A.BIN --real-params A.param --trials 15
    python tune_plant.py --apply-result best_plant.json
    python tune_plant.py --apply-result best_plant.json --as-new pasifik_cal

MULTI-LOG: every --real log is one fitting condition. Per trial each log's
mission is flown in sequence — under THAT log's onboard wind estimate and
THAT log's parameters — and the trial score is the mean of the per-log
scores. Two logs from different days (different wind, different throttle
operating points) over-determine the drag/thrust trade far better than one.

PER-LOG PARAMS: by default each log's ArduPilot parameters are EXTRACTED
FROM THE LOG ITSELF (dataflash PARM messages — exactly what the vehicle flew
that day) and uploaded over MAVLink before its flight; --real-params FILE
(repeat per --real, '' = extract) overrides with an explicit file. Hardware/
boot-only prefixes (SIM_/BRD_/SERIAL/CAN_/LOG_/NET_/SCR_, SYSID_THISMAV,
FORMAT_VERSION) are never uploaded.

v1 search space (cruise fidelity — the cd0 + pusher-η PAIRED calibration,
docs/INVARIANTS.md): cd0, oswald_e, pusher_vpitch_ms, drag_coeff_forward.

How a trial reaches the sim: trial values are written into the vehicle's
JSONs (originals backed up, ALWAYS restored afterwards), the log's wind is
patched into params.parm, and the SITL container is restarted — UE reloads
the vehicle config on the new handshake (ReloadConfigOnHandshake +
the bridge's servo-silence restart detection). Requires the UE level running
in ue_physics mode.

--accept writes the best values permanently at the end of a fit;
--apply-result does the same later from a saved best_plant.json (the init
menu's Calibration section uses it for "update existing"), and --as-new
first duplicates the vehicle folder under the given name ("add as new").
"""
import argparse
import json
import os
import re
import shutil
import subprocess
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
import run_mission_compare as rmc             # noqa: E402  (helpers reused)
from mission_from_log import extract_mission, extract_wind  # noqa: E402
from compare_logs import (parse_dataflash, pick_segment, build_flight,
                          regime_metrics)     # noqa: E402
from tune_params import (FallbackSampler, OptunaSampler, Progress,
                         HAVE_OPTUNA, EMERGENCY_MODES, fmt_vals)  # noqa: E402


# ---------------------------------------------------------------------------
# Search space — plant parameters in the vehicle JSONs
# ---------------------------------------------------------------------------
PLANT_SPACE = {
    'cd0':                ('mechanical', (0.020, 0.120)),
    'oswald_e':           ('mechanical', (0.60, 0.95)),
    'pusher_vpitch_ms':   ('electrical', (0.0, 200.0)),   # 0 = static pusher
    'drag_coeff_forward': ('electrical', (0.0, 1.0)),
}

# metric -> (regime, scorecard key, weight, absolute ref | None = 15% of the
# real value with a 0.3 floor). Refs mirror the acceptance gates.
OBJECTIVE = {
    'cruise_aspd': ('cruise', 'airspeed_mean', 0.30, 0.5),   # m/s
    'cruise_thr':  ('cruise', 'throttle_mean', 0.40, 3.0),   # percent points
    'climb_rate':  ('cruise', 'climb_rate_mean', 0.10, None),
    'sink_rate':   ('cruise', 'sink_rate_mean', 0.10, None),
    'hover_thr':   ('hover',  'throttle_mean', 0.10, 2.0),   # guard
}

# Params extracted from a real log are filtered by an ALLOW-list of
# flight-BEHAVIOR prefixes (controller gains, speed/throttle targets,
# navigation, transition). Uploading the full set flips the sim on takeoff:
# the real vehicle's SENSOR CALIBRATION (INS_ accel offsets, COMPASS_,
# AHRS_TRIM, BARO_, EK3_ tuning, BATT_ monitors, SERVO/RC endpoints)
# poisons SITL's perfect sensors and the delicate JSON-link EKF setup
# (tilt -143 deg 0.3 s after arming, 2026-07-19). The vehicle's own
# params.parm already boots the correct SITL sensor/output configuration.
PARAM_ALLOW_PREFIXES = ('Q_', 'TECS_', 'NAVL1_', 'WP_', 'TRIM_', 'THR_',
                        'LIM_', 'PTCH', 'RLL', 'YAW', 'KFF_', 'TKOFF_',
                        'LAND_', 'RTL_', 'MIS_', 'LOIT_', 'CRUISE_',
                        'FBWB_', 'AIRSPEED_', 'MIXING_', 'ACRO_', 'STAB_',
                        'FLAP', 'GLIDE_', 'STALL_')


# ---------------------------------------------------------------------------
# Vehicle JSON plumbing
# ---------------------------------------------------------------------------
def load_json(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def save_json(path, doc):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=2, ensure_ascii=False)
        f.write('\n')
    os.replace(tmp, path)


def vehicle_files(vehicle_dir):
    return {'mechanical': os.path.join(vehicle_dir, 'mechanical.json'),
            'electrical': os.path.join(vehicle_dir, 'electrical.json'),
            'parm': os.path.join(vehicle_dir, 'params.parm')}


def read_baseline(files):
    docs = {k: load_json(files[k]) for k in ('mechanical', 'electrical')}
    defaults = {'pusher_vpitch_ms': 0.0}
    out = {}
    for name, (which, (lo, hi)) in PLANT_SPACE.items():
        v = docs[which].get(name, defaults.get(name))
        if v is None:
            raise SystemExit(f"{files[which]}: missing '{name}' and no "
                             "default — add it to the vehicle JSON first")
        out[name] = float(np.clip(float(v), lo, hi))
    return out


def apply_values(files, vals, provenance=None):
    docs = {k: load_json(files[k]) for k in ('mechanical', 'electrical')}
    for name, val in vals.items():
        which = PLANT_SPACE[name][0]
        docs[which][name] = round(float(val), 5)
        if provenance and name in provenance:
            docs[which].setdefault('_doc', {})[name] = provenance[name]
    for k in ('mechanical', 'electrical'):
        save_json(files[k], docs[k])


def patch_parm_wind(parm_path, wind):
    """Set SIM_WIND_SPD/DIR/TURB in params.parm (UE re-reads it on every SITL
    handshake since the 2026-07-18 config-reload hook)."""
    with open(parm_path, encoding='utf-8') as f:
        lines = f.read().splitlines()
    values = {'SIM_WIND_SPD': wind['spd_ms'], 'SIM_WIND_DIR': wind['dir_deg'],
              'SIM_WIND_TURB': wind['turb_ms']}
    seen = set()
    out = []
    for ln in lines:
        tok = ln.split()
        if tok and tok[0] in values:
            out.append(f"{tok[0]:<16}{values[tok[0]]}")
            seen.add(tok[0])
        else:
            out.append(ln)
    for k, v in values.items():
        if k not in seen:
            out.append(f"{k:<16}{v}")
    with open(parm_path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(out) + '\n')


def warn_wind_override():
    path = os.path.join(PROJECT_DIR, 'Config', 'wind_settings.json')
    try:
        doc = load_json(path)
    except (OSError, ValueError):
        return
    for k, v in doc.items():
        if 'override' in k.lower() and v is True:
            print(f"[WARN] {path}: '{k}' is ON — the Settings wind override "
                  "replaces the vehicle's SIM_WIND_* and replays will NOT "
                  "fly the real flights' wind. Disable it in Settings → Wind "
                  "before trusting this calibration.")
            return


def restart_sitl(container):
    print(f"=== SITL RESTART: {container} ===")
    subprocess.run(['docker', 'restart', '-t', '5', container],
                   check=True, timeout=180)


def extract_params_from_bin(bin_path, out_parm):
    """Dataflash PARM messages -> uploadable .parm (last value wins).
    Only flight-BEHAVIOR params are kept (PARAM_ALLOW_PREFIXES) — the
    real vehicle's sensor calibration must never reach SITL."""
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(bin_path)
    vals = {}
    while True:
        msg = m.recv_match(type=['PARM'], blocking=False)
        if msg is None:
            break
        vals[str(msg.Name)] = float(msg.Value)
    kept = {name: v for name, v in vals.items()
            if any(name.startswith(p) for p in PARAM_ALLOW_PREFIXES)}
    if not kept:
        return None
    with open(out_parm, 'w', encoding='utf-8', newline='\n') as f:
        f.write(f"# extracted from {os.path.basename(bin_path)} "
                f"({len(kept)}/{len(vals)} params; flight-behavior "
                f"allow-list, sensor/hardware calibration dropped)\n")
        for k in sorted(kept):
            f.write(f"{k:<16}{kept[k]:.6f}\n")
    return out_parm


# ---------------------------------------------------------------------------
# Scoring — per-regime scorecard of a sim flight vs one real flight
# ---------------------------------------------------------------------------
def real_regimes(real_path, seg=None):
    raw = parse_dataflash(real_path)
    fl = build_flight(raw, pick_segment(raw, seg), 'real', '#000',
                      align='takeoff', takeoff_alt=2.0, shift=0.0)
    reg = regime_metrics(fl)
    if 'cruise' not in reg:
        raise SystemExit(f"{real_path}: no cruise regime found — the cruise "
                         "objective needs a fixed-wing phase in the real log")
    return reg


def score_sim(bin_path, real_reg):
    """(score 0..100, deltas dict, fail | None) for one pulled sim BIN."""
    raw = parse_dataflash(bin_path)
    fl = build_flight(raw, pick_segment(raw, None), 'sim', '#000',
                      align='takeoff', takeoff_alt=2.0, shift=0.0)
    # Emergency modes only fail the trial when they happen IN THE AIR —
    # ground-level QLAND at the end of VTOL_LAND is a touchdown artifact.
    for t0, _, name in fl.modes:
        if name in EMERGENCY_MODES:
            alt = float(np.interp(t0, fl.df['t'], fl.df['alt_rel']))
            if alt > 5.0:
                return 0.0, {}, (f"emergency/failsafe mode {name} at "
                                 f"{alt:.0f} m during mission")
    sim_reg = regime_metrics(fl)
    if 'cruise' not in sim_reg:
        return 0.0, {}, "sim flight has no cruise regime"

    m, penalty = {}, 0.0
    for name, (regime, key, weight, ref) in OBJECTIVE.items():
        rv = real_reg.get(regime, {}).get(key)
        sv = sim_reg.get(regime, {}).get(key)
        if rv is None or sv is None:
            continue
        delta = sv - rv
        r = ref if ref is not None else max(0.3, 0.15 * abs(rv))
        m[name] = round(delta, 3)
        penalty += weight * abs(delta) / r
    if not m:
        return 0.0, {}, "no comparable regime metrics between real and sim"
    return 100.0 / (1.0 + penalty), m, None


# ---------------------------------------------------------------------------
# One flight of one log's mission (used once per log per trial)
# ---------------------------------------------------------------------------
def fly_one(log, container, addr, timeouts, out_stem):
    """Restart SITL under the log's wind, upload its params + mission, fly,
    pull the BIN. Returns (bin_path | None, fail | None)."""
    if log['wind']:
        patch_parm_wind(log['parm_target'], log['wind'])
    try:
        restart_sitl(container)
    except Exception as e:
        return None, f"docker restart failed: {e}"

    mav = rmc.connect_with_retry(addr, timeouts['connect'])
    try:
        s = fly_mission.Telem(mav)
        if not fly_mission.gate(s, timeout=timeouts['gate']):
            return None, ('EKF gate failed — is the UE level running in '
                          'ue_physics mode?')
        if log['params']:
            rmc.upload_params(mav, log['params'])
        if fly_mission.upload_mission(mav, log['mission']) == 0:
            return None, 'mission upload failed'
        if not fly_mission.fly(s, log['n_items'], timeouts['fly'],
                               out_stem + '.csv'):
            return None, 'mission did not complete (timeout/never flew)'
        time.sleep(3)                            # let AP close the log file
        if not rmc.pull_dataflash(container, out_stem + '.bin'):
            return None, 'could not pull the dataflash log'
        return out_stem + '.bin', None
    finally:
        try:
            mav.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Apply mode (init-menu Calibration section: update existing / add as new)
# ---------------------------------------------------------------------------
def apply_result(result_path, as_new=None):
    best = load_json(result_path)
    vehicle = best['vehicle']
    src_dir = os.path.join(PROJECT_DIR, 'Vehicles', vehicle)
    if as_new:
        new_name = re.sub(r'[^A-Za-z0-9_\-]', '_', as_new).strip('_')
        if not new_name:
            raise SystemExit(f"--as-new: '{as_new}' leaves no usable folder name")
        dst_dir = os.path.join(PROJECT_DIR, 'Vehicles', new_name)
        if os.path.isdir(dst_dir):
            raise SystemExit(f"vehicle '{new_name}' already exists — pick "
                             "another name or use plain --apply-result on it")
        shutil.copytree(src_dir, dst_dir)
        mj = os.path.join(dst_dir, 'model.json')
        if os.path.isfile(mj):
            try:
                doc = load_json(mj)
                for key in ('name', 'display_name'):
                    if key in doc:
                        doc[key] = as_new
                save_json(mj, doc)
            except ValueError:
                pass
        vehicle, src_dir = new_name, dst_dir
        print(f"[OK] vehicle duplicated: {new_name}")

    files = vehicle_files(src_dir)
    logs = ', '.join(os.path.basename(p) for p in best.get('real_logs', []))
    note = (f"Fitted by tune_plant.py {best.get('generated', '?')} against "
            f"{logs or 'real flight logs'} under each flight's own onboard "
            f"wind and params (score {best.get('score', 0):.1f}, baseline "
            f"{best.get('baseline_score')}). cd0 and pusher_vpitch_ms are ONE "
            "paired calibration — never change one alone; re-run the cruise "
            "gate after any edit (docs/INVARIANTS.md).")
    apply_values(files, best['params'], provenance={n: note for n in best['params']})
    winds = best.get('winds') or []
    if winds and winds[0]:
        patch_parm_wind(files['parm'], winds[0])
    print(f"[OK] calibration applied to Vehicles/{vehicle} "
          f"({fmt_vals(best['params'])})")
    return vehicle


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Fit plant (physics-model) parameters against one or "
                    "more real flights by replaying their missions.")
    ap.add_argument('--real', action='append', default=[],
                    help="real flight dataflash log (repeatable — each log "
                         "is one fitting condition flown per trial)")
    ap.add_argument('--real-params', action='append', default=[],
                    help="params for the matching --real (by order; '' or "
                         "omitted = extract from the log itself)")
    ap.add_argument('--vehicle-name',
                    help="vehicle model under Vehicles/ (default: "
                         "Vehicles/active_vehicle.txt)")
    ap.add_argument('--trials', type=int, default=15,
                    help="number of tuning trials (default 15; each trial "
                         "flies EVERY log's mission once)")
    ap.add_argument('--accept', action='store_true',
                    help="write the best values into the vehicle JSONs at "
                         "the end (default: restore originals untouched)")
    ap.add_argument('--apply-result',
                    help="no tuning: apply a saved best_plant.json to the "
                         "vehicle (Calibration menu 'update existing')")
    ap.add_argument('--as-new',
                    help="with --apply-result: duplicate the vehicle under "
                         "this name first and apply there ('add as new')")
    ap.add_argument('--no-wind-from-real', action='store_true',
                    help="keep the vehicle's own SIM_WIND_* instead of each "
                         "log's onboard wind estimate")
    ap.add_argument('--addr', default='tcp:localhost:5760')
    ap.add_argument('--timeout', type=float, default=600,
                    help="per-flight timeout, seconds (default 600 — a stuck "
                         "flight fails fast; real missions are ~2-5 min)")
    ap.add_argument('--connect-timeout', type=float, default=600)
    ap.add_argument('--gate-timeout', type=float, default=600)
    ap.add_argument('--progress-json',
                    help="progress file rewritten after every trial (the "
                         "init-menu Calibration section polls it)")
    ap.add_argument('--seed', type=int, default=42)
    a = ap.parse_args()

    if a.apply_result:
        apply_result(os.path.abspath(a.apply_result), a.as_new)
        return
    if not a.real:
        raise SystemExit("give at least one --real flight log "
                         "(or --apply-result to apply a saved fit)")

    reals = [os.path.abspath(p) for p in a.real]
    for p in reals:
        if not os.path.isfile(p):
            raise SystemExit(f"real flight log not found: {p}")
    given_params = list(a.real_params) + [''] * (len(reals) - len(a.real_params))

    vehicle = a.vehicle_name or rmc.active_vehicle_name()
    vehicle_dir = os.path.join(PROJECT_DIR, 'Vehicles', vehicle)
    files = vehicle_files(vehicle_dir)
    for p in files.values():
        if not os.path.isfile(p):
            raise SystemExit(f"vehicle file not found: {p}")

    ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    session_dir = os.path.join(LOG_DIR, f"plant_{ts}_{vehicle}")
    backup_dir = os.path.join(session_dir, 'originals')
    os.makedirs(backup_dir, exist_ok=True)
    prog = Progress(os.path.abspath(a.progress_json)
                    if a.progress_json else
                    os.path.join(session_dir, 'progress.json'))
    prog.update(tool='tune_plant', vehicle=vehicle, trials_total=a.trials,
                trials_done=0, session_dir=session_dir, n_logs=len(reals),
                optimizer='optuna' if HAVE_OPTUNA else 'builtin',
                message='parsing the real flight(s)...')

    # ---- per-log preparation: regimes + mission + wind + params -------------
    logs = []
    for i, real in enumerate(reals):
        tag = f"log{i + 1}"
        print(f"=== REAL FLIGHT {i + 1}/{len(reals)}: {real} ===")
        reg = real_regimes(real)
        rc = reg['cruise']
        print(f"  cruise: {rc['airspeed_mean']:.1f} m/s @ "
              f"{rc['throttle_mean']:.1f}% thr, {rc['duration_s']:.0f} s")
        mission = os.path.join(session_dir, f'mission_{tag}.waypoints')
        mission, n_items = extract_mission(real, mission)
        wind = None if a.no_wind_from_real else extract_wind(real)
        if wind and wind['spd_ms'] < 0.05:
            wind = None                # EKF2-era logs report zero wind states
        if wind:
            print(f"  wind: {wind['spd_ms']} m/s from {wind['dir_deg']} deg, "
                  f"turb {wind['turb_ms']}")
        if given_params[i]:
            params = os.path.abspath(given_params[i])
            if not os.path.isfile(params):
                raise SystemExit(f"--real-params file not found: {params}")
        else:
            params = extract_params_from_bin(
                real, os.path.join(session_dir, f'params_{tag}.parm'))
            print(f"  params: {'extracted from the log' if params else 'NONE in log'}")
        logs.append({'real': real, 'regimes': reg, 'mission': mission,
                     'n_items': n_items, 'wind': wind, 'params': params,
                     'parm_target': files['parm'], 'tag': tag})
    warn_wind_override()

    for p in files.values():
        shutil.copy2(p, os.path.join(backup_dir, os.path.basename(p)))
    print(f"=== ORIGINALS BACKED UP: {backup_dir} ===")

    baseline = read_baseline(files)
    space = {name: rng for name, (_, rng) in PLANT_SPACE.items()}
    print(f"=== SEARCH: {len(space)} plant params, {a.trials} trials x "
          f"{len(reals)} log(s), optimizer="
          f"{'optuna TPE' if HAVE_OPTUNA else 'builtin'} ===")
    print(f"    baseline: {fmt_vals(baseline)}")

    if HAVE_OPTUNA:
        sampler = OptunaSampler(space, a.trials, a.seed)
        sampler.enqueue(baseline)
    else:
        sampler = FallbackSampler(space, a.trials, a.seed)

    container = rmc.find_sitl_container()
    if not container:
        raise SystemExit("no running cezeri_sitl_* container — start the "
                         "Docker fleet (and the UE level) first")

    timeouts = {'connect': a.connect_timeout, 'gate': a.gate_timeout,
                'fly': a.timeout}
    best = {'trial': 0, 'score': -1.0, 'params': {}, 'per_log': {}}
    baseline_score = None
    fail_streak = 0

    try:
        for k in range(1, a.trials + 1):
            vals = baseline if (k == 1 and not HAVE_OPTUNA) else sampler.ask()
            print(f"\n=== TRIAL {k}/{a.trials}: {fmt_vals(vals)} ===")
            prog.update(state='running', trial_running=k,
                        message=f"trial {k}/{a.trials} flying "
                                f"({len(reals)} log(s))...",
                        last_params=vals)
            apply_values(files, vals)

            per_log, scores, fails = {}, [], []
            for log in logs:
                stem = os.path.join(session_dir, f"trial_{k:02d}_{log['tag']}")
                bin_path, fail = fly_one(log, container, a.addr, timeouts, stem)
                if not fail:
                    try:
                        s_score, deltas, fail = score_sim(bin_path,
                                                          log['regimes'])
                    except Exception as e:
                        s_score, deltas, fail = 0.0, {}, f"scoring failed: {e}"
                else:
                    s_score, deltas = 0.0, {}
                scores.append(s_score)
                per_log[log['tag']] = {'score': round(s_score, 1),
                                       'deltas': deltas, 'fail': fail}
                if fail:
                    fails.append(f"{log['tag']}: {fail}")
                    print(f"  {log['tag']} FAILED: {fail}")
                else:
                    print(f"  {log['tag']} score {s_score:.1f}  "
                          + '  '.join(f"{n}={v:+.2f}"
                                      for n, v in deltas.items()))

            score = float(np.mean(scores)) if scores else 0.0
            all_failed = len(fails) == len(logs)
            sampler.tell(vals, score)
            fail_streak = fail_streak + 1 if all_failed else 0
            print(f"  TRIAL {k} mean score {score:.1f}"
                  + (f"  ({'; '.join(fails)})" if fails else ''))
            if k == 1 and not all_failed:
                baseline_score = score
            if score > best['score']:
                best.update(trial=k, score=score, params=dict(vals),
                            per_log=per_log)
            prog.update(trials_done=k,
                        last={'trial': k, 'score': round(score, 1),
                              'per_log': per_log},
                        baseline_score=(round(baseline_score, 1)
                                        if baseline_score is not None
                                        else None),
                        best={'trial': best['trial'],
                              'score': round(best['score'], 1),
                              'params': {n: round(v, 4) for n, v
                                         in best['params'].items()}},
                        message=f"trial {k}/{a.trials} done")
            if fail_streak >= 3:
                prog.update(state='failed',
                            message='3 trials in a row failed on every log '
                                    '— aborting. Check the simulator.')
                raise SystemExit(1)
    except KeyboardInterrupt:
        print("\n[STOP] interrupted — keeping the best result so far")
    finally:
        for p in files.values():
            src = os.path.join(backup_dir, os.path.basename(p))
            if os.path.isfile(src):
                shutil.copy2(src, p)
        print("=== ORIGINAL VEHICLE FILES RESTORED ===")

    if best['score'] < 0 or not best['params']:
        prog.update(state='failed', message='no trial produced a score')
        raise SystemExit(1)

    print(f"\n=== BEST: trial {best['trial']} score {best['score']:.1f} "
          f"(baseline {baseline_score if baseline_score is not None else '?'})"
          f" ===\n    {fmt_vals(best['params'])}")
    for tag, r in best['per_log'].items():
        print(f"    {tag}: score {r['score']}  "
              + '  '.join(f"{n}={v:+.2f}" for n, v in r['deltas'].items()))

    result_path = os.path.join(session_dir, 'best_plant.json')
    with open(result_path, 'w', encoding='utf-8') as f:
        json.dump({'vehicle': vehicle, 'real_logs': reals,
                   'score': best['score'], 'baseline_score': baseline_score,
                   'params': best['params'], 'per_log': best['per_log'],
                   'winds': [lg['wind'] for lg in logs],
                   'trials': a.trials, 'generated': ts},
                  f, indent=2)
    print(f"[OK] result: {result_path}")
    prog.update(state='done', result=result_path,
                message=f"finished — best score {best['score']:.1f} on trial "
                        f"{best['trial']} (baseline {baseline_score}). "
                        "Use Save As New / Update Existing to apply it.")

    if a.accept:
        apply_result(result_path)
        try:
            restart_sitl(container)
        except Exception:
            pass


if __name__ == '__main__':
    main()

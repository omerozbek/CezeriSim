#!/usr/bin/env python3
"""
run_mission_compare.py — Fly a mission in the simulator and (optionally)
compare against a real flight, end to end:

  1. Mission source: --mission <file.waypoints> when given, otherwise the
     mission is extracted from the --real flight log (mission_from_log.py).
  2. Wait for the SITL container + UE bridge (retries until --connect-timeout).
  3. Optional: upload a --params .parm/.param file over MAVLink before the
     flight (like a GCS "Load params"; boot-time SIM_* values excepted).
  4. Upload and fly the mission in AUTO (fly_mission.py machinery), telemetry
     CSV at ~4 Hz.
  5. Pull the dataflash .BIN out of the SITL container (auto-detected
     cezeri_sitl_* name) -> Scripts/control/logs/<date-time>_<vehicle>.bin —
     named for future use (date/time + the vehicle model that flew).
  6. If --real was given: run compare_logs.py real-vs-sim — interactive HTML
     report (auto-opens) + optional --summary-json for the UE Compare menu.

Launched by the init level's COMPARE menu ("Start Mission Simulation"), which
starts the Docker fleet and opens the simulation level in parallel — this
script simply waits for the vehicle to appear. Also usable standalone:

    python run_mission_compare.py --real "C:/path/real_flight.bin"
    python run_mission_compare.py --mission "C:/path/plan.waypoints"
"""
import argparse
import os
import re
import subprocess
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
CONTROL_DIR = os.path.normpath(os.path.join(HERE, '..', 'control'))
PROJECT_DIR = os.path.normpath(os.path.join(HERE, '..', '..'))
LOG_DIR = os.path.join(CONTROL_DIR, 'logs')
sys.path.insert(0, CONTROL_DIR)
sys.path.insert(0, HERE)

import fly_mission                       # noqa: E402  (Scripts/control)
import param_manager                     # noqa: E402  (Scripts/control)
from mission_from_log import extract_mission  # noqa: E402


def connect_with_retry(addr, total_timeout):
    """fly_mission.connect(), retried until the SITL MAVLink port answers."""
    print(f"=== CONNECT: {addr} (up to {total_timeout:.0f}s — the Docker "
          f"container and the UE level may still be starting) ===")
    t0 = time.time()
    attempt = 0
    while time.time() - t0 < total_timeout:
        attempt += 1
        try:
            mav = fly_mission.connect(addr)
            if mav.target_system:
                return mav
            mav.close()
            print(f"  [{attempt}] connected but no heartbeat yet — retrying")
        except Exception as e:
            print(f"  [{attempt}] SITL not reachable yet "
                  f"({type(e).__name__}) — retrying in 5 s")
        time.sleep(5)
    raise SystemExit(f"CONNECT FAIL: no SITL heartbeat on {addr} after "
                     f"{total_timeout:.0f}s — is the Docker fleet running?")


def find_sitl_container():
    """Name of the running cezeri_sitl_* container (first one, VTOL first)."""
    try:
        r = subprocess.run(
            ["docker", "ps", "--format", "{{.Names}}",
             "--filter", "name=cezeri_sitl"],
            capture_output=True, text=True, timeout=15)
        names = [n for n in r.stdout.split() if n]
    except Exception as e:
        print(f"[WARN] docker ps failed: {e}")
        return None
    if not names:
        return None
    names.sort(key=lambda n: (0 if 'vtol' in n else 1, n))
    return names[0]


def pull_dataflash(container, dest_bin):
    """Copy the newest dataflash .BIN out of the SITL container."""
    try:
        r = subprocess.run(
            ["docker", "exec", container, "sh", "-c",
             "ls -t $(find / -name '*.BIN' -path '*logs*' 2>/dev/null) "
             "2>/dev/null | head -1"],
            capture_output=True, text=True, timeout=30)
        newest = r.stdout.strip()
        if not newest:
            print(f"[WARN] no dataflash .BIN found in {container}")
            return None
        subprocess.run(["docker", "cp", f"{container}:{newest}", dest_bin],
                       check=True, timeout=60)
        sz = os.path.getsize(dest_bin) / 1e6
        print(f"[OK] dataflash log: {dest_bin} ({sz:.1f} MB, from "
              f"{container}:{newest})")
        return dest_bin
    except Exception as e:
        print(f"[WARN] dataflash pull failed: {e}")
        return None


def active_vehicle_name():
    """Vehicles/active_vehicle.txt (the VTOL a mission replay flies)."""
    try:
        with open(os.path.join(PROJECT_DIR, 'Vehicles', 'active_vehicle.txt'),
                  encoding='utf-8') as f:
            name = f.read().strip()
        return name or 'vehicle'
    except OSError:
        return 'vehicle'


def upload_params(mav, path):
    """Upload a .parm/.param file over MAVLink (GCS-style 'Load params')."""
    from pathlib import Path
    params = param_manager.parse_parm_file(Path(path))
    if not params:
        print(f"[WARN] no parameters parsed from {path} — skipping upload")
        return
    print(f"=== PARAMS: uploading {len(params)} value(s) from "
          f"{os.path.basename(path)} ===")
    ok = 0
    from pymavlink import mavutil
    for pname, pval in params.items():
        mav.mav.param_set_send(
            mav.target_system, mav.target_component,
            pname.encode('ascii')[:16], pval,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.2)
        if msg and msg.param_id.rstrip('\x00') == pname:
            ok += 1
        time.sleep(0.005)
    print(f"[OK] params uploaded ({ok}/{len(params)} acked — unacked values "
          f"are usually still applied). Boot-time SIM_* values (SIM_RATE_HZ) "
          f"still need the vehicle model's own params.parm.")


def main():
    ap = argparse.ArgumentParser(
        description="Fly a mission in the simulator and optionally compare "
                    "against a real flight log.")
    ap.add_argument('--real',
                    help="real flight log (.bin/.BIN/.log/.tlog) — comparison "
                         "baseline, and the mission source when --mission is "
                         "not given")
    ap.add_argument('--mission',
                    help="QGC WPL 110 .waypoints file to fly (skips the "
                         "extraction from the real log)")
    ap.add_argument('--params',
                    help="ArduPilot .parm/.param file uploaded over MAVLink "
                         "before the flight")
    ap.add_argument('--vehicle-name',
                    help="vehicle model name used in the saved sim log's "
                         "file name (default: Vehicles/active_vehicle.txt)")
    ap.add_argument('--addr', default='tcp:localhost:5760',
                    help="SITL MAVLink address (default tcp:localhost:5760 "
                         "= first UAV)")
    ap.add_argument('--timeout', type=float, default=1500,
                    help="mission flight timeout, seconds (default 1500)")
    ap.add_argument('--connect-timeout', type=float, default=600,
                    help="how long to wait for the SITL heartbeat (default "
                         "600 s)")
    ap.add_argument('--gate-timeout', type=float, default=600,
                    help="how long to wait for EKF GPS fusion (default 600 s "
                         "— needs the UE level up in ue_physics mode)")
    ap.add_argument('--summary-json',
                    help="write the comparison's closeness stats here as "
                         "JSON (read by the UE Compare menu)")
    ap.add_argument('--no-compare', action='store_true',
                    help="fly and pull the log, skip the comparison report")
    a = ap.parse_args()

    if not a.real and not a.mission:
        raise SystemExit("give the mission: --mission plan.waypoints, or "
                         "--real flight.bin to extract it from")

    real = os.path.abspath(a.real) if a.real else None
    if real and not os.path.isfile(real):
        raise SystemExit(f"real flight log not found: {real}")
    if a.params and not os.path.isfile(a.params):
        raise SystemExit(f"params file not found: {a.params}")
    os.makedirs(LOG_DIR, exist_ok=True)
    # Log stamp: date/time + the vehicle that flew — "for future use" naming.
    ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    vehicle = re.sub(r'[^A-Za-z0-9_\-]', '_',
                     a.vehicle_name or active_vehicle_name())

    # --- 1. mission source --------------------------------------------------
    if a.mission:
        mission = os.path.abspath(a.mission)
        if not os.path.isfile(mission):
            raise SystemExit(f"waypoints file not found: {mission}")
        # fly_mission needs the item count for progress/completion tracking.
        with open(mission, encoding='utf-8') as f:
            lines = [ln for ln in f.read().splitlines()[1:] if ln.strip()]
        n = len(lines)
        print(f"=== MISSION: {mission} ({n} items) ===")
    else:
        stem = os.path.splitext(os.path.basename(real))[0]
        mission = os.path.join(LOG_DIR, f"mission_from_{stem}.waypoints")
        print(f"=== MISSION EXTRACT: {real} ===")
        mission, n = extract_mission(real, mission)
        print(f"  {n} items -> {mission}")

    # --- 2..3. connect, gate, params, upload, fly ---------------------------
    mav = connect_with_retry(a.addr, a.connect_timeout)
    s = fly_mission.Telem(mav)
    if not fly_mission.gate(s, timeout=a.gate_timeout):
        raise SystemExit(1)
    if a.params:
        upload_params(mav, os.path.abspath(a.params))
    if fly_mission.upload_mission(mav, mission) == 0:
        raise SystemExit(1)
    csv_path = os.path.join(LOG_DIR, f"{ts}_{vehicle}_telemetry.csv")
    ok = fly_mission.fly(s, n, a.timeout, csv_path)
    if not ok:
        print("[WARN] mission did not complete cleanly — the pulled log may "
              "only cover part of the flight")

    # --- 4. pull the sim dataflash log -------------------------------------
    time.sleep(3)                        # let AP close the log file
    container = find_sitl_container()
    if not container:
        raise SystemExit("no running cezeri_sitl_* container found — "
                         "cannot pull the simulation log")
    sim_bin = pull_dataflash(container,
                             os.path.join(LOG_DIR, f"{ts}_{vehicle}.bin"))
    if not sim_bin:
        raise SystemExit(1)

    # --- 5. compare ---------------------------------------------------------
    if a.no_compare or not real:
        why = "comparison skipped" if a.no_compare else \
              "no --real log given, nothing to compare against"
        print(f"[OK] done ({why}). Sim log saved for future use: {sim_bin}")
        return
    cmd = [sys.executable, os.path.join(HERE, 'compare_logs.py'),
           real, sim_bin, '--mission', mission]
    if a.summary_json:
        cmd += ['--summary-json', a.summary_json]
    print("=== COMPARE: real vs simulated flight ===")
    r = subprocess.run(cmd)
    raise SystemExit(r.returncode)


if __name__ == '__main__':
    main()

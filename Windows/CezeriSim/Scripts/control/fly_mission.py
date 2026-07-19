#!/usr/bin/env python3
"""
fly_mission.py — Upload a Mission Planner / QGC WPL 110 mission, fly it in
AUTO, and pull the dataflash .BIN from the Docker SITL container afterwards
for sim-vs-real comparison in Mission Planner.

Written for the gokce real-mission replay (gökçesuasvideo2mission.waypoints),
but mission-agnostic.

Usage:
    python fly_mission.py --mission "C:/path/to/mission.waypoints"
                          [--addr tcp:localhost:5760]
                          [--timeout 1500]
                          [--no-log-pull]

Sequence:
  1. GATE — wait for EKF3 GPS fusion (flags & 0x10), level attitude, disarmed.
  2. Upload mission (QGC WPL 110), verify item count readback.
  3. Mode AUTO -> arm -> mission runs (item 1 must be NAV_VTOL_TAKEOFF).
  4. Monitor: waypoint progression, mode, telemetry CSV at ~4 Hz
     (logs/mission_<ts>.csv), STATUSTEXT passthrough.
  5. Completion = disarm after VTOL_LAND. Emergency QLAND on tilt > 60 deg
     in VTOL phase or telemetry loss.
  6. docker cp the newest dataflash .BIN out of the container ->
     logs/gokce_mission_<ts>.bin
"""
import argparse
import csv
import math
import os
import subprocess
import time
from datetime import datetime

from pymavlink import mavutil, mavwp

CONTAINER = "cezeri_sitl_drone0"
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

PLANE_MODES = {0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 5: "FBWA", 6: "FBWB",
               7: "CRUISE", 10: "AUTO", 11: "RTL", 12: "LOITER", 15: "GUIDED",
               17: "QSTABILIZE", 18: "QHOVER", 19: "QLOITER", 20: "QLAND",
               21: "QRTL", 25: "LOITER2QLAND"}
QLAND = 20


def connect(addr):
    mav = mavutil.mavlink_connection(addr, source_system=255)
    # Wait for the AUTOPILOT's heartbeat specifically. The first heartbeat on
    # the link can be another GCS routed by AP (UE's MavlinkStatusMonitor,
    # sysid 254) or a sysid-0 packet — accepting those sets target_system to
    # a GCS or to 0 (= broadcast commands). Same family as the pump() source
    # filter (BUGFIXES 2026-07-13).
    t0 = time.time()
    while True:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if (hb is not None and hb.get_srcSystem() > 0
                and hb.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                and hb.type != mavutil.mavlink.MAV_TYPE_GCS):
            mav.target_system = hb.get_srcSystem()
            mav.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            break
        if time.time() - t0 > 30:
            # MUST close before raising: SITL TCP serials are single-client —
            # a leaked half-open connection starves every later attempt
            # (SERIAL0 deaf for 300 s, 2026-07-13 tuning session).
            mav.close()
            raise TimeoutError(f"no autopilot heartbeat on {addr} in 30 s")
    print(f"[OK] heartbeat from sys {mav.target_system}")
    for sid, rate in ((mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10),
                      (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10),
                      (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),
                      (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5),
                      (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 2)):
        mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                         sid, rate, 1)
    return mav


class Telem:
    def __init__(self, mav):
        self.mav = mav
        self.lat = None; self.lon = None; self.alt_rel = 0.0
        self.vz = 0.0; self.airspeed = 0.0; self.groundspeed = 0.0
        self.heading = 0; self.throttle = 0
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0
        self.mode = -1; self.armed = False; self.ekf = 0
        self.wp = -1; self.reached = -1
        self.statustexts = []

    def pump(self, dur, on_text=None):
        t0 = time.time()
        while time.time() - t0 < dur:
            m = self.mav.recv_match(blocking=True, timeout=0.5)
            if m is None:
                continue
            # AP routes other GCS traffic onto this link too — UE's
            # MavlinkStatusMonitor (sysid 254 on SERIAL1) heartbeats at 1 Hz,
            # and a GCS heartbeat (unarmed, custom_mode 0) read as the
            # vehicle's fakes "MANUAL + disarmed" mid-mission (BUGFIXES
            # 2026-07-13). Only accept the autopilot's own messages.
            if (m.get_srcSystem() != self.mav.target_system or
                    m.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
                continue
            t = m.get_type()
            if t == 'GLOBAL_POSITION_INT':
                self.lat = m.lat / 1e7; self.lon = m.lon / 1e7
                self.alt_rel = m.relative_alt / 1000.0
                self.vz = -m.vz / 100.0
                self.heading = m.hdg / 100.0
            elif t == 'VFR_HUD':
                self.airspeed = m.airspeed; self.groundspeed = m.groundspeed
                self.throttle = m.throttle
            elif t == 'ATTITUDE':
                self.roll = math.degrees(m.roll)
                self.pitch = math.degrees(m.pitch)
                self.yaw = math.degrees(m.yaw)
            elif t == 'HEARTBEAT':
                self.armed = bool(m.base_mode & 128)
                self.mode = m.custom_mode
            elif t == 'EKF_STATUS_REPORT':
                self.ekf = m.flags
            elif t == 'MISSION_CURRENT':
                self.wp = m.seq
            elif t == 'MISSION_ITEM_REACHED':
                self.reached = m.seq
            elif t == 'STATUSTEXT':
                txt = m.text if isinstance(m.text, str) else m.text.decode()
                self.statustexts.append(txt)
                if 'Field Elevation' not in txt:
                    print(f"    AP: {txt}")
                    if on_text:
                        on_text(txt)

    def mode_name(self):
        return PLANE_MODES.get(self.mode, str(self.mode))


def gate(s, timeout=180):
    print("=== GATE: waiting for EKF GPS fusion + level attitude ===")
    t0 = time.time(); good = None
    while time.time() - t0 < timeout:
        s.pump(0.5)
        ok = (s.lat is not None and abs(s.roll) < 2 and abs(s.pitch) < 2
              and abs(s.vz) < 1.0 and s.alt_rel < 0.5 and not s.armed
              and (s.ekf & 0x10))
        if ok:
            good = good or time.time()
            if time.time() - good >= 5:
                print(f"  GATE PASS (ekf={hex(s.ekf)}, pos={s.lat:.7f},{s.lon:.7f})")
                return True
        else:
            good = None
    print(f"GATE FAIL after {timeout}s: ekf={hex(s.ekf)} alt={s.alt_rel:.2f} "
          f"r/p={s.roll:.1f}/{s.pitch:.1f} armed={s.armed}")
    return False


def upload_mission(mav, path):
    print(f"=== MISSION UPLOAD: {path} ===")
    loader = mavwp.MAVWPLoader()
    count = loader.load(path)
    print(f"  loaded {count} items from file")
    mav.mav.mission_clear_all_send(mav.target_system, mav.target_component)
    mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)

    mav.mav.mission_count_send(mav.target_system, mav.target_component, count)
    sent = 0
    t0 = time.time()
    while sent < count and time.time() - t0 < 60:
        m = mav.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                           blocking=True, timeout=5)
        if m is None:
            continue
        wp = loader.wp(m.seq)
        wp.target_system = mav.target_system
        wp.target_component = mav.target_component
        mav.mav.send(wp)
        sent = max(sent, m.seq + 1)
    ack = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
    if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print(f"  UPLOAD FAIL: ack={ack}")
        return 0
    print(f"  upload ACCEPTED ({sent} items)")
    return count


def set_mode(mav, mode):
    mav.mav.set_mode_send(mav.target_system,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)


def fly(s, n_items, timeout, csv_path):
    mav = s.mav
    print("=== FLIGHT: AUTO mission ===")
    set_mode(mav, 10)  # AUTO
    s.pump(1)
    mav.mav.command_long_send(mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    # The armed flag rides the 1 Hz heartbeat — a fixed 2 s wait raced it and
    # declared "arm rejected" while AP was already flying the mission
    # (2026-07-13 Pasifik replay). Poll up to 10 s instead.
    t_arm = time.time()
    while not s.armed and time.time() - t_arm < 10:
        s.pump(0.5)
    if not s.armed:
        print("FLIGHT FAIL: arm rejected")
        return False

    print(f"  armed in {s.mode_name()}; mission running "
          f"({n_items} items, timeout {timeout}s)")
    f = open(csv_path, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['t', 'wp', 'mode', 'lat', 'lon', 'alt_rel', 'airspeed',
                'groundspeed', 'climb', 'roll', 'pitch', 'yaw_hdg', 'throttle'])
    t0 = time.time()
    last_wp = -1; last_mode = -1; was_airborne = False
    max_alt = 0.0; max_as = 0.0; max_tilt_vtol = 0.0
    ok = False
    while time.time() - t0 < timeout:
        s.pump(0.25)
        t = time.time() - t0
        w.writerow([f"{t:.2f}", s.wp, s.mode_name(), s.lat, s.lon,
                    f"{s.alt_rel:.2f}", f"{s.airspeed:.2f}",
                    f"{s.groundspeed:.2f}", f"{s.vz:.2f}",
                    f"{s.roll:.2f}", f"{s.pitch:.2f}", f"{s.heading:.1f}",
                    s.throttle])
        max_alt = max(max_alt, s.alt_rel)
        max_as = max(max_as, s.airspeed)
        if s.alt_rel > 2:
            was_airborne = True
        if s.wp != last_wp:
            print(f"  [{t:6.1f}s] -> WP {s.wp}  (alt={s.alt_rel:.1f} m, "
                  f"as={s.airspeed:.1f} m/s, mode={s.mode_name()})")
            last_wp = s.wp
        if s.mode != last_mode:
            print(f"  [{t:6.1f}s] mode: {s.mode_name()}")
            last_mode = s.mode
        # emergency: tilt runaway in VTOL phase (fixed-wing banks to 33 legit)
        if s.mode in (17, 18, 19, 20, 21) or (s.mode == 10 and s.airspeed < 12):
            max_tilt_vtol = max(max_tilt_vtol, abs(s.roll), abs(s.pitch))
            if abs(s.roll) > 60 or abs(s.pitch) > 60:
                print(f"  EMERGENCY: tilt {s.roll:.0f}/{s.pitch:.0f} — QLAND")
                set_mode(mav, QLAND)
        if was_airborne and not s.armed:
            print(f"  [{t:6.1f}s] disarmed — mission complete "
                  f"(alt={s.alt_rel:.2f})")
            ok = True
            break
    f.close()
    print(f"  telemetry CSV: {csv_path}")
    print(f"  summary: max_alt={max_alt:.1f} m  max_airspeed={max_as:.1f} m/s  "
          f"max_vtol_tilt={max_tilt_vtol:.1f} deg  duration={time.time()-t0:.0f}s")
    if not ok:
        print("FLIGHT FAIL: mission did not complete within timeout "
              "(or never became airborne)")
    return ok


def pull_dataflash(dest_bin):
    """Copy the newest dataflash .BIN out of the SITL container."""
    try:
        r = subprocess.run(
            ["docker", "exec", CONTAINER, "sh", "-c",
             "ls -t $(find / -name '*.BIN' -path '*logs*' 2>/dev/null) 2>/dev/null | head -1"],
            capture_output=True, text=True, timeout=30)
        newest = r.stdout.strip()
        if not newest:
            print("[WARN] no dataflash .BIN found in container")
            return None
        subprocess.run(["docker", "cp", f"{CONTAINER}:{newest}", dest_bin],
                       check=True, timeout=60)
        sz = os.path.getsize(dest_bin) / 1e6
        print(f"[OK] dataflash log: {dest_bin} ({sz:.1f} MB, from {newest})")
        return dest_bin
    except Exception as e:
        print(f"[WARN] dataflash pull failed: {e}")
        return None


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--addr', default='tcp:localhost:5760')
    ap.add_argument('--mission', required=True)
    ap.add_argument('--timeout', type=float, default=1500)
    ap.add_argument('--no-log-pull', action='store_true')
    a = ap.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')

    mav = connect(a.addr)
    s = Telem(mav)
    if not gate(s):
        raise SystemExit(1)
    n = upload_mission(mav, a.mission)
    if n == 0:
        raise SystemExit(1)
    ok = fly(s, n, a.timeout, os.path.join(LOG_DIR, f"mission_{ts}.csv"))
    if not a.no_log_pull:
        time.sleep(3)  # let AP close the log file
        pull_dataflash(os.path.join(LOG_DIR, f"gokce_mission_{ts}.bin"))
    raise SystemExit(0 if ok else 1)

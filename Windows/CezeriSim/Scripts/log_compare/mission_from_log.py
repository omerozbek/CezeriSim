#!/usr/bin/env python3
"""
mission_from_log.py — Extract the flight mission out of an ArduPilot log into
a QGC WPL 110 .waypoints file (the format fly_mission.py uploads).

A dataflash log (.bin/.BIN/.log) stores the loaded mission as CMD messages;
a telemetry log (.tlog) carries it as MISSION_ITEM_INT/MISSION_ITEM traffic.
This tool replays either into a mission file, so a REAL flight's mission can
be re-flown in the simulator for a like-for-like comparison
(run_mission_compare.py drives that end to end).

Usage:
    python mission_from_log.py FLIGHT_LOG [-o OUT.waypoints]
    python mission_from_log.py FLIGHT_LOG --wind      (also print the flight's
                                                       onboard wind estimate)

If the log contains several mission dumps (e.g. re-uploads), the LAST
complete one wins.

extract_wind() reads ArduPilot's own in-flight wind estimate (EKF3 XKF2
VWN/VWE, EKF2 NKF2 fallback) so a replay can be flown under the REAL
flight's wind instead of whatever SIM_WIND_* happens to be in params.parm —
tune_plant.py uses this to keep plant calibration honest.
"""
import argparse
import math
import os


def _cmd_items_from_dataflash(path):
    """CMD messages -> {seq: (frame, cmd, p1..p4, lat, lon, alt)} (last wins)."""
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    items, total = {}, 0
    while True:
        msg = m.recv_match(type=['CMD'], blocking=False)
        if msg is None:
            break
        # Frame was added to the CMD logger in newer ArduPilot versions;
        # older logs get the QGC defaults (0 = abs for home, 3 = relative).
        frame = getattr(msg, 'Frame', 0 if msg.CNum == 0 else 3)
        items[int(msg.CNum)] = (int(frame), int(msg.CId),
                                float(msg.Prm1), float(msg.Prm2),
                                float(msg.Prm3), float(msg.Prm4),
                                float(msg.Lat), float(msg.Lng), float(msg.Alt))
        total = max(total, int(msg.CTot))
    return items, total


def _cmd_items_from_tlog(path):
    """MISSION_ITEM(_INT) traffic -> same {seq: item} map (last wins)."""
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    items, total = {}, 0
    while True:
        msg = m.recv_match(type=['MISSION_ITEM_INT', 'MISSION_ITEM',
                                 'MISSION_COUNT'], blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == 'MISSION_COUNT':
            total = max(total, int(msg.count))
            continue
        if getattr(msg, 'mission_type', 0) != 0:      # waypoints only
            continue
        if t == 'MISSION_ITEM_INT':
            lat, lon = msg.x / 1e7, msg.y / 1e7
        else:
            lat, lon = msg.x, msg.y
        items[int(msg.seq)] = (int(msg.frame), int(msg.command),
                               float(msg.param1), float(msg.param2),
                               float(msg.param3), float(msg.param4),
                               lat, lon, float(msg.z))
    return items, total


def extract_mission(log_path, out_path):
    """Extract the mission from log_path into a QGC WPL 110 file.
    Returns (out_path, item_count). Raises ValueError when no mission found."""
    ext = os.path.splitext(log_path)[1].lower()
    if ext == '.tlog':
        items, total = _cmd_items_from_tlog(log_path)
    else:
        items, total = _cmd_items_from_dataflash(log_path)

    if not items:
        raise ValueError(f"{log_path}: no mission (CMD / MISSION_ITEM "
                         f"messages) found in the log")
    count = max(total, max(items) + 1)
    missing = [i for i in range(count) if i not in items]
    if missing:
        raise ValueError(f"{log_path}: mission incomplete — items "
                         f"{missing} of {count} never appear in the log")

    with open(out_path, 'w', newline='\n', encoding='utf-8') as f:
        f.write("QGC WPL 110\n")
        for seq in range(count):
            frame, cmd, p1, p2, p3, p4, lat, lon, alt = items[seq]
            current = 1 if seq == 0 else 0
            f.write(f"{seq}\t{current}\t{frame}\t{cmd}\t"
                    f"{p1:.8f}\t{p2:.8f}\t{p3:.8f}\t{p4:.8f}\t"
                    f"{lat:.8f}\t{lon:.8f}\t{alt:.6f}\t1\n")
    return out_path, count


def extract_wind(log_path):
    """ArduPilot's onboard wind estimate from a dataflash log.

    Reads EKF3 XKF2 (VWN/VWE, first core only; NKF2 fallback for EKF2 logs)
    and takes the median over the last 60% of samples — the estimate needs
    fixed-wing flight time to converge, so early samples are dropped.

    Returns {'spd_ms', 'dir_deg', 'turb_ms', 'vwn', 'vwe', 'n'} with
    SIM_WIND_* semantics (dir_deg = compass direction the wind blows FROM;
    turb_ms = a gust sigma from the spread of the estimate), or None when the
    log has no usable wind states."""
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(log_path)
    vwn, vwe = [], []
    while True:
        msg = m.recv_match(type=['XKF2', 'NKF2'], blocking=False)
        if msg is None:
            break
        if getattr(msg, 'C', 0) != 0:           # first EKF core only
            continue
        n = getattr(msg, 'VWN', None)
        e = getattr(msg, 'VWE', None)
        if n is None or e is None:
            continue
        vwn.append(float(n))
        vwe.append(float(e))
    if len(vwn) < 50:
        return None
    import numpy as np
    k = int(len(vwn) * 0.4)                     # drop the convergence phase
    an = np.asarray(vwn[k:])
    ae = np.asarray(vwe[k:])
    mn, me = float(np.median(an)), float(np.median(ae))
    spd = math.hypot(mn, me)
    # (VWN, VWE) is the air-mass velocity vector; the wind blows FROM the
    # opposite compass direction (SIM_WIND_DIR convention).
    dir_from = (math.degrees(math.atan2(me, mn)) + 180.0) % 360.0
    # Gust sigma: on airframes without an airspeed sensor the EKF wind
    # MAGNITUDE is only weakly observable — on the 1-jul pasifik flight the
    # estimate swings 0..21 m/s (std 4-6) even during cruise, which fed
    # into SIM_WIND_TURB's fast OU gust model produces violence the real
    # flight never saw (QLAND on landing). The median speed/direction stay
    # trustworthy (7.75/012 vs the team's hand-recorded 8.2/010), so keep
    # them, and cap the gust sigma at 25% of the mean speed — the SITL
    # convention band — unless the measured spread is genuinely smaller.
    spread = float(np.std(np.hypot(an, ae)))
    spread = min(spread, 0.25 * spd)
    return {'spd_ms': round(spd, 2), 'dir_deg': round(dir_from, 1),
            'turb_ms': round(spread, 2), 'vwn': round(mn, 2),
            'vwe': round(me, 2), 'n': len(an)}


def main():
    ap = argparse.ArgumentParser(
        description="Extract the mission from an ArduPilot log into a "
                    "QGC WPL 110 .waypoints file.")
    ap.add_argument('log', help="flight log (.bin/.BIN/.log/.tlog)")
    ap.add_argument('-o', '--out',
                    help="output .waypoints path (default: next to the log)")
    ap.add_argument('--wind', action='store_true',
                    help="also print the flight's onboard wind estimate "
                         "(SIM_WIND_* values for a like-for-like replay)")
    a = ap.parse_args()

    out = a.out or os.path.splitext(a.log)[0] + '.waypoints'
    out, n = extract_mission(a.log, out)
    print(f"[OK] {n} mission items -> {out}")

    if a.wind:
        w = extract_wind(a.log)
        if w:
            print(f"[OK] onboard wind estimate ({w['n']} samples): "
                  f"SIM_WIND_SPD {w['spd_ms']}  SIM_WIND_DIR {w['dir_deg']}  "
                  f"SIM_WIND_TURB {w['turb_ms']}")
        else:
            print("[WARN] no usable XKF2/NKF2 wind states in the log")


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
wp_tune.py — Fly the loaded mission in AUTO with live parameter overrides and
score how close the vehicle passes each NAV waypoint (ROADMAP 5.6 mission
tuning). Reuses fly_mission.py machinery; designed for repeated flights
against ONE running SITL+UE stack (params are set over MAVLink, no restart —
AP-side nav params only; UE-plant values like wind/aero need a stack restart).

    python wp_tune.py --mission logs/mission_from_00000004.waypoints ^
                      --set WP_RADIUS=25 --set NAVL1_PERIOD=17 --tag iter1

Per flight it reports, for every NAV_WAYPOINT (cmd 16) of the mission:
  miss  = min horizontal point-to-segment distance of the flown track
  alt@  = altitude at the closest approach
plus the risk stats the tuning has to respect: min airspeed while in
fixed-wing flight, max |roll|, seconds below AIRSPEED_MIN, altitude range.
Track/summary saved to logs/wp_tune_<tag>.csv / .json.
"""
import argparse
import json
import math
import os
import time

from pymavlink import mavutil, mavwp

import fly_mission

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
M_LAT = 111320.0


def set_params(mav, overrides):
    """param_set + readback-verify each NAME=VALUE override."""
    ok = True
    for name, val in overrides.items():
        mav.mav.param_set_send(mav.target_system, mav.target_component,
                               name.encode(), float(val),
                               mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        got = None
        t0 = time.time()
        while time.time() - t0 < 5:
            m = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if m and m.param_id.strip('\x00') == name:
                got = m.param_value
                break
        if got is None or abs(got - float(val)) > max(1e-3, abs(val) * 1e-4):
            print(f"  [FAIL] {name} = {val} (readback {got})")
            ok = False
        else:
            print(f"  [set] {name} = {got:g}")
    return ok


def mission_waypoints(path):
    """NAV_WAYPOINT (16) items of a QGC WPL file -> [(seq, lat, lon, alt)]."""
    loader = mavwp.MAVWPLoader()
    loader.load(path)
    out = []
    for i in range(loader.count()):
        wp = loader.wp(i)
        if wp.command == 16 and i > 0:
            out.append((i, wp.x, wp.y, wp.z))
    return out


def score(track, wps):
    """Min point-to-segment horizontal miss per waypoint."""
    if len(track) < 2:
        return []
    lat0, lon0 = track[0][1], track[0][2]
    m_lon = M_LAT * math.cos(math.radians(lat0))

    def xy(lat, lon):
        return ((lat - lat0) * M_LAT, (lon - lon0) * m_lon)

    P = [xy(la, lo) for (_, la, lo, _) in track]
    A = [a for (_, _, _, a) in track]
    res = []
    for seq, wla, wlo, walt in wps:
        w = xy(wla, wlo)
        best = (float('inf'), 0.0)
        for i in range(len(P) - 1):
            x1, y1 = P[i]
            x2, y2 = P[i + 1]
            dx, dy = x2 - x1, y2 - y1
            L2 = dx * dx + dy * dy
            u = 0.0 if L2 == 0 else max(0.0, min(1.0,
                ((w[0] - x1) * dx + (w[1] - y1) * dy) / L2))
            d = math.hypot(w[0] - (x1 + u * dx), w[1] - (y1 + u * dy))
            if d < best[0]:
                best = (d, A[i] + u * (A[i + 1] - A[i]))
        res.append({"seq": seq, "miss_m": round(best[0], 1),
                    "alt_at_closest": round(best[1], 1), "alt_target": walt})
    return res


def fly_and_score(s, n_items, timeout, wps, airspeed_min):
    """fly_mission.fly() equivalent that records the track + risk stats."""
    mav = s.mav
    fly_mission.set_mode(mav, 10)
    s.pump(1)
    mav.mav.command_long_send(mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    t_arm = time.time()
    while not s.armed and time.time() - t_arm < 10:
        s.pump(0.5)
    if not s.armed:
        print("FAIL: arm rejected")
        return None, None

    print(f"  armed; flying ({n_items} items)")
    track = []
    flt = []                     # (t, airspeed, roll, pitch, throttle)
    t0 = time.time()
    min_as_fw = float('inf')
    max_roll = 0.0
    slow_s = 0.0
    was_airborne = False
    last = t0
    while time.time() - t0 < timeout:
        s.pump(0.25)
        now = time.time()
        dt, last = now - last, now
        if s.lat is not None:
            track.append((now - t0, s.lat, s.lon, s.alt_rel))
            flt.append((now - t0, s.airspeed, s.roll, s.pitch, s.throttle))
        # Link-death detection (SITL restarted under us): heartbeats stop.
        hb = s.mav.messages.get('HEARTBEAT')
        if was_airborne and hb is not None and now - hb._timestamp > 10:
            print("  LINK LOST mid-flight — scoring the partial track")
            break
        if s.alt_rel > 2:
            was_airborne = True
        # fixed-wing phase: past transition speed and airborne
        if was_airborne and s.airspeed > 12 and s.armed:
            min_as_fw = min(min_as_fw, s.airspeed)
            max_roll = max(max_roll, abs(s.roll))
            if s.airspeed < airspeed_min:
                slow_s += dt
        if was_airborne and not s.armed:
            print(f"  [{now-t0:6.1f}s] disarmed — mission complete")
            break
    else:
        print("FAIL: timeout")
        return None, None
    if not track:
        print("FAIL: no track recorded")
        return None, None

    stats = {
        "duration_s": round(track[-1][0], 1) if track else 0,
        "min_airspeed_fw_ms": round(min_as_fw, 1) if min_as_fw < 1e9 else None,
        "max_roll_deg": round(max_roll, 1),
        "s_below_airspeed_min": round(slow_s, 1),
        "max_alt_m": round(max(p[3] for p in track), 1) if track else 0,
    }
    return score(track, wps), (stats, track, flt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mission', required=True)
    ap.add_argument('--addr', default='tcp:localhost:5760')
    ap.add_argument('--set', action='append', default=[],
                    metavar='NAME=VALUE', help="param override (repeatable)")
    ap.add_argument('--tag', default=time.strftime('%H%M%S'))
    ap.add_argument('--timeout', type=float, default=400)
    ap.add_argument('--airspeed-min', type=float, default=17.0)
    ap.add_argument('--skip-gate', action='store_true',
                    help="skip the EKF gate (repeat flight on a warm stack)")
    a = ap.parse_args()

    overrides = {}
    for kv in a.set:
        k, v = kv.split('=', 1)
        overrides[k.strip()] = float(v)

    os.makedirs(LOG_DIR, exist_ok=True)
    mav = None
    t0 = time.time()
    while mav is None:
        try:
            mav = fly_mission.connect(a.addr)
        except Exception as e:
            if time.time() - t0 > 300:
                raise SystemExit(f"no SITL heartbeat on {a.addr} after 300 s")
            print(f"  SITL not ready ({type(e).__name__}) — retrying in 10 s")
            time.sleep(10)
    s = fly_mission.Telem(mav)
    if not a.skip_gate and not fly_mission.gate(s, timeout=300):
        raise SystemExit(1)
    # Guard against a stale UE instance: after a fresh stack start the vehicle
    # must sit at the mission home. An orphaned UnrealEditor-Cmd holding port
    # 9002 keeps its old vehicle position (seen 2026-07-13: "fresh" flight
    # took off from the previous flight's landing point).
    wps_all = mavwp.MAVWPLoader()
    wps_all.load(a.mission)
    valid_starts = [("home", wps_all.wp(0).x, wps_all.wp(0).y)]
    for i in range(wps_all.count()):
        if wps_all.wp(i).command == 85:      # NAV_VTOL_LAND: repeat flights
            valid_starts.append(("land WP", wps_all.wp(i).x, wps_all.wp(i).y))
    dists = [(math.hypot((s.lat - la) * M_LAT,
                         (s.lon - lo) * M_LAT * math.cos(math.radians(la))),
              nm) for (nm, la, lo) in valid_starts]
    d_best, at = min(dists)
    if d_best > 40:
        raise SystemExit(f"vehicle is {d_best:.0f} m from any valid start — "
                         f"stale UE instance? kill UnrealEditor-Cmd orphans "
                         f"and restart the stack")
    print(f"  start point: {at} ({d_best:.0f} m off)")
    if overrides:
        print(f"=== PARAM OVERRIDES ({a.tag}) ===")
        if not set_params(mav, overrides):
            raise SystemExit(1)
    if fly_mission.upload_mission(mav, a.mission) == 0:
        raise SystemExit(1)

    wps = mission_waypoints(a.mission)
    res, extra = fly_and_score(s, len(wps) + 3, a.timeout, wps, a.airspeed_min)
    if res is None:
        raise SystemExit(1)
    stats, track, flt = extra

    print(f"=== SCORE [{a.tag}] ===")
    tot = worst = 0.0
    for r in res:
        print(f"  WP{r['seq']}: miss {r['miss_m']:6.1f} m   "
              f"alt@closest {r['alt_at_closest']:5.1f} (target {r['alt_target']:.0f})")
        tot += r['miss_m']
        worst = max(worst, r['miss_m'])
    print(f"  mean {tot/len(res):.1f} m   worst {worst:.1f} m")
    print(f"  risk: min FW airspeed {stats['min_airspeed_fw_ms']} m/s, "
          f"max roll {stats['max_roll_deg']} deg, "
          f"{stats['s_below_airspeed_min']} s below AIRSPEED_MIN, "
          f"max alt {stats['max_alt_m']} m")

    out = {"tag": a.tag, "overrides": overrides, "waypoints": res,
           "mean_miss_m": round(tot / len(res), 1),
           "worst_miss_m": round(worst, 1), **stats}
    with open(os.path.join(LOG_DIR, f"wp_tune_{a.tag}.json"), 'w') as f:
        json.dump(out, f, indent=1)
    with open(os.path.join(LOG_DIR, f"wp_tune_{a.tag}.csv"), 'w') as f:
        f.write("t,lat,lon,alt_rel,airspeed,roll,pitch,throttle\n")
        for p, q in zip(track, flt):
            f.write(f"{p[0]:.2f},{p[1]:.7f},{p[2]:.7f},{p[3]:.2f},"
                    f"{q[1]:.2f},{q[2]:.2f},{q[3]:.2f},{q[4]}\n")
    print(f"  saved logs/wp_tune_{a.tag}.json / .csv")


if __name__ == '__main__':
    main()

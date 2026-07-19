#!/usr/bin/env python3
"""Pasifik 'risk of falling' analysis — quantifies every crash path visible in
the flight logs + the vehicle's parameter-level protections.

Usage: python risk_analysis.py LOG[:SEG] [LOG[:SEG] ...]
SEG is the 1-based armed segment (default: longest).
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from compare_logs import parse_dataflash  # noqa: E402

# ---- vehicle constants (pasifik1temmuzparamroll+qass / real dump) -----------
VS0        = 15.5     # AIRSPEED_STALL, 1 g wings-level stall (m/s)
ASPD_MIN   = 17.0     # AIRSPEED_MIN — TECS floor
Q_ANGLE_MAX = 30.0    # deg, VTOL angle limit (Q_ANGLE_MAX 3000)
Q_ASSIST_SPEED = 15.0 # m/s — quad assist below this airspeed (< VS0!)
Q_ASSIST_ALT   = 45.0 # m   — quad assist below this altitude
GUST_TAU   = 2.0      # s, assumed gust correlation time for the event count


def seg_slice(t, v, seg):
    m = (t >= seg[0]) & (t <= seg[1])
    return t[m], v[m]


def interp_to(t_ref, t, v):
    if len(t) == 0:
        return np.full_like(t_ref, np.nan)
    return np.interp(t_ref, t, v)


def parse_cached(path):
    import pickle
    cache = path + ".riskcache.pkl"
    mt = os.path.getmtime(path)
    if os.path.exists(cache):
        try:
            with open(cache, 'rb') as f:
                key, raw = pickle.load(f)
            if key == mt:
                return raw
        except Exception:
            pass
    raw = parse_dataflash(path)
    with open(cache, 'wb') as f:
        pickle.dump((mt, raw), f)
    return raw


def analyze(path, seg_req=None):
    raw = parse_cached(path)
    segs = sorted(raw.segments, key=lambda s: s[1] - s[0], reverse=True)
    seg = raw.segments[seg_req - 1] if seg_req else segs[0]
    dur = seg[1] - seg[0]
    name = os.path.basename(path)
    print(f"\n{'=' * 78}\n{name}  — armed segment {raw.segments.index(seg) + 1}"
          f"/{len(raw.segments)}, {dur:.0f} s\n{'=' * 78}")

    t_as, aspd = seg_slice(*raw.series['aspd'], seg)
    t_r, roll  = seg_slice(*raw.series['roll'], seg)
    t_p, pitch = seg_slice(*raw.series['pitch'], seg)
    t_a, alt   = seg_slice(*raw.series['alt_rel'], seg)
    t_c, climb = seg_slice(*raw.series['climb_baro'], seg)
    t_tq, thq  = seg_slice(*raw.series['thr_qt'], seg)
    t_tc, thc  = seg_slice(*raw.series['thr_ct'], seg)

    grid = np.arange(seg[0], seg[1], 0.2)
    As   = interp_to(grid, t_as, aspd)
    R    = interp_to(grid, t_r, roll)
    P    = interp_to(grid, t_p, pitch)
    Alt  = interp_to(grid, t_a, alt)
    Clm  = interp_to(grid, t_c, climb)

    # Quad-lifting mask: QTUN can log through cruise with the motors idle, so
    # "quad is carrying the vehicle" = QTUN throttle actually above idle
    # within 1.5 s of the grid point; wing-borne = everything else.
    vtol = np.zeros(len(grid), bool)
    if len(t_tq):
        thq_i = interp_to(grid, t_tq, thq)
        near = np.zeros(len(grid), bool)
        for i, g in enumerate(grid):
            j = np.searchsorted(t_tq, g)
            d = min([abs(t_tq[k] - g) for k in (max(j - 1, 0),
                     min(j, len(t_tq) - 1))])
            near[i] = d < 1.5
        vtol = near & (thq_i > 0.08)
    airborne = Alt > 5.0
    fw = ~vtol & airborne          # wing-borne AND actually flying

    # ---- 1. STALL (fixed-wing phase, bank-adjusted) --------------------------
    print("\n[1] STALL — fixed-wing phase, bank-adjusted stall speed "
          "Vs = 15.5*sqrt(1/cos(bank))")
    if fw.sum() > 5:
        bank = np.abs(R[fw])
        vs_eff = VS0 * np.sqrt(1.0 / np.clip(np.cos(np.radians(bank)), 0.3, 1))
        margin = As[fw] - vs_eff
        tfw = fw.sum() * 0.2
        print(f"  FW time {tfw:.0f} s | airspeed min {np.nanmin(As[fw]):.1f} "
              f"mean {np.nanmean(As[fw]):.1f} m/s | max bank {bank.max():.1f} deg")
        print(f"  worst bank-adjusted stall margin: {np.nanmin(margin):.2f} m/s "
              f"(Vs_eff up to {np.nanmax(vs_eff):.1f} m/s)")
        for thr in (0.0, 1.0, 2.0):
            s = np.nansum(margin < thr) * 0.2
            print(f"  time with margin < {thr:.0f} m/s: {s:5.1f} s")
        # gust-induced stall probability: margin/sigma Gaussian exceedance per
        # independent gust (correlation GUST_TAU) — P(any stall) over the phase
        n_per = max(int(GUST_TAU / 0.2), 1)
        m_ev = np.array([np.nanmin(margin[i:i + n_per])
                         for i in range(0, len(margin), n_per)])
        from math import erf
        for sig in (0.5, 1.0, 1.5, 2.0):
            p_each = 0.5 * (1 - np.array([erf(m / (sig * math.sqrt(2)))
                                          for m in m_ev]))
            p_any = 1 - np.prod(1 - np.clip(p_each, 0, 1))
            print(f"  P(gust-induced stall this flight | gust sigma={sig:.1f} "
                  f"m/s) = {p_any * 100:6.2f} %")
        # altitude at the worst margin — is there room to recover / assist?
        i_w = np.nanargmin(margin)
        alt_w = Alt[fw][i_w]
        print(f"  worst margin occurs at alt {alt_w:.0f} m "
              f"({'below' if alt_w < Q_ASSIST_ALT else 'ABOVE'} Q_ASSIST_ALT "
              f"{Q_ASSIST_ALT:.0f} m safety net)")
    else:
        print("  no fixed-wing phase found")

    # ---- 2. ASSIST GAP -------------------------------------------------------
    print("\n[2] Q_ASSIST GAP — assist speed 15.0 < stall 15.5 m/s")
    if fw.sum() > 5:
        gap = (As[fw] < VS0) & (As[fw] >= Q_ASSIST_SPEED) & \
              (Alt[fw] > Q_ASSIST_ALT)
        print(f"  time stalled-but-no-assist (15.0-15.5 m/s above 45 m): "
              f"{gap.sum() * 0.2:.1f} s")

    # ---- 3. VTOL ATTITUDE ----------------------------------------------------
    print("\n[3] VTOL ATTITUDE — tilt vs Q_ANGLE_MAX 30 deg (loss of control "
          ">> 45-60 deg)")
    if vtol.sum() > 5:
        tilt = np.sqrt(R[vtol] ** 2 + P[vtol] ** 2)
        print(f"  VTOL time {vtol.sum() * 0.2:.0f} s | max tilt "
              f"{np.nanmax(tilt):.1f} deg | time > 25 deg: "
              f"{np.nansum(tilt > 25) * 0.2:.1f} s")

    # ---- 4. TRANSITIONS ------------------------------------------------------
    print("\n[4] TRANSITIONS — wing-only spans (quad throttle < 0.08 airborne)")
    spans = []
    in_s = None
    for i, f in enumerate(fw):
        if f and in_s is None:
            in_s = i
        elif not f and in_s is not None:
            spans.append((in_s, i)); in_s = None
    if in_s is not None:
        spans.append((in_s, len(fw)))
    for a, b in spans:
        if (b - a) * 0.2 < 1.0:
            continue
        seg_as = As[a:b]
        as0, as1 = seg_as[0], seg_as[-1]
        print(f"  quad off t+{grid[a] - seg[0]:5.1f}s @ {as0:4.1f} m/s "
              f"alt {Alt[a]:3.0f} m -> quad back t+{grid[b - 1] - seg[0]:5.1f}s "
              f"@ {as1:4.1f} m/s | span {(b - a) * 0.2:4.1f} s, min airspeed "
              f"{np.nanmin(seg_as):4.1f} m/s (Vs margin {np.nanmin(seg_as) - VS0:+.1f})")

    # ---- 5. SINK / DESCENT ---------------------------------------------------
    print("\n[5] SINK RATE — vertical speed extremes")
    if len(Clm):
        print(f"  max sink {np.nanmin(Clm):.1f} m/s | max climb "
              f"{np.nanmax(Clm):.1f} m/s")
        low = (Alt < 15) & (Clm < -2.5)
        print(f"  time sinking >2.5 m/s below 15 m alt (hard-landing zone): "
              f"{low.sum() * 0.2:.1f} s")

    # ---- 6. THRUST HEADROOM --------------------------------------------------
    print("\n[6] THRUST HEADROOM")
    if len(thq):
        sat_q = np.sum(thq > 0.9) / len(thq) * 100
        print(f"  quad throttle mean {np.nanmean(thq):.2f} max "
              f"{np.nanmax(thq):.2f} (0-1) | saturated(>0.9): {sat_q:.1f} % "
              f"of QTUN samples")
    if len(thc):
        print(f"  plane throttle mean {np.nanmean(thc):.0f} max "
              f"{np.nanmax(thc):.0f} % | at 100%: "
              f"{np.sum(thc >= 99) / len(thc) * 100:.1f} % of time")

    # ---- 7. LOAD FACTOR ------------------------------------------------------
    if fw.sum() > 5:
        n_max = 1.0 / max(math.cos(math.radians(np.nanmax(np.abs(R[fw])))), .3)
        print(f"\n[7] LOAD FACTOR — max n = {n_max:.2f} g (bank "
              f"{np.nanmax(np.abs(R[fw])):.1f} deg)")


if __name__ == '__main__':
    for arg in sys.argv[1:]:
        if ':' in arg[2:]:  # allow C:\ drive colon
            path, seg = arg.rsplit(':', 1)
            analyze(path, int(seg))
        else:
            analyze(arg)

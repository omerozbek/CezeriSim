#!/usr/bin/env python3
"""
latency_sysid.py — calibrate the simulated ArduPilot↔Unreal link latency
(Settings → Comms) so the sim's tracking error matches a REAL flight.

The idea (system identification): fly the SAME mission in the simulator at a
few AP-link latency settings, compare each sim log against the real flight,
and pick the latency that reproduces the real vehicle's track/attitude error
best. Beyond ~zero latency the sim tracks tighter than a real airframe; adding
transport delay softens control exactly like the real sensor/ESC pipeline, so
the closest match is a physically meaningful estimate of your real link.

USAGE — two modes:

1. Compare already-flown sim logs (no sim needed):
     python latency_sysid.py --real REAL.bin \\
         --candidate 0 SIM_0ms.bin  --candidate 15 SIM_15ms.bin \\
         --candidate 30 SIM_30ms.bin
   Runs compare_logs.py for each candidate and prints a ranked table; the
   winner is the latency whose sim-vs-real track separation + attitude RMS is
   smallest.

2. Print the end-to-end recipe to collect those logs:
     python latency_sysid.py --recipe

Each candidate is (LATENCY_MS, SIM_LOG). LATENCY_MS is just the label the sim
was flown at — this script does not change settings or fly anything itself.
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

RECIPE = """\
=== Latency system-ID recipe ===

For each candidate latency L (e.g. 0, 10, 20, 30 ms):

  1. Settings -> Comms tab: set BOTH "Sensor Latency" and "Command Latency"
     min AND max to L ms (min = max = L for a clean, jitter-free sweep).
     Press Apply.
  2. Compare menu -> pick the REAL flight log -> Start Mission Simulation.
     The sim flies the real mission and saves the log as
     <date-time>_<vehicle>.bin under Scripts/control/logs.
  3. Rename/note that log as SIM_<L>ms.bin.

Then rank them:

  python latency_sysid.py --real REAL.bin \\
      --candidate 0  logs/SIM_0ms.bin \\
      --candidate 10 logs/SIM_10ms.bin \\
      --candidate 20 logs/SIM_20ms.bin \\
      --candidate 30 logs/SIM_30ms.bin

The winner is your real link's effective latency — set it in the Comms tab and
leave it on for all future testing so the sim degrades like the real vehicle.
Tip: sweep coarse first (0/10/20/30), then fine around the best (e.g. 12/15/18).
"""


def run_compare(real, sim):
    """compare_logs.py --summary-json for one pair; returns the stats dict."""
    tmp = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
    tmp.close()
    cmd = [sys.executable, os.path.join(HERE, "compare_logs.py"),
           real, sim, "--summary-json", tmp.name, "--no-open"]
    print(f"  comparing {os.path.basename(sim)} ...")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"  [WARN] compare failed for {sim}:\n{r.stderr[-500:]}")
        return None
    try:
        with open(tmp.name, encoding="utf-8") as f:
            stats = json.load(f)
    except Exception as e:
        print(f"  [WARN] no summary for {sim}: {e}")
        stats = None
    finally:
        os.unlink(tmp.name)
    return stats


def score(stats):
    """Lower = closer to the real flight. Track separation (m) dominates;
    attitude RMS (roll+pitch+heading, deg) breaks ties."""
    if not stats:
        return float("inf"), {}
    sep = (stats.get("track_separation_m") or {}).get("mean")
    deltas = stats.get("deltas") or {}
    att = 0.0
    n = 0
    for k in ("roll", "pitch", "heading"):
        rms = (deltas.get(k) or {}).get("rms")
        if rms is not None:
            att += rms
            n += 1
    att = att / n if n else float("nan")
    sep_v = sep if sep is not None else 1e9
    att_v = att if att == att else 0.0     # NaN -> 0 contribution
    return sep_v + 0.1 * att_v, {"track_sep_m": sep, "att_rms_deg": att}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--real", help="real flight log (comparison baseline)")
    ap.add_argument("--candidate", nargs=2, action="append",
                    metavar=("LATENCY_MS", "SIM_LOG"), default=[],
                    help="a (latency-ms label, sim log) pair; repeatable")
    ap.add_argument("--recipe", action="store_true",
                    help="print the log-collection recipe and exit")
    a = ap.parse_args()

    if a.recipe or (not a.real and not a.candidate):
        print(RECIPE)
        return

    if not a.real or not os.path.isfile(a.real):
        raise SystemExit(f"real flight log not found: {a.real}")
    if not a.candidate:
        raise SystemExit("give at least one --candidate LATENCY_MS SIM_LOG")

    rows = []
    for lat_ms, sim in a.candidate:
        if not os.path.isfile(sim):
            print(f"  [SKIP] {sim} not found")
            continue
        s, extra = score(run_compare(os.path.abspath(a.real),
                                     os.path.abspath(sim)))
        rows.append((float(lat_ms), s, extra, sim))

    if not rows:
        raise SystemExit("no candidate produced a comparison")

    rows.sort(key=lambda r: r[1])
    print("\n=== Latency ranking (closest to the real flight first) ===")
    print(f"  {'latency':>8}  {'score':>8}  {'track sep':>10}  {'att rms':>8}")
    for lat, sc, extra, _sim in rows:
        sep = extra.get("track_sep_m")
        att = extra.get("att_rms_deg")
        print(f"  {lat:6.0f}ms  {sc:8.2f}  "
              f"{(f'{sep:.1f} m' if sep is not None else '  n/a'):>10}  "
              f"{(f'{att:.1f}°' if att == att else ' n/a'):>8}")
    best = rows[0][0]
    print(f"\n>>> Best match: {best:.0f} ms — set BOTH Comms link latencies to "
          f"this and keep it on for realistic testing. <<<")


if __name__ == "__main__":
    main()

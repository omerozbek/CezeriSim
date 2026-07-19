# Log Compare — Real Flight vs Simulation

Compares a **real flight log** (Mission Planner / ArduPilot) with a
**simulation log** (CezeriSim) and generates a single-file interactive HTML
report: track overlay (local East/North + OpenStreetMap), synced time series,
mode timelines, and sim-minus-real difference statistics.

All of this is also driveable from inside the sim: the init level's corner
**COMPARE** menu (top-right, below MODELS) runs these scripts — see
`docs/HOW_TO_RUN.md`.

## Quick start

```powershell
# GUI (or just double-click "Compare Logs.bat"):
python compare_logs.py

# CLI:
python compare_logs.py "C:\path\to\real_flight.bin" "..\control\logs\gokce_mission_20260702_160605.bin"

# One-shot mission replay + comparison (Docker SITL + UE level running or
# starting — the script waits for the vehicle, then flies the real log's
# mission and compares):
python run_mission_compare.py --real "C:\path\to\real_flight.bin"

# Just extract the mission out of a log (QGC WPL 110):
python mission_from_log.py "C:\path\to\real_flight.bin" -o mission.waypoints
```

The report is written next to the real log
(`compare_<real>__vs__<sim>.html`) and opened in the browser.

## Accepted formats (both sides, any mix)

| Format | Source |
|---|---|
| `.bin` / `.BIN` / `.log` | ArduPilot dataflash — Mission Planner "Download DataFlash Log", or the sim's `Scripts/control/logs/gokce_mission_*.bin` pulled by `fly_mission.py` |
| `.tlog` | Mission Planner / GCS telemetry log |
| `.csv` | `fly_mission.py` 4 Hz telemetry (`logs/mission_*.csv`) |

## How it aligns the two flights

1. **Segment** — a log may contain several armed spans (the sim mission `.bin`
   has a hover acceptance test + the mission). The **longest armed segment**
   is used by default; override with `--real-seg N` / `--sim-seg N`
   (see them with `--list-segments`).
2. **Time** — `t = 0` where the aircraft first climbs through 2 m
   (`--align takeoff`, default; threshold `--takeoff-alt`). Use
   `--align arm` for t=0 at arming, and `--shift-sim SEC` for manual trim.
3. **Position** — tracks AND waypoints are drawn in **one shared East/North
   frame** (origin = the real flight's home), so the sim track sits exactly
   where it truly is relative to the mission waypoints and the real track —
   any home offset stays visible (default `--frame shared`; fixed 2026-07-02,
   the first version rebased each track to its own home, which shifted the
   sim track relative to the waypoints). Use `--frame own-home` for a
   pattern-only overlay that removes the home offset (waypoints omitted
   there — they have no unambiguous position in mixed frames). The map panel
   shows true lat/lon. Altitude stays home-relative on both sides, which
   already cancels the ~9.5 m AMSL field-elevation difference
   (see `Scripts/control/logs/gokce_mission_20260702_COMPARISON_NOTES.md`).

## Options

```
-o OUT.html          output path
--real-seg N         armed segment of the real log (1-based)
--sim-seg N          armed segment of the sim log (1-based)
--align takeoff|arm  t=0 reference (default takeoff)
--takeoff-alt M      takeoff altitude threshold (default 2 m)
--shift-sim SEC      extra time shift on the sim trace (+ = later)
--labels REAL SIM    display names (default: Real, Sim)
--frame shared|own-home  track frame (default shared; see above)
--mission FILE       QGC WPL .waypoints file to overlay (default: the CMD
                     messages found in the logs)
--list-segments      print armed segments and exit
--no-map             skip the OSM map panel (fully offline report)
--no-open            don't open the browser
--summary-json FILE  also write the closeness stats (flight summaries,
                     track separation, sim−real deltas) as strict JSON —
                     the UE COMPARE menu reads this to show results in-engine
```

## Companion scripts

- **`mission_from_log.py LOG [-o OUT.waypoints]`** — extracts the mission
  out of a flight log into a QGC WPL 110 file (dataflash `CMD` messages, or
  MISSION_ITEM traffic in a `.tlog`; the last complete upload wins).
- **`run_mission_compare.py --real REAL.bin`** — replays the real flight's
  mission in the sim end to end: waits for the SITL heartbeat (Docker + UE
  may still be booting), extracts + uploads the mission, flies it in AUTO
  (`fly_mission.py` machinery), pulls the newest dataflash `.BIN` from the
  auto-detected `cezeri_sitl_*` container to `..\control\logs\
  mission_replay_<ts>.bin`, then runs `compare_logs.py` on the pair.
  Options: `--addr`, `--timeout`, `--connect-timeout`, `--gate-timeout`,
  `--summary-json`, `--no-compare`.
- **`tune_params.py --mission PLAN.waypoints`** (or `--real REAL.bin`) —
  parameter fine-tuning: flies the mission repeatedly (`--flights`, default
  30), scores every flight from its pulled `.BIN` (cross-track to the
  mission path, per-waypoint closest approach, roll-rate RMS + bank
  reversals, detrended-altitude RMS; hard-fail on emergency modes /
  incomplete missions) and searches `NAVL1_* / WP_RADIUS / ROLL_LIMIT_DEG /
  TECS_* / Q_WP_*` for the best score. `--profile balanced|accuracy|smooth`
  picks the accuracy-vs-smoothness weighting. Uses Optuna TPE when
  installed (`pip install optuna`), else a built-in random+refine fallback.
  Everything lands in `..\control\logs\tune_<ts>_<vehicle>\` (per-flight
  CSV + BIN, `best_params.parm`); `--progress-json` is rewritten after
  every flight for the UE Compare menu. The mission must land back near
  its takeoff point (the next flight starts where the last one landed) —
  long sessions that trip a SITL battery failsafe simply score those
  flights 0. Driven by the Compare menu's FINE-TUNE PARAMETERS button.

## What's in the report

- **Stat tiles** — duration, max altitude, mean cruise airspeed, mean track
  separation (nearest-neighbour distance between the EN tracks: geometry
  match independent of timing).
- **Track overlay** — East/North with mission waypoints (from the log's CMD
  messages) + map view. Look for corner-cutting at `WP_RADIUS`, the WP11
  climb leg, landing approach direction.
- **Time series** — altitude, airspeed, groundspeed, climb, roll, pitch,
  throttle; shared time axis, unified hover.
- **Mode timelines** — one lane per flight (transition timing, Q phases).
- **Sim − real table** — bias / RMS / max |Δ| per channel over the aligned
  overlap (sim interpolated onto the real timeline; heading wrapped ±180°).
- **Aligned data table** — 1 Hz numbers behind every plot (collapsible).

## Channel sources (dataflash)

alt from `POS.RelHomeAlt`; airspeed `CTUN.As` (synthetic when no pitot);
groundspeed `GPS.Spd`; climb `BARO.CRt` (fallback `-GPS.VZ`); attitude
`ATT`; throttle `CTUN.ThO` (%) overlaid with `QTUN.ThO`×100 while the quad
motors are active; modes `MODE`; armed segments `EV` 10/11; waypoints `CMD`.

## Dependencies

`pymavlink`, `pandas`, `numpy`, `plotly` (all pip-installable; plotly added
2026-07-02 for this tool).

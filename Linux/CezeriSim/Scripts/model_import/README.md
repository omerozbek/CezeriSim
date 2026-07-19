# model_import — vehicle model import toolchain (AVL)

Turns the team's filled `*_VTOL_MECHANICAL_DATA.txt` / `*_VTOL_ELECTRICAL_DATA.txt`
request files into a CezeriSim vehicle: geometry is parsed, an AVL
(vortex-lattice, MIT/Drela 3.40) model is built and run, and the computed
stability/control derivatives are written into `Vehicles/<name>/mechanical.json`
— replacing the textbook guesses that were previously hand-written.

```
filled .txt ──parse──► geometry/<name>.json ──avl──► runs/<name>/derivs.json
                          (review this!)                    │
                                            write ──► Vehicles/<name>/*.json
```

## Commands

```bash
cd Scripts/model_import

# 1. parse the filled request files into a reviewable spec (+ gaps report)
python model_import.py parse "<MECHANICAL.txt>" --electrical "<ELECTRICAL.txt>" --name pasifik

# 2. run AVL, print the derivative table (writes nothing)
python model_import.py avl geometry/pasifik.json

# 2b. validation mode: computed vs a vehicle's current mechanical.json
python model_import.py compare geometry/gokce.json --vehicle gokce_flight16

# 3. write/update the vehicle folder (clones --base for params.parm/_doc schema)
python model_import.py write geometry/pasifik.json --vehicle pasifik --base vtol_default
#    (the Pasifik model lives in Vehicles/pasifik since 2026-07-13 — it was
#     first written as pasifik_vtol while Vehicles/pasifik still held the old
#     drone placeholder; NOTE: write does not overwrite params.parm, and
#     pasifik/params.parm is now the REAL 1-jul controller dump — keep it)
```

## What AVL does and does not provide

| Provided (computed) | NOT provided (needs data) |
|---|---|
| cl0, cl_alpha, cm_alpha, cm_q | cd0 (inviscid → estimate, calibrate to real cruise, gokce method) |
| cy_beta, cn_beta, cl_p, cn_r | stall_aoa (linear → from team CLmax via (CLmax−cl0)/clα) |
| cm_aileron/elevator/rudder | mass, CG, inertia (measured/CAD; else flagged estimate) |
| oswald_e (span efficiency), neutral point / static margin | k_thrust family (thrust stand / real .param) |

Model limits: wing + H-tail + fin only (no fuselage → static margin comes out
slightly optimistic); derivatives in stability axes at the cruise-CL trim
point; control-derivative units verified per-run by an elevator-step case.

## Files

- `bin/avl.exe` — official MIT binary (web.mit.edu/drela/Public/web/avl, v3.40)
- `airfoils/*.dat` — Selig-format coordinates (psu94097.dat from UIUC database)
- `geometry/*.json` — parsed/hand-built specs. `gokce.json` is the validation
  spec: wing from known planform numbers, tail estimated (never measured) —
  see its `source` field before trusting any gokce output.
- `runs/<name>/` — model.avl, raw .st outputs, derivs.json, avl_stdout.log

Sign conventions imposed on output (plant convention, see mechanical.json _doc):
cm_aileron > 0, cm_elevator < 0, cm_rudder > 0.

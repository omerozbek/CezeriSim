"""
param_manager.py - CezeriSim vehicle parameter manager
=======================================================
Switch between vehicle configs, load params to a running SITL, or dump
current SITL params to file - without restarting the Docker container.

Usage:
    python param_manager.py list                   # list available vehicles
    python param_manager.py status                 # show active vehicle
    python param_manager.py switch sitl_ue         # switch active vehicle
    python param_manager.py switch gokce           # switch to real vehicle
    python param_manager.py load                   # reload active vehicle params into SITL
    python param_manager.py load --vehicle gokce   # load specific vehicle without switching
    python param_manager.py dump                   # dump all SITL params to file
    python param_manager.py dump --out my.parm     # dump to specific file

Note: 'switch' only updates active_vehicle.txt + copies params.parm.
      The Docker container must be restarted for a backend change
      (ap_native <-> ue_physics).  Within the same backend, 'load' applies
      param changes without restart (live MAVLink param upload).
"""

import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("[ERROR] pymavlink not installed. Run:  pip install pymavlink")

SCRIPTS_DIR   = Path(__file__).parent.parent          # Scripts/
PROJECT_DIR   = SCRIPTS_DIR.parent                    # project root
VEHICLES_DIR  = PROJECT_DIR / "Vehicles"
ACTIVE_TXT    = VEHICLES_DIR / "active_vehicle.txt"
VEHICLE_PARM  = SCRIPTS_DIR / "vehicle.parm"

# ---------------------------------------------------------------------------
# Vehicle helpers
# ---------------------------------------------------------------------------

def read_active_vehicle() -> str:
    if ACTIVE_TXT.exists():
        return ACTIVE_TXT.read_text(encoding="utf-8").strip()
    return "(none)"


def set_active_vehicle(name: str) -> None:
    ACTIVE_TXT.write_text(name + "\n", encoding="utf-8")


def read_backend(vehicle_dir: Path) -> str:
    mech = vehicle_dir / "mechanical.json"
    if not mech.exists():
        return "ue_physics"
    try:
        data = json.loads(mech.read_text(encoding="utf-8"))
        return data.get("backend", "ue_physics").strip()
    except Exception:
        return "ue_physics"


def list_vehicles() -> None:
    active = read_active_vehicle()
    print(f"\nAvailable vehicles in {VEHICLES_DIR}:\n")
    for d in sorted(VEHICLES_DIR.iterdir()):
        if not d.is_dir() or d.name.startswith("_"):
            continue
        ok = all((d / f).exists() for f in ("mechanical.json", "electrical.json", "params.parm"))
        backend = read_backend(d) if (d / "mechanical.json").exists() else "?"
        marker  = " <-- active" if d.name == active else ""
        print(f"  {d.name:<22} [{'OK' if ok else 'INCOMPLETE'}]  backend={backend}{marker}")
    print()


def switch_vehicle(name: str) -> int:
    vdir = VEHICLES_DIR / name
    if not vdir.is_dir():
        print(f"[ERROR] Vehicle '{name}' not found in {VEHICLES_DIR}")
        list_vehicles()
        return 1

    parm_src = vdir / "params.parm"
    if not parm_src.exists():
        print(f"[ERROR] Missing {parm_src}")
        return 1

    old = read_active_vehicle()
    old_backend = read_backend(VEHICLES_DIR / old) if (VEHICLES_DIR / old).is_dir() else "?"
    new_backend = read_backend(vdir)

    set_active_vehicle(name)
    shutil.copy2(parm_src, VEHICLE_PARM)

    print(f"[OK] Active vehicle: {old} -> {name}")
    print(f"     Backend: {old_backend} -> {new_backend}")
    print(f"     Params copied to {VEHICLE_PARM}")

    if old_backend != new_backend:
        print()
        print(f"[!] Backend changed ({old_backend} -> {new_backend}).")
        print(f"    You must RESTART the Docker container for the change to take effect:")
        print(f"      python Scripts/start_sitl_docker.py --stop")
        print(f"      python Scripts/start_sitl_docker.py")
    else:
        print()
        print(f"[i] Same backend ({new_backend}) - you can reload params without restarting:")
        print(f"      python Scripts/control/param_manager.py load")
    return 0


# ---------------------------------------------------------------------------
# MAVLink param upload / dump
# ---------------------------------------------------------------------------

def parse_parm_file(path: Path) -> dict[str, float]:
    """Parse an ArduPilot .parm file -> {name: value}."""
    params: dict[str, float] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) >= 2:
            try:
                params[parts[0].upper()] = float(parts[1])
            except ValueError:
                pass
    return params


def connect_mav(addr: str, timeout: float = 30.0):
    print(f"Connecting to {addr}...")
    mav = mavutil.mavlink_connection(addr, source_system=255)
    hb  = mav.wait_heartbeat(timeout=timeout)
    if hb is None:
        print("[ERROR] No heartbeat. Is ArduPilot running and connected?")
        return None
    print(f"Connected. sysid={mav.target_system}")
    return mav


def load_params(vehicle_name: str | None, addr: str) -> int:
    """Upload params from a vehicle's params.parm to SITL via MAVLink."""
    if vehicle_name:
        vdir = VEHICLES_DIR / vehicle_name
    else:
        vdir = VEHICLES_DIR / read_active_vehicle()

    parm_file = vdir / "params.parm"
    if not parm_file.exists():
        print(f"[ERROR] {parm_file} not found.")
        return 1

    params = parse_parm_file(parm_file)
    print(f"Loaded {len(params)} params from {parm_file.name}")

    mav = connect_mav(addr)
    if mav is None:
        return 1

    # Skip read-back confirmation for speed - use PARAM_SET only
    ok_count   = 0
    fail_count = 0

    for pname, pval in params.items():
        pname_bytes = pname.encode("ascii")[:16]
        mav.mav.param_set_send(
            mav.target_system, mav.target_component,
            pname_bytes, pval,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        # Brief drain to prevent flooding
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
        if msg and msg.param_id.rstrip("\x00") == pname:
            ok_count += 1
        else:
            fail_count += 1

        time.sleep(0.005)

    print(f"[OK] Uploaded {ok_count}/{len(params)} params "
          f"({fail_count} without ack - likely still applied).")
    print("[i] ArduPilot applies params immediately. "
          "If SIM_RATE_HZ changed, a container restart is required.")
    return 0


def dump_params(out_path: Path, addr: str) -> int:
    """Download all params from SITL and save to file."""
    mav = connect_mav(addr)
    if mav is None:
        return 1

    print("Requesting all parameters (PARAM_REQUEST_LIST)...")
    mav.mav.param_request_list_send(mav.target_system, mav.target_component)

    params: dict[str, float] = {}
    t_start   = time.time()
    last_recv = time.time()

    while True:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            if time.time() - last_recv > 3.0:
                break
            continue
        name = msg.param_id.rstrip("\x00")
        params[name] = msg.param_value
        last_recv = time.time()
        if msg.param_index + 1 >= msg.param_count:
            break

    if not params:
        print("[ERROR] No params received.")
        return 1

    lines = [
        f"# Dumped from SITL via param_manager.py",
        f"# sysid={mav.target_system}  count={len(params)}",
        "",
    ]
    for k, v in sorted(params.items()):
        lines.append(f"{k:<30} {v!r}")

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[OK] {len(params)} params saved to {out_path}")
    return 0


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description="CezeriSim vehicle parameter manager")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list",   help="List available vehicles")
    sub.add_parser("status", help="Show active vehicle")

    sw = sub.add_parser("switch", help="Switch active vehicle")
    sw.add_argument("name", help="Vehicle folder name (e.g. sitl_ue, gokce)")

    ld = sub.add_parser("load", help="Upload active vehicle params to running SITL via MAVLink")
    ld.add_argument("--vehicle", default=None, help="Override vehicle (default: active)")
    ld.add_argument("--addr",    default="tcp:localhost:5760")

    dp = sub.add_parser("dump", help="Download all SITL params to file")
    dp.add_argument("--out",  default=None, help="Output file (default: logs/dump_<ts>.parm)")
    dp.add_argument("--addr", default="tcp:localhost:5760")

    args = ap.parse_args()

    if args.cmd == "list":
        list_vehicles()
        return 0

    elif args.cmd == "status":
        active  = read_active_vehicle()
        vdir    = VEHICLES_DIR / active
        backend = read_backend(vdir) if vdir.is_dir() else "?"
        print(f"Active vehicle : {active}")
        print(f"Backend        : {backend}")
        print(f"Params file    : {vdir / 'params.parm'}")
        return 0

    elif args.cmd == "switch":
        return switch_vehicle(args.name)

    elif args.cmd == "load":
        return load_params(args.vehicle, args.addr)

    elif args.cmd == "dump":
        out = Path(args.out) if args.out else \
              Path(__file__).parent / "logs" / \
              f"dump_{__import__('datetime').datetime.now().strftime('%Y%m%d_%H%M%S')}.parm"
        out.parent.mkdir(exist_ok=True)
        return dump_params(out, args.addr)

    return 0


if __name__ == "__main__":
    sys.exit(main())

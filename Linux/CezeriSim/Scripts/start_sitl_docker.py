#!/usr/bin/env python3
"""
start_sitl_docker.py  —  Run from Windows (PowerShell or CMD)
Starts CezeriSim ArduPilot SITL container(s) via Docker Compose.

Single-vehicle (legacy) mode — no --drones/--vtols flags:
  Reads the active vehicle from <project>/Vehicles/active_vehicle.txt (or
  --vehicle flag), copies its params.parm to Scripts/vehicle.parm, then
  launches docker compose (docker-compose.yml, one ArduPlane container).

Fleet mode — --drones N and/or --vtols M:
  Starts one SITL container per UAV: ArduCopter for drones, ArduPlane for
  VTOLs. Generates Scripts/generated/uav<i>.parm + docker-compose.fleet.yml
  and one servo relay per instance. Ports and homes follow the SAME
  conventions as UCezeriFleetSubsystem in UE (docs/INVARIANTS.md):

     global index g: drones 0..N-1, VTOLs N..N+M-1
     UE JSON (SimPort)         9002 + 10*g
     AP JSON in (host->cont)   9003 + 10*g  -> container 9003
     AP servo out -> relay      9006 + 10*g  -> forwards to 9002 + 10*g
     MAVLink SERIAL0 (control)  tcp:127.0.0.1:(5760 + 10*g)
     MAVLink SERIAL1 (spare)    tcp:127.0.0.1:(5762 + 10*g)
     MAV_SYSID                  g + 1
     GPS home                   base SIM_OPOS + g*5 m east

Requirements:
  - Docker Desktop for Windows installed and running
  - Run `docker compose build` once in Scripts/ before first use
    (rebuild once after updating to fleet support: adds the arducopter binary)

Usage:
    python start_sitl_docker.py                      # legacy: single active vehicle
    python start_sitl_docker.py --drones 3           # 3 ArduCopter drones
    python start_sitl_docker.py --drones 2 --vtols 1 # mixed fleet
    python start_sitl_docker.py --vehicle pasifik    # override active vehicle (legacy)
    python start_sitl_docker.py --build              # rebuild image then start
    python start_sitl_docker.py --stop               # stop containers + relays
    python start_sitl_docker.py --logs               # tail container logs
    python start_sitl_docker.py --list               # list available vehicle configs
    python start_sitl_docker.py --drones 2 --dry-run # generate files, don't start
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

SCRIPTS_DIR   = Path(__file__).parent
PROJECT_DIR   = SCRIPTS_DIR.parent
VEHICLES_DIR  = PROJECT_DIR / "Vehicles"
ACTIVE_TXT    = VEHICLES_DIR / "active_vehicle.txt"
ACTIVE_DRONE_TXT = VEHICLES_DIR / "active_drone_vehicle.txt"
VEHICLE_PARM  = SCRIPTS_DIR / "vehicle.parm"   # mounted into container (legacy mode)
COMPOSE_FILE  = SCRIPTS_DIR / "docker-compose.yml"
GENERATED_DIR = SCRIPTS_DIR / "generated"
FLEET_COMPOSE = GENERATED_DIR / "docker-compose.fleet.yml"
FLEET_PROJECT = "cezeri-fleet"

# ---- Fleet port/home conventions — MUST match UCezeriFleetSubsystem ---------
JSON_PORT_BASE   = 9002   # UE JSONBridge bind (SimPort)
AP_JSON_IN_BASE  = 9003   # host port mapped to container:9003 (AP JSON in)
SERVO_OUT_BASE   = 9006   # AP servo out -> servo_relay listen
MAVLINK_TCP_BASE = 5760   # SERIAL0 (control scripts)
MAVLINK_TCP2_BASE= 5762   # SERIAL1 (spare/visualizer)
PORT_STRIDE      = 10
EAST_SPACING_M   = 5.0
OFFSET_M_PER_DEG = 111319.4908   # spherical constant for the east offset ONLY


def run(args: list[str], env: dict | None = None, **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(args, cwd=SCRIPTS_DIR, env=env, **kwargs)


def read_backend(vehicle_dir: Path) -> str:
    """Read the `backend` field from a vehicle's mechanical.json.
    Returns 'ap_native' or 'ue_physics'. Defaults to 'ue_physics' for legacy vehicles."""
    mech = vehicle_dir / "mechanical.json"
    if not mech.exists():
        return "ue_physics"
    try:
        data = json.loads(mech.read_text(encoding="utf-8"))
        backend = data.get("backend", "ue_physics").strip()
        if backend not in ("ap_native", "ue_physics"):
            print(f"[WARN] Unknown backend '{backend}' in {mech}; defaulting to ue_physics")
            return "ue_physics"
        return backend
    except json.JSONDecodeError as e:
        print(f"[WARN] Could not parse {mech}: {e}; defaulting to ue_physics")
        return "ue_physics"


def resolve_host_ipv4() -> str:
    """Resolve the Docker host-gateway IPv4 the container must send servos to.

    `host.docker.internal` resolves IPv6-FIRST inside the container but only the
    IPv4 path routes to the Windows host where servo_relay.py listens. Passing
    the bare name lets AP pick IPv6 and its servos vanish ("AP silent / no
    servos" flaky link). Probe the real IPv4 with a throwaway container and pass
    it explicitly to --sim-address. See BUGFIXES 2026-06-07.
    """
    default = "192.168.65.254"   # Docker Desktop host-gateway fallback
    try:
        r = subprocess.run(
            ["docker", "run", "--rm", "--entrypoint", "getent",
             "--add-host", "host.docker.internal:host-gateway",
             "cezeri-sitl:latest", "ahosts", "host.docker.internal"],
            capture_output=True, text=True, timeout=30,
        )
        for line in r.stdout.splitlines():
            parts = line.split()
            if parts and parts[0].count(".") == 3 and all(
                    p.isdigit() for p in parts[0].split(".")):
                print(f"[info] Resolved host.docker.internal IPv4 = {parts[0]}")
                return parts[0]
    except Exception as e:
        print(f"[WARN] IPv4 resolve failed ({e}); using fallback {default}")
    print(f"[info] Using fallback host IPv4 {default}")
    return default


def read_vehicle_home(vehicle_dir: Path) -> str:
    """Derive the SITL --home string from the vehicle's SIM_OPOS_* params.

    Per INVARIANTS.md the --home lat/lon/alt must match the vehicle's SIM_OPOS
    and the UE actor's DefaultHomeLat/Lon/Alt. The params.parm is the single
    source of truth; this makes home per-vehicle (gokce flies at its real
    mission site, sitl_ue keeps the legacy Istanbul home)."""
    default = "41.0082000,28.9784000,0.00,0"
    parm = vehicle_dir / "params.parm"
    vals = {"SIM_OPOS_LAT": None, "SIM_OPOS_LNG": None,
            "SIM_OPOS_ALT": "0.0", "SIM_OPOS_HDG": "0"}
    try:
        for line in parm.read_text(encoding="utf-8").splitlines():
            parts = line.split("#", 1)[0].split()
            if len(parts) >= 2 and parts[0] in vals:
                vals[parts[0]] = parts[1]
    except OSError:
        return default
    if vals["SIM_OPOS_LAT"] is None or vals["SIM_OPOS_LNG"] is None:
        return default
    home = (f"{float(vals['SIM_OPOS_LAT']):.7f},{float(vals['SIM_OPOS_LNG']):.7f},"
            f"{float(vals['SIM_OPOS_ALT']):.2f},{float(vals['SIM_OPOS_HDG']):.0f}")
    return home


def read_opos(vehicle_dir: Path) -> dict | None:
    """Read SIM_OPOS_* from a vehicle's params.parm as the fleet base home."""
    parm = vehicle_dir / "params.parm"
    vals = {"SIM_OPOS_LAT": None, "SIM_OPOS_LNG": None,
            "SIM_OPOS_ALT": "0.0", "SIM_OPOS_HDG": "0"}
    try:
        for line in parm.read_text(encoding="utf-8").splitlines():
            parts = line.split("#", 1)[0].split()
            if len(parts) >= 2 and parts[0] in vals:
                vals[parts[0]] = parts[1]
    except OSError:
        return None
    if vals["SIM_OPOS_LAT"] is None or vals["SIM_OPOS_LNG"] is None:
        return None
    return {"lat": float(vals["SIM_OPOS_LAT"]), "lon": float(vals["SIM_OPOS_LNG"]),
            "alt": float(vals["SIM_OPOS_ALT"]), "hdg": float(vals["SIM_OPOS_HDG"])}


def instance_home(base: dict, g: int) -> dict:
    """home_g = base + g x EAST_SPACING_M east.
    IDENTICAL formula to UCezeriFleetSubsystem::ComputeInstanceHome — do not
    change one without the other (docs/INVARIANTS.md)."""
    cos_lat = max(math.cos(math.radians(base["lat"])), 1e-5)
    return {"lat": base["lat"],
            "lon": base["lon"] + (EAST_SPACING_M * g) / (OFFSET_M_PER_DEG * cos_lat),
            "alt": base["alt"], "hdg": base["hdg"]}


def home_str(home: dict) -> str:
    return f"{home['lat']:.7f},{home['lon']:.7f},{home['alt']:.2f},{home['hdg']:.0f}"


def resolve_type_vehicle(kind: str, override: str | None) -> Path:
    """Resolve the vehicle config folder for a fleet vehicle type."""
    if override:
        name = override.strip()
    elif kind == "drone":
        name = (ACTIVE_DRONE_TXT.read_text(encoding="utf-8").strip()
                if ACTIVE_DRONE_TXT.exists() else "sitl_copter")
    else:
        name = read_active_vehicle_name() or "sitl_ue"

    vehicle_dir = VEHICLES_DIR / name
    if not vehicle_dir.is_dir() or not (vehicle_dir / "params.parm").exists():
        print(f"[ERROR] {kind} vehicle config not found/incomplete: {vehicle_dir}")
        print("        Run `python start_sitl_docker.py --list` to see vehicles.")
        sys.exit(1)

    backend = read_backend(vehicle_dir)
    if backend != "ue_physics":
        print(f"[ERROR] Fleet mode requires backend=ue_physics, but {name} is {backend}.")
        print("        Use the legacy single-vehicle mode for ap_native.")
        sys.exit(1)
    return vehicle_dir


def write_instance_parm(vehicle_dir: Path, g: int, home: dict) -> Path:
    """Copy the vehicle's params.parm to generated/uav<g>.parm with SIM_OPOS_*
    rewritten to the instance home and the per-instance MAV_SYSID appended —
    the parm stays the single source of truth inside each container."""
    GENERATED_DIR.mkdir(exist_ok=True)
    out_lines = []
    replacements = {"SIM_OPOS_LAT": f"{home['lat']:.7f}",
                    "SIM_OPOS_LNG": f"{home['lon']:.7f}",
                    "SIM_OPOS_ALT": f"{home['alt']:.2f}"}
    for line in (vehicle_dir / "params.parm").read_text(encoding="utf-8").splitlines():
        parts = line.split("#", 1)[0].split()
        if len(parts) >= 2 and parts[0] in replacements:
            out_lines.append(f"{parts[0]:<15} {replacements[parts[0]]}")
        else:
            out_lines.append(line)
    sysid = g + 1
    out_lines += [
        "",
        f"# Fleet instance {g} — appended by start_sitl_docker.py",
        f"MAV_SYSID       {sysid}",
        f"SYSID_THISMAV   {sysid}   # pre-4.7 name; unknown-param warning is harmless",
    ]
    parm_path = GENERATED_DIR / f"uav{g}.parm"
    parm_path.write_text("\n".join(out_lines) + "\n", encoding="utf-8")
    return parm_path


def parse_home_override(home_arg: str | None) -> dict | None:
    """Parse --home 'lat,lon[,alt[,hdg]]' into a base-home dict (hdg optional —
    None means 'keep the vehicle's SIM_OPOS_HDG')."""
    if not home_arg:
        return None
    parts = [p.strip() for p in home_arg.split(",")]
    if len(parts) < 2:
        print(f"[ERROR] --home needs 'lat,lon[,alt[,hdg]]', got: {home_arg}")
        sys.exit(1)
    try:
        return {"lat": float(parts[0]), "lon": float(parts[1]),
                "alt": float(parts[2]) if len(parts) > 2 else 0.0,
                "hdg": float(parts[3]) if len(parts) > 3 else None}
    except ValueError:
        print(f"[ERROR] --home values must be numbers: {home_arg}")
        sys.exit(1)


def build_fleet_instances(drones: int, vtols: int,
                          drone_vehicle: str | None,
                          vtol_vehicle: str | None,
                          home_override: dict | None = None) -> list[dict]:
    """Instances in global-index order: drones 0..N-1, then VTOLs N..N+M-1."""
    drone_dir = resolve_type_vehicle("drone", drone_vehicle) if drones > 0 else None
    vtol_dir  = resolve_type_vehicle("vtol",  vtol_vehicle)  if vtols  > 0 else None

    # Base home priority mirrors UCezeriFleetSubsystem::EnsureBaseHome:
    # the VTOL vehicle's SIM_OPOS when any VTOL flies, else the drone's.
    base = (read_opos(vtol_dir) if vtol_dir else None) \
        or (read_opos(drone_dir) if drone_dir else None)
    if base is None:
        print("[WARN] No SIM_OPOS_* in vehicle params — using legacy default home.")
        base = {"lat": 41.0082000, "lon": 28.9784000, "alt": 0.0, "hdg": 0.0}

    # --home (the UE world-origin override / active custom world): replaces
    # the base LAT/LON/ALT; heading stays the vehicle's unless given. UE's
    # EnsureBaseHome applies the SAME override so both sides agree.
    if home_override is not None:
        base["lat"] = home_override["lat"]
        base["lon"] = home_override["lon"]
        base["alt"] = home_override["alt"]
        if home_override["hdg"] is not None:
            base["hdg"] = home_override["hdg"]
        print(f"[info] Base home OVERRIDDEN (--home): {home_str(base)}")

    instances = []
    for g in range(drones + vtols):
        is_vtol = g >= drones
        vdir    = vtol_dir if is_vtol else drone_dir
        home    = instance_home(base, g)
        instances.append({
            "g": g,
            "kind": "vtol" if is_vtol else "drone",
            "type_index": (g - drones) if is_vtol else g,
            "vehicle": vdir.name,
            "binary": "arduplane" if is_vtol else "arducopter",
            "home": home,
            "parm": write_instance_parm(vdir, g, home),
            "json_port":   JSON_PORT_BASE    + PORT_STRIDE * g,
            "ap_in_port":  AP_JSON_IN_BASE   + PORT_STRIDE * g,
            "servo_port":  SERVO_OUT_BASE    + PORT_STRIDE * g,
            "mav_port":    MAVLINK_TCP_BASE  + PORT_STRIDE * g,
            "mav2_port":   MAVLINK_TCP2_BASE + PORT_STRIDE * g,
            "sysid": g + 1,
        })
    return instances


def generate_fleet_compose(instances: list[dict], host_ipv4: str) -> None:
    """Write generated/docker-compose.fleet.yml — one service per UAV.
    Container-internal ports stay at the defaults (9003 JSON-in, 5760/5762
    MAVLink); only the HOST side is offset by 10 per instance. The servo-out
    port is dialled outward to the host, so it uses the real host port."""
    lines = [
        "# AUTO-GENERATED by start_sitl_docker.py --drones/--vtols — do not edit.",
        "# Stop with: python start_sitl_docker.py --stop",
        "services:",
    ]
    for ins in instances:
        lines += [
            f"  uav{ins['g']}:",
            f"    image: cezeri-sitl:latest",
            f"    container_name: cezeri_sitl_uav{ins['g']}_{ins['kind']}{ins['type_index']}",
            f"    environment:",
            f"      AP_BINARY: {ins['binary']}",
            f"    command: >",
            f"      --model JSON",
            f"      --defaults /home/ardupilot/uav.parm",
            f"      --sim-address={host_ipv4}",
            f"      --sim-port-in=9003",
            f"      --sim-port-out={ins['servo_port']}",
            f"      --home={home_str(ins['home'])}",
            f"      --serial0=tcp:5760",
            f"      --serial1=tcp:5762",
            f"      -I0",
            f"    volumes:",
            f"      - ./{ins['parm'].name}:/home/ardupilot/uav.parm:ro",
            f"    ports:",
            f"      - \"{ins['ap_in_port']}:9003/udp\"",
            f"      - \"{ins['mav_port']}:5760/tcp\"",
            f"      - \"{ins['mav2_port']}:5762/tcp\"",
            f"    extra_hosts:",
            f"      - \"host.docker.internal:host-gateway\"",
            f"    restart: unless-stopped",
            "",
        ]
    FLEET_COMPOSE.write_text("\n".join(lines), encoding="utf-8")


def check_image_has_copter() -> None:
    """Fleet drones need the arducopter binary — old images only built plane.

    The actual check spins up a container, which costs several seconds on
    every fleet start — cache the verdict per image ID (a fast metadata call)
    so only a rebuilt image pays for the container run again."""
    image_id = subprocess.run(
        ["docker", "image", "inspect", "-f", "{{.Id}}", "cezeri-sitl:latest"],
        capture_output=True, text=True).stdout.strip()
    cache = GENERATED_DIR / ".arducopter_image_check"
    if image_id and cache.exists() and cache.read_text().strip() == image_id:
        return
    r = subprocess.run(
        ["docker", "run", "--rm", "--entrypoint", "test", "cezeri-sitl:latest",
         "-x", "/home/ardupilot/ardupilot/build/sitl/bin/arducopter"],
        capture_output=True)
    if r.returncode != 0:
        print("[ERROR] The cezeri-sitl image has no arducopter binary.")
        print("        Rebuild once:  python start_sitl_docker.py --build")
        sys.exit(1)
    if image_id:
        GENERATED_DIR.mkdir(exist_ok=True)
        cache.write_text(image_id)


def kill_stale_relays() -> None:
    """Kill any servo_relay.py left over from a previous run. Stale relays keep
    their UDP ports bound (SO_REUSEADDR makes the double-bind silent) and eat
    servo packets — the classic 'works sometimes' link."""
    try:
        if sys.platform == "win32":
            ps = ("Get-CimInstance Win32_Process -Filter \"Name like 'python%'\" | "
                  "Where-Object { $_.CommandLine -match 'servo_relay\\.py' } | "
                  "ForEach-Object { Stop-Process -Id $_.ProcessId -Force; $_.ProcessId }")
            r = subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                               capture_output=True, text=True, timeout=30)
            killed = [p for p in r.stdout.split() if p.strip().isdigit()]
            if killed:
                print(f"[info] Killed stale servo relay pid(s): {', '.join(killed)}")
        else:
            r = subprocess.run(["pkill", "-f", "servo_relay.py"],
                               capture_output=True, text=True, timeout=30)
            if r.returncode == 0:
                print("[info] Killed stale servo relay(s).")
    except Exception as e:
        print(f"[WARN] Stale-relay cleanup failed ({e}) — continuing.")


def start_servo_relay_for(listen: int, dest: int) -> subprocess.Popen | None:
    """Launch one servo_relay.py instance (detached, silent — see the legacy
    start_servo_relay for why stdout/stderr MUST be DEVNULL)."""
    relay = SCRIPTS_DIR / "control" / "servo_relay.py"
    if not relay.exists():
        print(f"[WARN] servo_relay.py not found at {relay}; servo packets won't reach UE.")
        return None
    try:
        flags = 0
        if sys.platform == "win32":
            flags = subprocess.DETACHED_PROCESS | subprocess.CREATE_NEW_PROCESS_GROUP
        proc = subprocess.Popen(
            [sys.executable, str(relay), "--listen", str(listen), "--dest", str(dest)],
            cwd=SCRIPTS_DIR,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            creationflags=flags)
        print(f"  Servo relay       : pid {proc.pid}  port {listen} -> 127.0.0.1:{dest}")
        return proc
    except OSError as e:
        print(f"[WARN] Could not start servo_relay.py ({listen}->{dest}): {e}")
        return None


def print_fleet_summary(instances: list[dict]) -> None:
    print()
    print("Fleet connection strings (also shown on-screen in UE):")
    for ins in instances:
        cam_note = ""
        print(f"  {ins['kind'].upper():<5} {ins['type_index'] + 1}  "
              f"[{ins['vehicle']}]  "
              f"MAVLink tcp:127.0.0.1:{ins['mav_port']}  SYSID {ins['sysid']}  "
              f"JSON udp {ins['json_port']}{cam_note}")
    print()
    print("Camera streams (when enabled in the UE init menu):")
    print(f"  UAV g camera k -> udp 127.0.0.1:{5001}+10*g+k "
          f"(e.g. first drone cam 0 = 5001)")


def cmd_fleet_start(drones: int, vtols: int, drone_vehicle: str | None,
                    vtol_vehicle: str | None, rebuild: bool, dry_run: bool,
                    home_override: dict | None = None) -> None:
    if not dry_run:
        check_docker()
    if rebuild:
        cmd_build()

    instances = build_fleet_instances(drones, vtols, drone_vehicle, vtol_vehicle,
                                      home_override)
    host_ipv4 = resolve_host_ipv4() if not dry_run else "192.168.65.254"
    generate_fleet_compose(instances, host_ipv4)

    print()
    print(f"Fleet: {drones} drone(s) [ArduCopter] + {vtols} VTOL(s) [ArduPlane]")
    print(f"  Compose file      : {FLEET_COMPOSE}")
    for ins in instances:
        print(f"  uav{ins['g']} ({ins['kind']} {ins['type_index'] + 1}, {ins['vehicle']}): "
              f"home {home_str(ins['home'])}")

    if dry_run:
        print_fleet_summary(instances)
        print("\n[dry-run] Files generated; containers NOT started.")
        return

    if drones > 0:
        check_image_has_copter()

    kill_stale_relays()
    relays: list[subprocess.Popen | None] = []
    for ins in instances:
        relays.append(start_servo_relay_for(ins["servo_port"], ins["json_port"]))

    print_fleet_summary(instances)
    print("Press Ctrl+C to stop.\n")

    compose = ["docker", "compose", "-p", FLEET_PROJECT, "-f", str(FLEET_COMPOSE)]
    up_started = time.monotonic()
    try:
        run(compose + ["up"], check=True)
    except KeyboardInterrupt:
        print("\nStopping fleet...")
        run(compose + ["down", "--remove-orphans", "-t", "2"])
    except subprocess.CalledProcessError as e:
        # `compose up` returns non-zero when the fleet is taken down externally
        # (--stop from the UE menus); that must exit 0 or the `|| pause` guard
        # in the console launcher keeps this window open forever. A failure
        # within seconds of starting is a real startup error — keep it loud.
        if time.monotonic() - up_started < 15:
            raise
        print(f"\nFleet stopped externally (compose up exited with {e.returncode}).")
    finally:
        for proc in relays:
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()


def docker_env_for_backend(backend: str, vehicle_dir: Path,
                           home_override: dict | None = None) -> dict:
    """Build the env dict passed to `docker compose up` based on backend."""
    env = os.environ.copy()
    if home_override is not None:
        base = read_opos(vehicle_dir) or {"lat": 0.0, "lon": 0.0,
                                          "alt": 0.0, "hdg": 0.0}
        base["lat"], base["lon"] = home_override["lat"], home_override["lon"]
        base["alt"] = home_override["alt"]
        if home_override["hdg"] is not None:
            base["hdg"] = home_override["hdg"]
        env["AP_HOME"] = home_str(base)
        print(f"[info] Home OVERRIDDEN (--home): {env['AP_HOME']}")
    else:
        env["AP_HOME"] = read_vehicle_home(vehicle_dir)
    if backend == "ap_native":
        env["AP_MODEL"] = "quadplane"
        env["AP_EXTRA_ARGS"] = ""   # no --sim-address: AP uses internal physics, no external server
    else:  # ue_physics
        env["AP_MODEL"] = "JSON"
        # --sim-port-in=9003: AP listens for JSON from UE on container:9003 (Docker maps host:9003)
        # --sim-port-out=9006: AP sends servo to host:9006 (servo_relay.py receives and forwards
        #   to 127.0.0.1:9002 where UE listens).
        # --sim-address: resolved IPv4 (NOT the bare name) so AP never picks the
        #   unrouted IPv6 host.docker.internal address. See resolve_host_ipv4().
        host_ipv4 = resolve_host_ipv4()
        env["AP_EXTRA_ARGS"] = f"--sim-address={host_ipv4} --sim-port-in=9003 --sim-port-out=9006"
    return env


def check_docker() -> None:
    # `docker version` is a lightweight daemon ping — `docker info` collects
    # the full system report and is noticeably slower on a busy machine.
    # NOTE: if Docker Desktop's Resource Saver has paused the Linux VM (it is
    # ON by default and kicks in when no container ran for a while), the first
    # docker command blocks here until the VM resumes — that wake-up, not this
    # script, is what takes tens of seconds. Warn when it happens.
    started = time.monotonic()
    result = run(["docker", "version", "--format", "{{.Server.Version}}"],
                 capture_output=True)
    if result.returncode != 0:
        print("[ERROR] Docker is not running (Docker Desktop / Colima / Docker Engine).")
        print("        Start your Docker runtime and try again.")
        sys.exit(1)
    waited = time.monotonic() - started
    if waited > 5.0:
        print(f"[info] Docker engine took {waited:.0f}s to answer — Docker "
              "Desktop's Resource Saver likely paused the VM while idle. "
              "Disable it under Docker Desktop Settings > Resources > "
              "Resource Saver to make start/stop immediate again.")


def list_vehicles() -> None:
    if not VEHICLES_DIR.exists():
        print(f"[ERROR] Vehicles directory not found: {VEHICLES_DIR}")
        sys.exit(1)
    active = read_active_vehicle_name()
    print(f"Available vehicle configs in {VEHICLES_DIR}:\n")
    for d in sorted(VEHICLES_DIR.iterdir()):
        if d.is_dir() and not d.name.startswith("_"):
            has_mech = (d / "mechanical.json").exists()
            has_elec = (d / "electrical.json").exists()
            has_parm = (d / "params.parm").exists()
            status = "OK" if (has_mech and has_elec and has_parm) else "INCOMPLETE"
            backend = read_backend(d) if has_mech else "?"
            marker = " <-- active" if d.name == active else ""
            print(f"  {d.name:<20} [{status}]  backend={backend}{marker}")
    print()


def read_active_vehicle_name() -> str:
    if ACTIVE_TXT.exists():
        return ACTIVE_TXT.read_text(encoding="utf-8").strip()
    return ""


def resolve_vehicle(override: str | None) -> Path:
    name = override.strip() if override else read_active_vehicle_name()
    if not name:
        print("[ERROR] No active vehicle configured.")
        print(f"        Create {ACTIVE_TXT} with the vehicle folder name,")
        print("        or pass --vehicle <name>.")
        sys.exit(1)

    vehicle_dir = VEHICLES_DIR / name
    if not vehicle_dir.is_dir():
        print(f"[ERROR] Vehicle folder not found: {vehicle_dir}")
        print(f"        Run `python start_sitl_docker.py --list` to see available vehicles.")
        sys.exit(1)

    parm_src = vehicle_dir / "params.parm"
    if not parm_src.exists():
        print(f"[ERROR] Missing params.parm in {vehicle_dir}")
        sys.exit(1)

    return vehicle_dir


def prepare_vehicle_parm(vehicle_dir: Path) -> None:
    src = vehicle_dir / "params.parm"
    shutil.copy2(src, VEHICLE_PARM)
    print(f"[OK] Vehicle: {vehicle_dir.name}")
    print(f"     Params:  {src} -> {VEHICLE_PARM}")


def cmd_build() -> None:
    print("Building SITL image (first build takes ~30-45 minutes)...")
    result = run(["docker", "compose", "build"], check=False)
    if result.returncode != 0:
        print("[ERROR] docker compose build failed.")
        sys.exit(result.returncode)
    print("[OK] Image built successfully.")


def start_visualizer_bridge() -> subprocess.Popen | None:
    """Launch Scripts/control/visualizer_bridge.py in the background so UE can
    visualize ap_native flight. Returns the Popen handle (so caller can kill it)
    or None if it failed to start."""
    bridge = SCRIPTS_DIR / "control" / "visualizer_bridge.py"
    if not bridge.exists():
        print(f"[WARN] visualizer_bridge.py not found at {bridge}; UE will not visualize.")
        return None
    try:
        proc = subprocess.Popen(
            [sys.executable, str(bridge), "--mav", "tcp:localhost:5762"],
            cwd=SCRIPTS_DIR,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        print(f"  Visualizer bridge : pid {proc.pid} -> MAVLink SERIAL1 (5762) -> UDP 127.0.0.1:9100")
        return proc
    except OSError as e:
        print(f"[WARN] Could not start visualizer_bridge.py: {e}")
        return None


def start_servo_relay() -> subprocess.Popen | None:
    """Launch Scripts/control/servo_relay.py in the background.
    Relays AP servo packets from port 9006 → 127.0.0.1:9002 (UE JSONBridge).
    Required in ue_physics mode: Windows Firewall blocks Docker-NAT UDP to
    UE's port 9002 from non-loopback sources; via 127.0.0.1 it always works."""
    relay = SCRIPTS_DIR / "control" / "servo_relay.py"
    if not relay.exists():
        print(f"[WARN] servo_relay.py not found at {relay}; servo packets won't reach UE.")
        return None
    try:
        # stdout/stderr MUST be DEVNULL, not PIPE: nothing ever read the pipe,
        # so the relay's 5 s status prints filled the OS pipe buffer after
        # ~30-60 min and the relay blocked forever inside print() — the
        # "relay silently dies mid-session" bug (2026-06-09). DETACHED_PROCESS
        # additionally lets the relay survive this launcher exiting.
        flags = 0
        if sys.platform == "win32":
            flags = subprocess.DETACHED_PROCESS | subprocess.CREATE_NEW_PROCESS_GROUP
        proc = subprocess.Popen(
            [sys.executable, str(relay)],
            cwd=SCRIPTS_DIR,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=flags,
        )
        print(f"  Servo relay       : pid {proc.pid} -> port 9006 -> 127.0.0.1:9002 (UE JSONBridge)")
        return proc
    except OSError as e:
        print(f"[WARN] Could not start servo_relay.py: {e}")
        return None


def cmd_start(rebuild: bool, vehicle_override: str | None,
              home_override: dict | None = None) -> None:
    check_docker()
    vehicle_dir = resolve_vehicle(vehicle_override)
    prepare_vehicle_parm(vehicle_dir)

    backend = read_backend(vehicle_dir)
    env = docker_env_for_backend(backend, vehicle_dir, home_override)

    if rebuild:
        cmd_build()

    print()
    print("Starting ArduPlane SITL container...")
    print(f"  Backend           : {backend}")
    print(f"  AP --model        : {env['AP_MODEL']}")
    if backend == "ap_native":
        print(f"  Physics           : ArduPilot built-in (UE is visualizer only)")
    else:
        print(f"  Physics           : UE UVTOLPhysicsComponent via JSON udp/9003")
    print(f"  MAVLink out port  : 5760/tcp")
    print(f"  Home (--home)     : {env['AP_HOME']}  (from vehicle SIM_OPOS_*)")

    # Launch background helpers based on backend mode.
    vis_proc: subprocess.Popen | None = None
    relay_proc: subprocess.Popen | None = None

    if backend == "ap_native":
        vis_proc = start_visualizer_bridge()
    else:  # ue_physics
        kill_stale_relays()   # a leftover fleet relay on 9006 would double-bind
        relay_proc = start_servo_relay()

    print()
    print("Connect pymavlink (Windows):")
    print("  mav = mavutil.mavlink_connection('tcp:localhost:5760')")
    print()
    print("Press Ctrl+C to stop.\n")

    try:
        run(["docker", "compose", "up"], env=env, check=True)
    except KeyboardInterrupt:
        print("\nStopping...")
        run(["docker", "compose", "down", "-t", "2"], env=env)
    finally:
        for proc in [vis_proc, relay_proc]:
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()


def cmd_stop() -> None:
    check_docker()
    print("Stopping SITL container(s)...")
    # -t 2: SITL containers are disposable (state lives on the host), so give
    # SIGTERM only 2 s before SIGKILL instead of Docker's default 10 s grace
    # per project — the SITL processes ignore SIGTERM anyway, which made every
    # stop pay the full grace period. Fleet project first (the common case).
    if FLEET_COMPOSE.exists():
        run(["docker", "compose", "-p", FLEET_PROJECT, "-f", str(FLEET_COMPOSE),
             "down", "--remove-orphans", "-t", "2"], check=False)
    # Legacy single-vehicle compose project
    run(["docker", "compose", "down", "-t", "2"], check=False)
    kill_stale_relays()
    print("[OK] Containers stopped.")


def cmd_logs() -> None:
    check_docker()
    run(["docker", "compose", "logs", "--follow", "--tail=100"], check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="CezeriSim Docker SITL launcher")
    parser.add_argument("--vehicle", metavar="NAME",
                        help="Override active vehicle (folder name under Vehicles/, legacy mode)")
    parser.add_argument("--drones", type=int, default=None, metavar="N",
                        help="Fleet mode: number of quadrotor drones (ArduCopter)")
    parser.add_argument("--vtols", type=int, default=None, metavar="M",
                        help="Fleet mode: number of VTOL QuadPlanes (ArduPlane)")
    parser.add_argument("--drone-vehicle", metavar="NAME",
                        help="Drone vehicle config (default: Vehicles/active_drone_vehicle.txt)")
    parser.add_argument("--vtol-vehicle", metavar="NAME",
                        help="VTOL vehicle config (default: Vehicles/active_vehicle.txt)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Fleet mode: generate parms + compose file, don't start")
    parser.add_argument("--home", metavar="LAT,LON[,ALT[,HDG]]",
                        help="Override the base home (UE world-origin override / "
                             "active custom world) — replaces the vehicle's "
                             "SIM_OPOS lat/lon/alt; heading stays the vehicle's "
                             "unless given")
    parser.add_argument("--restart", action="store_true",
                        help="Stop any running containers/relays first, then "
                             "start fresh (clean-restart from the pause menu)")
    parser.add_argument("--build",  action="store_true", help="Rebuild image before starting")
    parser.add_argument("--stop",   action="store_true", help="Stop containers and relays")
    parser.add_argument("--logs",   action="store_true", help="Tail container logs")
    parser.add_argument("--list",   action="store_true", help="List available vehicle configs")
    args = parser.parse_args()

    if args.list:
        list_vehicles()
    elif args.stop:
        cmd_stop()
    elif args.logs:
        cmd_logs()
    else:
        if args.restart:
            # Clean restart (pause menu Restart button): tear down whatever is
            # running — containers AND relays — before starting fresh.
            cmd_stop()
        home_override = parse_home_override(args.home)
        if args.drones is not None or args.vtols is not None:
            drones = max(args.drones or 0, 0)
            vtols  = max(args.vtols or 0, 0)
            if drones + vtols == 0:
                print("[ERROR] Fleet mode needs at least one vehicle "
                      "(--drones N and/or --vtols M).")
                sys.exit(1)
            cmd_fleet_start(drones, vtols, args.drone_vehicle, args.vtol_vehicle,
                            rebuild=args.build, dry_run=args.dry_run,
                            home_override=home_override)
        else:
            cmd_start(rebuild=args.build, vehicle_override=args.vehicle,
                      home_override=home_override)


if __name__ == "__main__":
    main()

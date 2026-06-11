#!/usr/bin/env python3
"""
start_sitl_docker.py  —  Run from Windows (PowerShell or CMD)
Starts the CezeriSim ArduPlane SITL container via Docker Compose.

Reads the active vehicle from <project>/Vehicles/active_vehicle.txt (or --vehicle flag),
copies its params.parm to Scripts/vehicle.parm, then launches docker compose.

Requirements:
  - Docker Desktop for Windows installed and running
  - Run `docker compose build` once in Scripts/ before first use

Usage:
    python start_sitl_docker.py                      # use active_vehicle.txt
    python start_sitl_docker.py --vehicle pasifik    # override active vehicle
    python start_sitl_docker.py --build              # rebuild image then start
    python start_sitl_docker.py --stop               # stop the container
    python start_sitl_docker.py --logs               # tail container logs
    python start_sitl_docker.py --list               # list available vehicle configs
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPTS_DIR   = Path(__file__).parent
PROJECT_DIR   = SCRIPTS_DIR.parent
VEHICLES_DIR  = PROJECT_DIR / "Vehicles"
ACTIVE_TXT    = VEHICLES_DIR / "active_vehicle.txt"
VEHICLE_PARM  = SCRIPTS_DIR / "vehicle.parm"   # mounted into container
COMPOSE_FILE  = SCRIPTS_DIR / "docker-compose.yml"


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


def docker_env_for_backend(backend: str) -> dict:
    """Build the env dict passed to `docker compose up` based on backend."""
    env = os.environ.copy()
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
    result = run(["docker", "info"], capture_output=True)
    if result.returncode != 0:
        print("[ERROR] Docker Desktop is not running or not installed.")
        print("        Start Docker Desktop and try again.")
        sys.exit(1)


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


def cmd_start(rebuild: bool, vehicle_override: str | None) -> None:
    check_docker()
    vehicle_dir = resolve_vehicle(vehicle_override)
    prepare_vehicle_parm(vehicle_dir)

    backend = read_backend(vehicle_dir)
    env = docker_env_for_backend(backend)

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

    # Launch background helpers based on backend mode.
    vis_proc: subprocess.Popen | None = None
    relay_proc: subprocess.Popen | None = None

    if backend == "ap_native":
        vis_proc = start_visualizer_bridge()
    else:  # ue_physics
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
        run(["docker", "compose", "down"], env=env)
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
    print("Stopping SITL container...")
    run(["docker", "compose", "down"], check=True)
    print("[OK] Container stopped.")


def cmd_logs() -> None:
    check_docker()
    run(["docker", "compose", "logs", "--follow", "--tail=100"], check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="CezeriSim Docker SITL launcher")
    parser.add_argument("--vehicle", metavar="NAME",
                        help="Override active vehicle (folder name under Vehicles/)")
    parser.add_argument("--build",  action="store_true", help="Rebuild image before starting")
    parser.add_argument("--stop",   action="store_true", help="Stop the running container")
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
        cmd_start(rebuild=args.build, vehicle_override=args.vehicle)


if __name__ == "__main__":
    main()

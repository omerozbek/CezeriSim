"""
servo_relay.py - UDP relay: AP servo packets -> UE JSONBridge

Problem: Docker->host UDP arrives at Windows as source 127.0.0.1, but Windows
Firewall blocks it from reaching UE on port 9002 for non-loopback sources.
AP sends servo OUT to host.docker.internal:9006 (relay port).
This script receives on port 9006 (Docker-to-host UDP works here) and
re-sends as 127.0.0.1 -> 127.0.0.1:9002 (UE's JSONBridge, always reachable).

Usage (auto-started by start_sitl_docker.py in ue_physics mode):
    python servo_relay.py                       # legacy single vehicle: 9006 -> 9002
    python servo_relay.py --listen 9016 --dest 9012   # fleet instance 1

Fleet mode starts one relay per SITL instance i with
    --listen 9006 + 10*i   --dest 9002 + 10*i
matching the port conventions in docs/ARCHITECTURE.md.
"""

import argparse
import socket
import threading
import time

_ap = argparse.ArgumentParser(description="AP servo -> UE JSONBridge UDP relay")
_ap.add_argument("--listen", type=int, default=9006,
                 help="port AP sends servo packets to (--sim-port-out)")
_ap.add_argument("--dest", type=int, default=9002,
                 help="UE JSONBridge SimPort to forward to on 127.0.0.1")
_args = _ap.parse_args()

RELAY_PORT  = _args.listen   # AP sends servo here (--sim-port-out)
UE_PORT     = _args.dest     # UE JSONBridge listens here
UE_HOST     = "127.0.0.1"
RELAY_HOST  = "0.0.0.0"

def _make_recv_socket(family, host):
    """Create a bound, non-blocking UDP recv socket, or None if it fails."""
    try:
        s = socket.socket(family, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if family == socket.AF_INET6:
            # v6-only so it doesn't clash with the separate IPv4 socket on the
            # same port; we listen on both families explicitly.
            s.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
        s.bind((host, RELAY_PORT))
        s.setblocking(False)
        return s
    except OSError as e:
        print(f"[servo_relay] could not bind {family} on {host}:{RELAY_PORT}: {e}")
        return None


def main():
    import select

    # ArduPilot resolves host.docker.internal and may send servos over EITHER
    # IPv4 (192.168.65.254) or IPv6 (fdc4:...::254) depending on the resolver
    # order. Listen on BOTH so servos always reach us — this is the root-cause
    # fix for the intermittent "AP silent / no servos" link drop.
    socks = []
    s4 = _make_recv_socket(socket.AF_INET, "0.0.0.0")
    if s4:
        socks.append(s4)
        print(f"[servo_relay] Listening on 0.0.0.0:{RELAY_PORT} (IPv4)")
    s6 = _make_recv_socket(socket.AF_INET6, "::")
    if s6:
        socks.append(s6)
        print(f"[servo_relay] Listening on [::]:{RELAY_PORT} (IPv6)")
    if not socks:
        print("[servo_relay] FATAL: could not bind any socket on "
              f"port {RELAY_PORT}."); return

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[servo_relay] Forwarding to {UE_HOST}:{UE_PORT}")
    pkt_count = 0
    last_log  = time.time()

    try:
        while True:
            ready, _, _ = select.select(socks, [], [], 0.5)
            for s in ready:
                try:
                    data, _ = s.recvfrom(256)
                except OSError:
                    continue
                send_sock.sendto(data, (UE_HOST, UE_PORT))
                pkt_count += 1

            now = time.time()
            if now - last_log >= 60.0:
                last_log = now
                # Crash-proof logging: if stdout is a closed/full pipe, print()
                # blocks or raises and the relay dies silently mid-session
                # (2026-06-09). The relay must NEVER die because of logging.
                try:
                    print(f"[servo_relay] {pkt_count} packets relayed in last 60s", flush=True)
                except OSError:
                    pass
                pkt_count = 0
    except KeyboardInterrupt:
        print("[servo_relay] Stopped.")
    finally:
        for s in socks:
            s.close()
        send_sock.close()

if __name__ == "__main__":
    main()

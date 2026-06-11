#!/bin/bash
# Resolve host.docker.internal to IPv4 explicitly.
# 'getent hosts' prefers IPv6; 'getent ahosts' returns all records — we filter
# out any address containing ':' (IPv6) to get the IPv4 one.
HOST_IP=$(getent ahosts host.docker.internal \
    | awk '/STREAM/ && $1 !~ /:/ {print $1; exit}')

if [ -z "$HOST_IP" ]; then
    echo "[entrypoint] ERROR: could not resolve host.docker.internal to IPv4"
    exit 1
fi

echo "[entrypoint] host.docker.internal -> $HOST_IP (IPv4)"

# Always start from a clean EEPROM so --defaults is the authoritative param source.
rm -f /home/ardupilot/sitl/eeprom.bin

# Export HOST_IP so AP_EXTRA_ARGS can reference it if needed (ue_physics mode).
export HOST_IP

exec /home/ardupilot/ardupilot/build/sitl/bin/arduplane "$@"

#!/usr/bin/env python3
"""
Apply required patches to ArduPilot SITL source before the waf build.
Run from the ardupilot/ source root.

Patches:
  1. SIM_JSON.cpp  — bind the receive socket to port_in so Docker's
                     static UDP port mapping (9003:9003/udp) can route
                     JSON packets from UE into the container.  Without
                     this the OS assigns an ephemeral source port that
                     Docker has no mapping for.

  2. SIM_I2CDevice.cpp, SIM_Airspeed_DLVR.cpp
                   — replace AP_HAL::panic() calls with safe returns to
                     prevent crash-loops caused by driver/SITL version
                     mismatches.
"""

import sys
import os
import re

ARDUPILOT_ROOT = os.path.dirname(os.path.abspath(__file__))

# When run from the Dockerfile RUN step, CWD is /home/ardupilot/ardupilot
ARDUPILOT_ROOT = os.getcwd()


def read(path):
    with open(path, "r") as f:
        return f.read()


def write(path, text):
    with open(path, "w") as f:
        f.write(text)


def patch_file(relpath, old, new, description, required=True):
    path = os.path.join(ARDUPILOT_ROOT, relpath)
    if not os.path.exists(path):
        if required:
            print(f"  ERROR: file not found: {path}", file=sys.stderr)
            sys.exit(1)
        else:
            print(f"  SKIP  (file absent): {description}")
            return False

    txt = read(path)

    # Already applied?
    if old not in txt:
        if new.splitlines()[0].strip() in txt or "sock.bind" in txt:
            print(f"  SKIP  (already applied): {description}")
            return False
        print(f"  ERROR: patch target not found in {relpath}:\n    {old!r}", file=sys.stderr)
        sys.exit(1)

    write(path, txt.replace(old, new, 1))
    print(f"  OK    {description}")
    return True


# ── 1. SIM_JSON.cpp — bind receive socket to port_in ─────────────────────────
#
# The function set_interface_ports() stores the port numbers but never binds
# the socket.  The OS then assigns a random ephemeral source port when the
# socket first sends.  Docker Desktop only maps 9003:9003/udp, so the reply
# from UE (sent to container:9003) is never routed to AP's ephemeral port.
#
# We try two common variable-name patterns used across ArduPilot versions:
SIM_JSON = "libraries/SITL/SIM_JSON.cpp"

for pattern, bind_line in [
    # ArduPlane-stable: insert after reuseaddress() inside set_interface_ports()
    ("    sock.reuseaddress();\n",
     '    sock.reuseaddress();\n    sock.bind("0.0.0.0", (uint16_t)port_in);\n'),
    # Older branches: insert after _port_in assignment
    ("    _port_in = port_in;\n",
     '    _port_in = port_in;\n    sock.bind("0.0.0.0", (uint16_t)port_in);\n'),
    ("    port_in_ = port_in;\n",
     '    port_in_ = port_in;\n    sock.bind("0.0.0.0", (uint16_t)port_in);\n'),
]:
    if patch_file(SIM_JSON, pattern, bind_line,
                  "SIM_JSON: sock.bind(port_in) to fix Docker NAT", required=False):
        break
else:
    # Fallback: insert bind right before the closing brace of set_interface_ports
    path = os.path.join(ARDUPILOT_ROOT, SIM_JSON)
    txt = read(path)
    if "sock.bind" in txt:
        print("  SKIP  (already applied): SIM_JSON: sock.bind(port_in)")
    else:
        # Find set_interface_ports and inject before its closing brace
        pattern = r'(void SIM_JSON::set_interface_ports\([^)]+\)\s*\{[^}]+)(})'
        replacement = r'\1    sock.bind("0.0.0.0", (uint16_t)port_in);\n\2'
        new_txt = re.sub(pattern, replacement, txt, count=1, flags=re.DOTALL)
        if new_txt == txt:
            print("  WARN  SIM_JSON: could not apply sock.bind patch — check source manually",
                  file=sys.stderr)
        else:
            write(path, new_txt)
            print("  OK    SIM_JSON: sock.bind(port_in) [fallback regex]")

# ── 2 & 3. Panic suppression — replace AP_HAL::panic() with safe returns ──────
# The return value (void vs int) is determined by parsing the enclosing function
# signature, so this works correctly for mixed return types.

def suppress_panics(relpath, default_ret="void"):
    path = os.path.join(ARDUPILOT_ROOT, relpath)
    if not os.path.exists(path):
        print(f"  SKIP  (not found): {relpath}")
        return

    lines = open(path).readlines()
    original = "".join(lines)

    if "AP_HAL::panic" not in original:
        print(f"  SKIP  (no panics): {relpath}")
        return

    current_ret = default_ret
    result = []
    for line in lines:
        s = line.strip()

        # Track function return type from definition lines
        m = re.match(r'^(int|ssize_t|bool|uint\w+|size_t)\s+\S+::\S+\s*\(', s)
        if m:
            current_ret = m.group(1)
        elif re.match(r'^void\s+\S+::\S+\s*\(', s):
            current_ret = "void"

        # Replace panic with appropriate return
        if "AP_HAL::panic(" in line:
            ret_stmt = "return;" if current_ret == "void" else "return -1;"
            line = re.sub(r'AP_HAL::panic\([^)]*\);', ret_stmt, line)

        result.append(line)

    new_txt = "".join(result)
    if new_txt != original:
        open(path, "w").write(new_txt)
        print(f"  OK    {relpath.split('/')[-1]}: panics → safe returns")
    else:
        print(f"  SKIP  {relpath.split('/')[-1]}: already patched")

suppress_panics("libraries/SITL/SIM_I2CDevice.cpp", default_ret="void")
suppress_panics("libraries/SITL/SIM_Airspeed_DLVR.cpp", default_ret="int")

print("Patches complete.")

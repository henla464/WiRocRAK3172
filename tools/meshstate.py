#!/usr/bin/env python3
"""Query ATC+MESHSTATE from a RAK3172 over serial and decode the fields.

Usage: ./meshstate.py [port]   (default /dev/ttyUSB0, 115200 8N1)
"""

import os
import select
import sys
import termios
import time
import tty

FIELDS = ["enabled", "is_root", "addr", "txq", "state", "alloc", "parent",
          "hops", "path_cost", "neighbours", "epoch", "overhead_pct",
          "ack_timeout_ms", "standby", "tx_ok", "tx_busy", "tx_assign",
          "rx_any", "rx_assign", "adopt", "last_type"]
STATE = {0: "unassigned", 1: "joining", 2: "joined"}
ERRORS = ("AT_BUSY_ERROR", "AT_PARAM_ERROR", "AT_ERROR")


def open_port(dev):
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSADRAIN, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def query(dev, name, timeout=2.0):
    fd = open_port(dev)
    try:
        os.write(fd, f"ATC+{name}=?\r\n".encode())
        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not select.select([fd], [], [], 0.1)[0]:
                continue
            buf += os.read(fd, 256)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode(errors="replace").strip()
                if f"{name}=" in line:
                    return line.split(f"{name}=", 1)[1]
                if line in ERRORS or line.startswith("AT_"):
                    return line
    finally:
        os.close(fd)
    return None


def raw(dev, cmd, timeout=2.0):
    fd = open_port(dev)
    try:
        os.write(fd, (cmd + "\r\n").encode())
        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not select.select([fd], [], [], 0.1)[0]:
                continue
            buf += os.read(fd, 256)
            if b"\n" in buf:
                break
        for line in buf.decode(errors="replace").splitlines():
            if line.strip():
                print(line.strip())
    finally:
        os.close(fd)
    return 0


def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    if len(sys.argv) > 2:
        return raw(dev, " ".join(sys.argv[2:]))
    try:
        reply = query(dev, "MESHSTATE")
    except OSError as e:
        print(f"cannot open {dev}: {e}")
        return 1
    if reply is None:
        print(f"no reply from {dev}")
        return 1
    print(reply)
    vals = reply.split(":")
    if len(vals) != len(FIELDS):
        print(f"expected {len(FIELDS)} fields, got {len(vals)}: {reply}")
        return 1
    for name, v in zip(FIELDS, vals):
        note = ""
        if name == "state":
            note = f"  ({STATE.get(int(v), '?')})"
        elif name in ("enabled", "is_root", "standby"):
            note = "  (yes)" if v == "1" else "  (no)"
        print(f"{name:16} {v}{note}")
    is_root = vals[FIELDS.index("is_root")] == "1"
    if not is_root and (vals[FIELDS.index("state")] != "2" or vals[FIELDS.index("parent")] == "0"):
        print("-> uplinks refused: node not attached")

    for name in ("MESHTOPO", "MESHNODEDEVICEID"):
        try:
            out = query(dev, name)
        except OSError as e:
            print(f"cannot open {dev}: {e}")
            return 1
        print(f"{name:16} {out if out is not None else '(no reply)'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Publish wheel odometry over local UDP so other programs (a localizer, a navigator, ROS bridge, logging)
can use it without touching the serial port.

One JSON datagram per control tick (~30 Hz) to 127.0.0.1:5601 by default:

  {"type": "odom", "boot": <id of this run>, "seq": n, "t": <time.monotonic() of the encoder read>,
   "x": m, "y": m, "theta": rad, "v": m/s, "w": rad/s, "left_m": m, "right_m": m, "healthy": bool,
   "wheel_diameter_m": .., "track_width_m": ..}

`t` is CLOCK_MONOTONIC, which on Linux is shared by all processes on the machine, so the
receiver can line it up with its own camera timestamps. `boot` changes every time the base
program starts (the odometry frame restarts at 0,0,0), so receivers know to re-anchor.
UDP never blocks and nothing breaks if no one is listening.
"""

from __future__ import annotations

import json
import socket
import time


def parse_addr(text: str | None) -> tuple[str, int] | None:
    if not text or text.lower() in ("off", "none", "0"):
        return None
    host, _, port = text.rpartition(":")
    return (host or "127.0.0.1", int(port))


class OdomPublisher:
    def __init__(self, base, addr: tuple[str, int] = ("127.0.0.1", 5601)) -> None:
        self.base = base
        self.addr = addr
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.boot = f"{time.time():.3f}"
        self.seq = 0
        self.last_stamp = None
        self.errors = 0

    def publish(self, force: bool = False) -> bool:
        """Send the latest odometry if it changed since the last call. Returns True if sent."""
        o = self.base.get_odometry()
        if not force and o.stamp == self.last_stamp:
            return False
        self.last_stamp = o.stamp
        self.seq += 1
        msg = {"type": "odom", "boot": self.boot, "seq": self.seq, "t": o.stamp,
               "x": o.x, "y": o.y, "theta": o.theta, "v": o.v, "w": o.w,
               "left_m": o.left_m, "right_m": o.right_m, "healthy": o.healthy,
               "wheel_diameter_m": self.base.wheel_circ / 3.141592653589793,
               "track_width_m": self.base.track}
        try:
            self.sock.sendto(json.dumps(msg, separators=(",", ":")).encode(), self.addr)
            return True
        except OSError:                 # nobody listening / buffer full: never disturb driving
            self.errors += 1
            return False

    def close(self) -> None:
        self.sock.close()

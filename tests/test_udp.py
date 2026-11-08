"""Tests for the UDP interfaces: odometry out (5601) and navigation commands in (5603).

No hardware, no pyserial or websockets needed.  Run:  python tests/test_udp.py
"""

from __future__ import annotations

import json
import socket
import sys
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import joystick_server as js                 # noqa: E402
from odom_udp import OdomPublisher, parse_addr  # noqa: E402


class StubBase:
    wheel_circ = 0.125 * 3.141592653589793
    track = 0.508

    def __init__(self) -> None:
        self.stamp = 1.0
        self.commands = []

    def get_odometry(self):
        return SimpleNamespace(stamp=self.stamp, x=0.1, y=0.2, theta=0.3, v=0.05, w=0.0,
                               left_m=0.4, right_m=0.4, healthy=True)

    def set_velocity(self, v, w):
        self.commands.append((v, w))


def test_parse_addr():
    assert parse_addr("127.0.0.1:5601") == ("127.0.0.1", 5601)
    assert parse_addr(":6000") == ("127.0.0.1", 6000)
    assert parse_addr("off") is None and parse_addr(None) is None


def test_odometry_datagram():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(1.0)
    base = StubBase()
    pub = OdomPublisher(base, rx.getsockname())
    assert pub.publish()
    assert not pub.publish(), "unchanged sample must not be re-sent"
    msg = json.loads(rx.recv(4096))
    assert msg["type"] == "odom" and msg["seq"] == 1
    assert (msg["x"], msg["y"], msg["theta"]) == (0.1, 0.2, 0.3)
    assert abs(msg["wheel_diameter_m"] - 0.125) < 1e-9 and msg["track_width_m"] == 0.508
    base.stamp = 2.0
    assert pub.publish()
    assert json.loads(rx.recv(4096))["seq"] == 2
    pub.close()
    rx.close()


def test_publish_without_listener_never_raises():
    pub = OdomPublisher(StubBase(), ("127.0.0.1", 9))     # nothing listens on port 9
    for _ in range(5):
        pub.publish(force=True)
    pub.close()


def test_nav_command_and_joystick_priority():
    base = StubBase()
    js.base = base
    js.nav_state.update(last=-1e9, joy_until=-1e9, ignored=0, count=0)
    proto = js.NavProtocol()
    proto.datagram_received(json.dumps({"type": "cmd", "v": 0.2, "w": 0.1}).encode(), None)
    assert base.commands == [(0.2, 0.1)]
    for bad in (b"not json", b'{"type": "other", "v": 1, "w": 1}', b'{"type": "cmd", "v": 1}'):
        proto.datagram_received(bad, None)
    assert base.commands == [(0.2, 0.1)], "malformed or foreign messages must be ignored"
    js.nav_state["joy_until"] = time.monotonic() + 1.0   # phone joystick was just pushed
    proto.datagram_received(json.dumps({"type": "cmd", "v": 0.3, "w": 0}).encode(), None)
    assert base.commands == [(0.2, 0.1)] and js.nav_state["ignored"] == 1


if __name__ == "__main__":
    tests = [f for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    for t in tests:
        t()
        print("pass", t.__name__)
    print(f"OK: {len(tests)} tests")

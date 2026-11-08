"""Decide which connected device is driving.

Problem it solves: a phone with two joystick pages open (e.g. an old tab) sends
"drive" from one page and "stop" (x=0, y=0 heartbeat) from the other, so the robot
alternates between moving and stopping.

Rule: exactly one connection controls the robot at a time.
  * the controller's messages are always applied (including stop);
  * when nobody controls, or the controller has gone quiet for `idle_s`,
    the next message from any connection takes control;
  * while the controller is active, another connection only takes over by
    actually pushing its joystick (a non-zero command); its "stop" heartbeats
    are ignored.
Every connection's emergency stop still works once it is the controller; and if
the controlling page closes, the base's fail-safe stops the wheels anyway.
"""

from __future__ import annotations

import time


class ControlArbiter:
    def __init__(self, idle_s: float = 0.5, clock=time.monotonic) -> None:
        self.idle_s = idle_s
        self.clock = clock
        self.controller = None
        self.last_msg = -1e9
        self.ignored = 0          # diagnostics: messages dropped from non-controllers

    def accept(self, client, x: float, y: float) -> tuple[bool, bool]:
        """Return (apply_this_command, control_changed)."""
        now = self.clock()
        changed = False
        if client is not self.controller:
            idle = self.controller is None or now - self.last_msg > self.idle_s
            if idle or (x != 0 or y != 0):
                self.controller = client
                changed = True
            else:
                self.ignored += 1
                return False, False
        self.last_msg = now
        return True, changed

    def disconnect(self, client) -> None:
        if client is self.controller:
            self.controller = None

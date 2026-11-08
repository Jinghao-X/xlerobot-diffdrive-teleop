"""A fake serial port that behaves like a bus of STS3215 servos.

Used by the tests and by `"port": "fake"` in robot_config.json, so the joystick
server, calibration and odometry can be tried without hardware.
Wheel-mode speed is modelled as `steps_per_unit` encoder steps per second per unit.
"""

from __future__ import annotations

import time
from collections import deque

from sts_bus import (ADDR_GOAL_VELOCITY, ADDR_LOCK, ADDR_OPERATING_MODE, ADDR_PRESENT_POSITION,
                     ADDR_PRESENT_VELOCITY, ADDR_TORQUE_ENABLE, INST_READ, INST_SYNC_WRITE, INST_WRITE,
                     STEPS_PER_REV, checksum, decode_signed, encode_signed)


class FakeMotor:
    def __init__(self) -> None:
        self.regs = {ADDR_OPERATING_MODE: 0, ADDR_LOCK: 1, ADDR_TORQUE_ENABLE: 0,   # as after power-up
                     0x3E: 120}                                                    # 12.0 V supply
        self.goal = 0            # signed units
        self.position = 0.0      # continuous steps
        self.eeprom_writes = 0
        self.ignored_mode_writes = 0


class FakeSTSSerial:
    def __init__(self, port: str = "fake", baudrate: int = 1_000_000, timeout: float = 0.02, *,
                 motor_ids=(7, 8), steps_per_unit: float = 1.0, clock=time.monotonic,
                 reply_to_writes: bool = True, reply_latency: bool = True) -> None:
        self.motors = {i: FakeMotor() for i in motor_ids}
        self.steps_per_unit = steps_per_unit
        self.clock = clock
        self.reply_to_writes = reply_to_writes
        self._last = clock()
        self._out: deque[int] = deque()
        # Real USB serial: a status reply is still "in flight" for a moment after the
        # request, so it can arrive *after* reset_input_buffer(). With reply_latency=True,
        # write replies only become readable at the next read(), surviving a reset.
        self.reply_latency = reply_latency
        self._in_flight: list[int] = []
        self.eeprom_busy_reads = 0   # a few reads go unanswered right after an EEPROM write
        self.is_open = True
        self.fail_next_writes = 0   # tests: raise OSError on the next N writes

    # -------------------------------------------------------------- physics
    def _advance(self) -> None:
        now = self.clock()
        dt = now - self._last
        self._last = now
        for m in self.motors.values():
            if m.regs[ADDR_OPERATING_MODE] == 1 and m.regs[ADDR_TORQUE_ENABLE] == 1:
                m.position += m.goal * self.steps_per_unit * dt   # no torque: the wheel does not drive

    # -------------------------------------------------------------- serial API
    def reset_input_buffer(self) -> None:
        self._out.clear()                # replies still in flight are NOT cleared

    def read(self, n: int = 1) -> bytes:
        if self._in_flight:                       # sent earlier, so it arrives first
            self._out.extendleft(reversed(self._in_flight))
            self._in_flight.clear()
        out = bytes(self._out.popleft() for _ in range(min(n, len(self._out))))
        return out

    def close(self) -> None:
        self.is_open = False

    def write(self, data: bytes) -> int:
        if self.fail_next_writes > 0:
            self.fail_next_writes -= 1
            raise OSError("fake serial failure")
        self._advance()
        buf = list(data)
        while len(buf) >= 6:
            if buf[0] != 0xFF or buf[1] != 0xFF:
                buf.pop(0)
                continue
            mid, length, inst = buf[2], buf[3], buf[4]
            packet = buf[:4 + length]
            buf = buf[4 + length:]
            params, chk = packet[5:-1], packet[-1]
            if checksum([mid, length, inst, *params]) != chk:
                continue                       # real servos ignore corrupt packets
            if inst == INST_SYNC_WRITE and mid == 0xFE:      # broadcast, no replies
                addr, n = params[0], params[1]
                body = params[2:]
                for k in range(0, len(body) - n, n + 1):
                    target = self.motors.get(body[k])
                    if target is not None:
                        self._write_regs(target, addr, body[k + 1:k + 1 + n])
                continue
            m = self.motors.get(mid)
            if m is None:
                continue
            if inst == INST_WRITE:
                self._write_regs(m, params[0], params[1:])
                if self.reply_to_writes:
                    self._reply(mid, [], delayed=self.reply_latency)
            elif inst == INST_READ:
                if self.eeprom_busy_reads > 0:
                    self.eeprom_busy_reads -= 1
                    continue
                self._reply(mid, self._read_regs(m, params[0], params[1]))
        return len(data)

    def _write_regs(self, m: FakeMotor, addr: int, data: list[int]) -> None:
        if addr == ADDR_GOAL_VELOCITY and len(data) == 2:
            m.goal = decode_signed(data[0] | (data[1] << 8))
        elif addr == ADDR_OPERATING_MODE:
            if m.regs[ADDR_LOCK] == 0:
                m.eeprom_writes += 1
                self.eeprom_busy_reads = 1
            if m.regs[ADDR_TORQUE_ENABLE] == 0:          # like LeRobot: change mode with torque off
                m.regs[ADDR_OPERATING_MODE] = data[0]
            else:
                m.ignored_mode_writes += 1
        else:
            for k, v in enumerate(data):
                m.regs[addr + k] = v

    def _read_regs(self, m: FakeMotor, addr: int, length: int) -> list[int]:
        if addr == ADDR_PRESENT_POSITION and length == 2:
            p = int(m.position) % STEPS_PER_REV
            return [p & 0xFF, p >> 8]
        if addr == ADDR_PRESENT_VELOCITY and length == 2:
            raw = encode_signed(round(m.goal * self.steps_per_unit))
            return [raw & 0xFF, raw >> 8]
        return [m.regs.get(addr + k, 0) for k in range(length)]

    def _reply(self, mid: int, data: list[int], delayed: bool = False) -> None:
        body = [mid, len(data) + 2, 0, *data]
        (self._in_flight if delayed else self._out).extend([0xFF, 0xFF, *body, checksum(body)])

"""Tests for the base driver, using the fake STS3215 bus (no hardware, no pyserial needed).

Run:  python tests/test_base.py     (or pytest)
"""

from __future__ import annotations

import copy
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from base_driver import DiffDriveBase, load_config            # noqa: E402
from fake_sts import FakeSTSSerial                              # noqa: E402
from joystick_mapping import joystick_to_velocity, v4_wheel_targets  # noqa: E402
from sts_bus import STSBus, build_packet, MODE_WHEEL, STEPS_PER_REV  # noqa: E402

CFG = load_config()


class Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t


def make(cfg=None, **fake_kw):
    cfg = copy.deepcopy(cfg or CFG)
    clock = Clock()
    holder = {}

    def factory(port, baudrate, timeout):
        holder["ser"] = FakeSTSSerial(port, baudrate, timeout, motor_ids=(7, 8), clock=clock, **fake_kw)
        return holder["ser"]
    bus = STSBus("fake", serial_factory=factory)
    base = DiffDriveBase(cfg, bus=bus, clock=clock)
    base.start(threaded=False)
    return base, holder["ser"], clock


def run(base, clock, seconds, command=None):
    """Advance simulated time in control ticks, re-sending `command` (v, w) each tick like the phone does."""
    dt = 1.0 / base.tick_hz
    for _ in range(round(seconds * base.tick_hz)):
        if command is not None:
            base.set_velocity(*command)
        clock.t += dt
        base.tick()


def truth(ser, base):
    """Forward distance of each wheel from the fake motors' continuous positions."""
    l = ser.motors[7].position * base.enc_left / STEPS_PER_REV * base.wheel_circ
    r = ser.motors[8].position * base.enc_right / STEPS_PER_REV * base.wheel_circ
    return l, r


# ------------------------------------------------------------------ protocol

def test_packets_match_v4():
    """Byte-for-byte identical to what xle_wheel_joystick_server_v4.py sends today."""
    def v4_speed_packet(motor_id, speed):
        speed_val = abs(speed) | 0x8000 if speed < 0 else speed
        data = [motor_id, 0x05, 0x03, 0x2E, speed_val & 0xFF, (speed_val >> 8) & 0xFF]
        return bytes([0xFF, 0xFF] + data + [(~sum(data)) & 0xFF])
    for mid in (7, 8):
        for speed in (0, 1, 5, 500, -500, 1000, -1000, -1):
            raw = abs(speed) | 0x8000 if speed < 0 else speed
            assert build_packet(mid, 0x03, [0x2E, raw & 0xFF, raw >> 8]) == v4_speed_packet(mid, speed)


def test_read_ignores_write_replies_and_bad_checksums():
    base, ser, clock = make()
    ser.motors[7].position = 1234
    base.bus.set_goal_velocity(7, 0)                     # leaves a status reply in the buffer
    assert base.bus.read_position(7) == 1234
    real_reply = ser._reply
    ser._reply = lambda mid, data: (real_reply(mid, data), ser._out.__setitem__(-1, ser._out[-1] ^ 0xFF))
    assert base.bus.read_position(7) is None             # corrupted checksum is rejected
    ser._reply = real_reply
    assert base.bus.read_position(99) is None            # no such motor: timeout, no exception


def test_read_after_write_with_usb_latency():
    """Regression for the real-robot failure: a write's reply arriving after reset_input_buffer()."""
    base, ser, clock = make()
    ser.motors[8].position = 777
    for _ in range(20):
        base.bus.set_goal_velocity(7, 0)                 # reply is still "in flight"
        base.bus.set_goal_velocity(8, 0)
        assert base.bus.read_position(8) == 777


def test_sync_write_packet_format():
    """Feetech SYNC WRITE: FF FF FE LEN 83 ADDR N (ID D1 D2)... CHK, LEN = (N+1)*count + 4."""
    sent = []
    base, ser, clock = make()
    real_write = ser.write
    ser.write = lambda data: (sent.append(bytes(data)), real_write(data))[1]
    base.bus.set_goal_velocities({7: -500, 8: 300})
    body = [0xFE, 10, 0x83, 0x2E, 2, 7, 0xF4, 0x81, 8, 0x2C, 0x01]
    assert sent[-1] == bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])
    assert ser.motors[7].goal == -500 and ser.motors[8].goal == 300


def test_mode_switch_like_real_motors():
    """Motors arrive in position mode with torque on (as read on the real robot): switch must stick."""
    base, ser, clock = make()
    for mid in (7, 8):
        m = ser.motors[mid]
        assert m.regs[0x21] == MODE_WHEEL            # switched by start()
        assert m.regs.get(0x28) == 1                 # torque re-enabled
        assert m.ignored_mode_writes == 0
    run(base, clock, 2.0, (0.05, 0.0))
    o = base.get_odometry()
    assert o.healthy and o.left_m > 0.05 and o.right_m > 0.05


def test_torque_enabled_after_power_cycle():
    """Regression: motors already in wheel mode after a power cycle still have torque OFF.

    This is what happened on the real robot: reads worked, the joystick did nothing,
    and the robot could be pushed by hand.
    """
    base, ser, clock = make()
    for m in ser.motors.values():          # power cycle: mode kept in EEPROM, torque back to 0
        m.regs[0x28] = 0
    base.start(threaded=False)
    assert all(m.regs[0x28] == 1 for m in ser.motors.values())
    run(base, clock, 2.0, (0.05, 0.0))
    assert base.get_odometry().left_m > 0.05


def test_low_motor_voltage_is_reported():
    """Regression: motor supply off, board fed ~4.9 V over USB -> motors answer but cannot turn."""
    import contextlib
    import io
    base, ser, clock = make()
    for m in ser.motors.values():
        m.regs[0x3E] = 49
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        assert base.check_power() is False
    assert "4.9 V" in out.getvalue()
    for m in ser.motors.values():
        m.regs[0x3E] = 120
    assert base.check_power() is True


def test_wheels_released_on_exit():
    import contextlib
    import io
    base, ser, clock = make()
    run(base, clock, 1.0, (0.05, 0.0))
    with contextlib.redirect_stdout(io.StringIO()):
        base.close()
    assert all(m.regs[0x28] == 0 and m.goal == 0 for m in ser.motors.values())


def test_wheel_mode_written_once():
    base, ser, clock = make()                            # start() switched both motors to wheel mode
    assert ser.motors[7].regs[0x21] == MODE_WHEEL and ser.motors[7].eeprom_writes == 1
    assert ser.motors[7].regs[0x37] == 1                 # EEPROM locked again
    base.start(threaded=False)                           # restart: already in wheel mode, no new write
    assert ser.motors[7].eeprom_writes == 1


# ------------------------------------------------------------------ joystick equivalence

def test_joystick_matches_v4_without_ratio_preservation():
    cfg = copy.deepcopy(CFG)
    cfg["control"]["preserve_turn_ratio"] = False
    base, ser, clock = make(cfg)
    rng = random.Random(4)
    for _ in range(2000):
        x, y = rng.uniform(-1, 1), rng.uniform(-1, 1)
        base.set_velocity(*joystick_to_velocity(x, y, cfg))
        want = v4_wheel_targets(x, y, cfg)
        assert all(abs(a - b) < 1e-6 for a, b in zip(base.target, want)), (x, y, base.target, want)


def test_ratio_preserved_when_saturated():
    base, ser, clock = make()
    base.set_velocity(*joystick_to_velocity(1.0, 1.0, CFG))   # v4 would give left=cap, right=0
    l, r = base.target[0] * base.sign_left, base.target[1] * base.sign_right
    assert abs(max(abs(l), abs(r)) - base.max_units) < 1e-6
    base.set_velocity(*joystick_to_velocity(0.3, 0.5, CFG))   # unsaturated: same as v4
    want = v4_wheel_targets(0.3, 0.5, CFG)
    assert all(abs(a - b) < 1e-6 for a, b in zip(base.target, want))


def test_joystick_directions():
    v, w = joystick_to_velocity(0, 1, CFG)
    assert v > 0 and w == 0
    v, w = joystick_to_velocity(1, 0, CFG)
    assert v == 0 and w < 0                                   # push right = turn clockwise
    assert joystick_to_velocity(0.05, -0.05, CFG) == (0.0, 0.0)  # dead zone
    assert joystick_to_velocity(float("nan"), 1, CFG)[1] == 0.0


def test_left_motor_inverted_on_the_wire():
    base, ser, clock = make()
    run(base, clock, 1.0, (0.05, 0.0))
    assert ser.motors[7].goal < 0 < ser.motors[8].goal        # forward: left motor spins "backwards"


# ------------------------------------------------------------------ odometry

def test_straight_line():
    base, ser, clock = make()
    run(base, clock, 5.0, (0.05, 0.0))
    run(base, clock, 1.0)
    o = base.get_odometry()
    tl, tr = truth(ser, base)
    assert o.healthy
    assert abs(o.x - (tl + tr) / 2) < 0.002 and abs(o.y) < 1e-6 and abs(o.theta) < 1e-6
    # 5 s at 0.05 m/s = 0.25 m; the smoothing ramp-up and ramp-down roughly cancel, and with no
    # explicit stop the fail-safe keeps the last command for 0.5 s (+0.025 m), exactly like v4
    assert 0.265 < o.x < 0.285


def test_rotate_in_place_ccw():
    base, ser, clock = make()
    run(base, clock, 3.0, (0.0, 0.4))
    run(base, clock, 1.0)
    o = base.get_odometry()
    tl, tr = truth(ser, base)
    assert o.theta > 0 and abs(o.x) < 1e-3 and abs(o.y) < 1e-3
    assert abs(o.theta - (tr - tl) / base.track) < 0.01


def test_full_circle_returns_home():
    base, ser, clock = make()
    v, w = 0.04, 0.1                                          # radius 0.4 m, below the wheel speed limit
    base.reset_odometry()
    run(base, clock, 0.5, (v, w))                             # let smoothing settle first
    o0 = base.get_odometry()
    run(base, clock, 2 * math.pi / w, (v, w))
    o1 = base.get_odometry()
    assert math.hypot(o1.x - o0.x, o1.y - o0.y) < 0.02
    assert abs(math.atan2(math.sin(o1.theta - o0.theta), math.cos(o1.theta - o0.theta))) < 0.05


def test_many_revolutions_wraparound():
    base, ser, clock = make()
    run(base, clock, 60.0, (base.max_speed_mps, 0.0))         # far more than one encoder revolution
    tl, tr = truth(ser, base)
    o = base.get_odometry()
    assert tl > 2 * base.wheel_circ
    assert abs(o.left_m - tl) < 0.003 and abs(o.right_m - tr) < 0.003


def test_missed_reads_lose_no_distance():
    base, ser, clock = make()
    run(base, clock, 1.0, (0.05, 0.0))
    real_read = base.bus.read_position
    base.bus.read_position = lambda mid: None                 # 4 ticks of failed reads
    run(base, clock, 4 / base.tick_hz, (0.05, 0.0))
    base.bus.read_position = real_read
    run(base, clock, 1.0, (0.05, 0.0))
    tl, tr = truth(ser, base)
    o = base.get_odometry()
    assert abs(o.left_m - tl) < 0.002 and abs(o.right_m - tr) < 0.002 and o.healthy


# ------------------------------------------------------------------ safety

def test_failsafe_stops_wheels():
    base, ser, clock = make()
    run(base, clock, 1.0, (0.05, 0.0))
    assert ser.motors[8].goal != 0
    run(base, clock, 0.5)                                     # silence: still within the timeout window
    run(base, clock, 1.0)                                     # past the timeout: decays to zero
    assert base.failsafe_active and ser.motors[7].goal == 0 and ser.motors[8].goal == 0


def test_non_finite_velocity_means_stop():
    base, ser, clock = make()
    base.set_velocity(float("nan"), 0.0)
    assert base.target == [0.0, 0.0]


def test_control_thread_survives_serial_errors():
    cfg = copy.deepcopy(CFG)
    cfg["wheels"]["port"] = "fake"
    base = DiffDriveBase(cfg)
    base.start()
    try:
        base.set_velocity(0.05, 0.0)
        time.sleep(0.2)
        base.bus.ser.fail_next_writes = 3                     # three failing ticks
        for _ in range(10):
            base.set_velocity(0.05, 0.0)
            time.sleep(0.05)
        assert base._thread.is_alive() and base._errors == 0
        assert base.bus.ser.motors[8].goal > 0                # driving again after the errors
    finally:
        base.close()


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        fn()
        print("pass", name)
    print(f"OK: {len(tests)} tests")

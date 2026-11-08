"""Differential-drive base for XLeRobot 0.4 (two STS3215 wheels).

One object, two kinds of callers:
  * the phone joystick (joystick_server.py) and, later, navigation call
    set_velocity(v, w) in physical units: v in m/s (forward +), w in rad/s (left/CCW +);
  * everyone reads get_odometry() for the pose integrated from the wheel encoders.

Safety behaviour carried over from xle_wheel_joystick_server_v4.py:
  * fail-safe: if nobody calls set_velocity / set_wheel_units for command_timeout_s, wheels stop;
  * exponential smoothing toward the target, tiny speeds snapped to 0;
  * the control thread never dies on serial errors; after 5 errors in a row it reopens the port.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path

from sts_bus import ADDR_PRESENT_VOLTAGE, MODE_WHEEL, STEPS_PER_REV, STSBus

CONFIG_PATH = Path(__file__).resolve().parent / "robot_config.json"


def load_config(path: str | Path = CONFIG_PATH) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def save_config(cfg: dict, path: str | Path = CONFIG_PATH) -> None:
    Path(path).write_text(json.dumps(cfg, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def open_wheel_bus(cfg: dict, **fake_kwargs) -> STSBus:
    w = cfg["wheels"]
    if w["port"] == "fake":
        from fake_sts import FakeSTSSerial

        def factory(port, baudrate, timeout):
            return FakeSTSSerial(port, baudrate, timeout, motor_ids=(w["left_id"], w["right_id"]), **fake_kwargs)
        return STSBus("fake", w["baudrate"], serial_factory=factory)
    return STSBus(w["port"], w["baudrate"])


@dataclass(frozen=True)
class Odometry:
    x: float = 0.0          # m, in the odometry frame (where the robot was at reset)
    y: float = 0.0
    theta: float = 0.0      # rad, CCW positive, wrapped to (-pi, pi]
    v: float = 0.0          # m/s, measured from encoders
    w: float = 0.0          # rad/s, measured from encoders
    left_m: float = 0.0     # total distance rolled by each wheel since reset (forward +)
    right_m: float = 0.0
    stamp: float = 0.0      # clock() of the last update
    healthy: bool = False   # False until the first good encoder read, or after repeated read failures


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class DiffDriveBase:
    def __init__(self, cfg: dict, bus: STSBus | None = None, clock=time.monotonic) -> None:
        self.cfg = cfg
        self.bus = bus if bus is not None else open_wheel_bus(cfg)
        self.clock = clock
        w, g, c, k = cfg["wheels"], cfg["geometry"], cfg["control"], cfg["calibration"]
        self.left_id, self.right_id = w["left_id"], w["right_id"]
        self.sign_left = -1 if w["invert_left"] else 1     # motor command sign for "forward"
        self.sign_right = -1 if w["invert_right"] else 1
        # encoder sign for "forward": normally the same as the command sign; flip if
        # `calibrate_base.py check` reports that a wheel's encoder counts the other way
        self.enc_left = self.sign_left * (-1 if w.get("encoder_flip_left", False) else 1)
        self.enc_right = self.sign_right * (-1 if w.get("encoder_flip_right", False) else 1)
        self.max_units = float(w["max_units"])
        self.wheel_circ = math.pi * float(g["wheel_diameter_m"])
        self.track = float(g["track_width_m"])
        self.units_per_mps = float(k["speed_units_per_mps"])
        self.tick_hz = float(c["tick_hz"])
        self.smoothing = float(c["smoothing"])
        self.timeout = float(c["command_timeout_s"])
        self.preserve_ratio = bool(c["preserve_turn_ratio"])
        self.stop_threshold = float(c["stop_threshold_units"])

        self.lock = threading.Lock()
        self.target = [0.0, 0.0]     # motor units (already sign-corrected), left/right
        self.current = [0.0, 0.0]
        self.last_cmd = -1e9
        self._last_raw: list[int | None] = [None, None]
        self._odom = Odometry()
        self._read_failures = 0
        self._errors = 0
        self._thread: threading.Thread | None = None
        self._running = False
        self.failsafe_active = True
        self.io_lock = threading.Lock()   # the serial bus is not thread safe
        self.stats = {"ticks": 0, "io_ms_max": 0.0, "read_failures": 0, "failsafe_stops": 0,
                      "since": time.monotonic()}

    # ------------------------------------------------------------------ lifecycle
    def check_power(self) -> bool:
        """Read each wheel motor's supply voltage. With the motor supply off, the board can
        still be powered through USB (~5 V): the motors answer but cannot turn."""
        min_v = float(self.cfg["wheels"].get("min_motor_voltage", 6.0))
        ok = True
        for mid in (self.left_id, self.right_id):
            raw = self.bus.read_u8(mid, ADDR_PRESENT_VOLTAGE)
            if raw is None:
                print(f"❌ 电机 {mid} 没有回应：检查电机电源和 USB 线")
                ok = False
            elif raw / 10 < min_v:
                print(f"❌ 电机 {mid} 电压只有 {raw / 10:.1f} V（至少需要 {min_v:.0f} V）："
                      "电机电源没接、没打开或电池没电，轮子不会转")
                ok = False
        return ok

    def _prepare_motors(self) -> bool:
        """Wheel mode, speed 0, torque on, for both motors. Done at start and after a reconnect."""
        ok = self.check_power()
        for mid in (self.left_id, self.right_id):
            if not self.bus.ensure_mode(mid, MODE_WHEEL):
                print(f"⚠️ 电机 {mid} 没有应答或无法切换到轮子模式，请检查接线和串口")
                ok = False
        self.bus.set_goal_velocities({self.left_id: 0, self.right_id: 0})
        for mid in (self.left_id, self.right_id):
            if not self.bus.set_torque(mid, True):
                print(f"⚠️ 电机 {mid} 无法打开扭矩，轮子不会转动，请检查电机电源")
                ok = False
        return ok

    def start(self, *, threaded: bool = True) -> None:
        if self._prepare_motors():
            volts = [self.bus.read_u8(m, ADDR_PRESENT_VOLTAGE) for m in (self.left_id, self.right_id)]
            print(f"✅ 两个轮子电机：轮子模式、扭矩已打开，电压 "
                  + " / ".join(f"{v / 10:.1f}V" for v in volts if v is not None))
        if threaded:
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        with self.lock:
            self.target = [0.0, 0.0]
            self.current = [0.0, 0.0]
        with self.io_lock:
            for _ in range(3):                     # no replies to SYNC WRITE: send a few times
                try:
                    self.bus.set_goal_velocities({self.left_id: 0, self.right_id: 0})
                except Exception:
                    pass
                time.sleep(0.02)
            # Release the wheels (torque off) so the robot can be pushed by hand after exit.
            # Set "release_on_exit": false in the config to keep them locked instead.
            if self.cfg["control"].get("release_on_exit", True):
                released = True
                for mid in (self.left_id, self.right_id):
                    try:
                        released &= self.bus.set_torque(mid, False)
                    except Exception:
                        released = False
                print("🔓 轮子已释放，可以用手推动" if released else "⚠️ 未能释放轮子扭矩（电机可能未通电）")
        self.bus.close()

    # ------------------------------------------------------------------ commands
    def set_velocity(self, v: float, w: float) -> None:
        """v: m/s forward; w: rad/s, positive = turn left (counter-clockwise)."""
        if not (math.isfinite(v) and math.isfinite(w)):
            v, w = 0.0, 0.0
        left = (v - w * self.track / 2) * self.units_per_mps
        right = (v + w * self.track / 2) * self.units_per_mps
        self.set_wheel_units(left, right)

    def set_wheel_units(self, left: float, right: float) -> None:
        """Raw speed units for each wheel, forward positive (before motor inversion)."""
        if self.preserve_ratio:
            peak = max(abs(left), abs(right))
            if peak > self.max_units:
                left, right = left * self.max_units / peak, right * self.max_units / peak
        else:
            left = max(-self.max_units, min(self.max_units, left))
            right = max(-self.max_units, min(self.max_units, right))
        with self.lock:
            self.target = [left * self.sign_left, right * self.sign_right]
            self.last_cmd = self.clock()

    def stop(self) -> None:
        self.set_wheel_units(0.0, 0.0)

    @property
    def max_speed_mps(self) -> float:
        return self.max_units / self.units_per_mps

    # ------------------------------------------------------------------ odometry
    def get_odometry(self) -> Odometry:
        with self.lock:
            return self._odom

    def reset_odometry(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        with self.lock:
            self._odom = replace(self._odom, x=x, y=y, theta=wrap_angle(theta), left_m=0.0, right_m=0.0)

    def _update_odometry(self, raw_left: int | None, raw_right: int | None, now: float) -> None:
        if raw_left is None or raw_right is None:
            self._read_failures += 1
            if self._read_failures >= 5:
                self._odom = replace(self._odom, healthy=False)
            return                        # keep the last good raw values; the next good read catches up
        self._read_failures = 0
        prev_l, prev_r = self._last_raw
        self._last_raw = [raw_left, raw_right]
        if prev_l is None or prev_r is None:
            self._odom = replace(self._odom, stamp=now, healthy=True)
            return

        def steps(new: int, old: int) -> int:     # shortest signed difference on the 0..4095 circle
            return (new - old + STEPS_PER_REV // 2) % STEPS_PER_REV - STEPS_PER_REV // 2

        dl = steps(raw_left, prev_l) * self.enc_left / STEPS_PER_REV * self.wheel_circ
        dr = steps(raw_right, prev_r) * self.enc_right / STEPS_PER_REV * self.wheel_circ
        ds, dth = (dl + dr) / 2, (dr - dl) / self.track
        o = self._odom
        mid = o.theta + dth / 2
        dt = max(now - o.stamp, 1e-6)
        self._odom = Odometry(
            x=o.x + ds * math.cos(mid), y=o.y + ds * math.sin(mid), theta=wrap_angle(o.theta + dth),
            v=ds / dt, w=dth / dt, left_m=o.left_m + dl, right_m=o.right_m + dr, stamp=now, healthy=True)

    # ------------------------------------------------------------------ control loop
    def tick(self) -> None:
        """One control step: fail-safe, smoothing, write speeds, read encoders, integrate.

        Serial I/O happens outside self.lock, so set_velocity() (called from the
        phone's WebSocket handler) never waits for the bus.
        """
        now = self.clock()
        with self.lock:
            was_active = self.failsafe_active
            self.failsafe_active = now - self.last_cmd > self.timeout
            if self.failsafe_active and not was_active and any(self.target):
                self.stats["failsafe_stops"] += 1          # stopped because commands stopped arriving
            if self.failsafe_active:
                self.target = [0.0, 0.0]
            for i in (0, 1):
                self.current[i] += (self.target[i] - self.current[i]) * self.smoothing
                if abs(self.current[i]) < self.stop_threshold:
                    self.current[i] = 0.0
            speeds = {self.left_id: self.current[0], self.right_id: self.current[1]}
        t0 = time.perf_counter()
        with self.io_lock:
            self.bus.set_goal_velocities(speeds)
            raw_l = self.bus.read_position(self.left_id)
            raw_r = self.bus.read_position(self.right_id)
        io_ms = (time.perf_counter() - t0) * 1000
        with self.lock:
            self._update_odometry(raw_l, raw_r, now)
            st = self.stats
            st["io_ms_max"] = max(st["io_ms_max"], io_ms)
            st["read_failures"] += (raw_l is None) + (raw_r is None)
            st["ticks"] += 1

    def take_stats(self) -> dict:
        """Diagnostics since the previous call: control rate, worst serial time, failures."""
        with self.lock:
            now = time.monotonic()
            st = dict(self.stats)
            elapsed = max(now - st.pop("since"), 1e-6)
            st["tick_hz"] = st.pop("ticks") / elapsed
            self.stats.update(ticks=0, io_ms_max=0.0, read_failures=0, since=now)
            return st

    def _loop(self) -> None:
        interval = 1.0 / self.tick_hz
        next_t = time.monotonic()
        while self._running:
            try:
                self.tick()
                self._errors = 0
            except Exception as e:                  # never let this thread die
                self._errors += 1
                print(f"\n⚠️ 控制循环第{self._errors}次异常: {e}")
                with self.lock:
                    self.target = [0.0, 0.0]
                    self.current = [0.0, 0.0]
                    self._last_raw = [None, None]   # encoder continuity is lost
                    self._odom = replace(self._odom, healthy=False)
                if self._errors >= 5:
                    print("🛑 连续异常过多，尝试重连串口...")
                    try:
                        self.bus.reopen()
                        self._prepare_motors()
                        print("✅ 串口重连成功")
                        self._errors = 0
                    except Exception as err:
                        print(f"❌ 重连失败: {err}，稍后重试")
                        time.sleep(0.5)
            next_t += interval
            time.sleep(max(0.0, next_t - time.monotonic()))
            if time.monotonic() - next_t > 1.0:     # fell far behind (e.g. after a reconnect)
                next_t = time.monotonic()

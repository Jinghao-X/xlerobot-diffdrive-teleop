"""Minimal Feetech STS3215 serial protocol (the servos used by SO-101 and XLeRobot).

Packet:   FF FF  ID  LEN  INSTR  PARAMS...  CHK      LEN = len(PARAMS) + 2
Status:   FF FF  ID  LEN  ERROR  PARAMS...  CHK
CHK = ~(ID + LEN + INSTR/ERROR + sum(PARAMS)) & 0xFF

Register addresses (same table LeRobot uses for STS3215):
  0x21 Operating_Mode  (EEPROM, 1 byte)  0 = position, 1 = wheel / velocity
  0x28 Torque_Enable   (1 byte)
  0x37 Lock            (1 byte)          0 = EEPROM writable, 1 = locked
  0x2E Goal_Velocity   (2 bytes)         bit 15 = sign
  0x38 Present_Position(2 bytes)         0..4095 per revolution
  0x3A Present_Velocity(2 bytes)         bit 15 = sign
  0x3E Present_Voltage (1 byte)          0.1 V
"""

from __future__ import annotations

import time

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83
BROADCAST_ID = 0xFE

ADDR_OPERATING_MODE = 0x21
ADDR_TORQUE_ENABLE = 0x28
ADDR_LOCK = 0x37
ADDR_GOAL_VELOCITY = 0x2E
ADDR_PRESENT_POSITION = 0x38
ADDR_PRESENT_VELOCITY = 0x3A
ADDR_PRESENT_VOLTAGE = 0x3E     # 1 byte, 0.1 V units

MODE_POSITION = 0
MODE_WHEEL = 1
STEPS_PER_REV = 4096


def checksum(body: list[int]) -> int:
    """body = [ID, LEN, INSTR, *params]"""
    return (~sum(body)) & 0xFF


def build_packet(motor_id: int, instruction: int, params: list[int]) -> bytes:
    body = [motor_id, len(params) + 2, instruction, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def encode_signed(value: int) -> int:
    """Sign-magnitude, bit 15 = negative (Feetech convention)."""
    value = int(value)
    return (abs(value) & 0x7FFF) | (0x8000 if value < 0 else 0)


def decode_signed(raw: int) -> int:
    return -(raw & 0x7FFF) if raw & 0x8000 else raw


class STSBus:
    """One serial bus (one control board). Not thread safe: callers hold their own lock."""

    def __init__(self, port: str, baudrate: int = 1_000_000, *, timeout: float = 0.02, serial_factory=None):
        if serial_factory is None:
            import serial  # pyserial, imported only when talking to real hardware
            serial_factory = serial.Serial
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._factory = serial_factory
        self.ser = serial_factory(port, baudrate, timeout=timeout)
        time.sleep(0.05)

    # ------------------------------------------------------------ connection
    def reopen(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = self._factory(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(0.05)

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    # ------------------------------------------------------------ raw I/O
    def write(self, motor_id: int, address: int, data: list[int]) -> None:
        """WRITE instruction. The status reply (if any) is discarded before the next read."""
        self.ser.write(build_packet(motor_id, INST_WRITE, [address, *data]))

    def read(self, motor_id: int, address: int, length: int) -> bytes | None:
        """READ instruction; returns the data bytes, or None on timeout.

        Status replies to earlier WRITEs can still arrive after reset_input_buffer()
        (USB latency), so every packet is parsed and anything that is not the reply
        we asked for (wrong ID, wrong length, bad checksum) is skipped.
        """
        self.ser.reset_input_buffer()
        self.ser.write(build_packet(motor_id, INST_READ, [address, length]))
        deadline = time.monotonic() + max(self.timeout * 3, 0.01)
        prev = None
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if not (prev == 0xFF and b[0] == 0xFF):
                prev = b[0]
                continue
            prev = None
            head = self.ser.read(2)                    # ID LEN
            if len(head) != 2:
                continue
            rid, rlen = head
            if rlen < 2 or rlen > 64:
                continue
            rest = self.ser.read(rlen)                 # ERROR, data..., CHK
            if len(rest) != rlen:
                continue
            err, data, chk = rest[0], rest[1:-1], rest[-1]
            if checksum([rid, rlen, err, *data]) != chk:
                continue
            if rid == motor_id and len(data) == length:
                return bytes(data)
            # a reply to something else (e.g. an earlier write): keep looking
        return None

    def sync_write(self, address: int, values: dict[int, list[int]]) -> None:
        """SYNC WRITE the same register on several motors in one packet; motors do not reply."""
        if not values:
            return
        n = len(next(iter(values.values())))
        params = [address, n]
        for mid, data in values.items():
            assert len(data) == n
            params += [mid, *data]
        self.ser.write(build_packet(BROADCAST_ID, INST_SYNC_WRITE, params))

    # ------------------------------------------------------------ registers
    def read_u8(self, motor_id: int, address: int) -> int | None:
        d = self.read(motor_id, address, 1)
        return None if d is None else d[0]

    def read_u16(self, motor_id: int, address: int) -> int | None:
        d = self.read(motor_id, address, 2)
        return None if d is None else d[0] | (d[1] << 8)

    def set_goal_velocity(self, motor_id: int, units: float) -> None:
        raw = encode_signed(round(units))
        self.write(motor_id, ADDR_GOAL_VELOCITY, [raw & 0xFF, (raw >> 8) & 0xFF])

    def set_goal_velocities(self, units: dict[int, float]) -> None:
        """All wheels in one SYNC WRITE: no status replies to get in the way of reads."""
        data = {}
        for mid, u in units.items():
            raw = encode_signed(round(u))
            data[mid] = [raw & 0xFF, (raw >> 8) & 0xFF]
        self.sync_write(ADDR_GOAL_VELOCITY, data)

    def read_position(self, motor_id: int) -> int | None:
        return self.read_u16(motor_id, ADDR_PRESENT_POSITION)

    def read_velocity(self, motor_id: int) -> int | None:
        raw = self.read_u16(motor_id, ADDR_PRESENT_VELOCITY)
        return None if raw is None else decode_signed(raw)

    def set_torque(self, motor_id: int, on: bool) -> bool:
        """Torque_Enable lives in RAM and is OFF after every power-up: the motor answers
        reads but does not drive, and the wheel can be pushed by hand. Returns True if
        the read-back matches."""
        value = 1 if on else 0
        for _ in range(3):
            self.write(motor_id, ADDR_TORQUE_ENABLE, [value])
            time.sleep(0.01)
            if self.read_u8(motor_id, ADDR_TORQUE_ENABLE) == value:
                return True
        return False

    def ensure_mode(self, motor_id: int, mode: int) -> bool:
        """Write Operating_Mode only when it differs; it is kept in EEPROM.

        Same sequence LeRobot uses: torque off, unlock EEPROM, write, lock, torque on.
        The motor is briefly busy after an EEPROM write, so the read-back is retried.
        Returns True if the motor answered and is now in `mode`.
        """
        current = None
        for _ in range(3):
            current = self.read_u8(motor_id, ADDR_OPERATING_MODE)
            if current is not None:
                break
        if current == mode:
            return True
        if current is None:
            return False
        self.write(motor_id, ADDR_TORQUE_ENABLE, [0])
        time.sleep(0.01)
        self.write(motor_id, ADDR_LOCK, [0])
        time.sleep(0.01)
        self.write(motor_id, ADDR_OPERATING_MODE, [mode])
        time.sleep(0.05)
        self.write(motor_id, ADDR_LOCK, [1])
        time.sleep(0.05)
        self.write(motor_id, ADDR_TORQUE_ENABLE, [1])
        time.sleep(0.01)
        for _ in range(10):
            if self.read_u8(motor_id, ADDR_OPERATING_MODE) == mode:
                return True
            time.sleep(0.02)
        return False

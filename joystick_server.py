"""Phone joystick server, drop-in replacement for xle_wheel_joystick_server_v4.py.

Same WebSocket protocol and port, so joystick4.html works unchanged: the phone sends
{"x": .., "y": ..} about every 100 ms. Driving now goes through DiffDriveBase, which
adds wheel-encoder odometry; the pose is printed while the robot moves.

  python joystick_server.py                 # uses robot_config.json
  python joystick_server.py --port fake     # no hardware: simulated motors
  python joystick_server.py --port /dev/serial/by-id/usb-...   # Raspberry Pi

Wheel odometry is also sent as JSON over local UDP (127.0.0.1:5601, ~30 Hz) for a
localizer or navigator; --odom-udp off disables it. See odom_udp.py.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import socket
import time

from base_driver import DiffDriveBase, load_config
from control_arbiter import ControlArbiter
from joystick_mapping import joystick_to_velocity
from odom_udp import OdomPublisher, parse_addr

base: DiffDriveBase | None = None
odom_pub: OdomPublisher | None = None
cfg: dict = {}
arbiter = ControlArbiter()
clients: set = set()
msg_gap = {"last": None, "max": 0.0, "count": 0}   # controller message timing, for diagnostics
# Navigation commands (UDP, from your own navigator; see README). The phone always wins: a pushed joystick
# blocks navigation for JOY_PRIORITY_S; the page's idle (0, 0) heartbeats do not stop navigation.
JOY_PRIORITY_S = 1.0
nav_state = {"last": -1e9, "joy_until": -1e9, "ignored": 0, "count": 0}


class NavProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        try:
            msg = json.loads(data)
            if msg.get("type") != "cmd":
                return
            v, w = float(msg["v"]), float(msg["w"])
        except (ValueError, KeyError, TypeError):
            return
        now = time.monotonic()
        if now < nav_state["joy_until"]:
            nav_state["ignored"] += 1
            return
        nav_state["last"] = now
        nav_state["count"] += 1
        base.set_velocity(v, w)


def peer(websocket) -> str:
    try:
        return str(websocket.remote_address[0])
    except Exception:
        return "?"


async def handler(websocket) -> None:
    clients.add(websocket)
    who = peer(websocket)
    print(f"📱 设备已连接：{who}（当前共 {len(clients)} 个连接）")
    if len(clients) > 1:
        print("   ⚠️ 有多个遥控页面同时连接；只有正在操作的那个能控制机器人")
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                x, y = float(data.get("x", 0)), float(data.get("y", 0))
                if not (math.isfinite(x) and math.isfinite(y)):
                    raise ValueError("non-finite joystick value")
            except (json.JSONDecodeError, ValueError, TypeError, AttributeError) as e:
                print(f"⚠️ 消息解析错误: {e}")
                continue
            apply, changed = arbiter.accept(websocket, x, y)
            if changed and len(clients) > 1:
                print(f"🎮 控制权 → {who}")
            if not apply:
                continue
            now = time.monotonic()
            if msg_gap["last"] is not None:
                msg_gap["max"] = max(msg_gap["max"], now - msg_gap["last"])
            msg_gap["last"] = now
            msg_gap["count"] += 1
            if x != 0 or y != 0:
                if now - nav_state["last"] < 0.5 and now >= nav_state["joy_until"]:
                    print("🎮 摇杆接管，导航暂停", flush=True)
                nav_state["joy_until"] = now + JOY_PRIORITY_S
            elif now - nav_state["last"] < 0.5:
                continue                       # navigation is driving: ignore the page's idle heartbeat
            base.set_velocity(*joystick_to_velocity(x, y, cfg))
    except Exception as e:  # connection closed or broken
        if type(e).__name__ not in ("ConnectionClosed", "ConnectionClosedOK", "ConnectionClosedError"):
            print(f"⚠️ 连接异常: {e}")
    finally:
        clients.discard(websocket)
        was_controller = arbiter.controller is websocket
        arbiter.disconnect(websocket)
        if was_controller:
            msg_gap["last"] = None
        print(f"📱 设备已断开：{who}（剩余 {len(clients)} 个连接）"
              + ("，轮子将在失联保护下自动停止" if was_controller else ""))


async def report_pose() -> None:
    """Every second: pose (when moving) plus a diagnostics line.

    控制: control loop rate (should be ~30 Hz)
    串口: worst serial write+read time in a tick (a few ms is normal)
    遥控: messages from the controlling device and the longest gap between two
          (~0.1 s is normal; gaps near 0.5 s trigger the fail-safe and make driving jerky)
    忽略: messages dropped from other connected pages (should be 0 with one page open)
    Printed only while the robot moves or something looks wrong (encoder read failures,
    a gap long enough to trigger the fail-safe, a new fail-safe stop), so an idle robot
    keeps the terminal quiet even though the page sends heartbeats every 0.1 s.
    """
    last_failsafe = 0
    warned_ignoring = False
    warned_background = False
    while True:
        await asyncio.sleep(1.0)
        o = base.get_odometry()
        st = base.take_stats()
        gap_ms = msg_gap["max"] * 1000
        ignored, arbiter.ignored = arbiter.ignored, 0
        if ignored and not warned_ignoring:
            print("ℹ️ 另一个遥控页面也连着，它的停止指令被忽略（正常操作不受影响）", flush=True)
        warned_ignoring = bool(ignored)
        moving = abs(o.v) > 0.005 or abs(o.w) > 0.02
        commanding = any(base.target)
        new_failsafe = st["failsafe_stops"] > last_failsafe
        last_failsafe = st["failsafe_stops"]
        # A long gap only matters while the robot is being driven; an idle page in the
        # background sends one heartbeat per second, which is harmless.
        background = msg_gap["count"] <= 2 and gap_ms > 800 and not commanding
        if background and not warned_background:
            print("ℹ️ 遥控页面在后台（每秒只有1条心跳）；切回前台即可继续遥控", flush=True)
        warned_background = background
        problem = st["read_failures"] or (gap_ms > 300 and commanding) or new_failsafe or not o.healthy
        if not (moving or problem):
            msg_gap.update(max=0.0, count=0)
            continue
        line = (f"{st['tick_hz']:.0f}Hz 串口{st['io_ms_max']:.0f}ms 失败{st['read_failures']} | "
                f"遥控{msg_gap['count']}条 间隔{gap_ms:.0f}ms 忽略{ignored} | 失联刹车{st['failsafe_stops']}")
        if moving:
            line += f" | x{o.x:+.2f} y{o.y:+.2f} {math.degrees(o.theta):+.0f}° {o.v:+.2f}m/s"
        elif not o.healthy:
            line += " | ⚠️ 编码器读取失败"
        print(line, flush=True)
        msg_gap.update(max=0.0, count=0)


async def publish_odometry() -> None:
    """Send each new odometry sample to the local UDP port (for a localizer / navigator)."""
    if odom_pub is None:
        return
    interval = 0.5 / base.tick_hz          # poll at twice the control rate; only new samples are sent
    while True:
        odom_pub.publish()
        await asyncio.sleep(interval)


def get_local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "无法自动获取，请手动查看（Windows: ipconfig，树莓派: hostname -I）"


async def main(args) -> None:
    global base, cfg, odom_pub
    import websockets  # imported here so the rest of the package works without it

    cfg = load_config(args.config) if args.config else load_config()
    if args.port:
        cfg["wheels"]["port"] = args.port
    print(f"📡 连接轮子串口 {cfg['wheels']['port']}...")
    base = DiffDriveBase(cfg)
    base.start()
    addr = parse_addr(args.odom_udp)
    if addr:
        odom_pub = OdomPublisher(base, addr)
        print(f"📤 里程计发送到 UDP {addr[0]}:{addr[1]}（给定位/导航程序用）")
    print(f"✅ 底盘已启动，最高速度约 {base.max_speed_mps:.2f} m/s"
          f"{'' if cfg['calibration']['calibrated'] else '（尚未标定，仅为估计值）'}")
    print("\n" + "=" * 50)
    print(f"🚀 WebSocket 服务已启动，监听端口 {cfg['joystick']['ws_port']}")
    ip = get_local_ip()
    print(f"📶 本机局域网IP / LAN IP: {ip}")
    print(f"📱 手机打开 / Open on your phone: http://{ip}:8000/joystick4.html")
    print("   （需要先用 start.sh / start.bat 启动网页服务 / needs the page server from start.sh or start.bat）")
    print("=" * 50 + "\n")
    if args.nav_port:
        loop = asyncio.get_running_loop()
        await loop.create_datagram_endpoint(NavProtocol, local_addr=("127.0.0.1", args.nav_port))
        print(f"🧭 接收导航指令：UDP 127.0.0.1:{args.nav_port}（手机摇杆优先）")
    async with websockets.serve(handler, "0.0.0.0", cfg["joystick"]["ws_port"]):
        await asyncio.gather(report_pose(), publish_odometry())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="XLeRobot 手机摇杆遥控服务")
    parser.add_argument("--config", help="配置文件路径，默认为同目录的 robot_config.json")
    parser.add_argument("--port", help="覆盖配置里的轮子串口，例如 COM3、/dev/ttyACM0 或 fake")
    parser.add_argument("--odom-udp", default="127.0.0.1:5601", help="里程计 UDP 发送地址，off 表示不发送")
    parser.add_argument("--nav-port", type=int, default=5603, help="接收导航速度指令的本机 UDP 端口，0 表示不接收")
    args = parser.parse_args()
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\n🛑 收到停止指令")
    finally:
        if odom_pub:
            odom_pub.close()
        if base:
            base.close()
            print("🔌 已安全停止并关闭串口")

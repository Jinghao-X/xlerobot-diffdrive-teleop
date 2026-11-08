"""Measure the numbers the base driver needs, and write them into robot_config.json.

  python calibrate_base.py check      # 1. encoder directions (1 second, slow)
  python calibrate_base.py straight   # 2. speed units per m/s, and the real wheel diameter
  python calibrate_base.py rotate     # 3. effective track width (distance between wheel contact points)

Run them in this order on a flat floor with at least 1.5 m of free space in front
and a clear circle about 0.8 m across for the rotation.
Keep a hand near the robot; Ctrl+C stops it at any time.
Add --port fake to try everything without hardware.
"""

from __future__ import annotations

import argparse
import math
import time

from base_driver import DiffDriveBase, load_config, save_config


def drive(base: DiffDriveBase, left: float, right: float, seconds: float) -> None:
    """Hold wheel speeds (units, forward +) for `seconds`, feeding the fail-safe, then stop."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        base.set_wheel_units(left, right)
        time.sleep(0.05)
    base.stop()
    time.sleep(1.0)            # let smoothing bring the wheels to rest and odometry settle


def ask_float(prompt: str) -> float | None:
    text = input(prompt).strip()
    if not text:
        return None
    try:
        value = float(text)
    except ValueError:
        print("  输入不是数字，跳过")
        return None
    return value if math.isfinite(value) and value != 0 else None


def confirm(prompt: str) -> bool:
    return input(prompt + " [y/N] ").strip().lower() in ("y", "yes", "是")


def cmd_check(base: DiffDriveBase, cfg: dict, args) -> bool:
    print(f"机器人将以低速向前开 {args.seconds:.0f} 秒。")
    if not confirm("准备好了吗？"):
        return False
    base.reset_odometry()
    drive(base, args.units, args.units, args.seconds)
    o = base.get_odometry()
    print(f"编码器读数：左轮 {o.left_m * 100:+.1f} cm，右轮 {o.right_m * 100:+.1f} cm")
    if not o.healthy:
        print("❌ 读不到编码器，请检查接线、串口和电机 ID")
        return False
    if not confirm("机器人刚才是向前走的吗？"):
        print("请先修正 invert_left / invert_right，让摇杆向前推时机器人向前走，再运行本检查。")
        return False
    changed = False
    for side, dist in (("left", o.left_m), ("right", o.right_m)):
        if dist < -0.005:
            key = f"encoder_flip_{side}"
            cfg["wheels"][key] = not cfg["wheels"].get(key, False)
            print(f"  {'左' if side == 'left' else '右'}轮编码器方向与指令相反，将设置 {key} = {cfg['wheels'][key]}")
            changed = True
        elif abs(dist) <= 0.005:
            print(f"  ⚠️ {'左' if side == 'left' else '右'}轮几乎没有读数，请确认轮子在转、电机 ID 正确")
    if not changed:
        print("✅ 两个轮子的编码器方向都正确")
    return changed


def cmd_straight(base: DiffDriveBase, cfg: dict, args) -> bool:
    print(f"机器人将以速度值 {args.units:.0f} 直行 {args.seconds:.0f} 秒。在地上用胶带标出车身某一点的起始位置。")
    if not confirm("准备好了吗？"):
        return False
    base.reset_odometry()
    end = time.monotonic() + args.seconds
    t_steady, d_steady = None, None
    while time.monotonic() < end:
        base.set_wheel_units(args.units, args.units)
        o = base.get_odometry()
        if t_steady is None and time.monotonic() > end - args.seconds + 1.5:   # after the speed ramp
            t_steady, d_steady = o.stamp, (o.left_m + o.right_m) / 2
        time.sleep(0.05)
    o = base.get_odometry()
    t_end, d_end = o.stamp, (o.left_m + o.right_m) / 2
    base.stop()
    time.sleep(1.0)
    o = base.get_odometry()
    d_enc = (o.left_m + o.right_m) / 2
    if not o.healthy or t_steady is None or t_end - t_steady < 1.0:
        print("❌ 编码器数据不足，请先运行 check")
        return False
    v_enc = (d_end - d_steady) / (t_end - t_steady)
    print(f"编码器测得：总距离 {d_enc * 100:.1f} cm，匀速段速度 {v_enc:.3f} m/s（按轮径 "
          f"{cfg['geometry']['wheel_diameter_m'] * 1000:.0f} mm 计算）")
    if abs(o.left_m - o.right_m) > 0.05 * max(abs(d_enc), 1e-6):
        print(f"  ⚠️ 两轮距离相差较大：左 {o.left_m * 100:.1f} cm，右 {o.right_m * 100:.1f} cm，可能打滑或两轮直径不同")
    tape = ask_float("用尺子量出实际前进了多少厘米（直接回车跳过，只标定速度）：")
    ratio = tape / 100 / d_enc if tape else 1.0
    if tape and not 0.8 < ratio < 1.25:
        print(f"  ⚠️ 尺量与编码器差了 {abs(ratio - 1) * 100:.0f}%，超出合理范围，请重测，本次不修改轮径")
        ratio = 1.0
    new_diameter = cfg["geometry"]["wheel_diameter_m"] * ratio
    new_k = args.units / (v_enc * ratio)
    print(f"结果：每 1 m/s 对应速度值 {new_k:.0f}；最高速度值 {cfg['wheels']['max_units']} ≈ "
          f"{cfg['wheels']['max_units'] / new_k:.2f} m/s；有效轮径 {new_diameter * 1000:.1f} mm")
    if not confirm("写入配置文件？"):
        return False
    cfg["calibration"]["speed_units_per_mps"] = round(new_k, 1)
    cfg["calibration"]["calibrated"] = True
    cfg["geometry"]["wheel_diameter_m"] = round(new_diameter, 5)
    return True


def cmd_rotate(base: DiffDriveBase, cfg: dict, args) -> bool:
    print(f"机器人将原地逆时针旋转 {args.seconds:.0f} 秒。先在地上贴一条胶带，对准车头方向作为 0°。")
    if not confirm("准备好了吗？"):
        return False
    base.reset_odometry()
    drive(base, -args.units, args.units, args.seconds)
    o = base.get_odometry()
    turned = o.right_m - o.left_m                       # = theta * track
    print(f"编码器测得：按当前轮距 {cfg['geometry']['track_width_m'] * 1000:.0f} mm 计算，"
          f"转了 {math.degrees(turned / cfg['geometry']['track_width_m']):.0f}°")
    deg = ask_float("实际转了多少度？（例如转了 2 圈又 90° 就填 810）：")
    if not deg:
        return False
    new_track = turned / math.radians(deg)
    if not 0.7 < new_track / cfg["geometry"]["track_width_m"] < 1.4:
        print(f"  ⚠️ 算出的有效轮距 {new_track * 1000:.0f} mm 与尺量值相差太大，请检查后重测")
        return False
    print(f"结果：有效轮距 {new_track * 1000:.1f} mm")
    if not confirm("写入配置文件？"):
        return False
    cfg["geometry"]["track_width_m"] = round(new_track, 4)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="XLeRobot 底盘标定")
    parser.add_argument("step", choices=["check", "straight", "rotate"])
    parser.add_argument("--units", type=float, default=None, help="速度值（默认 check 200，straight 800，rotate 600）")
    parser.add_argument("--seconds", type=float, default=None,
                        help="持续时间（默认 straight 约走 0.8 m，rotate 约转 2 圈，按当前估计速度计算）")
    parser.add_argument("--port", help="覆盖配置里的轮子串口，可填 fake")
    parser.add_argument("--config", help="配置文件路径")
    args = parser.parse_args()

    cfg = load_config(args.config) if args.config else load_config()
    k = cfg["calibration"]["speed_units_per_mps"]
    args.units = args.units or {"check": 200, "straight": 800, "rotate": 600}[args.step]
    if not args.seconds:
        wheel_mps = args.units / k
        if args.step == "check":
            args.seconds = 1.0
        elif args.step == "straight":
            args.seconds = min(30.0, max(3.0, 0.8 / wheel_mps))
        else:
            omega = 2 * wheel_mps / cfg["geometry"]["track_width_m"]
            args.seconds = min(60.0, max(5.0, 4 * math.pi / omega))
        print(f"预计持续 {args.seconds:.0f} 秒（按当前估计速度计算）")
    original_port = cfg["wheels"]["port"]
    if args.port:
        cfg["wheels"]["port"] = args.port
    if args.units > cfg["wheels"]["max_units"]:
        raise SystemExit("速度值不能超过 max_units")
    base = DiffDriveBase(cfg)
    base.start()
    try:
        changed = {"check": cmd_check, "straight": cmd_straight, "rotate": cmd_rotate}[args.step](base, cfg, args)
    except KeyboardInterrupt:
        print("\n🛑 已中止")
        changed = False
    finally:
        base.close()
    if changed:
        cfg["wheels"]["port"] = original_port      # never save a --port override
        save_config(cfg, args.config) if args.config else save_config(cfg)
        print("✅ 已写入配置文件")


if __name__ == "__main__":
    main()

# XLeRobot 0.4 Differential-Drive Base: Teleop + Odometry

<p align="center">
  <img src="docs/demo.gif" height="480" alt="XLeRobot 0.4 driven with the phone joystick">
  &nbsp;&nbsp;
  <img src="docs/joystick.jpg" height="480" alt="Phone joystick page">
</p>
<p align="center"><sub>Driving with the phone joystick (2× speed) · the joystick page</sub></p>

Drive the **XLeRobot 0.4.0 two-wheel base** from your phone. Plug the robot's wheel board into a computer (Windows, Mac, Linux or a Raspberry Pi), start one script, and open a web page on your phone. No app to install.

It also gives you:

- **Wheel odometry**: the robot's position (x, y), heading and measured speed, read from the motor encoders.
- **A simple velocity interface**: `set_velocity(v m/s, ω rad/s)`, shared by the joystick and any program you write.
- **UDP ports** for your own navigation code: send speed commands in, get odometry out.
- **Calibration scripts** that measure your robot's real wheel size and speed.
- **A fake-motor mode**, so you can try everything without a robot.

The official XLeRobot docs cover the 0.3 omni-wheel base; this project covers the 0.4 two-wheel version.

> Terminal messages and the phone page are in Chinese. The code, comments and this README are in English.

## Contents

1. [What you need](#1-what-you-need)
2. [Install](#2-install)
3. [Try it without the robot](#3-try-it-without-the-robot)
4. [Connect the robot](#4-connect-the-robot)
5. [Drive with your phone](#5-drive-with-your-phone)
6. [Calibrate](#6-calibrate-recommended)
7. [UDP interfaces for your own code](#7-udp-interfaces-for-your-own-code)
8. [Configuration](#8-configuration)
9. [Troubleshooting](#9-troubleshooting)
10. [Files](#10-files)

## 1. What you need

- **XLeRobot 0.4.0 base**: 2 × Feetech STS3215 wheel motors, IDs **7 (left)** and **8 (right)**, on a USB serial board.
- **The motor power supply connected** (12 V). USB alone powers the board but not the motors.
- **A computer** next to or on the robot: Windows 10/11, macOS, Linux, or a Raspberry Pi 5.
- **Python 3.9 or newer.**
- **A phone on the same Wi-Fi** as the computer.

## 2. Install

### Windows

1. Install Python from [python.org](https://www.python.org/downloads/). On the first installer screen, tick **"Add python.exe to PATH"**.
2. Download this project: click the green **Code** button above → **Download ZIP**, then unzip it. Or use `git clone` if you have Git.
3. Open the unzipped folder in File Explorer, click the address bar, type `cmd` and press Enter. A Command Prompt opens in that folder.
4. Run:

```bat
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

If `python` is not found, use `py` instead (for example `py -m venv venv`).

### macOS, Linux, Raspberry Pi

```bash
git clone https://github.com/Jinghao-X/xlerobot-diffdrive-teleop.git
cd xlerobot-diffdrive-teleop
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

**Each time** you open a new terminal later, go into the folder and activate the environment again: `venv\Scripts\activate` on Windows, or `source venv/bin/activate` on Mac/Linux/Pi.

## 3. Try it without the robot

```bash
python joystick_server.py --port fake
```

`--port fake` uses simulated motors. You should see `✅ 底盘已启动` ("base started"). Press **Ctrl+C** to stop.

Optional, to run the tests:

```bash
python tests/test_base.py
python tests/test_control.py
python tests/test_udp.py
```

## 4. Connect the robot

Plug the wheel board into the computer with USB, turn on the motor power, then find the board's serial port name.

| System | How to find the port | Looks like |
| --- | --- | --- |
| **Windows** | Open **Device Manager** → **Ports (COM & LPT)**. Unplug and re-plug the board to see which entry appears. | `COM3` |
| **Raspberry Pi / Linux** | Run `ls /dev/serial/by-id/` | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_...-if00` |
| **macOS** | Run `ls /dev/cu.*` | `/dev/cu.usbmodem...`, `/dev/cu.usbserial-...` or `/dev/cu.wchusbserial...` |

Open `robot_config.json` in a text editor and put the port name in `wheels.port`:

```json
"wheels": {
  "port": "COM3",
```

Don't want to edit the file? Pass the port when starting instead, e.g. `--port COM3`.

**Windows:** if no port appears in Device Manager, or it shows a yellow warning sign, install the CH340 USB-serial driver (search "CH340 driver" on the manufacturer's site, WCH).

**Raspberry Pi / Linux:** give your user serial access once, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

On Linux, use the `/dev/serial/by-id/...` name rather than `/dev/ttyACM0`, because `ttyACM` numbers can swap after re-plugging.

## 5. Drive with your phone

**Before the first drive, lift the robot so the wheels are off the ground**, until you've checked the directions are right.

**Start**

- **Windows:** double-click `start.bat`, or run `start.bat` in the Command Prompt. Add a port if needed: `start.bat --port COM3`.
- **Mac / Linux / Pi:** `./start.sh` (or `./start.sh --port /dev/cu.usbmodem...`).

The script starts two things: a small web server that serves the joystick page (port 8000), and the robot control server (port 8765).

**Open the page on your phone**

The terminal prints a line like:

```
📱 手机打开 / Open on your phone: http://192.168.1.23:8000/joystick4.html
```

Open that address in your phone's browser. The phone must be on the **same Wi-Fi** as the computer.

**Windows:** the first time, Windows Firewall asks whether to allow Python. Tick **Private networks** and click **Allow**, or the phone can't connect.

**Using the page**

- The dot at the top turns **green** when connected. Red means "connecting…".
- **Hold and drag the joystick**: up = forward, down = backward, left/right = turn.
- **Let go** and the robot stops. Switching to another app or locking the phone also stops it.
- The red **急停** button is an **emergency stop**.
- If the phone loses connection, the robot stops by itself within 0.5 s.
- Keep **only one** joystick page open. A second page (for example a forgotten laptop tab) can make driving jerky.

**Wrong direction?**

- Robot goes backward when you push forward → in `robot_config.json`, flip both `invert_left` and `invert_right` (`true` ↔ `false`).
- Robot spins when you push forward → flip just one of them.

**Stop the program:** press **Ctrl+C** in the terminal. The wheels are released, so you can push the robot by hand.

## 6. Calibrate (recommended)

Calibration measures your robot's real wheel diameter, speed and track width, so the speeds and odometry are accurate. The default values were measured on the author's robot.

You need a flat floor, about 1.5 m of free space ahead, and a ~0.8 m circle for turning. Keep a hand near the robot; **Ctrl+C stops it at any time**. Run the three steps in order:

```bash
python calibrate_base.py check      # 1. checks encoder direction, ~1 s (lift the wheels the first time)
python calibrate_base.py straight   # 2. drives straight, ~10 s; you measure the distance with a tape measure
python calibrate_base.py rotate     # 3. turns in place ~2 times; you count the degrees
```

Each step explains what it will do, asks before moving, shows the result, and asks before saving to `robot_config.json`. The prompts are in Chinese:

- `[y/N]` questions: type `y` and press Enter to go ahead, or just Enter to cancel.
- `straight` asks how many **centimetres** the robot actually moved (`实际前进了多少厘米`). Measure it and type the number.
- `rotate` asks how many **degrees** it actually turned (`实际转了多少度`). For example, 2 full turns plus 90° = `810`.

## 7. UDP interfaces for your own code

While `joystick_server.py` is running, your own program on the same computer can drive the robot and read its position. Both ports listen on **127.0.0.1 only**, so other devices on the network can't drive the robot. Each message is one JSON object per UDP packet.

**Odometry out: `127.0.0.1:5601`, about 30 times per second**

```json
{"type": "odom", "boot": "1730000000.123", "seq": 42, "t": 12345.678,
 "x": 0.52, "y": -0.03, "theta": 0.12, "v": 0.10, "w": 0.0,
 "left_m": 0.51, "right_m": 0.53, "healthy": true,
 "wheel_diameter_m": 0.1217, "track_width_m": 0.5129}
```

- `x`, `y`: metres; `theta`: radians, counter-clockwise positive. Measured from where the robot was when the program started; x points forward.
- `t`: `time.monotonic()` at the encoder read, useful for matching with camera timestamps.
- `boot`: changes every time the program restarts, which resets the position to (0, 0, 0).
- `healthy: false` means encoder reads are failing.
- Change the address with `--odom-udp HOST:PORT`, or turn it off with `--odom-udp off`.

**Speed commands in: `127.0.0.1:5603`**

```json
{"type": "cmd", "v": 0.15, "w": 0.0}
```

- `v`: forward speed in m/s (negative = backward). `w`: turning speed in rad/s (positive = left).
- Send at least **10 times per second**. If commands stop for 0.5 s, the robot stops.
- **The phone always wins:** while someone pushes the joystick, UDP commands are ignored until 1 s after they let go.
- Change the port with `--nav-port`, or turn it off with `--nav-port 0`.

**Example: drive forward slowly for 2 seconds and print the position**

```python
import json, socket, time

cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
odom = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
odom.bind(("127.0.0.1", 5601))

end = time.monotonic() + 2.0
while time.monotonic() < end:
    cmd.sendto(json.dumps({"type": "cmd", "v": 0.1, "w": 0.0}).encode(), ("127.0.0.1", 5603))
    o = json.loads(odom.recv(4096))
    print(f"x={o['x']:.2f} m  y={o['y']:.2f} m  heading={o['theta']:.2f} rad")
    time.sleep(0.05)
```

Run it in a second terminal while the server is running.

## 8. Configuration

All settings are in `robot_config.json`. The ones you're most likely to change:

| Setting | What it does |
| --- | --- |
| `wheels.port` | Serial port of the wheel board (`COM3`, `/dev/serial/by-id/...`, or `fake`) |
| `wheels.left_id`, `right_id` | Motor IDs (default 7 and 8) |
| `wheels.invert_left`, `invert_right` | Flip a motor's direction |
| `wheels.max_units` | Top speed (2000 ≈ 0.19 m/s with the default calibration) |
| `wheels.min_motor_voltage` | Warn at startup if the motor supply is below this voltage |
| `control.command_timeout_s` | Stop if no command arrives for this long (default 0.5 s) |
| `control.smoothing` | Acceleration smoothing, 0–1; lower = gentler starts and stops |
| `control.release_on_exit` | Release the wheels when the program exits, so the robot can be pushed |
| `joystick.deadzone`, `curve_exponent` | How the joystick feels near the centre and at full push |
| `joystick.ws_port` | Control server port (default 8765) |

Calibration writes `geometry` and `calibration.speed_units_per_mps` for you.

## 9. Troubleshooting

**The phone page stays red ("连接中…", connecting)**
- The phone and computer must be on the same Wi-Fi. Guest networks often block devices from seeing each other.
- **Windows:** allow Python through Windows Firewall (Private networks). If you missed the prompt: Start → "Allow an app through Windows Firewall" → find Python → tick Private.
- Check the terminal is still running and shows no error.

**Can't open the serial port**
- Check the port name (section 4).
- Close any other program using the board: another copy of this script, Feetech's debug tool, Arduino serial monitor, etc. Only one program can use the port at a time.
- **Linux/Pi:** "Permission denied" → run the `usermod` command in section 4 and log in again.

**Motors don't move, or move weakly, and the startup voltage reads about 5 V**
The motor power isn't connected; the board is running on USB power only. Check the 12 V supply. The program warns at startup when the voltage is low.

**Driving is jerky or stutters**
Usually two joystick pages are open. Close the extra one. The terminal's status line shows `忽略` (ignored) > 0 when a second page is sending messages.

**Motors don't respond after power-up**
The motors turn their torque off after a power cycle. This program turns it back on at startup; just restart it. If another tool on the same bus turned torque off, restart this program.

**Odometry jumps, or encoder read failures**
Make sure only one program is using the serial port.

**Robot drifts to one side when driving straight**
Check both wheel motors are the same model (same voltage version and gear ratio), then run `python calibrate_base.py straight` again.

## 10. Files

| File | Purpose |
| --- | --- |
| `start.bat` / `start.sh` | Start everything (Windows / Mac, Linux, Pi) |
| `joystick_server.py` | Main program: phone joystick server + UDP ports |
| `joystick4.html` | The phone joystick page |
| `calibrate_base.py` | Calibration: `check`, `straight`, `rotate` |
| `robot_config.json` | All settings |
| `base_driver.py` | Two-wheel drive control, fail-safe, odometry |
| `sts_bus.py` | STS3215 serial protocol |
| `joystick_mapping.py` | Joystick position → speed and turn rate |
| `control_arbiter.py` | Lets only one phone page control the robot at a time |
| `odom_udp.py` | Sends odometry over UDP |
| `fake_sts.py` | Simulated motors for `--port fake` and the tests |
| `tests/` | Tests; no robot or extra packages needed |

## Contributing

Issues and pull requests are welcome. This project is independent of, and built for, [XLeRobot](https://github.com/Vector-Wangel/XLeRobot).

## License

[Apache License 2.0](LICENSE), the same as XLeRobot.

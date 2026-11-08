#!/bin/bash
# Start the phone joystick: web page (port 8000) + control server (port 8765). Ctrl+C stops both.
# 启动手机遥控：网页服务（8000 端口）+ 遥控服务（8765 端口）。Ctrl+C 同时关闭两者。
cd "$(dirname "$0")"
[ -f venv/bin/activate ] && source venv/bin/activate
python3 -m http.server 8000 > /dev/null 2>&1 &
HTTP_PID=$!
trap "kill $HTTP_PID 2>/dev/null" EXIT
echo "📱 Open on your phone / 手机打开：http://$(hostname -I | awk '{print $1}'):8000/joystick4.html"
python3 joystick_server.py "$@"

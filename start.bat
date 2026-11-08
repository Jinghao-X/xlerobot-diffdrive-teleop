@echo off
REM Start the phone joystick on Windows: web page (port 8000) + control server (port 8765).
REM Press Ctrl+C to stop both.
cd /d "%~dp0"
if exist venv\Scripts\activate.bat call venv\Scripts\activate.bat
start "" /b python -m http.server 8000 >nul 2>&1
python joystick_server.py %*
pause

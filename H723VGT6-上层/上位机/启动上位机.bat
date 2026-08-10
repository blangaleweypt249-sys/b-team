@echo off
cd /d "%~dp0"
python -c "import serial" >nul 2>&1
if not errorlevel 1 goto launch
python -m pip install -r requirements.txt
if not errorlevel 1 goto launch
echo pyserial 安装失败，请检查网络后重试。
pause
exit /b 1
:launch
start "" pythonw.exe main.py
exit /b 0

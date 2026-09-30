@echo off
rem Launch the HS force-sensor Modbus-TCP host application (GUI).
rem Optional args, e.g.:  run_host.bat --host 192.168.1.100 --order CDAB
cd /d "%~dp0"
python hs_app.py %*
echo.
echo [hs_app] exited.
pause

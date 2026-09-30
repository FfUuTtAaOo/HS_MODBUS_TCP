@echo off
rem Launch the sensor simulator (no hardware needed).
rem Optional args, e.g.:  run_simulator.bat --port 502 --rate 500 --order CDAB
rem Default: listen on 0.0.0.0:502, 500 Hz, float32 CDAB.
cd /d "%~dp0"
python hs_simulator.py %*
echo.
echo [simulator] exited.
pause

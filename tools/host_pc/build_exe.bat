@echo off
rem ============================================================
rem  Build a standalone single-file hs_host.exe
rem  Target PC needs NO Python installed.
rem  Requires: Python 3.9+ WITH tkinter + PyInstaller (auto-installed).
rem ============================================================
setlocal
cd /d "%~dp0"

set "MANAGED=%USERPROFILE%\.workbuddy\binaries\python\envs\default\Scripts\python.exe"
if exist "%MANAGED%" (
    set "PYPATH=%MANAGED%"
) else (
    where python >nul 2>&1
    if errorlevel 1 (
        echo [ERROR] Python not found. Install Python 3.9+ with tkinter first.
        pause
        exit /b 1
    )
    set "PYPATH=python"
)

rem --- tkinter is mandatory: PyInstaller cannot bundle what the interpreter lacks
"%PYPATH%" -c "import tkinter" 2>nul
if errorlevel 1 (
    echo [ERROR] This Python has no tkinter module.
    echo         Pick another interpreter, or the exe will crash on startup
    echo         with: ModuleNotFoundError: No module named 'tkinter'
    pause
    exit /b 1
)

rem --- PyInstaller
"%PYPATH%" -m PyInstaller --version >nul 2>&1
if errorlevel 1 (
    echo [INFO] Installing PyInstaller ...
    "%PYPATH%" -m pip install pyinstaller
    if errorlevel 1 (
        echo [ERROR] pip install pyinstaller failed.
        pause
        exit /b 1
    )
)

rem --- icon
if not exist "hs_host.ico" "%PYPATH%" make_icon.py

echo.
echo [INFO] Building single-file exe ...
"%PYPATH%" -m PyInstaller --onefile --windowed --name hs_host ^
    --icon hs_host.ico --version-file version_info.py --clean --noconfirm hs_app.py
if errorlevel 1 (
    echo [ERROR] Build failed.
    pause
    exit /b 1
)

echo.
echo [OK] Done: %~dp0dist\hs_host.exe
echo      Size is about 11 MB with tkinter bundled. Copy this one file anywhere.
echo.
pause

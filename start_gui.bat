@echo off
chcp 65001 >nul
REM Basketball Hoop Calibration Tool - GUI Launcher (Conda)

echo ========================================
echo   Basketball Hoop Calibration Tool
echo ========================================
echo.

REM Change to script directory
cd /d "%~dp0"

REM Set conda environment name
set CONDA_ENV=basketball_env

REM Initialize conda for batch script
call conda activate %CONDA_ENV% 2>nul
if %errorlevel% neq 0 (
    echo [ERROR] Failed to activate conda environment: %CONDA_ENV%
    echo [ERROR] Please check if conda is installed and environment exists
    echo.
    echo To create the environment, run:
    echo   conda create -n %CONDA_ENV% python=3.8
    echo   conda activate %CONDA_ENV%
    echo   pip install -r requirements.txt
    pause
    exit /b 1
)

echo [OK] Activated conda environment: %CONDA_ENV%

REM Check if gui.py exists
if not exist "gui.py" (
    echo [ERROR] gui.py not found!
    pause
    exit /b 1
)

echo [OK] Starting GUI...
echo.

REM Start GUI
python gui.py

REM If GUI exits with error
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] GUI exited with error code: %errorlevel%
    pause
)

REM Deactivate conda environment
call conda deactivate

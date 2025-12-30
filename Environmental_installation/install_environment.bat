@echo off
REM Basketball Calibration Tool - Environment Installation Script

echo ========================================
echo   Environment Installation Script
echo ========================================
echo.

REM Change to script directory
cd /d "%~dp0"

REM Check if conda is installed
where conda >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Conda not found! Please install Anaconda or Miniconda first.
    pause
    exit /b 1
)

echo [OK] Conda found
echo.

REM Step 1: Create conda environment
echo ========================================
echo Step 1/4: Creating conda environment
echo ========================================
echo.
echo Creating environment: basketball_env (Python 3.10)
echo.

call conda create -n basketball_env python=3.10 -y
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Failed to create conda environment
    pause
    exit /b 1
)

echo.
echo [OK] Environment created successfully
echo.

REM Step 2: Activate environment
echo ========================================
echo Step 2/4: Activating environment
echo ========================================
echo.

call conda activate basketball_env
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Failed to activate environment
    pause
    exit /b 1
)

echo [OK] Environment activated
echo.

REM Step 3: Install requirements from requirements.txt
echo ========================================
echo Step 3/4: Installing requirements
echo ========================================
echo.

if not exist "requirements.txt" (
    echo [ERROR] requirements.txt not found!
    pause
    exit /b 1
)

echo Installing packages from requirements.txt...
echo.

pip install -r requirements.txt
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Failed to install requirements
    pause
    exit /b 1
)

echo.
echo [OK] Requirements installed successfully
echo.

REM Step 4: Install pyorbbecsdk wheel file
echo ========================================
echo Step 4/4: Installing pyorbbecsdk
echo ========================================
echo.

if not exist "pyorbbecsdk-2.0.9-cp310-cp310-win_amd64.whl" (
    echo [WARNING] pyorbbecsdk-2.0.9-cp310-cp310-win_amd64.whl not found!
    echo [WARNING] Skipping pyorbbecsdk installation
    echo.
) else (
    echo Installing pyorbbecsdk...
    echo.

    pip install pyorbbecsdk-2.0.9-cp310-cp310-win_amd64.whl
    if %errorlevel% neq 0 (
        echo.
        echo [ERROR] Failed to install pyorbbecsdk
        pause
        exit /b 1
    )

    echo.
    echo [OK] pyorbbecsdk installed successfully
    echo.
)

REM Installation complete
echo ========================================
echo   Installation Complete!
echo ========================================
echo.
echo Environment name: basketball_env
echo.
echo To activate the environment, run:
echo   conda activate basketball_env
echo.
echo To deactivate, run:
echo   conda deactivate
echo.

@echo off
REM Basketball Calibration Tool - Environment Installation Script

echo ========================================
echo   Environment Installation Script
echo ========================================
echo.

REM Change to script directory
cd /d "%~dp0"

REM Check if this is a restarted instance
if "%1"=="CONDA_INITIALIZED" goto run_installation

REM Step 1: Check if conda is available in PATH
where conda >nul 2>&1
if %errorlevel% equ 0 goto run_installation

REM Step 2: Try to initialize conda in current window
echo [INFO] Conda not found in PATH, attempting to initialize...
echo.
call :init_conda
if %errorlevel% equ 0 goto run_installation

REM Step 3: Restart in new Anaconda Prompt window
if defined CONDA_ROOT (
    echo [INFO] Restarting in Anaconda Prompt environment...
    echo [INFO] A new window will open to continue installation.
    echo.
    start "Basketball Environment Installation" cmd /K ""%CONDA_ROOT%\Scripts\activate.bat" && cd /d "%~dp0" && "%~f0" CONDA_INITIALIZED"
    exit
)

REM Step 4: All methods failed
echo ========================================
echo [ERROR] Cannot initialize conda!
echo ========================================
echo.
echo Conda was not found in the following locations:
echo   - %USERPROFILE%\anaconda3
echo   - %USERPROFILE%\miniconda3
echo   - C:\ProgramData\Anaconda3
echo   - C:\ProgramData\Miniconda3
echo   - %LOCALAPPDATA%\anaconda3
echo   - %LOCALAPPDATA%\miniconda3
echo.
echo Please do one of the following:
echo   1. Run this script in "Anaconda Prompt" (recommended)
echo   2. Add conda to your system PATH
echo   3. Install Anaconda/Miniconda if not installed
echo.
pause
exit /b 1

REM ============================================================================
REM Function: Initialize conda by searching common installation paths
REM ============================================================================
:init_conda
echo [INFO] Searching for conda installation...

REM Search in user profile directory
if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
    set CONDA_ROOT=%USERPROFILE%\anaconda3
    echo [OK] Found conda at: %CONDA_ROOT%
    call "%CONDA_ROOT%\Scripts\activate.bat"
    exit /b 0
)
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    set CONDA_ROOT=%USERPROFILE%\miniconda3
    echo [OK] Found conda at: %CONDA_ROOT%
    call "%CONDA_ROOT%\Scripts\activate.bat"
    exit /b 0
)

REM Search in ProgramData directory
if exist "C:\ProgramData\Anaconda3\Scripts\activate.bat" (
    set CONDA_ROOT=C:\ProgramData\Anaconda3
    echo [OK] Found conda at: %CONDA_ROOT%
    call "%CONDA_ROOT%\Scripts\activate.bat"
    exit /b 0
)
if exist "C:\ProgramData\Miniconda3\Scripts\activate.bat" (
    set CONDA_ROOT=C:\ProgramData\Miniconda3
    echo [OK] Found conda at: %CONDA_ROOT%
    call "%CONDA_ROOT%\Scripts\activate.bat"
    exit /b 0
)

REM Search in LocalAppData directory
if exist "%LOCALAPPDATA%\anaconda3\Scripts\activate.bat" (
    set CONDA_ROOT=%LOCALAPPDATA%\anaconda3
    echo [OK] Found conda at: %CONDA_ROOT%
    call "%CONDA_ROOT%\Scripts\activate.bat"
    exit /b 0
)
if exist "%LOCALAPPDATA%\miniconda3\Scripts\activate.bat" (
    set CONDA_ROOT=%LOCALAPPDATA%\miniconda3
    echo [OK] Found conda at: %CONDA_ROOT%
    call "%CONDA_ROOT%\Scripts\activate.bat"
    exit /b 0
)

REM Conda not found
echo [WARNING] Conda not found in common locations
exit /b 1

REM ============================================================================
REM Main Installation Process
REM ============================================================================
:run_installation

echo [OK] Conda found and ready
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

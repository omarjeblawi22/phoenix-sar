@echo off
:: PHOENIX IMPPF Runner — Windows
:: Double-click this file to run the particle filter on collected RTT data.
:: Requires: Python 3 with numpy and matplotlib installed.
::   pip install numpy matplotlib

setlocal EnableDelayedExpansion

echo ============================================================
echo  PHOENIX IMPPF — AP Localisation Particle Filter
echo ============================================================
echo.

:: Check Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python not found. Install Python 3 from https://python.org
    pause
    exit /b 1
)

:: Check dependencies
python -c "import numpy, matplotlib" >nul 2>&1
if errorlevel 1 (
    echo Installing required packages...
    pip install numpy matplotlib
)

:: Prompt for dataset directory
echo Enter the path to your dataset folder (e.g. C:\Users\you\run01)
echo Or press Enter to use the example path below:
echo   C:\Users\omarj\OneDrive\Desktop\FINAL GRAD\run01
echo.
set /p DATA_DIR="Dataset path: "
if "!DATA_DIR!"=="" set DATA_DIR=C:\Users\omarj\OneDrive\Desktop\FINAL GRAD\run01

:: Check files exist
if not exist "!DATA_DIR!\rtt.csv" (
    echo ERROR: rtt.csv not found in !DATA_DIR!
    echo Make sure you transferred the data from the Pi first.
    pause
    exit /b 1
)

echo.
echo Dataset: !DATA_DIR!
echo.

:: Prompt for key parameters
echo === PARAMETERS ===
echo (Press Enter to use defaults shown in brackets)
echo.

set /p OFFSET_B="FTM offset-b [5.5]: "
if "!OFFSET_B!"=="" set OFFSET_B=5.5

set /p SIGMA_LOS="sigma-los [0.5]: "
if "!SIGMA_LOS!"=="" set SIGMA_LOS=0.5

set /p SIGMA_NLOS="sigma-nlos [1.5]: "
if "!SIGMA_NLOS!"=="" set SIGMA_NLOS=1.5

set /p BIAS="bias [0.8]: "
if "!BIAS!"=="" set BIAS=0.8

set /p ROUGH="rough [0.3]: "
if "!ROUGH!"=="" set ROUGH=0.3

set /p LIKELIHOOD="likelihood gaussian/student_t [student_t]: "
if "!LIKELIHOOD!"=="" set LIKELIHOOD=student_t

set SAVE_PREFIX=!DATA_DIR!\result

echo.
echo Running IMPPF...
echo.

python "%~dp0\04_imppf_prototype.py" ^
  --map      "!DATA_DIR!\map.npy" ^
  --map-meta "!DATA_DIR!\map_meta.json" ^
  --traj     "!DATA_DIR!\trajectory.csv" ^
  --rtt      "!DATA_DIR!\rtt.csv" ^
  --particles 1000 ^
  --sigma-los !SIGMA_LOS! ^
  --sigma-nlos !SIGMA_NLOS! ^
  --bias !BIAS! ^
  --offset-b !OFFSET_B! ^
  --likelihood !LIKELIHOOD! ^
  --rough !ROUGH! ^
  --save "!SAVE_PREFIX!"

if errorlevel 1 (
    echo.
    echo IMPPF failed. Check the error above.
) else (
    echo.
    echo Results saved to:
    echo   !SAVE_PREFIX!.main.png
    echo   !SAVE_PREFIX!.evolution.png
    echo.
    echo Opening results...
    start "" "!SAVE_PREFIX!.main.png"
    start "" "!SAVE_PREFIX!.evolution.png"
)

pause

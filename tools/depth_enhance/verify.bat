@echo off
:: ATLAS Depth Verification Viewer
::
:: Usage:
::   verify.bat colmap.zip                          (RGB vs depth)
::   verify.bat colmap_enhanced.zip                 (RGB vs enhanced depth)
::   verify.bat colmap.zip colmap_enhanced.zip      (A/B compare)

setlocal enabledelayedexpansion

set PYTHON_EXE=

if exist ".python_exe" (
    set /p PYTHON_EXE=<.python_exe
    "!PYTHON_EXE!" --version >nul 2>&1
    if errorlevel 1 set PYTHON_EXE=
)

if "!PYTHON_EXE!"=="" (
    where py >nul 2>&1
    if not errorlevel 1 (
        for %%V in (3.12 3.11 3.10) do (
            if "!PYTHON_EXE!"=="" (
                py -%%V --version >nul 2>&1
                if not errorlevel 1 set PYTHON_EXE=py -%%V
            )
        )
    )
)
if "!PYTHON_EXE!"=="" (
    python --version >nul 2>&1
    if not errorlevel 1 set PYTHON_EXE=python
)

if "!PYTHON_EXE!"=="" (
    echo ERROR: No Python found. Run install.bat first.
    pause
    exit /b 1
)

if "%~1"=="" (
    echo ATLAS Depth Verification Viewer
    echo.
    echo Usage:
    echo   verify.bat colmap.zip
    echo   verify.bat colmap_enhanced.zip
    echo   verify.bat colmap.zip colmap_enhanced.zip   (A/B compare)
    echo.
    set /p ZIP_A="Drag a colmap zip here: "
    "!PYTHON_EXE!" verify_depth.py "!ZIP_A!"
) else (
    "!PYTHON_EXE!" verify_depth.py %*
)

if errorlevel 1 pause

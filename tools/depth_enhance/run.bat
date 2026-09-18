@echo off
:: SPDX-License-Identifier: MIT
:: Copyright (c) 2026 Orion. All rights reserved.
::
:: ATLAS Depth Enhancement Tool - Launcher
:: Uses the exact Python recorded by install.bat, so packages are always found.
::
:: Usage:
::   run.bat colmap.zip
::   run.bat colmap.zip --output colmap_enhanced.zip

setlocal enabledelayedexpansion

:: ---------------------------------------------------------------------------
:: Use the Python recorded by install.bat (.python_exe file)
:: This guarantees we use the same interpreter that has all packages installed.
:: ---------------------------------------------------------------------------
set PYTHON_EXE=

if exist ".python_exe" (
    set /p PYTHON_EXE=<.python_exe
    :: Verify it still works
    "!PYTHON_EXE!" --version >nul 2>&1
    if errorlevel 1 (
        echo WARNING: Recorded Python "!PYTHON_EXE!" not found, falling back to detection.
        set PYTHON_EXE=
    ) else (
        for /f "tokens=2" %%v in ('"!PYTHON_EXE!" --version 2^>^&1') do set PYVER=%%v
        echo Using Python !PYVER! (!PYTHON_EXE!)
    )
)

:: Fall back to py launcher detection if no recorded Python
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

:: ---------------------------------------------------------------------------
:: Run enhance_depth.py
:: ---------------------------------------------------------------------------
if "%~1"=="" (
    echo ATLAS Depth Enhancement Tool
    echo.
    echo Usage: run.bat colmap.zip [--output colmap_enhanced.zip]
    echo.
    set /p INPUT_ZIP="Drag colmap.zip here or type path: "
    "!PYTHON_EXE!" enhance_depth.py "!INPUT_ZIP!"
) else (
    "!PYTHON_EXE!" enhance_depth.py %*
)

if errorlevel 1 (
    echo.
    echo Failed. If you see a missing package error, run install.bat first.
    pause
)

@echo off
:: SPDX-License-Identifier: MIT
:: Copyright (c) 2026 Orion. All rights reserved.
::
:: ATLAS Depth Enhancement Tool - Windows Setup
::
:: Usage: Double-click install.bat  OR  run from Command Prompt

setlocal enabledelayedexpansion

echo ============================================================
echo  ATLAS Depth Enhancement Tool - Windows Setup
echo ============================================================
echo.

:: ---------------------------------------------------------------------------
:: 1. Find any Python 3.x to run the setup helper
::    We only need it to be runnable - the helper will check the version itself
:: ---------------------------------------------------------------------------
echo --- [1/5] Locating Python ---

set BOOTSTRAP_PY=

:: Try py launcher with explicit versions first (most reliable on Windows)
for %%V in (3.12 3.11 3.10 3.9 3.8 3.7) do (
    if "!BOOTSTRAP_PY!"=="" (
        py -%%V --version >nul 2>&1
        if not errorlevel 1 set BOOTSTRAP_PY=py -%%V
    )
)

:: Fall back to python / python3 on PATH
if "!BOOTSTRAP_PY!"=="" (
    python --version >nul 2>&1
    if not errorlevel 1 set BOOTSTRAP_PY=python
)
if "!BOOTSTRAP_PY!"=="" (
    python3 --version >nul 2>&1
    if not errorlevel 1 set BOOTSTRAP_PY=python3
)

if "!BOOTSTRAP_PY!"=="" (
    echo ERROR: No Python found at all.
    echo Install Python 3.11 from https://www.python.org/downloads/
    echo or run:  winget install Python.Python.3.11
    pause
    exit /b 1
)

for /f "tokens=2" %%v in ('!BOOTSTRAP_PY! --version 2^>^&1') do set BOOTSTRAP_VER=%%v
echo [OK] Bootstrap Python: !BOOTSTRAP_PY! (!BOOTSTRAP_VER!)

:: ---------------------------------------------------------------------------
:: 2. Write and run the Python setup helper
::    All version checks, CUDA detection, and pip calls happen in Python
::    to avoid Windows batch numeric comparison bugs
:: ---------------------------------------------------------------------------
echo.
echo --- [2/5] Running setup helper ---

!BOOTSTRAP_PY! setup_helper.py
if errorlevel 1 (
    echo.
    echo Setup failed. See messages above.
    pause
    exit /b 1
)

echo.
echo ============================================================
echo  Setup complete!
echo.
echo  Usage:
echo    python enhance_depth.py colmap.zip
echo    python enhance_depth.py colmap.zip --output colmap_enhanced.zip
echo.
echo  First run downloads PromptDA weights (~2 GB) from HuggingFace.
echo  Weights are cached in %USERPROFILE%\.cache\huggingface after that.
echo ============================================================
echo.
pause

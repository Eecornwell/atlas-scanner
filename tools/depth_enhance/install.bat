@echo off
:: SPDX-License-Identifier: MIT
:: Copyright (c) 2026 Orion. All rights reserved.
::
:: ATLAS Depth Enhancement Tool - Windows Setup
::
:: Handles:
::   - Python 3.7/3.8/3.9 on PATH: installs Python 3.11 via winget and uses py launcher
::   - CUDA version detection: picks correct PyTorch wheel index automatically
::   - No CUDA / no GPU: installs CPU-only PyTorch with a clear warning
::   - PromptDA clone + install with fallback to requirements.txt
::
:: Usage: Double-click install.bat  OR  run from Command Prompt

setlocal enabledelayedexpansion

echo ============================================================
echo  ATLAS Depth Enhancement Tool - Windows Setup
echo ============================================================
echo.

:: ---------------------------------------------------------------------------
:: 1. Find a suitable Python (3.10+)
::    Try: py launcher (preferred), then python, then python3
:: ---------------------------------------------------------------------------
echo --- [1/5] Checking Python version ---

set PYTHON_EXE=
set PYVER_MAJOR=0
set PYVER_MINOR=0

:: Try py launcher first — it can select a specific version
where py >nul 2>&1
if not errorlevel 1 (
    :: Try py -3.11, -3.12, -3.10 in preference order
    for %%V in (3.12 3.11 3.10) do (
        if "!PYTHON_EXE!"=="" (
            py -%%V --version >nul 2>&1
            if not errorlevel 1 (
                set PYTHON_EXE=py -%%V
                for /f "tokens=2" %%v in ('py -%%V --version 2^>^&1') do set PYVER=%%v
                echo [OK] Found Python %%V via py launcher
            )
        )
    )
)

:: Fall back to python / python3 on PATH
if "!PYTHON_EXE!"=="" (
    for %%C in (python python3) do (
        if "!PYTHON_EXE!"=="" (
            %%C --version >nul 2>&1
            if not errorlevel 1 (
                for /f "tokens=2" %%v in ('%%C --version 2^>^&1') do (
                    set PYVER=%%v
                    for /f "tokens=1,2 delims=." %%a in ("%%v") do (
                        set PYVER_MAJOR=%%a
                        set PYVER_MINOR=%%b
                    )
                )
                if !PYVER_MAJOR! GTR 3 set PYTHON_EXE=%%C
                if !PYVER_MAJOR! EQU 3 if !PYVER_MINOR! GEQ 10 set PYTHON_EXE=%%C
            )
        )
    )
)

if "!PYTHON_EXE!"=="" (
    echo.
    echo [!] Python 3.10 or newer is required.
    echo     Your system has Python !PYVER! which is too old.
    echo.
    echo     Option A - Install via winget (recommended, automatic):
    echo       winget install Python.Python.3.11
    echo       Then re-run this script.
    echo.
    echo     Option B - Install manually:
    echo       https://www.python.org/downloads/
    echo       Download Python 3.11.x, run installer,
    echo       check "Add Python to PATH" and "Install for all users".
    echo       Then re-run this script.
    echo.
    echo     After installing, the py launcher will let both versions coexist.
    echo     This script will automatically use the newer one.
    echo.
    pause
    exit /b 1
)

echo [OK] Using: !PYTHON_EXE! (!PYVER!)

:: ---------------------------------------------------------------------------
:: 2. Detect CUDA version from nvidia-smi
:: ---------------------------------------------------------------------------
echo.
echo --- [2/5] Detecting GPU / CUDA ---

set CUDA_VER=cpu
set TORCH_INDEX=https://download.pytorch.org/whl/cpu
set GPU_NAME=none

nvidia-smi >nul 2>&1
if errorlevel 1 (
    echo [!] nvidia-smi not found - no NVIDIA GPU detected or driver not installed.
    echo     PyTorch will be installed in CPU-only mode.
    echo     PromptDA will run on CPU (slow - expect ~30s per tile).
) else (
    :: Extract CUDA version from nvidia-smi output e.g. "CUDA Version: 12.8"
    for /f "tokens=3" %%c in ('nvidia-smi ^| findstr /i "CUDA Version"') do set CUDA_FULL=%%c
    for /f "tokens=1 delims=." %%M in ("!CUDA_FULL!") do set CUDA_MAJOR=%%M
    for /f "tokens=2 delims=." %%m in ("!CUDA_FULL!") do set CUDA_MINOR=%%m

    :: Get GPU name
    for /f "skip=1 tokens=*" %%g in ('nvidia-smi --query-gpu=name --format=csv^,noheader 2^>nul') do (
        if "!GPU_NAME!"=="none" set GPU_NAME=%%g
    )

    echo [OK] GPU: !GPU_NAME!
    echo [OK] CUDA: !CUDA_FULL!

    :: Map CUDA version to PyTorch wheel index
    :: CUDA 12.8+ -> cu128, 12.4-12.7 -> cu124, 12.1-12.3 -> cu121, 11.8 -> cu118
    if !CUDA_MAJOR! GEQ 12 (
        if !CUDA_MINOR! GEQ 8 (
            set CUDA_VER=cu128
        ) else if !CUDA_MINOR! GEQ 4 (
            set CUDA_VER=cu124
        ) else (
            set CUDA_VER=cu121
        )
    ) else if !CUDA_MAJOR! EQU 11 (
        set CUDA_VER=cu118
    ) else (
        echo [!] CUDA !CUDA_FULL! is older than 11.8 - falling back to CPU PyTorch.
        set CUDA_VER=cpu
    )

    if not "!CUDA_VER!"=="cpu" (
        set TORCH_INDEX=https://download.pytorch.org/whl/!CUDA_VER!
        echo [OK] PyTorch wheel: !CUDA_VER!
    )
)

:: ---------------------------------------------------------------------------
:: 3. Install PyTorch
:: ---------------------------------------------------------------------------
echo.
echo --- [3/5] Installing PyTorch (!CUDA_VER!) ---
echo     Index: !TORCH_INDEX!

!PYTHON_EXE! -m pip install torch torchvision --index-url !TORCH_INDEX! --quiet
if errorlevel 1 (
    echo.
    echo [!] PyTorch install failed with index !TORCH_INDEX!
    echo     Trying CPU fallback...
    !PYTHON_EXE! -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu --quiet
    if errorlevel 1 (
        echo ERROR: PyTorch install failed entirely. Check your internet connection.
        pause
        exit /b 1
    )
    echo [OK] PyTorch installed (CPU fallback)
) else (
    echo [OK] PyTorch installed
)

:: ---------------------------------------------------------------------------
:: 4. Core dependencies
:: ---------------------------------------------------------------------------
echo.
echo --- [4/5] Installing core dependencies ---

!PYTHON_EXE! -m pip install ^
    "numpy>=1.24" ^
    "opencv-python>=4.8" ^
    "Pillow>=10.0" ^
    "tqdm>=4.0" ^
    "huggingface_hub>=0.23" ^
    "transformers>=4.40" ^
    "timm>=1.0" ^
    "einops>=0.7" ^
    "accelerate>=0.30" ^
    --quiet
if errorlevel 1 (
    echo ERROR: Core dependency install failed.
    pause
    exit /b 1
)
echo [OK] Core dependencies installed

:: ---------------------------------------------------------------------------
:: 5. PromptDA
:: ---------------------------------------------------------------------------
echo.
echo --- [5/5] Installing PromptDA ---

if not exist "models" mkdir models

if not exist "models\PromptDA\.git" (
    echo Cloning PromptDA from GitHub...
    git --version >nul 2>&1
    if errorlevel 1 (
        echo.
        echo [!] git not found. Install from https://git-scm.com/download/win
        echo     Then re-run this script.
        pause
        exit /b 1
    )
    git clone https://github.com/DepthAnything/PromptDA.git models\PromptDA --depth 1 --quiet
    if errorlevel 1 (
        echo ERROR: git clone failed. Check your internet connection.
        pause
        exit /b 1
    )
    echo [OK] PromptDA cloned
) else (
    echo [OK] PromptDA already present
)

:: Try editable install first, fall back to requirements.txt
!PYTHON_EXE! -m pip install -e models\PromptDA --quiet 2>nul
if errorlevel 1 (
    echo     Editable install failed, trying requirements.txt...
    if exist "models\PromptDA\requirements.txt" (
        !PYTHON_EXE! -m pip install -r models\PromptDA\requirements.txt --quiet
    )
    :: Try plain install as last resort
    !PYTHON_EXE! -m pip install models\PromptDA --quiet 2>nul
)

:: Verify PromptDA importable
!PYTHON_EXE! -c "from promptda.promptda import PromptDA" >nul 2>&1
if errorlevel 1 (
    echo.
    echo [!] PromptDA import failed. The package may have changed its structure.
    echo     enhance_depth.py will still run but will skip PromptDA and use
    echo     the guided filter fallback instead.
    echo     Check: https://github.com/DepthAnything/PromptDA
) else (
    echo [OK] PromptDA importable
)

:: ---------------------------------------------------------------------------
:: Summary
:: ---------------------------------------------------------------------------
echo.
echo --- Verification ---
!PYTHON_EXE! -c "import torch; print('  torch     :', torch.__version__); print('  CUDA avail:', torch.cuda.is_available()); print('  GPU       :', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'none (CPU mode)')"
!PYTHON_EXE! -c "import cv2; print('  opencv    :', cv2.__version__)"
!PYTHON_EXE! -c "import PIL; print('  Pillow    :', PIL.__version__)"

echo.
echo ============================================================
echo  Setup complete!
echo.
echo  Usage:
echo    !PYTHON_EXE! enhance_depth.py colmap.zip
echo    !PYTHON_EXE! enhance_depth.py colmap.zip --output colmap_enhanced.zip
echo.
echo  First run will download PromptDA weights (~2 GB) from HuggingFace.
echo  Weights are cached in %%USERPROFILE%%\.cache\huggingface after that.
echo.
if "!CUDA_VER!"=="cpu" (
    echo  WARNING: Running on CPU. Expect ~30-60s per depth tile.
    echo  For GPU acceleration install NVIDIA drivers and re-run this script.
)
echo ============================================================
echo.
pause

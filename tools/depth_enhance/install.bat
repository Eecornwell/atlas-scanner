@echo off
:: SPDX-License-Identifier: MIT
:: Copyright (c) 2026 Orion. All rights reserved.
::
:: ATLAS Depth Enhancement Tool — Windows Setup
:: Installs Python dependencies for enhance_depth.py
::
:: Requirements:
::   - Python 3.10+ (from python.org, added to PATH)
::   - NVIDIA GPU with CUDA 12.x drivers
::   - Internet connection (downloads ~4 GB of models on first run)
::
:: Usage:
::   Double-click install.bat  OR  run from Command Prompt

setlocal enabledelayedexpansion

echo ============================================================
echo  ATLAS Depth Enhancement Tool - Windows Setup
echo ============================================================
echo.

:: Check Python
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python not found. Install Python 3.10+ from https://python.org
    echo        Make sure to check "Add Python to PATH" during install.
    pause
    exit /b 1
)
for /f "tokens=2" %%v in ('python --version 2^>^&1') do set PYVER=%%v
echo [OK] Python %PYVER%

:: Check pip
pip --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: pip not found. Run: python -m ensurepip
    pause
    exit /b 1
)
echo [OK] pip found

echo.
echo --- [1/4] PyTorch with CUDA 12.8 ---
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128 --quiet
if errorlevel 1 (
    echo ERROR: PyTorch install failed.
    pause
    exit /b 1
)
echo [OK] PyTorch installed

echo.
echo --- [2/4] Core dependencies ---
pip install ^
    numpy ^
    opencv-python ^
    Pillow ^
    tqdm ^
    huggingface_hub ^
    transformers ^
    timm ^
    einops ^
    accelerate ^
    --quiet
if errorlevel 1 (
    echo ERROR: Core dependency install failed.
    pause
    exit /b 1
)
echo [OK] Core dependencies installed

echo.
echo --- [3/4] PromptDA ---
:: Clone PromptDA if not already present
if not exist "models\PromptDA\.git" (
    echo Cloning PromptDA...
    if not exist "models" mkdir models
    git clone https://github.com/DepthAnything/PromptDA.git models\PromptDA --quiet
    if errorlevel 1 (
        echo ERROR: git clone failed. Make sure git is installed: https://git-scm.com
        pause
        exit /b 1
    )
) else (
    echo PromptDA already cloned.
)
pip install -e models\PromptDA --quiet
if errorlevel 1 (
    echo WARNING: PromptDA editable install failed, trying requirements.txt...
    pip install -r models\PromptDA\requirements.txt --quiet
)
echo [OK] PromptDA installed

echo.
echo --- [4/4] Verifying installation ---
python -c "import torch; print('  torch:', torch.__version__); print('  CUDA:', torch.cuda.is_available()); print('  GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'none')"
python -c "from promptda.promptda import PromptDA; print('  PromptDA: OK')"
python -c "import cv2; print('  OpenCV:', cv2.__version__)"
if errorlevel 1 (
    echo ERROR: Verification failed. Check output above.
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
echo  The model weights (~2 GB) will be downloaded from HuggingFace
echo  on the first run and cached in %%USERPROFILE%%\.cache\huggingface
echo ============================================================
echo.
pause

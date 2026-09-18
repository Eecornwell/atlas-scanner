#!/usr/bin/env python
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# ATLAS Depth Enhancement Tool - Setup Helper
# Called by install.bat to handle all version checks, CUDA detection,
# and pip installs in Python rather than Windows batch to avoid
# batch numeric comparison bugs.

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()


def run(cmd, check=True, capture=False):
    kwargs = dict(check=check)
    if capture:
        kwargs.update(stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return subprocess.run(cmd, **kwargs)


def pip(args, python=None):
    exe = python or sys.executable
    return run([exe, '-m', 'pip'] + args, check=False)


# ---------------------------------------------------------------------------
# Step 1: Verify Python version is 3.10+
# ---------------------------------------------------------------------------
print('\n--- [1/5] Checking Python version ---')

major, minor = sys.version_info.major, sys.version_info.minor
print(f'  Running as: Python {major}.{minor} ({sys.executable})')

if (major, minor) < (3, 10):
    print()
    print(f'  ERROR: Python {major}.{minor} is too old. Python 3.10+ is required.')
    print()
    print('  Option A - Install via winget (recommended):')
    print('    winget install Python.Python.3.11')
    print('    Then re-run install.bat')
    print()
    print('  Option B - Install manually:')
    print('    https://www.python.org/downloads/')
    print('    Download Python 3.11.x, run installer,')
    print('    check "Add Python to PATH" and "Use py launcher".')
    print('    Then re-run install.bat')
    print()
    print('  After installing, install.bat will automatically find Python 3.11')
    print('  via the py launcher without affecting your existing Python install.')
    sys.exit(1)

print(f'  [OK] Python {major}.{minor}')

# ---------------------------------------------------------------------------
# Step 2: Detect CUDA version from nvidia-smi
# ---------------------------------------------------------------------------
print('\n--- [2/5] Detecting GPU / CUDA ---')

torch_index = 'https://download.pytorch.org/whl/cpu'
cuda_tag = 'cpu'
gpu_name = 'none'

nvidia_smi = shutil.which('nvidia-smi')
if nvidia_smi:
    try:
        result = subprocess.run(
            [nvidia_smi], capture_output=True, text=True, timeout=10
        )
        output = result.stdout + result.stderr

        # Parse CUDA version e.g. "CUDA Version: 12.8"
        m = re.search(r'CUDA Version:\s*(\d+)\.(\d+)', output)
        if m:
            cuda_major, cuda_minor = int(m.group(1)), int(m.group(2))
            print(f'  CUDA: {cuda_major}.{cuda_minor}')

            if cuda_major >= 13 or (cuda_major == 12 and cuda_minor >= 8):
                cuda_tag = 'cu128'
            elif cuda_major == 12 and cuda_minor >= 4:
                cuda_tag = 'cu124'
            elif cuda_major == 12:
                cuda_tag = 'cu121'
            elif cuda_major == 11 and cuda_minor >= 8:
                cuda_tag = 'cu118'
            else:
                print(f'  WARNING: CUDA {cuda_major}.{cuda_minor} < 11.8, using CPU PyTorch.')

            # Check GPU compute capability.
            # PyTorch >= 2.1 dropped sm_61 (Pascal/GTX 10xx).
            # Use PyTorch 2.0.1+cu118 for CC < 7.0.
            try:
                smi_cc = subprocess.run(
                    [nvidia_smi, '--query-gpu=compute_cap', '--format=csv,noheader'],
                    capture_output=True, text=True, timeout=5
                )
                cc_str = smi_cc.stdout.strip().splitlines()[0].strip()
                cc_major = int(cc_str.split('.')[0])
                print(f'  GPU compute capability: {cc_str}')
                if cc_major < 7:
                    print(f'  Pascal/Maxwell GPU (CC {cc_str}) — PyTorch >= 2.1 dropped sm_61.')
                    print(f'  Will install PyTorch 2.0.1+cu118 (last version with Pascal support).')
                    cuda_tag = 'cu118_legacy'
            except Exception:
                pass

        # Parse GPU name
        gm = re.search(r'^\|\s+\d+\s+(.*?)\s+(?:Off|On)\s+\|', output, re.MULTILINE)
        if not gm:
            gm = re.search(r'GPU\s+\d+:.*?:\s+(.*?)\s*\(', output)
        if gm:
            gpu_name = gm.group(1).strip()

        if cuda_tag != 'cpu':
            torch_index = f'https://download.pytorch.org/whl/{cuda_tag}'
            print(f'  GPU: {gpu_name}')
            print(f'  PyTorch wheel: {cuda_tag}')
        else:
            print(f'  GPU: {gpu_name}')
    except Exception as e:
        print(f'  WARNING: nvidia-smi failed ({e}), using CPU PyTorch.')
else:
    print('  nvidia-smi not found - no NVIDIA GPU or driver not installed.')
    print('  PyTorch will be installed CPU-only (slow: ~30-60s per tile).')

# ---------------------------------------------------------------------------
# Step 3: Install PyTorch
# ---------------------------------------------------------------------------
print(f'\n--- [3/5] Installing PyTorch ({cuda_tag}) ---')

# cu118_legacy = Pascal GPU (CC < 7.0): needs PyTorch 2.0.1 which still
# ships sm_61 kernels. Newer wheels only support sm_75+.
if cuda_tag == 'cu118_legacy':
    torch_index = 'https://download.pytorch.org/whl/cu118'
    torch_pkg = ['torch==2.0.1+cu118', 'torchvision==0.15.2+cu118']
    # Uninstall xformers if present — 0.0.35 requires torch>=2.10 and will
    # break numpy interop with torch 2.0.1
    pip(['uninstall', 'xformers', '-y', '--quiet'])
else:
    torch_index = f'https://download.pytorch.org/whl/{cuda_tag}' if cuda_tag != 'cpu' else 'https://download.pytorch.org/whl/cpu'
    torch_pkg = ['torch', 'torchvision']

print(f'  Index: {torch_index}')

r = pip(['install'] + torch_pkg + ['--index-url', torch_index, '--force-reinstall', '--quiet'])
if r.returncode != 0:
    if cuda_tag != 'cpu':
        print('  WARNING: CUDA wheel failed, retrying with CPU fallback...')
        r = pip(['install', 'torch', 'torchvision',
                 '--index-url', 'https://download.pytorch.org/whl/cpu', '--force-reinstall', '--quiet'])
    if r.returncode != 0:
        print('  ERROR: PyTorch install failed. Check your internet connection.')
        sys.exit(1)
    print('  [OK] PyTorch installed (CPU fallback)')
else:
    print('  [OK] PyTorch installed')

# ---------------------------------------------------------------------------
# Step 3b: xformers matched to installed torch version
# ---------------------------------------------------------------------------
print('\n--- [3b/5] Installing xformers ---')

try:
    import importlib.util, subprocess as _sp
    # Get the exact torch version that was just installed
    r = _sp.run([sys.executable, '-c', 'import torch; print(torch.__version__)'],
                capture_output=True, text=True)
    torch_ver = r.stdout.strip()  # e.g. "2.14.0+cu128"
    base_ver = torch_ver.split('+')[0]  # e.g. "2.14.0"
    print(f'  Installed torch: {torch_ver}')

    if '+cpu' in torch_ver:
        print('  Skipping xformers (CPU-only torch — xformers requires CUDA)')
    elif 'cu118' in torch_ver and cuda_tag == 'cu118_legacy':
        print('  Skipping xformers (Pascal GPU — xformers does not support CC < 7.0)')
    else:
        # xformers publishes wheels matching torch versions exactly.
        # pip will find the right one automatically when given the torch version constraint.
        xf_r = pip(['install', 'xformers', '--quiet',
                    '--index-url', f'https://download.pytorch.org/whl/{cuda_tag}'])
        if xf_r.returncode == 0:
            # Verify it actually loads
            vr = _sp.run([sys.executable, '-c', 'import xformers; print(xformers.__version__)'],
                         capture_output=True, text=True)
            if vr.returncode == 0:
                print(f'  [OK] xformers {vr.stdout.strip()}')
            else:
                print('  WARNING: xformers installed but import failed — will be skipped at runtime')
        else:
            print('  WARNING: No xformers wheel found for this torch version — skipping')
            print('  PromptDA will use standard attention (slightly slower, same quality)')
except Exception as e:
    print(f'  WARNING: xformers install skipped: {e}')

# ---------------------------------------------------------------------------
# Step 4: Core dependencies
# ---------------------------------------------------------------------------
print('\n--- [4/5] Installing core dependencies ---')

core_deps = [
    'numpy>=1.24,<2' if cuda_tag == 'cu118_legacy' else 'numpy>=1.24',
    'opencv-python>=4.8',
    'Pillow>=10.0',
    'tqdm>=4.0',
    'huggingface_hub>=0.23',
    'transformers>=4.40',
    'timm>=1.0',
    'einops>=0.7',
    'accelerate>=0.30',
    # xformers installed separately below to match exact torch version
]

r = pip(['install'] + core_deps + ['--quiet'])
if r.returncode != 0:
    # xformers may fail if no matching wheel exists for this torch/CUDA combo.
    # Retry without it — everything still works, just slower attention.
    core_deps_no_xformers = [d for d in core_deps if 'xformers' not in d]
    r = pip(['install'] + core_deps_no_xformers + ['--quiet'])
    if r.returncode != 0:
        print('  ERROR: Core dependency install failed.')
        sys.exit(1)
    print('  [OK] Core dependencies installed (xformers skipped - no matching wheel)')
else:
    print('  [OK] Core dependencies installed')

# ---------------------------------------------------------------------------
# Step 5: PromptDA
# ---------------------------------------------------------------------------
print('\n--- [5/5] Installing PromptDA ---')

models_dir = SCRIPT_DIR / 'models'
promptda_dir = models_dir / 'PromptDA'
models_dir.mkdir(exist_ok=True)

if not (promptda_dir / '.git').exists():
    git = shutil.which('git')
    if not git:
        print('  ERROR: git not found.')
        print('  Install from https://git-scm.com/download/win then re-run.')
        sys.exit(1)
    print('  Cloning PromptDA...')
    r = run(['git', 'clone', 'https://github.com/DepthAnything/PromptDA.git',
             str(promptda_dir), '--depth', '1'], check=False)
    if r.returncode != 0:
        print('  ERROR: git clone failed. Check your internet connection.')
        sys.exit(1)
    print('  [OK] PromptDA cloned')
else:
    print('  [OK] PromptDA already present')

# Try editable install, then plain install
installed = False
for attempt in [
    [sys.executable, '-m', 'pip', 'install', '-e', str(promptda_dir), '--quiet'],
    [sys.executable, '-m', 'pip', 'install', str(promptda_dir), '--quiet'],
]:
    if not installed:
        r = subprocess.run(attempt, check=False,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if r.returncode == 0:
            installed = True

if not installed:
    req = promptda_dir / 'requirements.txt'
    if req.exists():
        r = pip(['install', '-r', str(req), '--quiet'])
        installed = r.returncode == 0

# Regardless of whether pip install succeeded, write a .pth file directly
# into site-packages so the import always works from this Python.
# This is more reliable than editable installs which depend on pip internals.
try:
    import site
    sp = site.getsitepackages()
    pth_content = str(promptda_dir) + '\n'
    for sp_dir in sp:
        pth_path = Path(sp_dir) / 'promptda_atlas.pth'
        try:
            pth_path.write_text(pth_content)
            print(f'  [OK] Wrote {pth_path}')
            break
        except OSError:
            continue
except Exception as e:
    print(f'  WARNING: Could not write .pth file: {e}')

# Record which Python executable was used so run.bat uses the same one
python_record = SCRIPT_DIR / '.python_exe'
python_record.write_text(sys.executable)
print(f'  [OK] Recorded Python: {sys.executable}')

# Verify import
try:
    result = subprocess.run(
        [sys.executable, '-c', 'from promptda.promptda import PromptDA'],
        capture_output=True, timeout=15
    )
    if result.returncode == 0:
        print('  [OK] PromptDA importable')
    else:
        print('  WARNING: PromptDA import failed.')
        print('    enhance_depth.py will use the guided filter fallback.')
        print('    Check: https://github.com/DepthAnything/PromptDA')
except Exception:
    print('  WARNING: Could not verify PromptDA import.')

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print('\n--- Verification ---')
checks = [
    ('torch',   'import torch; print("  torch     :", torch.__version__); '
                'print("  CUDA avail:", torch.cuda.is_available()); '
                'print("  GPU       :", torch.cuda.get_device_name(0) '
                'if torch.cuda.is_available() else "none (CPU mode)")'),
    ('opencv',  'import cv2; print("  opencv    :", cv2.__version__)'),
    ('Pillow',  'import PIL; print("  Pillow    :", PIL.__version__)'),
]
for name, code in checks:
    r = subprocess.run([sys.executable, '-c', code],
                       capture_output=False, check=False)

if cuda_tag == 'cpu':
    print()
    print('  NOTE: Running CPU-only. Expect ~30-60s per depth tile.')
    print('  Install an NVIDIA GPU driver and re-run install.bat for GPU speed.')

print()
print('Setup complete.')

#!/bin/bash
# Atlas Mobile — Offline Pipeline Setup
#
# Installs Python dependencies for the post-processing pipeline:
#   Stage 1: PromptDA  (LiDAR-anchored dense depth completion)
#   Stage 2: StableNormal (diffusion-based surface normal estimation)
#   Stage 3: COLMAP assembly (erp_tile_slicer + assemble_colmap)
#
# Requirements:
#   - Python 3.10+
#   - CUDA-capable GPU with ≥8 GB VRAM recommended
#   - torch already installed (or will be installed below)
#
# Usage:
#   cd mobile/offline_pipeline
#   bash setup.sh
#
# Re-running is safe — all steps are idempotent.

set -e

if [ "$EUID" -eq 0 ]; then
  echo "ERROR: Do not run as root."
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DIR="$SCRIPT_DIR/models"
mkdir -p "$MODELS_DIR"

echo "=== Atlas Mobile Offline Pipeline Setup ==="
echo "Script dir : $SCRIPT_DIR"
echo "Models dir : $MODELS_DIR"
echo ""

# ---------------------------------------------------------------------------
# 1. Core Python dependencies
# ---------------------------------------------------------------------------
echo "--- [1/5] Core Python dependencies ---"

pip install --upgrade pip --quiet

# torch: skip if a CUDA build is already present to avoid downgrading
if python3 -c "import torch; assert '+cu' in torch.__version__" 2>/dev/null; then
  echo "  ✓ torch $(python3 -c 'import torch; print(torch.__version__)') (CUDA) already installed"
else
  echo "  Installing torch with CUDA 12.8 support..."
  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128 --quiet
fi

pip install \
  numpy>=1.24 \
  scipy>=1.11 \
  "opencv-python>=4.8" \
  "open3d>=0.18" \
  "Pillow>=10.0" \
  "pyyaml>=6.0" \
  --quiet

echo "  ✓ Core deps installed"

# ---------------------------------------------------------------------------
# 2. Hugging Face + diffusion stack (needed by both PromptDA and StableNormal)
# ---------------------------------------------------------------------------
echo ""
echo "--- [2/5] Hugging Face + diffusion stack ---"

pip install \
  "huggingface_hub>=0.23" \
  "transformers>=4.40" \
  "diffusers>=0.28" \
  "accelerate>=0.30" \
  "timm>=1.0" \
  "einops>=0.7" \
  "flask>=3.0" \
  --quiet

echo "  ✓ HuggingFace stack installed"

# ---------------------------------------------------------------------------
# 2b. CoreML tools (macOS only — for convert_models.py)
# ---------------------------------------------------------------------------
echo ""
echo "--- [2b] coremltools (macOS only) ---"
if python3 -c "import platform; exit(0 if platform.system()=='Darwin' else 1)" 2>/dev/null; then
  pip install "coremltools>=7.0" --quiet
  echo "  ✓ coremltools installed"
else
  echo "  Skipping coremltools (not macOS — convert_models.py requires macOS)"
fi

# ---------------------------------------------------------------------------
# 3. PromptDA (Prompt Depth Anything)
#    https://github.com/DepthAnything/PromptDA
# ---------------------------------------------------------------------------
echo ""
echo "--- [3/5] PromptDA ---"

PROMPTDA_DIR="$MODELS_DIR/PromptDA"
PROMPTDA_COMMIT="7b3b3b3"   # pin to a known-good commit

if [ ! -d "$PROMPTDA_DIR/.git" ]; then
  echo "  Cloning PromptDA..."
  git clone https://github.com/DepthAnything/PromptDA.git "$PROMPTDA_DIR" --quiet
else
  echo "  PromptDA repo already present, fetching updates..."
  git -C "$PROMPTDA_DIR" fetch --quiet
fi

# Pin to a stable commit
cd "$PROMPTDA_DIR"
LATEST=$(git rev-parse HEAD)
echo "  HEAD: $LATEST"

# Install as editable package so enhance_session.py can import it
pip install -e "$PROMPTDA_DIR" --quiet 2>/dev/null || \
  pip install -r "$PROMPTDA_DIR/requirements.txt" --quiet 2>/dev/null || true

echo "  ✓ PromptDA ready at $PROMPTDA_DIR"
cd "$SCRIPT_DIR"

# ---------------------------------------------------------------------------
# 4. StableNormal
#    https://github.com/Stable-X/StableNormal
# ---------------------------------------------------------------------------
echo ""
echo "--- [4/5] StableNormal ---"

STABLENORMAL_DIR="$MODELS_DIR/StableNormal"

if [ ! -d "$STABLENORMAL_DIR/.git" ]; then
  echo "  Cloning StableNormal..."
  git clone https://github.com/Stable-X/StableNormal.git "$STABLENORMAL_DIR" --quiet
else
  echo "  StableNormal repo already present, fetching updates..."
  git -C "$STABLENORMAL_DIR" fetch --quiet
fi

cd "$STABLENORMAL_DIR"
LATEST=$(git rev-parse HEAD)
echo "  HEAD: $LATEST"

pip install -e "$STABLENORMAL_DIR" --quiet 2>/dev/null || \
  pip install -r "$STABLENORMAL_DIR/requirements.txt" --quiet 2>/dev/null || true

echo "  ✓ StableNormal ready at $STABLENORMAL_DIR"
cd "$SCRIPT_DIR"

# ---------------------------------------------------------------------------
# 5. Verify environment
# ---------------------------------------------------------------------------
echo ""
echo "--- [5/5] Verification ---"

python3 - <<'PYEOF'
import sys

ok = True

def check(label, expr):
    global ok
    try:
        result = eval(expr)
        print(f"  ✓ {label}: {result}")
    except Exception as e:
        print(f"  ✗ {label}: {e}")
        ok = False

check("python",       "sys.version.split()[0]")
check("torch",        "__import__('torch').__version__")
check("torch CUDA",   "__import__('torch').cuda.is_available() or 'not available (CPU fallback)'")
check("numpy",        "__import__('numpy').__version__")
check("opencv",       "__import__('cv2').__version__")
check("open3d",       "__import__('open3d').__version__")
check("scipy",        "__import__('scipy').__version__")
check("PIL",          "__import__('PIL').__version__")
check("yaml",         "__import__('yaml').__version__")
check("huggingface_hub", "__import__('huggingface_hub').__version__")
check("transformers", "__import__('transformers').__version__")
check("diffusers",    "__import__('diffusers').__version__")
check("accelerate",   "__import__('accelerate').__version__")
check("timm",         "__import__('timm').__version__")
check("einops",       "__import__('einops').__version__")

# Pipeline scripts
import importlib.util, pathlib
script_dir = pathlib.Path(__file__).parent if '__file__' in dir() else pathlib.Path('.')
for script in ("erp_tile_slicer", "assemble_colmap", "enhance_session"):
    p = script_dir / f"{script}.py"
    if p.exists():
        print(f"  ✓ {script}.py found")
    else:
        print(f"  ✗ {script}.py NOT found")
        ok = False

sys.exit(0 if ok else 1)
PYEOF

echo ""
echo "=== Setup complete ==="
echo ""
echo "Usage:"
echo "  python3 enhance_session.py --session-dir /path/to/session_YYYY-MM-DD_HH-MM-SS"
echo ""
echo "Stages:"
echo "  1. PromptDA  — dense depth from sparse iPhone LiDAR + RGB"
echo "  2. StableNormal — surface normals from RGB"
echo "  3. COLMAP assembly — cameras.bin / images.bin / points3D.bin / depth_images/"
echo ""
echo "Note: PromptDA and StableNormal weights are downloaded from HuggingFace"
echo "on first run (~2-4 GB). Ensure you have an internet connection."

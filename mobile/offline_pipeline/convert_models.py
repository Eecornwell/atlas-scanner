"""
Convert PromptDA and StableNormal to CoreML .mlpackage for on-device inference.

Run once on a Mac with Apple Silicon (M1/M2/M3) — coremltools requires macOS.
Output .mlpackage files are added to the Xcode project as bundle resources.

Usage:
    cd mobile/offline_pipeline
    python3 convert_models.py [--output-dir /path/to/AtlasMobile/Resources/]

Requirements:
    pip install coremltools>=7.0
    # PromptDA and StableNormal must be installed (run setup.sh first)

Output:
    PromptDA.mlpackage     — ~400 MB, ViT-L encoder-decoder, ANE-optimised
    StableNormal.mlpackage — ~900 MB, YOSO coarse + diffusion UNet, ANE-optimised

After conversion:
    1. Drag both .mlpackage files into Xcode project navigator
    2. Ensure they are added to the AtlasMobile target (check "Add to targets")
    3. PostProcessingManager.swift will load them via Bundle.main.url(forResource:)
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
import numpy as np


# ---------------------------------------------------------------------------
# PromptDA conversion
# ---------------------------------------------------------------------------

def convert_promptda(output_dir: Path) -> Path:
    """Convert PromptDA ViT-L to CoreML with ANE compute units."""
    import coremltools as ct
    from promptda.promptda import PromptDA

    print("Loading PromptDA ViT-L...")
    model = PromptDA.from_pretrained("depth-anything/promptda_vitl").eval()

    # Trace with fixed input size (518x518 — PromptDA's native resolution)
    H, W = 518, 518
    dummy_image = torch.zeros(1, 3, H, W)
    dummy_depth = torch.zeros(1, 1, H, W)

    print("Tracing PromptDA...")
    with torch.no_grad():
        traced = torch.jit.trace(model, (dummy_image, dummy_depth))

    print("Converting to CoreML...")
    mlmodel = ct.convert(
        traced,
        inputs=[
            ct.TensorType(name="image",         shape=(1, 3, H, W), dtype=np.float32),
            ct.TensorType(name="prompt_depth",  shape=(1, 1, H, W), dtype=np.float32),
        ],
        outputs=[
            ct.TensorType(name="depth", dtype=np.float32),
        ],
        compute_units=ct.ComputeUnit.ALL,  # ANE + GPU + CPU
        minimum_deployment_target=ct.target.iOS16,
    )

    out_path = output_dir / "PromptDA.mlpackage"
    mlmodel.save(str(out_path))
    size_mb = sum(f.stat().st_size for f in out_path.rglob("*") if f.is_file()) // (1024 * 1024)
    print(f"  ✓ PromptDA.mlpackage ({size_mb} MB) -> {out_path}")
    return out_path


# ---------------------------------------------------------------------------
# StableNormal conversion
# ---------------------------------------------------------------------------

def convert_stablenormal(output_dir: Path) -> Path:
    """
    Convert StableNormal YOSO coarse pass to CoreML.

    The full two-step diffusion pipeline is too large to run efficiently
    on-device. We convert only the YOSO coarse pass (single forward pass,
    no iterative denoising) which produces good-quality normals at ~1-2s/image.
    The refined diffusion pass remains available on the host pipeline.
    """
    import coremltools as ct
    from stablenormal.pipeline_yoso_normal import YOSONormalsPipeline

    print("Loading StableNormal YOSO (coarse pass)...")
    pipe = YOSONormalsPipeline.from_pretrained(
        "Stable-X/yoso-normal-v0-3", trust_remote_code=True
    ).eval()

    # Extract just the UNet backbone for tracing
    unet = pipe.unet.eval()
    H, W = 768, 768
    dummy = torch.zeros(1, 3, H, W)

    print("Tracing YOSO UNet...")
    with torch.no_grad():
        traced = torch.jit.trace(unet, dummy)

    print("Converting to CoreML...")
    mlmodel = ct.convert(
        traced,
        inputs=[
            ct.TensorType(name="image", shape=(1, 3, H, W), dtype=np.float32),
        ],
        outputs=[
            ct.TensorType(name="normal", dtype=np.float32),
        ],
        compute_units=ct.ComputeUnit.ALL,
        minimum_deployment_target=ct.target.iOS16,
    )

    out_path = output_dir / "StableNormal.mlpackage"
    mlmodel.save(str(out_path))
    size_mb = sum(f.stat().st_size for f in out_path.rglob("*") if f.is_file()) // (1024 * 1024)
    print(f"  ✓ StableNormal.mlpackage ({size_mb} MB) -> {out_path}")
    return out_path


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Convert ML models to CoreML for iOS")
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path(__file__).parents[1] / "AtlasMobile/AtlasMobile/Resources",
        help="Output directory for .mlpackage files (default: AtlasMobile/Resources/)",
    )
    parser.add_argument("--skip-promptda",    action="store_true")
    parser.add_argument("--skip-stablenormal", action="store_true")
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    import platform
    if platform.system() != "Darwin":
        print("WARNING: coremltools ANE optimisation requires macOS.")
        print("         Models will still convert but may not use the Neural Engine.")

    if not args.skip_promptda:
        convert_promptda(args.output_dir)

    if not args.skip_stablenormal:
        convert_stablenormal(args.output_dir)

    print("\nNext steps:")
    print("  1. Drag .mlpackage files into Xcode project navigator")
    print("  2. Check 'Add to targets: AtlasMobile'")
    print("  3. Build and run — PostProcessingManager will load them automatically")


if __name__ == "__main__":
    main()

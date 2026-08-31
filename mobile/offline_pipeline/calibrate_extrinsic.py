"""
Extrinsic calibration for Insta360 ↔ iPhone rigid mount.

Port of atlas-scanner's calibration pipeline:
  - find_matches_superglue_erp.py
  - seed_calib.py / physical_seed.py
  - tune_calibration.py
  - verify_seed_overlay.py

Usage:
    uv run python calibrate_extrinsic.py \
        --session-dir /path/to/calibration_session \
        --camera-id insta360_primary \
        --seed-from-physical
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def load_physical_seed(
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    x: float = 0.0,
    y: float = 0.05,
    z: float = 0.03,
) -> dict[str, float]:
    """Initial extrinsic guess from physical mount measurements."""
    return {"roll": roll, "pitch": pitch, "yaw": yaw, "x": x, "y": y, "z": z}


def find_matches(
    session_dir: Path,
    camera_id: str,
    extrinsic_seed: dict[str, float],
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Find feature correspondences between iPhone depth projection and Insta360 ERP.

    Port of atlas-scanner's find_matches_superglue_erp.py.
    Uses SuperGlue for learned feature matching on equirectangular projections.
    """
    # TODO: Load iPhone depth maps and project to LiDAR intensity images
    # TODO: Load Insta360 ERP panoramas
    # TODO: Project LiDAR points into ERP frame using seed extrinsic
    # TODO: Run SuperGlue matching
    # TODO: Return matched point pairs
    print(f"  Finding matches for {camera_id}...")
    return []


def tune_calibration(
    matches: list[tuple[np.ndarray, np.ndarray]],
    seed: dict[str, float],
    max_iterations: int = 100,
) -> dict[str, float]:
    """Iteratively refine extrinsic by minimizing reprojection error.

    Port of atlas-scanner's tune_calibration.py.
    """
    # TODO: Set up optimization (scipy.optimize.minimize)
    # TODO: Cost function: sum of reprojection errors for matched features
    # TODO: Optimize 6-DOF (roll, pitch, yaw, x, y, z)
    print(f"  Refining calibration (max {max_iterations} iterations)...")
    return seed


def verify_overlay(
    session_dir: Path,
    camera_id: str,
    extrinsic: dict[str, float],
    output_path: Path,
) -> None:
    """Generate visual overlay of colorized LiDAR on Insta360 panorama.

    Port of atlas-scanner's verify_seed_overlay.py.
    """
    # TODO: Project iPhone LiDAR points into Insta360 ERP using calibrated extrinsic
    # TODO: Colorize points by depth
    # TODO: Overlay on panorama image
    # TODO: Save verification image
    print(f"  Verification overlay -> {output_path}")


def save_calibration(extrinsic: dict[str, float], output_path: Path) -> None:
    """Save calibration in YAML format compatible with multi_camera.yaml."""
    # TODO: Write YAML with roll/pitch/yaw/x/y/z
    print(f"  Saved calibration -> {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Atlas Mobile extrinsic calibration")
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--camera-id", type=str, required=True)
    parser.add_argument("--seed-from-physical", action="store_true")
    parser.add_argument("--seed-roll", type=float, default=0.0)
    parser.add_argument("--seed-pitch", type=float, default=0.0)
    parser.add_argument("--seed-yaw", type=float, default=0.0)
    parser.add_argument("--seed-x", type=float, default=0.0)
    parser.add_argument("--seed-y", type=float, default=0.05)
    parser.add_argument("--seed-z", type=float, default=0.03)
    args = parser.parse_args()

    print(f"Calibrating {args.camera_id} from session {args.session_dir}")

    seed = load_physical_seed(
        args.seed_roll, args.seed_pitch, args.seed_yaw,
        args.seed_x, args.seed_y, args.seed_z,
    )
    print(f"  Seed: {seed}")

    matches = find_matches(args.session_dir, args.camera_id, seed)
    if not matches:
        print("  No matches found — using seed as final calibration")
        refined = seed
    else:
        refined = tune_calibration(matches, seed)

    output_dir = args.session_dir / "calibration"
    output_dir.mkdir(parents=True, exist_ok=True)

    save_calibration(refined, output_dir / f"{args.camera_id}_extrinsic.yaml")
    verify_overlay(
        args.session_dir,
        args.camera_id,
        refined,
        output_dir / f"{args.camera_id}_overlay.png",
    )


if __name__ == "__main__":
    main()

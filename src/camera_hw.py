#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Shared helper to load camera hardware profile (ERP resolution,
# mask filenames, calibration path) for a given session or camera_hw name.

import json
import yaml
from pathlib import Path

_SRC = Path(__file__).resolve().parent  # atlas-scanner/src/
_MODEL_DIR = _SRC / 'config' / 'camera_models'
_CALIB_DIR = _SRC / 'config' / 'calibrations'
_MASK_DIR = _SRC / 'config' / 'masks'

_DEFAULTS = {
    'onex2': {
        'erp_width':         5760,
        'erp_height':        2880,
        'lidar_mask_dual':   'lidar_mask_dual_sdk.png',
        'lidar_mask_single': 'lidar_mask_single.png',
        'display_name':      'Insta360 One X2',
    },
    'x5': {
        'erp_width':         7680,
        'erp_height':        3840,
        'lidar_mask_dual':   'lidar_mask_dual_x5.png',
        'lidar_mask_single': 'lidar_mask_single_x5.png',
        'display_name':      'Insta360 X5',
    },
}


def load_camera_profile(camera_hw: str) -> dict:
    """Return merged profile dict for camera_hw."""
    defaults = dict(_DEFAULTS.get(camera_hw, _DEFAULTS['onex2']))
    hw_yaml  = _MODEL_DIR / f'{camera_hw}.yaml'
    if hw_yaml.exists():
        try:
            overrides = yaml.safe_load(hw_yaml.read_text()) or {}
            defaults.update({k: v for k, v in overrides.items() if v is not None})
        except Exception:
            pass
    defaults['camera_hw'] = camera_hw
    return defaults


def camera_hw_for_session(session_dir) -> str:
    """Read camera_hw from session_config.json, falling back to 'onex2'."""
    cfg = Path(session_dir) / 'session_config.json'
    if cfg.exists():
        try:
            return json.loads(cfg.read_text()).get('camera_hw', 'onex2')
        except Exception:
            pass
    return 'onex2'


def is_blur_skipped(scan_dir) -> bool:
    """Return True if this scan has been marked blurry by filter_blurry_scans.py."""
    return (Path(scan_dir) / '.blur_skip').exists()


def iter_scan_dirs(session_path) -> list:
    """Return sorted fusion_scan_* dirs that have not been blur-filtered."""
    return sorted(
        d for d in Path(session_path).iterdir()
        if d.is_dir() and d.name.startswith('fusion_scan_')
        and not (d / '.blur_skip').exists()
    )


def mask_path(profile: dict, camera_mode: str, sdk_stitch: bool = True) -> Path:
    """Return the absolute lidar mask path for the given mode.
    sdk_stitch parameter kept for API compatibility but ignored — SDK stitch
    is always used so the SDK mask is always selected.
    Looks in config/masks/ first, falls back to src/ root for legacy layouts.
    """
    if camera_mode == 'dual_fisheye':
        name = profile.get('lidar_mask_dual', 'lidar_mask_dual_sdk.png')
    else:
        name = profile.get('lidar_mask_single', 'lidar_mask_single.png')
    p = _MASK_DIR / name
    if p.exists():
        return p
    return _SRC / name  # legacy fallback


def calibration_path(camera_hw: str, cam_index: int = None) -> Path:
    """Return the per-model (and optionally per-camera) calibration YAML.
    If cam_index is provided, looks for calibrations/<hw>/cam_<N>/fusion_calibration.yaml
    first, then falls back to calibrations/<hw>/fusion_calibration.yaml.
    """
    if cam_index is not None:
        p = _CALIB_DIR / camera_hw / f'cam_{cam_index}' / 'fusion_calibration.yaml'
        if p.exists():
            return p
    p = _CALIB_DIR / camera_hw / 'fusion_calibration.yaml'
    if p.exists():
        return p
    return _SRC / 'config' / 'fusion_calibration.yaml'


def cam_index_for_scan(scan_dir) -> int:
    """Read .cam_index from a scan directory. Returns 0 if not present."""
    ci = Path(scan_dir) / '.cam_index'
    if ci.exists():
        try:
            return int(ci.read_text().strip())
        except (ValueError, OSError):
            pass
    return 0

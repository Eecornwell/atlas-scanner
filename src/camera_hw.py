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


def mask_path(profile: dict, camera_mode: str, sdk_stitch: bool = False) -> Path:
    """Return the absolute lidar mask path for the given mode."""
    if camera_mode == 'dual_fisheye':
        name = profile.get('lidar_mask_dual', 'lidar_mask_dual_sdk.png')
    else:
        name = profile.get('lidar_mask_single', 'lidar_mask_single.png')
    return _SRC / name


def calibration_path(camera_hw: str) -> Path:
    """Return the per-model calibration YAML, falling back to shared one."""
    p = _CALIB_DIR / camera_hw / 'fusion_calibration.yaml'
    if p.exists():
        return p
    return _SRC / 'config' / 'fusion_calibration.yaml'

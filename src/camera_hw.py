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
        'display_name':      'Insta360 One X2',
    },
    'x3': {
        'erp_width':         5760,
        'erp_height':        2880,
        'lidar_mask_dual':   'lidar_mask_dual_x3_cam_left.png',
        'display_name':      'Insta360 X3',
    },
    'x5': {
        'erp_width':         7680,
        'erp_height':        3840,
        'lidar_mask_dual':   'lidar_mask_dual_x5.png',
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
    """Return the absolute lidar dual mask path for the given camera profile.
    Single fisheye center masking is applied programmatically at runtime,
    not via a separate mask file.
    """
    name = profile.get('lidar_mask_dual', 'lidar_mask_dual_sdk.png')
    p = _MASK_DIR / name
    if p.exists():
        return p
    return _SRC / name  # legacy fallback


def calibration_path(camera_hw: str, cam_index: int = None) -> Path:
    """Return the calibration YAML for a camera.

    Resolution order:
    1. Per-camera slot path from multi_camera.yaml matched by BOTH cam_index
       AND camera_hw — prevents a single-camera session (cam_index=0) from
       picking up cam_0's slot when the connected camera is actually cam_1 or cam_2.
    2. Legacy per-hw/cam_N path: calibrations/<hw>/cam_<N>/fusion_calibration.yaml
    3. Per-hw path: calibrations/<hw>/fusion_calibration.yaml
    4. Active shared calibration: config/fusion_calibration.yaml
    """
    if cam_index is not None:
        mc_path = _SRC / 'config' / 'multi_camera.yaml'
        if mc_path.exists():
            try:
                mc = yaml.safe_load(mc_path.read_text()) or {}
                cameras = mc.get('cameras', {})

                # Priority 1a: exact match on both index AND hw
                cam_cfg = cameras.get(f'cam_{cam_index}', {})
                if cam_cfg.get('camera_hw', '') == camera_hw:
                    calib_rel = cam_cfg.get('calibration', '')
                    if calib_rel:
                        slot_path = _SRC / 'config' / calib_rel
                        if slot_path.exists():
                            return slot_path

                # Priority 1b: index matches but hw differs — find the slot
                # whose hw matches (handles single-camera sessions where the
                # SDK assigns index 0 regardless of physical slot position).
                for slot_key, slot_cfg in cameras.items():
                    if slot_cfg.get('camera_hw', '') == camera_hw:
                        calib_rel = slot_cfg.get('calibration', '')
                        if calib_rel:
                            slot_path = _SRC / 'config' / calib_rel
                            if slot_path.exists():
                                return slot_path
            except Exception:
                pass
        # Priority 2: legacy per-hw/cam_N path
        p = _CALIB_DIR / camera_hw / f'cam_{cam_index}' / 'fusion_calibration.yaml'
        if p.exists():
            return p
    # Priority 3: per-hw path
    p = _CALIB_DIR / camera_hw / 'fusion_calibration.yaml'
    if p.exists():
        return p
    # Priority 4: active shared calibration
    return _SRC / 'config' / 'fusion_calibration.yaml'


def cam_index_for_scan(scan_dir) -> int:
    """Read .cam_index from a scan directory.

    New format (written by main_multi.cpp): "<sdk_index> <serial>"
    Old format: "<sdk_index>"

    When a serial is present, resolve the multi_camera.yaml slot by serial
    so the correct calibration is used regardless of USB enumeration order.
    Returns the slot index (0/1/2) that matches the serial, or the raw
    sdk_index as fallback.
    """
    ci_path = Path(scan_dir) / '.cam_index'
    if not ci_path.exists():
        return 0
    try:
        parts = ci_path.read_text().strip().split()
        sdk_index = int(parts[0])
        if len(parts) < 2:
            return sdk_index  # old format — no serial
        serial = parts[1]
        # Look up which multi_camera.yaml slot has this serial
        mc_path = _SRC / 'config' / 'multi_camera.yaml'
        if mc_path.exists():
            try:
                mc = yaml.safe_load(mc_path.read_text()) or {}
                for slot_key, cam_cfg in mc.get('cameras', {}).items():
                    if cam_cfg.get('serial', '') == serial:
                        return int(slot_key.split('_')[1])
            except Exception:
                pass
        return sdk_index
    except (ValueError, OSError):
        return 0

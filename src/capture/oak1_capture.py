#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Capture node for the Luxonis OAK-1 (fixed-focus) camera.
# Compatible with DepthAI v3 (depthai >= 3.0).
#
# Architecture: each depthai session runs in a fresh subprocess (not fork)
# so depthai's internal threads start clean and pipeline.stop() segfaults
# or firmware crashes cannot affect the parent trigger loop.

import os
import sys
import time
import subprocess
import tempfile
import yaml
import numpy as np
from pathlib import Path
from datetime import datetime

_ALLOWED_DATA = Path.home() / "atlas_ws" / "data"
_SELF = Path(__file__).resolve()


def _safe(p: Path) -> Path:
    resolved = p.resolve()
    if _ALLOWED_DATA.resolve() not in [resolved, *resolved.parents]:
        raise ValueError(f"Path '{resolved}' escapes allowed root")
    return resolved


# ── helpers used only in subprocess workers ──────────────────────────────────

def _worker_imports():
    try:
        import depthai as dai
        import cv2
        return dai, cv2
    except ImportError as e:
        print(f"✗ Import error: {e}", flush=True)
        sys.exit(1)


def _build_pipeline(dai, width: int, height: int):
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.Camera, dai.CameraBoardSocket.CAM_A)
    cam.build()
    stream_out = cam.requestOutput((width, height), dai.ImgFrame.Type.BGR888p, fps=2.1)
    q = stream_out.createOutputQueue(maxSize=4, blocking=False)
    ctrl_q = cam.inputControl.createInputQueue()
    return pipeline, q, ctrl_q


def _apply_isp(dai, ctrl_q, settle_s: float = 0.0):
    if settle_s > 0:
        time.sleep(settle_s)
    ctrl = dai.CameraControl()
    ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
    ctrl.setSaturation(2)
    ctrl.setSharpness(1)
    ctrl_q.send(ctrl)


def _wait_ae(stream_q, timeout=6.0):
    """Wait for AE to stabilise: 3 consecutive frames within 5% brightness delta.
    Timeout reduced to 6s — with USB reset between captures device enumerates
    faster and AE converges in 2-4s on USB2."""
    prev, stable, last_frame = 0.0, 0, None
    t0 = time.time()
    while time.time() - t0 < timeout:
        f = stream_q.tryGet()
        if f is not None:
            import cv2
            fr = f.getCvFrame()
            m = float(fr.mean())
            if m > 10:
                if abs(m - prev) / max(prev, 1.0) < 0.05:
                    stable += 1
                    if stable >= 3:
                        return fr
                else:
                    stable = 0
                prev = m
                last_frame = fr
        time.sleep(0.05)
    return last_frame


# ── subprocess entry points ───────────────────────────────────────────────────

def _cmd_warmup(width: int, height: int, out_yaml: str):
    """Subprocess: connect, read intrinsics, warm AE, write yaml. No stop()."""
    dai, cv2 = _worker_imports()
    try:
        pipeline, stream_q, ctrl_q = _build_pipeline(dai, width, height)
        pipeline.start()
        device = pipeline.getDefaultDevice()
        spd = device.getUsbSpeed()
        tag = "⚠ USB2" if spd == dai.UsbSpeed.HIGH else "✓ USB3"
        print(f"✓ OAK-1 connected: {device.getDeviceId()} [{tag} — {spd.name}]", flush=True)
        calib = device.readCalibration()
        M = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height)
        D = calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        info = {
            "width": width, "height": height,
            "fx": float(M[0][0]), "fy": float(M[1][1]),
            "cx": float(M[0][2]), "cy": float(M[1][2]),
            "distortion_model": "rational_polynomial",
            "D": [float(d) for d in D],
        }
        print(f"✓ Intrinsics: fx={info['fx']:.1f} fy={info['fy']:.1f} "
              f"cx={info['cx']:.1f} cy={info['cy']:.1f}", flush=True)
        Path(out_yaml).write_text(yaml.dump(info, default_flow_style=False))
        _apply_isp(dai, ctrl_q)
        t0 = time.time()
        while time.time() - t0 < 6.0:
            f = stream_q.tryGet()
            if f is not None and f.getCvFrame().mean() > 40:
                print(f"✓ AE/AWB converged (mean={f.getCvFrame().mean():.1f})", flush=True)
                break
            time.sleep(0.05)
        # Never call pipeline.stop() — crashes firmware on USB2. OS cleans up.
        # Use os._exit to skip Python/DepthAI destructor which hangs on USB2.
        os._exit(0)
    except Exception as e:
        print(f"✗ Warmup error: {e}", flush=True)
        os._exit(1)


def _cmd_capture(width: int, height: int, out_png: str):
    """Subprocess: connect, capture one frame, write png. No stop()."""
    dai, cv2 = _worker_imports()
    try:
        pipeline, stream_q, ctrl_q = _build_pipeline(dai, width, height)
        pipeline.start()
        _apply_isp(dai, ctrl_q, settle_s=0.5)
        frame = _wait_ae(stream_q)
        # drain briefly for freshest frame
        if frame is not None:
            t1 = time.time()
            while time.time() - t1 < 1.5:
                f = stream_q.tryGet()
                if f is not None:
                    frame = f.getCvFrame()
                elif frame is not None:
                    break
                time.sleep(0.02)
        if frame is not None:
            cv2.imwrite(out_png, frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            os._exit(0)
        else:
            os._exit(1)
    except Exception as e:
        print(f"✗ Capture error: {e}", flush=True)
        os._exit(1)


# ── parent-side helpers ───────────────────────────────────────────────────────

def _run_subprocess(args: list, timeout: int) -> subprocess.CompletedProcess:
    """Run a fresh Python interpreter with the given args, streaming output."""
    return subprocess.run(
        [sys.executable, str(_SELF)] + args,
        timeout=timeout,
    )


def _usb_reset_oak1():
    """USBDEVFS_RESET ioctl, then poll until device is UNBOOTED."""
    import fcntl
    import depthai as dai
    USBDEVFS_RESET = 0x5514
    try:
        for vendor_file in Path("/sys/bus/usb/devices").glob("*/idVendor"):
            if vendor_file.read_text().strip() == "03e7":
                devpath = vendor_file.parent
                busnum = int((devpath / "busnum").read_text().strip())
                devnum = int((devpath / "devnum").read_text().strip())
                node = f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"
                with open(node, "wb") as f:
                    fcntl.ioctl(f, USBDEVFS_RESET, 0)
                print(f"✓ USB reset: {node}", flush=True)
                for _ in range(20):
                    time.sleep(0.5)
                    if dai.Device.getAllAvailableDevices():
                        print("✓ Device ready", flush=True)
                        return
                print("⚠ Device did not re-enumerate", flush=True)
                return
    except Exception as e:
        print(f"⚠ USB reset failed: {e}", flush=True)
        time.sleep(3.0)


def _enhance_frame(frame: np.ndarray, alpha: float = 1.0, beta: int = 0) -> np.ndarray:
    """Simple global contrast/brightness adjustment. alpha>1 increases contrast, beta shifts brightness."""
    import cv2
    return cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)


def _write_colmap_camera(info: dict, scan_dir: Path, actual_w: int, actual_h: int):
    sx, sy = actual_w / info["width"], actual_h / info["height"]
    D = info["D"]
    k1, k2, p1, p2 = D[0], D[1], D[2], D[3]
    k3 = D[4] if len(D) > 4 else 0.0
    k4 = D[5] if len(D) > 5 else 0.0
    k5 = D[6] if len(D) > 6 else 0.0
    k6 = D[7] if len(D) > 7 else 0.0
    line = (f"1 FULL_OPENCV {actual_w} {actual_h} "
            f"{info['fx']*sx} {info['fy']*sy} {info['cx']*sx} {info['cy']*sy} "
            f"{k1} {k2} {p1} {p2} {k3} {k4} {k5} {k6}")
    (scan_dir / "colmap_camera.txt").write_text("# COLMAP cameras.txt\n" + line + "\n")


def _undistort_and_save(frame: np.ndarray, info: dict, out_path: Path):
    import cv2
    h, w = frame.shape[:2]
    sx, sy = w / info["width"], h / info["height"]
    K = np.array([[info["fx"]*sx, 0, info["cx"]*sx],
                  [0, info["fy"]*sy, info["cy"]*sy],
                  [0, 0, 1]], dtype=np.float64)
    dist = np.array(info["D"][:8], dtype=np.float64)
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0.0)
    undist = cv2.undistort(frame, K, dist, None, new_K)
    cv2.imwrite(str(out_path), undist, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    # Save validity mask — white where undistortion produced valid pixels,
    # black where the crop left empty corners. Used by exact_match_fusion
    # to reject points that land on invalid pixels.
    ones = np.ones((h, w), dtype=np.uint8) * 255
    valid_mask = cv2.undistort(ones, K, dist, None, new_K)
    _, valid_mask = cv2.threshold(valid_mask, 128, 255, cv2.THRESH_BINARY)
    cv2.imwrite(str(out_path.with_name(out_path.stem + '_mask.png')), valid_mask)
    # Save the undistorted intrinsics alongside the image so downstream tools
    # (exact_match_fusion, oak1_lidar_colorize) use the correct K for projection.
    undist_info = {
        "width": w, "height": h,
        "fx": float(new_K[0, 0]), "fy": float(new_K[1, 1]),
        "cx": float(new_K[0, 2]), "cy": float(new_K[1, 2]),
        "distortion_model": "none",
        "D": [0.0, 0.0, 0.0, 0.0, 0.0],
    }
    out_path.with_suffix('.yaml').write_text(yaml.dump(undist_info, default_flow_style=False))


def _run_capture(width: int, height: int, max_attempts: int = 3):
    """Run capture subprocess, retry with USB reset on failure."""
    import cv2
    for attempt in range(max_attempts):
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tf:
            tmp = tf.name
        try:
            result = _run_subprocess(["--capture", str(width), str(height), tmp],
                                     timeout=30)
            if result.returncode == 0 and os.path.getsize(tmp) > 0:
                frame = cv2.imread(tmp)
                if frame is not None:
                    return frame
            print(f"✗ Capture attempt {attempt+1}/{max_attempts} failed "
                  f"(rc={result.returncode})", flush=True)
        except subprocess.TimeoutExpired:
            print(f"✗ Capture attempt {attempt+1}/{max_attempts} timed out", flush=True)
        finally:
            try:
                os.unlink(tmp)
            except OSError:
                pass
        if attempt < max_attempts - 1:
            _usb_reset_oak1()
    return None


# ── main trigger loop ─────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Usage: oak1_capture.py <scan_dir>", flush=True)
        sys.exit(1)

    scan_dir = Path(sys.argv[1])
    try:
        scan_dir = _safe(scan_dir)
    except ValueError as e:
        print(f"✗ {e}", flush=True)
        sys.exit(1)
    scan_dir.mkdir(parents=True, exist_ok=True)

    cam_index = "0"
    if "--cam-index" in sys.argv:
        idx = sys.argv.index("--cam-index")
        if idx + 1 < len(sys.argv):
            cam_index = sys.argv[idx + 1]

    _src = _SELF.parent.parent
    hw_yaml = _src / "config" / "camera_models" / "oak1.yaml"
    cfg = yaml.safe_load(hw_yaml.read_text()) if hw_yaml.exists() else {}
    width = int(cfg.get("image_width", 4056))
    height = int(cfg.get("image_height", 3040))

    # ── Warmup: read intrinsics via subprocess (safe isolation) ──────────────
    print(f"Building pipeline ({width}x{height})...", flush=True)
    with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as tf:
        tmp_info = tf.name
    try:
        result = _run_subprocess(["--warmup", str(width), str(height), tmp_info],
                                 timeout=30)
        if result.returncode != 0 or not os.path.exists(tmp_info):
            print("✗ Warmup failed", flush=True)
            sys.exit(1)
        info = yaml.safe_load(Path(tmp_info).read_text())
    except subprocess.TimeoutExpired:
        print("✗ Warmup timed out", flush=True)
        sys.exit(1)
    finally:
        try:
            os.unlink(tmp_info)
        except OSError:
            pass

    # USB reset after warmup so device is in UNBOOTED state for the persistent pipeline
    _usb_reset_oak1()

    # ── Start persistent pipeline — stays open for the whole session ──────────
    # This eliminates the per-capture subprocess overhead (~8-17s) and allows
    # capturing every X5 shot at 5s intervals.
    try:
        import depthai as dai
        import cv2
    except ImportError as e:
        print(f"✗ Import error: {e}", flush=True)
        sys.exit(1)

    def _start_pipeline(wait_ae=True):
        """Start pipeline and wait for AE. Returns (pipeline, stream_q, ctrl_q)."""
        p, sq, cq = _build_pipeline(dai, width, height)
        p.start()
        _apply_isp(dai, cq, settle_s=0.5)
        if wait_ae:
            print("Waiting for AE...", flush=True)
            _wait_ae(sq)
            # Verify frames are actually flowing after AE — XLink may have
            # reconnected internally leaving the pipeline in a bad state.
            t0 = time.time()
            verified = False
            while time.time() - t0 < 3.0:
                f = sq.tryGet()
                if f is not None:
                    verified = True
                    break
                time.sleep(0.05)
            if not verified:
                raise RuntimeError("Pipeline started but no frames flowing after AE")
            print("✓ AE ready", flush=True)
        return p, sq, cq

    # Retry initial pipeline start up to 3 times with USB reset on failure.
    # No full AE wait — warmup already converged AE, but do a stability soak
    # to confirm the pipeline is truly stable before signaling ready.
    for _attempt in range(3):
        try:
            pipeline, stream_q, ctrl_q = _start_pipeline(wait_ae=False)
            # Stability soak: read frames for 3s to confirm pipeline is stable
            # and AE has settled. XLink errors surface here rather than mid-session.
            _t0 = time.time()
            _frames_ok = 0
            _bright_ok = False
            while time.time() - _t0 < 5.0:
                _f = stream_q.tryGet()
                if _f is not None:
                    _frames_ok += 1
                    if _f.getCvFrame().mean() > 40:
                        _bright_ok = True
                time.sleep(0.05)
            if _frames_ok == 0:
                raise RuntimeError("No frames during stability soak")
            print(f"✓ Pipeline stable ({_frames_ok} frames, bright={_bright_ok})", flush=True)
            break
        except Exception as _e:
            print(f"⚠ Pipeline start failed (attempt {_attempt+1}/3): {_e}", flush=True)
            # os._exit to bypass DepthAI destructor which crashes on bad XLink state.
            # Re-launch self as a fresh process for the next attempt.
            if _attempt < 2:
                _usb_reset_oak1()
                # Re-exec self so DepthAI starts with clean internal state
                os.execv(sys.executable, [sys.executable, str(_SELF)] + sys.argv[1:])
            os._exit(1)
    else:
        print("✗ Could not start pipeline after 3 attempts", flush=True)
        sys.exit(1)

    _safe(scan_dir / "camera_info.yaml").write_text(yaml.dump(info, default_flow_style=False))
    (scan_dir / ".sdk_ready").touch()
    (scan_dir / ".oak1_ready").touch()
    print("Waiting for capture trigger...", flush=True)

    while True:
        if (scan_dir / ".oak1_quit_trigger").exists():
            (scan_dir / ".oak1_quit_trigger").unlink(missing_ok=True)
            print("Quit trigger — exiting", flush=True)
            os._exit(0)

        trigger = scan_dir / ".oak1_trigger"
        if trigger.exists():
            target_line = trigger.read_text().strip()
            trigger.unlink(missing_ok=True)
            target_dir = Path(target_line) if target_line else scan_dir
            try:
                target_dir = _safe(target_dir)
            except ValueError:
                target_dir = scan_dir
            target_dir.mkdir(parents=True, exist_ok=True)
            (target_dir / "camera_info.yaml").write_text(yaml.dump(info, default_flow_style=False))

            print("Capturing...", flush=True)
            trigger_time = time.time()
            frame = None
            frame_ts = None
            try:
                # Flush all frames captured before the trigger arrived,
                # then wait for the first frame captured after the trigger.
                # This ensures we capture while stationary (after X5 shutter)
                # not while moving (frames buffered before the trigger).
                while stream_q.tryGet() is not None:
                    pass  # flush pre-trigger frames
                # Wait up to 3s for first post-trigger frame
                t0 = time.time()
                while time.time() - t0 < 3.0:
                    f = stream_q.tryGet()
                    if f is not None:
                        frame = f.getCvFrame()
                        try:
                            ts = f.getTimestampSystem()
                            frame_ts = ts.total_seconds()
                        except Exception:
                            frame_ts = time.time()
                        break
                    time.sleep(0.02)
            except Exception as _e:
                print(f"⚠ Pipeline error: {_e} — reconnecting...", flush=True)
                # Do NOT call pipeline.stop() — destructor crashes on bad XLink state
                _usb_reset_oak1()
                try:
                    pipeline, stream_q, ctrl_q = _start_pipeline(wait_ae=False)
                    # Short stability soak after reconnect
                    _t0 = time.time()
                    _ok = 0
                    while time.time() - _t0 < 3.0:
                        _f = stream_q.tryGet()
                        if _f is not None:
                            _ok += 1
                        time.sleep(0.05)
                    if _ok == 0:
                        raise RuntimeError("No frames after reconnect")
                    # Discard ALL triggers written during reconnect — they're stale
                    while (scan_dir / '.oak1_trigger').exists():
                        (scan_dir / '.oak1_trigger').unlink(missing_ok=True)
                        print("⚠ Discarded stale trigger after reconnect", flush=True)
                        time.sleep(0.1)  # brief pause in case main_multi writes another
                except Exception as _e2:
                    print(f"✗ Reconnect failed: {_e2}", flush=True)

            if frame is not None:
                raw_mean = frame.mean()
                capture_ts = frame_ts if frame_ts is not None else time.time()
                frame_age = time.time() - capture_ts
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                frame = _enhance_frame(frame)
                out_path = _safe(target_dir / f"oak1_{timestamp}.png")
                cv2.imwrite(str(out_path), frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                print(f"✓ Saved: {out_path} "
                      f"(raw_mean={raw_mean:.1f} frame_age={frame_age:.2f}s)", flush=True)
                _write_colmap_camera(info, target_dir, frame.shape[1], frame.shape[0])
                undist_path = _safe(target_dir / f"oak1_{timestamp}_undistorted.png")
                _undistort_and_save(frame, info, undist_path)
                print(f"✓ Undistorted: {undist_path.name}", flush=True)
                _safe(target_dir / "capture_0.shutter_event").write_text(
                    f"{capture_ts:.6f} {cam_index}\n")
                _safe(target_dir / ".cam_index").write_text(f"{cam_index}\n")
                (target_dir / ".oak1_capture").touch()
                # Only write SDK sentinels when running as primary camera.
                # In secondary mode (alongside X5) these interfere with
                # the X5 capture wait loop in atlas_fusion_capture.sh.
                if cam_index == "0":
                    (scan_dir / ".sdk_downloads_pending").write_text("0")
                    (scan_dir / ".sdk_capture_done").touch()
            else:
                print("✗ No frame received", flush=True)
                if cam_index == "0":
                    (scan_dir / ".sdk_capture_failed").touch()

        else:
            # Idle — check pipeline is still alive by peeking at the queue
            try:
                stream_q.tryGet()
            except Exception as _e:
                print(f"⚠ Pipeline dropped in idle: {_e} — reconnecting...", flush=True)
                # Do NOT call pipeline.stop() — destructor crashes on bad XLink state
                _usb_reset_oak1()
                try:
                    pipeline, stream_q, ctrl_q = _start_pipeline(wait_ae=False)
                    # Short stability soak
                    _t0 = time.time()
                    while time.time() - _t0 < 3.0:
                        stream_q.tryGet()
                        time.sleep(0.05)
                except Exception as _e2:
                    print(f"✗ Reconnect failed: {_e2}", flush=True)

        time.sleep(0.05)


# ── subprocess dispatch ───────────────────────────────────────────────────────

if __name__ == "__main__":
    # Handle SIGTERM with os._exit to bypass DepthAI destructor which crashes
    import signal as _signal
    _signal.signal(_signal.SIGTERM, lambda *_: os._exit(0))
    if len(sys.argv) >= 2 and sys.argv[1] == "--warmup":
        _, _, w, h, out = sys.argv
        _cmd_warmup(int(w), int(h), out)
    elif len(sys.argv) >= 2 and sys.argv[1] == "--capture":
        _, _, w, h, out = sys.argv
        _cmd_capture(int(w), int(h), out)
    else:
        main()

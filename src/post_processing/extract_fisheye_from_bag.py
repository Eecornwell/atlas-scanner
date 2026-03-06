#!/usr/bin/env python3
"""
Extract the best dual-fisheye frame from a rosbag and convert to ERP.
Reads the full H.264 stream from /dual_fisheye/image/compressed,
writes it to a temp file, decodes with ffmpeg, then runs fisheye_to_erp.
"""

import sys
import os
import sqlite3
import subprocess
import tempfile
import argparse
from pathlib import Path

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("✗ rclpy not available — source your ROS workspace first")
    sys.exit(1)


def open_db3(bag_dir):
    bag_path = Path(bag_dir)
    db3_files = sorted(bag_path.glob("*.db3"))
    if db3_files:
        return sqlite3.connect(str(db3_files[0]))
    zstd = sorted(bag_path.glob("*.db3.zstd"))
    if not zstd:
        raise FileNotFoundError(f"No .db3 file in {bag_dir}")
    out = str(zstd[0]).replace(".zstd", "")
    subprocess.run(["zstd", "-d", str(zstd[0]), "-o", out, "-f"], check=True, capture_output=True)
    return sqlite3.connect(out)


def extract_and_convert(bag_dir, scan_dir, dual=True):
    con = open_db3(bag_dir)
    topics = {r[0]: (r[1], r[2]) for r in con.execute("SELECT id, name, type FROM topics")}

    tid = next((tid for tid, (n, _) in topics.items()
                if '/dual_fisheye/image/compressed' in n), None)
    if tid is None:
        print(f"✗ No /dual_fisheye/image/compressed topic in {bag_dir}")
        return False

    MsgType = get_message(topics[tid][1])
    rows = con.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
    ).fetchall()
    con.close()

    if not rows:
        print(f"✗ No compressed image messages in {bag_dir}")
        return False

    # Concatenate all H.264 packets into one stream so ffmpeg has SPS/PPS context
    with tempfile.NamedTemporaryFile(suffix='.h264', delete=False) as tmp:
        for (data,) in rows:
            msg = deserialize_message(bytes(data), MsgType)
            tmp.write(bytes(msg.data))
        tmp_path = tmp.name

    # Decode middle frame (more likely to be a clean I-frame than the first)
    mid = max(0, len(rows) // 2)
    fisheye_path = os.path.join(scan_dir, 'dual_fisheye.jpg')
    result = subprocess.run([
        'ffmpeg', '-y', '-i', tmp_path,
        '-vf', f'select=eq(n\\,{mid})', '-frames:v', '1', '-q:v', '2', fisheye_path
    ], capture_output=True)
    os.unlink(tmp_path)

    if result.returncode != 0 or not os.path.exists(fisheye_path):
        # Fallback: grab first decodable frame
        result = subprocess.run([
            'ffmpeg', '-y', '-i', tmp_path if os.path.exists(tmp_path) else '/dev/null',
            '-frames:v', '1', '-q:v', '2', fisheye_path
        ], capture_output=True)

    if not os.path.exists(fisheye_path):
        print(f"✗ ffmpeg could not decode any frame from {bag_dir}")
        return False

    print(f"✓ Decoded fisheye frame: {fisheye_path}")

    # Convert to ERP
    script_dir = os.path.dirname(os.path.abspath(__file__))
    erp_path = os.path.join(scan_dir, 'equirect_dual_fisheye.jpg')
    cmd = [sys.executable, os.path.join(script_dir, 'fisheye_to_erp.py'),
           fisheye_path, erp_path]
    if dual:
        cmd.append('--dual')
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"✗ ERP conversion failed: {result.stderr}")
        return False

    print(f"✓ ERP saved: {erp_path}")
    return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_dir', help='Path to rosbag_* directory')
    parser.add_argument('scan_dir', help='Scan output directory')
    parser.add_argument('--dual', action='store_true',
                        help='Apply dual-fisheye back-to-front calibration')
    args = parser.parse_args()
    ok = extract_and_convert(args.bag_dir, args.scan_dir, args.dual)
    sys.exit(0 if ok else 1)

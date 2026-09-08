#!/usr/bin/env python3
"""Fallback: project a single-fisheye JPEG to equirectangular ERP.
Usage: fisheye_to_erp.py <input.insp> <output.jpg> <erp_width> <erp_height>
"""
import sys, cv2, numpy as np

input_path, output_path = sys.argv[1], sys.argv[2]
erp_w, erp_h = int(sys.argv[3]), int(sys.argv[4])

data = open(input_path, 'rb').read()
fisheye = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
if fisheye is None:
    print(f'Could not decode {input_path}', file=sys.stderr)
    sys.exit(1)

fh, fw = fisheye.shape[:2]
if abs(fw - fh) > fw // 10:
    print(f'Not square ({fw}x{fh}), skipping fallback', file=sys.stderr)
    sys.exit(1)

print(f'Fisheye fallback: {fw}x{fh} -> ERP {erp_w}x{erp_h}')

cx, cy = fw / 2.0, fh / 2.0
f = fw / np.pi  # equidistant: r = f*theta, FOV~180deg

ey_idx = np.arange(erp_h)
ex_idx = np.arange(erp_w)
lat = (0.5 - (ey_idx + 0.5) / erp_h) * np.pi
lon = ((ex_idx + 0.5) / erp_w - 0.5) * 2.0 * np.pi
LON, LAT = np.meshgrid(lon, lat)

X = np.cos(LAT) * np.sin(LON)
Y = np.sin(LAT)
Z = np.cos(LAT) * np.cos(LON)
theta = np.arccos(np.clip(Z, -1, 1))
phi = np.arctan2(Y, X)
r = f * theta

map_x = (cx + r * np.cos(phi)).astype(np.float32)
map_y = (cy + r * np.sin(phi)).astype(np.float32)
valid = (theta < np.pi / 2) & (map_x >= 0) & (map_x < fw) & (map_y >= 0) & (map_y < fh)
map_x[~valid] = 0
map_y[~valid] = 0

erp = cv2.remap(fisheye, map_x, map_y, cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
erp[~valid] = 0

cv2.imwrite(output_path, erp, [cv2.IMWRITE_JPEG_QUALITY, 95])
print(f'Done (fisheye fallback) -> {output_path}')

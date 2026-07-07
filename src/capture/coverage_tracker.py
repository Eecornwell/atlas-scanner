#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Live 2.5D coverage tracker for Gaussian splatting sessions.
# Tracks scanner position in XY floor cells × Z height bands so the operator
# knows which areas need revisiting at a different height. A 360° fisheye
# captures the full horizontal sphere automatically; the gap is always vertical.
#
# Display shows two views side-by-side:
#   LEFT  — coverage quality (visits × height bands hit)
#   RIGHT — missing height bands as a hint (L=low, M=mid, H=high, ·=done)

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# ─── Config ───────────────────────────────────────────────────────────────────
CELL_SIZE        = 0.5   # XY metres per floor cell
HEIGHT_BANDS     = 3     # number of Z bands (low / mid / high)
HEIGHT_BAND_SIZE = 0.6   # metres per height band
HEIGHT_ORIGIN    = -0.6  # Low: [-0.6,0.0)  Mid: [0.0,+0.6)  High: [+0.6,+1.2)
                         # (mid elevation on the pole). With 0.6m bands:
                         # Low:  z in [-0.9, -0.3) — pole lowered ~0.6m from start
                         # Mid:  z in [-0.3, +0.3) — centred on starting height
                         # High: z in [+0.3, +0.9) — pole raised ~0.6m from start
MIN_VISITS_PER_BAND = 2  # visits in a band before it counts as "covered"
RENDER_INTERVAL  = 3.0   # seconds between redraws
MAX_GRID_CELLS   = 50    # max display width/height in cells

# Coverage quality thresholds (number of height bands covered out of HEIGHT_BANDS)
_BAND_LABELS = ['L', 'M', 'H', '4', '5']  # label per band index

def _height_band(z: float) -> int:
    """Map a Z coordinate to a height band index, clamped to [0, HEIGHT_BANDS-1]."""
    return max(0, min(HEIGHT_BANDS - 1, int(math.floor((z - HEIGHT_ORIGIN) / HEIGHT_BAND_SIZE))))

def _coverage_char(band_visits: dict) -> str:
    """Single char summarising coverage quality for a floor cell."""
    covered = sum(1 for v in band_visits.values() if v >= MIN_VISITS_PER_BAND)
    if covered == 0:
        return '·'
    if covered == 1:
        return '░'
    if covered == HEIGHT_BANDS - 1:
        return '▓'
    if covered >= HEIGHT_BANDS:
        return '█'
    return '▒'

def _missing_label(band_visits: dict) -> str:
    """Single char hint: which height bands still need coverage."""
    missing = [_BAND_LABELS[b] for b in range(HEIGHT_BANDS)
               if band_visits.get(b, 0) < MIN_VISITS_PER_BAND]
    if not missing:
        return '·'          # fully covered
    if len(missing) == HEIGHT_BANDS:
        return '?'          # never visited
    return missing[0]       # show the lowest missing band first


class CoverageTracker(Node):
    def __init__(self):
        super().__init__('coverage_tracker')
        # grid[(gx, gy)] = {band_index: visit_count}
        self._grid: dict[tuple, dict] = {}
        self._current = (0.0, 0.0, 0.0)  # x, y, z
        self._total_poses = 0

        self.create_subscription(Odometry, '/rko_lio/odometry', self._odom_cb, 10)
        self.create_timer(RENDER_INTERVAL, self._render)
        band_centres = [HEIGHT_ORIGIN + (b + 0.5) * HEIGHT_BAND_SIZE for b in range(HEIGHT_BANDS)]
        self.get_logger().info(
            f'Coverage tracker started — {HEIGHT_BANDS} height bands at '
            f'{[f"{z:.2f}m" for z in band_centres]}'
        )

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._current = (p.x, p.y, p.z)
        self._total_poses += 1

        key = (int(math.floor(p.x / CELL_SIZE)), int(math.floor(p.y / CELL_SIZE)))
        band = _height_band(p.z)

        cell = self._grid.setdefault(key, {})
        cell[band] = cell.get(band, 0) + 1

    def _render(self):
        if not self._grid:
            return

        keys   = list(self._grid.keys())
        xs     = [k[0] for k in keys]
        ys     = [k[1] for k in keys]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = min(max_x - min_x + 1, MAX_GRID_CELLS)
        span_y = min(max_y - min_y + 1, MAX_GRID_CELLS)

        cx = int(math.floor(self._current[0] / CELL_SIZE))
        cy = int(math.floor(self._current[1] / CELL_SIZE))

        left_rows, right_rows = [], []
        for gy in range(max_y, max_y - span_y, -1):
            left_row, right_row = [], []
            for gx in range(min_x, min_x + span_x):
                if gx == cx and gy == cy:
                    left_row.append('@')
                    right_row.append('@')
                elif (gx, gy) in self._grid:
                    bv = self._grid[(gx, gy)]
                    left_row.append(_coverage_char(bv))
                    right_row.append(_missing_label(bv))
                else:
                    left_row.append(' ')
                    right_row.append(' ')
            left_rows.append('│' + ''.join(left_row) + '│')
            right_rows.append('│' + ''.join(right_row) + '│')

        total_cells  = len(self._grid)
        full_cells   = sum(
            1 for bv in self._grid.values()
            if sum(1 for v in bv.values() if v >= MIN_VISITS_PER_BAND) >= HEIGHT_BANDS
        )
        partial_cells = sum(
            1 for bv in self._grid.values()
            if 0 < sum(1 for v in bv.values() if v >= MIN_VISITS_PER_BAND) < HEIGHT_BANDS
        )
        pct = (full_cells / total_cells * 100) if total_cells else 0

        border = '─' * span_x
        gap    = '   '
        band_labels = '/'.join(_BAND_LABELS[:HEIGHT_BANDS])
        lines = [
            '\033[2J\033[H',
            f'┌{border}┐{gap}┌{border}┐',
            *[f'{l}{gap}{r}' for l, r in zip(left_rows, right_rows)],
            f'└{border}┘{gap}└{border}┘',
            f'  COVERAGE QUALITY{gap}  MISSING HEIGHTS ({band_labels} = low→high)',
            f'  · visited  ░▒▓█ = 1→{HEIGHT_BANDS} height bands{gap}  · = done  L/M/H = needs that band',
            f'',
            f'  @ = current pos  |  Cell {CELL_SIZE}m  |  Band height {HEIGHT_BAND_SIZE}m  |  '
            f'Min visits/band: {MIN_VISITS_PER_BAND}',
            f'  Pos: ({self._current[0]:.1f}, {self._current[1]:.1f}, {self._current[2]:.1f}) m  '
            f'Height band: {_BAND_LABELS[_height_band(self._current[2])]}  |  Poses: {self._total_poses}',
            f'  Cells: {total_cells}  |  Full ({HEIGHT_BANDS}-band): {full_cells} ({pct:.0f}%)  |  '
            f'Partial: {partial_cells}  |  Unvisited: {total_cells - full_cells - partial_cells}',
        ]
        print('\n'.join(lines), flush=True)


def main():
    rclpy.init()
    node = CoverageTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

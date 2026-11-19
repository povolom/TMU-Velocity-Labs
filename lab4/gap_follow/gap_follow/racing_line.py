"""Plan a racing line through a known map, offline, for the obstacle_avoidance node.

Both ForzaETH's race_stack and the UNICORN racing stack drive from a global line computed
once from the mapped track (their global planners), and only step off it around obstacles
that aren't on the map. This is that first half, kept simple: the map's walls and static
obstacles are all known, so the line can be planned through the whole lap with nothing
hidden, which matters in corners the LiDAR can't see round in time.

The search is the node's own (route(), dynamic programming over offset from the centerline
and its slope), run once over the whole lap: every candidate line keeps the car's footprint
(as seen by the course simulator: a 0.33 x 0.31 m box centred on the LiDAR) clear of the
map, never turns tighter than the car can, and among those the one with the most room and
the least turning wins.

    ros2 run gap_follow make_racing_line levine_obs        # writes config/levine_obs_line.csv
"""

import argparse
import os

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

from gap_follow.obstacle_avoidance import CELL_D, DISC_RADIUS, DISCS, load_centerline

STEP = 0.15               # m along the centerline per step
# Tightest turn on the line (1/m), well inside what the car can do (~1.27 for the LiDAR
# point) so it has steering to spare to catch up with the line. Tighter limits buy a little
# room in tight corners but make S-bends the car can't swing its steering through in time.
KAPPA_MAX = 1.0
MARGIN = 0.05             # m the footprint keeps from the map, at least...
ROOM = 0.35               # ...and room up to this much more is rewarded...
ROOM_WEIGHT = 20.0        # ...at this cost per m short, per step
BEND_WEIGHT = 0.5         # per (1/m)^2 of turning, per m
SLOPE = 1.2               # most change in offset per m along the centerline
MAX_SLOPE_CHANGE = 4      # cells of slope change per step searched
OVERLAP = 8.0             # m planned past the lap's end, so the line closes on itself


def load_map(yaml_path):
    """Distance (m) to the nearest wall or obstacle, and a function to look it up."""
    meta = yaml.safe_load(open(yaml_path))
    image = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), meta['image']))
                     .convert('L'), dtype=float)
    free = image > 128.0          # as the simulator reads the map
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]
    dist = distance_transform_edt(free) * res
    rows, cols = free.shape

    def at(x, y):
        c = np.clip(((x - ox) / res).astype(int), 0, cols - 1)
        r = np.clip(rows - 1 - ((y - oy) / res).astype(int), 0, rows - 1)
        return dist[r, c]
    return at


def plan(track, distance_at):
    """The racing line: offsets d (m, left of the centerline) at s = 0, STEP, ... ."""
    cx, cy, tx, ty, left, right, s, length = track
    steps = int(np.ceil((length + OVERLAP) / STEP))
    row_s = np.arange(steps) * STEP

    def interp(v):
        return np.interp(np.mod(row_s, length), np.append(s, length), np.append(v, v[0]))
    bx, by = interp(cx), interp(cy)
    btx, bty = interp(tx), interp(ty)
    norm = np.hypot(btx, bty)
    btx, bty = btx / norm, bty / norm
    d_max = max(left.max(), right.max())
    offsets = np.arange(-d_max, d_max + 1e-9, CELL_D)
    cells = len(offsets)
    inside = (offsets[None, :] <= interp(left)[:, None]) & \
             (offsets[None, :] >= -interp(right)[:, None])
    px = bx[:, None] - offsets[None, :] * bty[:, None]
    py = by[:, None] + offsets[None, :] * btx[:, None]

    max_slope = int(SLOPE * STEP / CELL_D)
    slopes = np.arange(-max_slope, max_slope + 1)
    nslopes = len(slopes)
    cell_idx = np.arange(cells)[:, None]
    prev_cell = cell_idx - slopes[None, :]
    prev_ok = (prev_cell >= 0) & (prev_cell < cells)
    prev_cell = np.clip(prev_cell, 0, cells - 1)

    def room(r):
        """Room around the footprint for every (cell, slope) at step r: the car points
        from its previous point to this one."""
        hx = px[r][cell_idx] - px[r - 1][prev_cell]
        hy = py[r][cell_idx] - py[r - 1][prev_cell]
        h = np.arctan2(hy, hx)
        x, y = px[r][cell_idx], py[r][cell_idx]
        near = np.min([distance_at(x + o * np.cos(h), y + o * np.sin(h)) for o in DISCS],
                      axis=0)
        return near - DISC_RADIUS

    def turn(ax, ay, bx_, by_, cx_, cy_):
        cross = (bx_ - ax) * (cy_ - by_) - (by_ - ay) * (cx_ - bx_)
        sides = np.hypot(bx_ - ax, by_ - ay) * np.hypot(cx_ - bx_, cy_ - by_) * \
            np.hypot(cx_ - ax, cy_ - ay)
        return 2.0 * cross / np.maximum(sides, 1e-9)

    def step_cost(r):
        clear = room(r)
        ok = prev_ok & inside[r][:, None] & (clear >= MARGIN)
        cost = ROOM_WEIGHT * np.clip(MARGIN + ROOM - clear, 0.0, ROOM) * STEP
        return np.where(ok, cost, np.inf)

    # Step 1 from anywhere on the track (the line is free to start where it likes).
    total = step_cost(1)
    came_from = [np.zeros((cells, nslopes), dtype=np.int8)]
    for r in range(2, steps):
        here = step_cost(r)
        best = np.full_like(total, np.inf)
        choice = np.zeros(total.shape, dtype=np.int8)
        for change in range(-MAX_SLOPE_CHANGE, MAX_SLOPE_CHANGE + 1):
            prev_slope = slopes - change
            slope_ok = np.abs(prev_slope) <= max_slope
            back2 = prev_cell - prev_slope[None, :]
            ok = prev_ok & slope_ok[None, :] & (back2 >= 0) & (back2 < cells)
            back2 = np.clip(back2, 0, cells - 1)
            k = turn(px[r - 2][back2], py[r - 2][back2], px[r - 1][prev_cell],
                     py[r - 1][prev_cell], px[r][cell_idx], py[r][cell_idx])
            ok &= np.abs(k) <= KAPPA_MAX
            cand = total[prev_cell, np.clip(prev_slope + max_slope, 0, nslopes - 1)[None, :]]
            cand = np.where(ok, cand + BEND_WEIGHT * k ** 2 * STEP, np.inf)
            better = cand < best
            best[better] = cand[better]
            choice[better] = change
        best += here
        if not np.isfinite(best).any():
            raise RuntimeError(f'no drivable line past s = {row_s[r]:.1f} m '
                               f'({bx[r]:.2f}, {by[r]:.2f})')
        total = best
        came_from.append(choice)

    line = np.empty(steps)
    c, j = np.unravel_index(int(np.argmin(total)), total.shape)
    for r in range(steps - 1, 0, -1):
        line[r] = offsets[c]
        change = came_from[r - 1][c, j]
        c, j = c - slopes[j], j - change
    line[0] = offsets[c]
    # The planned stretch runs a lap and OVERLAP past s = 0; keep a lap that starts after
    # the free start has settled, where it meets its own end most smoothly (same offset,
    # same slope), mapped back onto s = 0 .. length.
    n = int(round(length / STEP))
    starts = np.arange(int(np.ceil(OVERLAP / 4.0 / STEP)), steps - n - 1)
    seam = np.abs(line[starts + n] - line[starts]) + \
        np.abs((line[starts + n + 1] - line[starts + n]) - (line[starts + 1] - line[starts]))
    first = int(starts[np.argmin(seam)])
    lap = np.arange(first, first + n)
    lap_s = np.mod(row_s[lap], length)
    order = np.argsort(lap_s)
    return lap_s[order], line[lap][order]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('map', help='map name (e.g. levine_obs) or path to its .yaml')
    parser.add_argument('--maps', default=os.path.expanduser('~/f1tenth_gym_ros/maps'))
    parser.add_argument('--centerline', default=None)
    parser.add_argument('--out', default=None)
    args = parser.parse_args()
    yaml_path = args.map if args.map.endswith('.yaml') else \
        os.path.join(args.maps, args.map + '.yaml')
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    centerline = args.centerline or os.path.join(here, 'config', 'levine_centerline.csv')
    name = os.path.splitext(os.path.basename(yaml_path))[0]
    out = args.out or os.path.join(here, 'config', name + '_line.csv')

    track = load_centerline(centerline)
    s, d = plan(track, load_map(yaml_path))
    cx, cy, tx, ty, _, _, cs, length = track

    def interp(v):
        return np.interp(s, np.append(cs, length), np.append(v, v[0]))
    tnx, tny = interp(tx), interp(ty)
    norm = np.hypot(tnx, tny)
    x = interp(cx) - d * tny / norm
    y = interp(cy) + d * tnx / norm
    np.savetxt(out, np.column_stack([s, d, x, y]), delimiter=',', fmt='%.4f',
               header=f'racing line for {name} (counter-clockwise; d is m left of the '
                      f'centerline at s), made by make_racing_line\ns_m, d_m, x_m, y_m')
    print(f'wrote {out}: {len(s)} points, offset {d.min():.2f} .. {d.max():.2f} m')

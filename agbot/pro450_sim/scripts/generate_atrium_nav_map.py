#!/usr/bin/env python3
"""Generate the deterministic occupancy map used by the atrium patrol demo.

The SLAM demo intentionally remains available for mapping experiments.  Its
saved map is not a good ground-truth map for the scripted patrol, however: a
robot or arm return can be baked into the static layer at the start pose.  The
patrol world geometry is known, so this generator creates a clean map in the
same map/world coordinate system and leaves runtime obstacle avoidance to the
local costmap.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path


RESOLUTION = 0.05
ORIGIN_X = -5.60
ORIGIN_Y = -5.60
WIDTH = 224
HEIGHT = 224
OUTER = 5.0
ATRIUM = 3.0
WALL_T = 0.12
PLANT_RADIUS = 0.14


def _occupied(x: float, y: float) -> bool:
    """Return the fixed collision geometry visible in a horizontal scan."""
    half_wall = WALL_T * 0.5

    # Four continuous outer walls.  The visual doors are closed in this demo.
    outer_wall = (
        (abs(abs(x) - OUTER) <= half_wall and abs(y) <= OUTER + half_wall)
        or (abs(abs(y) - OUTER) <= half_wall and abs(x) <= OUTER + half_wall)
    )

    # Four continuous walls around the central atrium.
    inner_wall = (
        (abs(abs(x) - ATRIUM) <= half_wall and abs(y) <= ATRIUM + half_wall)
        or (abs(abs(y) - ATRIUM) <= half_wall and abs(x) <= ATRIUM + half_wall)
    )
    if outer_wall or inner_wall:
        return True

    inset = OUTER - 0.35
    along = 2.5
    plant_centres = (
        (along, inset), (-along, inset),
        (along, -inset), (-along, -inset),
        (inset, along), (inset, -along),
        (-inset, along), (-inset, -along),
    )
    return any(math.hypot(x - px, y - py) <= PLANT_RADIUS
               for px, py in plant_centres)


def _cell_value(x: float, y: float) -> int:
    if _occupied(x, y):
        return 0       # occupied
    inside_outer = abs(x) < OUTER and abs(y) < OUTER
    inside_atrium = abs(x) < ATRIUM and abs(y) < ATRIUM
    if inside_outer and not inside_atrium:
        return 254     # known free corridor
    return 205         # unknown outside the navigable ring


def generate(output_yaml: Path) -> tuple[Path, Path]:
    output_yaml = output_yaml.resolve()
    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    output_pgm = output_yaml.with_suffix('.pgm')

    pixels = bytearray()
    # Netpbm rows start at the top; OccupancyGrid origin starts bottom-left.
    for row in range(HEIGHT):
        y = ORIGIN_Y + (HEIGHT - row - 0.5) * RESOLUTION
        for col in range(WIDTH):
            x = ORIGIN_X + (col + 0.5) * RESOLUTION
            pixels.append(_cell_value(x, y))

    with output_pgm.open('wb') as stream:
        stream.write(f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode('ascii'))
        stream.write(pixels)

    output_yaml.write_text(
        'image: ' + output_pgm.name + '\n'
        'mode: trinary\n'
        f'resolution: {RESOLUTION}\n'
        f'origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8')
    return output_yaml, output_pgm


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output-yaml', required=True,
        help='Destination YAML; a same-stem PGM is written beside it')
    args = parser.parse_args()
    yaml_path, pgm_path = generate(Path(args.output_yaml))
    print(f'generated patrol map: {yaml_path} ({pgm_path})')


if __name__ == '__main__':
    main()

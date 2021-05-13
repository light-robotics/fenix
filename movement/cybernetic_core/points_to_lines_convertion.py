#!/usr/bin/env python3.9

import math
from dataclasses import dataclass
from typing import List

from lines_3d import Line3D

@dataclass
class Point:
    x: float
    y: float
    z: float


def convert_points_to_3d_lines(D_points_history: List[List[Point]]) -> List[Line3D]:
    lines = []
    for i in range(len(D_points_history) - 1):
        for j in range(4):
            lines.append(Line3D(D_points_history[i][j], D_points_history[i+1][j]))

    return lines

if __name__ == '__main__':
    D_points_history = [
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=6.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=6.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=6.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=-0.0)],
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=6.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=-0.0)],
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=-0.0)],
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=-0.0)],
        [Point(x=15.0, y=15.0, z=-0.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=6.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=-0.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=6.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=-0.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=-0.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=6.0)],
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=6.0)],
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=0.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=15.0, z=11.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=25.0, z=11.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-15.0, z=0.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-15.0, z=6.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=-0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=6.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=-0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=0.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=-0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=-15.0, z=-0.0), Point(x=-15.0, y=15.0, z=-0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=-15.0, z=6.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=-5.0, z=6.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=-5.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=15.0, z=0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=0.0), Point(x=-15.0, y=-5.0, z=0.0), Point(x=-15.0, y=15.0, z=6.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=0.0), Point(x=-15.0, y=-5.0, z=0.0), Point(x=-15.0, y=25.0, z=6.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=0.0), Point(x=-15.0, y=-5.0, z=0.0), Point(x=-15.0, y=25.0, z=0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=0.0), Point(x=-15.0, y=-5.0, z=0.0), Point(x=-15.0, y=25.0, z=0.0)],
        [Point(x=15.0, y=25.0, z=5.0), Point(x=15.0, y=-5.0, z=0.0), Point(x=-15.0, y=-5.0, z=0.0), Point(x=-15.0, y=25.0, z=0.0)],
        [Point(x=15.0, y=35.0, z=11.0), Point(x=15.0, y=-5.0, z=0.0), Point(x=-15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=25.0, z=0.0)],
        [Point(x=15.0, y=35.0, z=0.0), Point(x=15.0, y=-5.0, z=0.0), Point(x=-15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=25.0, z=0.0)],
        [Point(x=15.0, y=35.0, z=-0.0), Point(x=15.0, y=-5.0, z=0.0), Point(x=-15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=25.0, z=0.0)],
        [Point(x=15.0, y=35.0, z=-0.0), Point(x=15.0, y=-5.0, z=6.0), Point(x=-15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=25.0, z=-0.0)],
        [Point(x=15.0, y=35.0, z=-0.0), Point(x=15.0, y=5.0, z=6.0), Point(x=-15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=25.0, z=-0.0)],
        [Point(x=15.0, y=35.0, z=-0.0), Point(x=15.0, y=5.0, z=0.0), Point(x=-15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=25.0, z=-0.0)],
        [Point(x=15.0, y=35.0, z=-0.0), Point(x=15.0, y=5.0, z=-0.0), Point(x=-15.0, y=-5.0, z=-0.0), Point(x=-15.0, y=25.0, z=-0.0)],
        [Point(x=15.0, y=35.0, z=0.0), Point(x=15.0, y=5.0, z=-0.0), Point(x=-15.0, y=-5.0, z=6.0), Point(x=-15.0, y=25.0, z=0.0)],
        [Point(x=15.0, y=35.0, z=0.0), Point(x=15.0, y=5.0, z=-0.0), Point(x=-15.0, y=5.0, z=6.0), Point(x=-15.0, y=25.0, z=0.0)],
        [Point(x=15.0, y=35.0, z=0.0), Point(x=15.0, y=5.0, z=-0.0), Point(x=-15.0, y=5.0, z=0.0), Point(x=-15.0, y=25.0, z=0.0)],
        [Point(x=15.0, y=35.0, z=0.0), Point(x=15.0, y=5.0, z=-0.0), Point(x=-15.0, y=5.0, z=-0.0), Point(x=-15.0, y=25.0, z=0.0)],
        [Point(x=15.0, y=35.0, z=0.0), Point(x=15.0, y=5.0, z=0.0), Point(x=-15.0, y=5.0, z=0.0), Point(x=-15.0, y=25.0, z=6.0)],
        [Point(x=15.0, y=35.0, z=0.0), Point(x=15.0, y=5.0, z=0.0), Point(x=-15.0, y=5.0, z=0.0), Point(x=-15.0, y=35.0, z=6.0)],
        [Point(x=15.0, y=35.0, z=0.0), Point(x=15.0, y=5.0, z=0.0), Point(x=-15.0, y=5.0, z=0.0), Point(x=-15.0, y=35.0, z=0.0)],
        [Point(x=15.0, y=35.0, z=0.0), Point(x=15.0, y=5.0, z=0.0), Point(x=-15.0, y=5.0, z=0.0), Point(x=-15.0, y=35.0, z=0.0)],
        [Point(x=15.0, y=35.0, z=0.0), Point(x=15.0, y=5.0, z=0.0), Point(x=-15.0, y=5.0, z=0.0), Point(x=-15.0, y=35.0, z=0.0)],
        [Point(x=15.0, y=35.0, z=6.0), Point(x=15.0, y=5.0, z=0.0), Point(x=-15.0, y=5.0, z=-0.0), Point(x=-15.0, y=35.0, z=0.0)],
        [Point(x=15.0, y=45.0, z=6.0), Point(x=15.0, y=5.0, z=0.0), Point(x=-15.0, y=5.0, z=-0.0), Point(x=-15.0, y=35.0, z=0.0)],
        [Point(x=15.0, y=45.0, z=0.0), Point(x=15.0, y=5.0, z=0.0), Point(x=-15.0, y=5.0, z=-0.0), Point(x=-15.0, y=35.0, z=0.0)],
        [Point(x=15.0, y=45.0, z=-0.0), Point(x=15.0, y=5.0, z=0.0), Point(x=-15.0, y=5.0, z=-0.0), Point(x=-15.0, y=35.0, z=0.0)],
        [Point(x=15.0, y=45.0, z=-0.0), Point(x=15.0, y=5.0, z=6.0), Point(x=-15.0, y=5.0, z=-0.0), Point(x=-15.0, y=35.0, z=-0.0)],
        [Point(x=15.0, y=45.0, z=-0.0), Point(x=15.0, y=15.0, z=6.0), Point(x=-15.0, y=5.0, z=-0.0), Point(x=-15.0, y=35.0, z=-0.0)],
        [Point(x=15.0, y=45.0, z=-0.0), Point(x=15.0, y=15.0, z=0.0), Point(x=-15.0, y=5.0, z=-0.0), Point(x=-15.0, y=35.0, z=-0.0)],
        [Point(x=15.0, y=45.0, z=-0.0), Point(x=15.0, y=15.0, z=-0.0), Point(x=-15.0, y=5.0, z=-0.0), Point(x=-15.0, y=35.0, z=-0.0)],
        [Point(x=15.0, y=45.0, z=0.0), Point(x=15.0, y=15.0, z=-0.0), Point(x=-15.0, y=5.0, z=6.0), Point(x=-15.0, y=35.0, z=0.0)],
        [Point(x=15.0, y=45.0, z=0.0), Point(x=15.0, y=15.0, z=-0.0), Point(x=-15.0, y=15.0, z=6.0), Point(x=-15.0, y=35.0, z=0.0)],
        [Point(x=15.0, y=45.0, z=0.0), Point(x=15.0, y=15.0, z=-0.0), Point(x=-15.0, y=15.0, z=0.0), Point(x=-15.0, y=35.0, z=0.0)],
        [Point(x=15.0, y=45.0, z=0.0), Point(x=15.0, y=15.0, z=-0.0), Point(x=-15.0, y=15.0, z=-0.0), Point(x=-15.0, y=35.0, z=0.0)],
        [Point(x=15.0, y=45.0, z=0.0), Point(x=15.0, y=15.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0), Point(x=-15.0, y=35.0, z=6.0)],
        [Point(x=15.0, y=45.0, z=0.0), Point(x=15.0, y=15.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0), Point(x=-15.0, y=45.0, z=6.0)],
        [Point(x=15.0, y=45.0, z=0.0), Point(x=15.0, y=15.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0), Point(x=-15.0, y=45.0, z=0.0)],
        [Point(x=15.0, y=45.0, z=0.0), Point(x=15.0, y=15.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0), Point(x=-15.0, y=45.0, z=0.0)],
        [Point(x=15.0, y=45.0, z=0.0), Point(x=15.0, y=15.0, z=0.0), Point(x=-15.0, y=15.0, z=0.0), Point(x=-15.0, y=45.0, z=0.0)]
    ]

    for item in convert_points_to_3d_lines(D_points_history):
        print(item)

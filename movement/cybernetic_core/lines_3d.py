#!/usr/bin/env python3.9

import math
from dataclasses import dataclass


@dataclass
class Point:
    x: float
    y: float
    z: float

"""
Step 1: Find the DR’s (Direction Ratios) by taking the difference of the corresponding 
position coordinates of the two given points. 
l = (x2 – x1), m = (y2 – y1), n = (z2 – z1); Here l, m, n are the DR’s.
Step 2: Choose either of the two given points say, we choose (x1, y1, z1).
Step 3: Write the required equation of the straight line passing through 
the points (x1, y1, z1) and (x2, y2, z2). L : (x – x1)/l = (y – y1)/m = (z – z1)/n
"""

class Line3D:
    def __init__(self, pnt1: Point, pnt2: Point):
        self.l = pnt1.x - pnt2.x
        self.m = pnt1.y - pnt2.y
        self.n = pnt1.z - pnt2.z
        self.anchor_point = pnt1
        self.target_point = pnt2

        self.min_x = min(self.anchor_point.x, self.target_point.x)
        self.max_x = max(self.anchor_point.x, self.target_point.x)
        self.min_y = min(self.anchor_point.y, self.target_point.y)
        self.max_y = max(self.anchor_point.y, self.target_point.y)
        self.min_z = min(self.anchor_point.z, self.target_point.z)
        self.max_z = max(self.anchor_point.z, self.target_point.z)
        # (x - pnt1.x)/l = (y - pnt1.y)/m = (z - pnt1.z)/n
    
    def __repr__(self):
        return f'Line3D({self.anchor_point}, {self.target_point})'

    def point_on_line(self, pnt: Point) -> bool:
        if (pnt.x - self.anchor_point.x) * self.m == (pnt.y - self.anchor_point.y) * self.l and \
            (pnt.x - self.anchor_point.x) * self.n == (pnt.z - self.anchor_point.z) * self.l:
                return True
        return False

    def intersect_with_plane_x(self, x: int) -> Point:
        if self.l == 0:
            return None
        y = round((x - self.anchor_point.x) * self.m / self.l + self.anchor_point.y, 1)
        z = round((x - self.anchor_point.x) * self.n / self.l + self.anchor_point.z, 1) 
        if y > self.max_y or y < self.min_y or z > self.max_z or z < self.min_z:            
            return None
        return Point(x, y, z)

    def intersect_with_plane_y(self, y: int) -> Point:
        if self.m == 0:
            return None
        x = round((y - self.anchor_point.y) * self.l / self.m + self.anchor_point.x, 1)
        z = round((y - self.anchor_point.y) * self.n / self.m + self.anchor_point.z, 1)
        #if x > self.max_x or x < self.min_x or z > self.max_z or z < self.min_z:
        #    return None
        #return Point(x, y, z)
        
        if self.min_x <= x <= self.max_x and \
           self.min_y <= y <= self.max_y and \
           self.min_z <= z <= self.max_z:
                return Point(x, y, z)
        
        return None

    def intersect_with_plane_z(self, z: int) -> Point:
        if self.n == 0:
            return None
        x = round((z - self.anchor_point.z) * self.l / self.n + self.anchor_point.x, 1)
        y = round((z - self.anchor_point.z) * self.m / self.n + self.anchor_point.y, 1)
        if x > self.max_x or x < self.min_x or y > self.max_y or y < self.min_y:
            return None
        return Point(x, y, z)

"""
A = Point(0, 0, 0)
B = Point(5, 10, 15)
C = Point(2, 2, 2)
D = Point(2, 2, 3)

ln = Line3D(A, B)
print(ln.point_on_line(C))
print(ln.point_on_line(D))
print(ln.intersect_with_plane_x(7))
print(ln.intersect_with_plane_x(5))
print(ln.intersect_with_plane_x(3))

print(ln.intersect_with_plane_y(7))
print(ln.intersect_with_plane_y(5))
print(ln.intersect_with_plane_y(3))

print(ln.intersect_with_plane_z(7))
print(ln.intersect_with_plane_z(5))
print(ln.intersect_with_plane_z(3))
"""

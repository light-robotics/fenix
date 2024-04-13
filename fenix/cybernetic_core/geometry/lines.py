#!/usr/bin/env python3.9

from __future__ import annotations
import math
from typing import List, Tuple


class Point:
    def __init__(self, x: float, y: float, z: float):
        self.x, self.y, self.z = x, y, z

    def move(self, 
            delta_x: float = 0, 
            delta_y: float = 0,
            delta_z: float = 0
            ) -> None:
        self.x += delta_x
        self.y += delta_y
        self.z += delta_z

    def __eq__(self, another: Point) -> bool:
        if abs(self.x - another.x) < 0.01 and \
            abs(self.y - another.y) < 0.01 and \
             abs(self.z - another.z) < 0.01:
             return True
        return False
    
    def __repr__(self):
        return f'Point(x={self.x}, y={self.y}, z={self.z})'

# 2D lines helper functions
class Line2D:
    def __init__(self, point1: Point, point2: Point):
        self.point1 = point1
        self.point2 = point2

        self.k, self.b, self.angle = self.get_linear_func()
        
    def get_linear_func(self) -> Tuple[float, float, float]:
        delta_x = self.point2.x - self.point1.x
        if delta_x == 0:
            delta_x = 0.01
        k = (self.point2.y - self.point1.y) / delta_x
        b = (self.point2.x * self.point1.y - self.point1.x * self.point2.y) / delta_x
        angle = math.atan2(self.point2.y - self.point1.y, self.point2.x - self.point1.x)
        return k, b, angle

    def calculate_intersection(self, another_line: Line2D) -> Tuple[float, float]:
        x = (self.b - another_line.b) / (another_line.k - self.k)
        y = self.k * x + self.b
        return x, y

    # function, that moves on a line from a given point to a target point for a margin distance
    def move_on_a_line(self, margin: float) -> Tuple[float, float]:
        new_point_x = round(self.point1.x +
                            math.cos(self.angle) * margin,
                            2)
        new_point_y = round(self.point1.y +
                            math.sin(self.angle) * margin,
                            2)

        return new_point_x, new_point_y


# 3D lines functions
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

    def intersect_with_plane_x(self, n):
        """
        Find the intersection point of the line with the plane X = n.
        
        Parameters:
        - n: Value of X for the plane
        
        Returns:
        - Point: Intersection point if exists, None otherwise
        """
        
        if self.anchor_point.x == n and self.target_point.x == n:
            # Line lies on the plane
            return self.anchor_point if self.anchor_point.y == self.target_point.y else None
        
        if self.anchor_point.x == n:
            # Anchor point lies on the plane
            return self.anchor_point
        
        if self.target_point.x == n:
            # Target point lies on the plane
            return self.target_point

        if self.anchor_point.x == self.target_point.x:
            # Line is parallel to YZ plane
            return None
        
        # Calculate intersection point
        t = (n - self.anchor_point.x) / (self.target_point.x - self.anchor_point.x)
        if 0 <= t <= 1:
            y = self.anchor_point.y + t * (self.target_point.y - self.anchor_point.y)
            z = self.anchor_point.z + t * (self.target_point.z - self.anchor_point.z)
            return Point(n, y, z)
        
        return None

    def intersect_with_plane_y(self, m):
        """
        Find the intersection point of the line with the plane Y = m.
        
        Parameters:
        - m: Value of Y for the plane
        
        Returns:
        - Point: Intersection point if exists, None otherwise
        """
        if self.anchor_point.y == m and self.target_point.y == m:
            # Line lies on the plane
            return self.anchor_point if self.anchor_point.x == self.target_point.x else None
        
        if self.anchor_point.y == m:
            # Anchor point lies on the plane
            return self.anchor_point
        
        if self.target_point.y == m:
            # Target point lies on the plane
            return self.target_point

        if self.anchor_point.y == self.target_point.y:
            # Line is parallel to XZ plane
            return None

        # Calculate intersection point
        t = (m - self.anchor_point.y) / (self.target_point.y - self.anchor_point.y)
        if 0 <= t <= 1:
            x = self.anchor_point.x + t * (self.target_point.x - self.anchor_point.x)
            z = self.anchor_point.z + t * (self.target_point.z - self.anchor_point.z)
            return Point(x, m, z)
        
        return None

    def old_intersect_with_plane_x(self, x: int) -> Point:
        if self.l == 0:
            return None
        y = round((x - self.anchor_point.x) * self.m / self.l + self.anchor_point.y, 1)
        z = round((x - self.anchor_point.x) * self.n / self.l + self.anchor_point.z, 1) 
        if y > self.max_y or y < self.min_y or z > self.max_z or z < self.min_z:            
            return None
        
        return Point(x, y, z)

    def old_intersect_with_plane_y(self, y: int) -> Point:
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
        if z < self.min_z or z > self.max_z:
            return None
        x = round((z - self.anchor_point.z) * self.l / self.n + self.anchor_point.x, 1)
        y = round((z - self.anchor_point.z) * self.m / self.n + self.anchor_point.y, 1)
        if x > self.max_x or x < self.min_x or y > self.max_y or y < self.min_y:
            return None
        return Point(x, y, z)

def convert_points_to_3d_lines(D_points_history: List[List[Point]]) -> List[Line3D]:
    lines = []
    for i in range(len(D_points_history) - 1):
        for j in range(4):
            if D_points_history[i][j] == D_points_history[i+1][j]:
                continue
            lines.append(Line3D(D_points_history[i][j], D_points_history[i+1][j]))

    return lines

class LinearFunc:
    def __init__(self, point1, point2):
        delta_x = (point2.x - point1.x)
        if delta_x == 0:
            delta_x = 0.01
        self.k = (point2.y - point1.y) / delta_x
        self.b = (point2.x * point1.y - point1.x * point2.y) / delta_x
        self.angle = math.atan2(point2.y - point1.y, point2.x - point1.x)


def calculate_intersection(func1, func2):
    x = (func1.b - func2.b) / (func2.k - func1.k)
    y = func1.k * x + func1.b
    return x, y


# function, that moves on a line from a given point to a target point for a margin distance
def move_on_a_line(intersection_point, target_point, margin):
    function = LinearFunc(intersection_point, target_point)
    new_point_x = round(intersection_point.x +
                        math.cos(function.angle) * margin,
                        2)
    new_point_y = round(intersection_point.y +
                        math.sin(function.angle) * margin,
                        2)

    return [new_point_x, new_point_y]
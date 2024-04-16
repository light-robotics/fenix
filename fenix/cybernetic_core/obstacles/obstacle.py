#!/usr/bin/env python3.9
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from cybernetic_core.geometry.lines import Point, Line3D
from configs import config as cfg


class ObstacleException(Exception):
    pass

class Obstacle:
    def __init__(self, min_x, max_x, min_y, max_y, min_z, max_z):
        # crutch, lidar scans and forward for it is Y+
        # for Fenix forward is X+
        # so we exchange values of obstacle
        # #fixlater
        self.min_x = min_y
        self.max_x = max_y
        self.min_y = min_x
        self.max_y = max_x
        self.min_z = min_z
        self.max_z = max_z

    def __repr__(self):
        return f'Obstacle: [{self.min_x}-{self.max_x}, {self.min_y}-{self.max_y}, {self.min_z}-{self.max_z}]'    
    
    def touching_the_obstacle(self, x: int, y: int) -> int:
        # returns obstacle z if it touches the top plane, 0 else
        # throws Exception if it is too close to the edge
        #danger_offset = 2
        danger_offset = cfg.obstacle["danger_offset"]
        
        if self.min_x <= x <= self.max_x and \
            self.min_y <= y <= self.max_y:

                # check for danger
                if x - self.min_x < danger_offset:
                    raise ObstacleException(f'x = {x} too close to min_x ({self.min_x})')
                if self.max_x - x < danger_offset:
                    raise ObstacleException(f'x = {x} too close to max_x ({self.max_x})')
                if y - self.min_y < danger_offset:
                    raise ObstacleException(f'y = {y} too close to min_y ({self.min_y})')
                if self.max_y - y < danger_offset:
                    raise ObstacleException(f'y = {y} too close to max_y ({self.max_y})')
                
                return self.max_z

        return 0

    def intersecting_the_obstacle(self, line: Line3D) -> bool:
        outer_danger_offset = cfg.obstacle["outer_danger_offset"]
        intersection_point_with_max_x_plane = line.intersect_with_plane_x(self.max_x + outer_danger_offset)
        if intersection_point_with_max_x_plane is not None:
            #print('1')
            if self.min_y <= intersection_point_with_max_x_plane.y <= self.max_y and \
               self.min_z <= intersection_point_with_max_x_plane.z <= self.max_z: 
                #print(f'1. Intersection in {intersection_point_with_max_x_plane}')
                return True

        intersection_point_with_min_x_plane = line.intersect_with_plane_x(self.min_x - outer_danger_offset)
        if intersection_point_with_min_x_plane is not None:
            #print(f'2. {intersection_point_with_min_x_plane}')
            if self.min_y <= intersection_point_with_min_x_plane.y <= self.max_y and \
               self.min_z <= intersection_point_with_min_x_plane.z <= self.max_z: 
                #print(f'2. Intersection in {intersection_point_with_min_x_plane}')
                return True
        
        intersection_point_with_max_y_plane = line.intersect_with_plane_y(self.max_y + outer_danger_offset)
        if intersection_point_with_max_y_plane is not None:
            #print('3')
            if self.min_x <= intersection_point_with_max_y_plane.x <= self.max_x and \
               self.min_z <= intersection_point_with_max_y_plane.z <= self.max_z:                
                #print(f'3. Intersection in {intersection_point_with_max_y_plane}')
                return True

        intersection_point_with_min_y_plane = line.intersect_with_plane_y(self.min_y - outer_danger_offset)
        if intersection_point_with_min_y_plane is not None:
            #print(f'4. {intersection_point_with_min_y_plane}')
            if self.min_x <= intersection_point_with_min_y_plane.x <= self.max_x and \
               self.min_z <= intersection_point_with_min_y_plane.z <= self.max_z: 
                #print(f'4. Intersection in {intersection_point_with_min_y_plane}')
                return True
        
        
        intersection_point_with_max_z_plane = line.intersect_with_plane_z(self.max_z - 0.1)
        if intersection_point_with_max_z_plane is not None:
            #print('5')
            if self.min_x - outer_danger_offset <= intersection_point_with_max_z_plane.x <= self.max_x + outer_danger_offset and \
               self.min_y - outer_danger_offset <= intersection_point_with_max_z_plane.y <= self.max_y + outer_danger_offset:
                #print(f'5. Intersection in {intersection_point_with_max_z_plane}')
                return True       
        
        return False

def obstacles_from_csv(file: str = '/fenix/fenix/wrk/obstacles.csv') -> list[Obstacle]:
    obstacles = []
    with open (file, 'r') as f:
        contents = f.readlines()
    
    for item in contents:        
        coords = [round(float(x)/10, 1) for x in item.split(',')]
        obstacles.append(Obstacle(*coords))

    return obstacles
    

if __name__ == '__main__':
    obss = obstacles_from_csv()
    obs = obss[0]
    print(obs.touching_the_obstacle(5, 50))
    print(obs.touching_the_obstacle(15, 50))

    line = Line3D(Point(10, 0, 0), Point(15, 15, 10))
    obs.intersecting_the_obstacle(line)
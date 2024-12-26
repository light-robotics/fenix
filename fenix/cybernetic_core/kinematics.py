import math
import copy
from dataclasses import dataclass
from typing import List, Dict
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from configs import config as cfg
from cybernetic_core.geometry.angles import FenixPosition, calculate_leg_angles, turn_on_angle, convert_legs_angles_C, calculate_D_point, convert_legs_angles_to_kinematic
from cybernetic_core.geometry.lines import Point, LinearFunc, calculate_intersection, move_on_a_line
import configs.code_config as code_config
import logging.config
from cybernetic_core.cybernetic_utils.moves import Move, MoveSnapshot


def get_turn_coords(between_legs, angle, x0=0, y0=0, x_delta=0, y_delta=0):

    x3, y3 = x0 + x_delta, y0 + y_delta

    a = between_legs
    alpha = math.radians(angle)
    beta = math.radians(90 - angle)

    x4 = x3 + a*math.cos(alpha)
    y4 = y3 + a*math.sin(alpha)

    x2 = x3 - a*math.cos(beta)
    y2 = y3 + a*math.sin(beta)

    x1 = x4 - a*math.sin(alpha)
    y1 = y4 + a*math.cos(alpha)

    print(f'{between_legs}, {x0}, {y0}, ({x1, y1}), ({x2, y2}), ({x3, y3}), ({x4, y4})')

    return x1, y1, x2, y2, x3, y3, x4, y4

import numpy as np

def rotate_point(x, y, angle):
    # Convert angle from degrees to radians
    angle_rad = np.radians(angle)
    
    # Rotation matrix
    rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                 [np.sin(angle_rad), np.cos(angle_rad)]])
    
    # Coordinates after rotation
    new_x, new_y = np.dot(rotation_matrix, np.array([x, y]))
    
    return new_x, new_y

def move_forward(x, y, angle, move_distance):
    # Calculate the new coordinates after moving forward
    new_x = x + move_distance * np.cos(np.radians(angle))
    new_y = y + move_distance * np.sin(np.radians(angle))
    
    return new_x, new_y

def move_robot_legs(leg1_x, leg1_y, leg2_x, leg2_y, leg3_x, leg3_y, leg4_x, leg4_y, angle, move_distance):
    # Rotate the legs by the given angle
    print(leg1_x, leg1_y, leg2_x, leg2_y, leg3_x, leg3_y, leg4_x, leg4_y, angle, move_distance)
    leg1_x, leg1_y = rotate_point(leg1_x, leg1_y, angle)
    leg2_x, leg2_y = rotate_point(leg2_x, leg2_y, angle)
    leg3_x, leg3_y = rotate_point(leg3_x, leg3_y, angle)
    leg4_x, leg4_y = rotate_point(leg4_x, leg4_y, angle)

    # Move the legs forward
    leg1_x, leg1_y = move_forward(leg1_x, leg1_y, angle, move_distance)
    leg2_x, leg2_y = move_forward(leg2_x, leg2_y, angle, move_distance)
    leg3_x, leg3_y = move_forward(leg3_x, leg3_y, angle, move_distance)
    leg4_x, leg4_y = move_forward(leg4_x, leg4_y, angle, move_distance)
    print(leg1_x, leg1_y, leg2_x, leg2_y, leg3_x, leg3_y, leg4_x, leg4_y)
    return leg1_x, leg1_y, leg2_x, leg2_y, leg3_x, leg3_y, leg4_x, leg4_y

class Leg:
    def __init__(self, O: Point, D: Point):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('angles_logger') #logging.getLogger('main_logger')
        self.O = O
        self.D = D
        #print(f'O: {O}. D: {D}')
        self.update_angles()

    def update_angles(self):
        # tetta is not fully correct, because it uses atan2
        # tetta is corrected via convert_tetta function
        #print(self.O, self.D)
        calculated_angles = calculate_leg_angles(self.O, self.D, self.logger)
        self.tetta, self.alpha, self.beta, self.gamma = calculated_angles
        #print(f'Best angles: {round(math.degrees(self.alpha), 2), round(math.degrees(self.beta), 2), round(math.degrees(self.gamma), 2)}')

    def move_mount_point(self, delta_x, delta_y, delta_z):
        self.O.move(delta_x, delta_y, delta_z)
        self.update_angles()
    
    def move_end_point(self, delta_x, delta_y, delta_z):
        self.D.move(delta_x, delta_y, delta_z)
        #print(f'D Move: {delta_x, delta_y, delta_z}')
        self.update_angles()

class FenixKinematics:
    """
    Either take initial position from config
    or provide horizontal x, y and vertical v
    or provide exact angles to create a kinematic model
    """
    def __init__(self, fenix_position: FenixPosition = None, init_snapshot=True):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('main_logger')

        if fenix_position is None:
            self.legs_offset_v = cfg.start['vertical']
            self.legs_offset_h_x = cfg.start['horizontal_x']
            self.legs_offset_h_y = cfg.start['horizontal_y']
            self.legs = self.initiate_legs()
        else:
            self.legs = self.build_legs_from_angles(fenix_position)
            # only correct when all leg's D's are equidistant from center
            self.legs_offset_v = self.legs[1].O.z - self.legs[1].D.z
            self.legs_offset_h_x = round((self.legs[1].D.x - self.legs[4].D.x)/2.0, 2)
            self.legs_offset_h_y = round((self.legs[1].D.y - self.legs[2].D.y)/2.0, 2)
            
        #print(f"""
        #    Kinematics created with 
        #    v = {self.legs_offset_v}, 
        #    h_x = {self.legs_offset_h_x},
        #    h_y = {self.legs_offset_h_y}
        #    """)

        self.current_legs_offset_v = self.legs_offset_v
        self.current_legs_offset_h_x = self.legs_offset_h_x
        self.current_legs_offset_h_y = self.legs_offset_h_y

        self.legs_deltas = {1 : [0, 0, 0], 2 : [0, 0, 0], 3 : [0, 0, 0], 4 : [0, 0, 0]}

        self.current_angle = 0 # up/down
        self.side_look_angle = 0 # left/right
        self.current_vertical_angle = 0
        self.current_horizontal_angle = 0
        self.current_body_delta = [0, 0, 0]
        self.margin = cfg.fenix["margin"][1]
        self.leg_up = cfg.fenix["leg_up"][2]
        self.leg_up_single = cfg.fenix["leg_up"][1]
        
        self.angles_history = []
        self.D_points_history = []
        if init_snapshot:
            self.add_angles_snapshot('init')

    def get_sequence_length(self):
        sum_diff = 0
        for i in range(len(self.sequence) - 1):
            max_diff = 0
            for a, b in zip(self.sequence[i].angles_snapshot, self.sequence[i+1].angles_snapshot):
                max_diff = max(max_diff, abs(a - b))
            sum_diff += max_diff
        
        return round(sum_diff)
    
    def reset_history(self):
        self.angles_history = []

    def add_angles_snapshot(self, move_type: str = 'unknown'):
        fp = FenixPosition(
            leg1_tetta=self.legs[1].tetta, 
            leg1_alpha=self.legs[1].alpha, 
            leg1_beta=self.legs[1].beta, 
            leg1_gamma=self.legs[1].gamma,  
            leg2_tetta=self.legs[2].tetta, 
            leg2_alpha=self.legs[2].alpha, 
            leg2_beta=self.legs[2].beta, 
            leg2_gamma=self.legs[2].gamma,
            leg3_tetta=self.legs[3].tetta, 
            leg3_alpha=self.legs[3].alpha, 
            leg3_beta=self.legs[3].beta, 
            leg3_gamma=self.legs[3].gamma,
            leg4_tetta=self.legs[4].tetta, 
            leg4_alpha=self.legs[4].alpha, 
            leg4_beta=self.legs[4].beta, 
            leg4_gamma=self.legs[4].gamma
        )

        #new_move = MoveSnapshot(move_type, convert_legs_angles(angles_in))
        new_move = MoveSnapshot(move_type, fp)
        self.angles_history.append(new_move)
        
        self.D_points_history.append(
            [
                copy.deepcopy(self.legs[1].D),
                copy.deepcopy(self.legs[2].D),
                copy.deepcopy(self.legs[3].D),
                copy.deepcopy(self.legs[4].D)
            ])

    @property
    def height(self):
        return sum([(leg.O.z - leg.D.z) for leg in self.legs.values()])/4
        #return self.current_legs_offset_v

    @property
    def sequence(self):
        sequence = []
        for move in self.angles_history:
            sequence.append(MoveSnapshot(move.move_type, convert_legs_angles_C(move.angles_snapshot, self.logger)))
        return sequence
        #return self.angles_history
    
    def build_legs_from_angles(self, fp: FenixPosition):
        O1 = Point(cfg.leg["mount_point_offset"],
                   cfg.leg["mount_point_offset"],
                   0)
        
        D1 = calculate_D_point(O1, fp.legs[1])
        self.logger.info('[Init] Building leg 1')
        #print(f'Building leg 1: {math.degrees(alpha1), math.degrees(beta1), math.degrees(gamma1)}')
        Leg1 = Leg(O1, D1)

        O2 = Point(cfg.leg["mount_point_offset"],
                   -cfg.leg["mount_point_offset"],
                   0)

        D2 = calculate_D_point(O2, fp.legs[2])
        self.logger.info('[Init] Building leg 2')
        Leg2 = Leg(O2, D2)
        #print(f'Leg2.2:{[round(math.degrees(x), 2) for x in [Leg2.tetta, Leg2.alpha, Leg2.beta, Leg2.gamma]]}')

        O3 = Point(-cfg.leg["mount_point_offset"],
                   -cfg.leg["mount_point_offset"],
                   0)
        D3 = calculate_D_point(O3, fp.legs[3])
        self.logger.info('[Init] Building leg 3')
        Leg3 = Leg(O3, D3)

        O4 = Point(-cfg.leg["mount_point_offset"],
                   cfg.leg["mount_point_offset"],
                   0)
        D4 = calculate_D_point(O4, fp.legs[4])
        self.logger.info('[Init] Building leg 4')
        Leg4 = Leg(O4, D4)

        self.logger.info('[Init] Build successful')

        return {1: Leg1, 2: Leg2, 3: Leg3, 4: Leg4}

    @property
    def current_position(self):
        #return convert_legs_angles_back(self.sequence[-1].angles_snapshot)
        return self.angles_history[-1].angles_snapshot

    def initiate_legs(self):
        O1 = Point(cfg.leg["mount_point_offset"],
                   cfg.leg["mount_point_offset"],
                   self.legs_offset_v)
        D1 = Point(self.legs_offset_h_x - cfg.start["x_offset_body"],
                   self.legs_offset_h_y - cfg.start["y_offset_body"],
                   0)
        self.logger.info('[Init] Initiating leg 1')
        Leg1 = Leg(O1, D1)

        O2 = Point(cfg.leg["mount_point_offset"],
                   -cfg.leg["mount_point_offset"],
                   self.legs_offset_v)
        D2 = Point(self.legs_offset_h_x - cfg.start["x_offset_body"],
                   -self.legs_offset_h_y - cfg.start["y_offset_body"],
                   0)
        self.logger.info('[Init] Initiating leg 2')
        Leg2 = Leg(O2, D2)

        O3 = Point(-cfg.leg["mount_point_offset"],
                   -cfg.leg["mount_point_offset"],
                   self.legs_offset_v)
        D3 = Point(-self.legs_offset_h_x - cfg.start["x_offset_body"],
                   -self.legs_offset_h_y - cfg.start["y_offset_body"],
                   0)
        self.logger.info('[Init] Initiating leg 3')
        Leg3 = Leg(O3, D3)

        O4 = Point(-cfg.leg["mount_point_offset"],
                   cfg.leg["mount_point_offset"],
                   self.legs_offset_v)
        D4 = Point(-self.legs_offset_h_x - cfg.start["x_offset_body"],
                   self.legs_offset_h_y - cfg.start["y_offset_body"],
                   0)
        self.logger.info('[Init] Initiating leg 4')
        Leg4 = Leg(O4, D4)

        self.logger.info('[Init] Initialization successful')

        return {1: Leg1, 2: Leg2, 3: Leg3, 4: Leg4}
    
    ################## MOVEMENTS START HERE ##################
    def leg_movement(self, leg_num, leg_delta, snapshot=True):
        self.logger.info(f'Leg move {leg_num}: {leg_delta}')
        leg = self.legs[leg_num]

        leg.move_end_point(leg_delta[0], leg_delta[1], leg_delta[2])
        if snapshot:
            self.add_angles_snapshot('endpoint')

    def body_movement(self, delta_x, delta_y, delta_z, snapshot=True):
        self.logger.info(f'Body movement [{delta_x}, {delta_y}, {delta_z}]')
        self.current_body_delta = [x + y for x, y in zip(self.current_body_delta, [delta_x, delta_y, delta_z])]
        #print(f'self.current_body_delta : {self.current_body_delta}')
        if delta_x == delta_y == delta_z == 0:
            return

        for leg in self.legs.values():
            leg.move_mount_point(delta_x, delta_y, delta_z)

        if snapshot:
            self.add_angles_snapshot('body')

        self.current_legs_offset_v -= delta_z

    # ?
    def start(self):
        self.body_movement(0, 0, -cfg.start["vertical"] + cfg.start["initial_z_position_delta"])
        self.body_movement(0, 0, cfg.start["vertical"] - cfg.start["initial_z_position_delta"])
    
    # ?
    def reset(self):
        self.logger.info('Processing reset command')
        self.body_to_center()
        #delta_z = self.current_body_delta[2]
        delta_z = self.legs[1].O.z - self.legs[1].D.z - cfg.start["vertical"]
        self.body_movement(0, 0, -delta_z)

    # ?
    def end(self):
        self.reset()
        self.body_movement(0, 0, -cfg.start["vertical"] + 
                                  cfg.start["initial_z_position_delta"])
            
    def legs_D_offsets(self):
        x_offset = abs(round((self.legs[1].D.x - self.legs[4].D.x)/2))
        y_offset = abs(round((self.legs[1].D.y - self.legs[2].D.y)/2))
        return {"x": x_offset, "y": y_offset}
    
    def switch_mode(self, mode: str):
        self.logger.info(f'Switching mode to {mode}')
        self.reset()
        required_xy = cfg.modes[mode]
        current_xy = self.legs_D_offsets()

        print(f'Current_xy: {current_xy}. Required: {required_xy}')
        delta_x = required_xy["x"] - current_xy["x"]
        delta_y = required_xy["y"] - current_xy["y"]
        if abs(delta_x) + abs(delta_y) != 0:
            self.reposition_legs(delta_x, delta_y)
    
    def body_delta_xy(self, delta_y=cfg.start["y_offset_body"], delta_x=cfg.start["x_offset_body"]):
        # move body to center
        avg_o_x, avg_o_y, avg_d_x, avg_d_y = 0, 0, 0, 0
        for leg in self.legs.values():
            avg_o_x += leg.O.x
            avg_o_y += leg.O.y
            avg_d_x += leg.D.x
            avg_d_y += leg.D.y

        avg_o_x /= 4
        avg_o_y /= 4
        avg_d_x /= 4
        avg_d_y /= 4

        return [round(avg_o_x - avg_d_x - delta_x, 2),
                round(avg_o_y - avg_d_y - delta_y, 2)]
                           

    def body_to_center(self, delta_y=cfg.start["y_offset_body"], delta_x=cfg.start["x_offset_body"], snapshot=True):
        # move body to center
        
        body_delta_xy = self.body_delta_xy(delta_y, delta_x)
        self.logger.info(f'Moving body: {body_delta_xy}')
        self.body_movement(-body_delta_xy[0],
                           -body_delta_xy[1],
                           0, 
                           snapshot)

    # body compensation for moving up one leg
    def target_body_position(self, leg_in_the_air_number):
        """
        provide the number of leg_in_the_air
        return target position of body to let the leg go into the air
        """

        # find intersection point of basement lines
        func1 = LinearFunc(self.legs[1].D, self.legs[3].D)
        func2 = LinearFunc(self.legs[2].D, self.legs[4].D)
        intersection = Point(*calculate_intersection(func1, func2), 0)

        target_leg_number_by_air_leg_number = {1: 3, 2: 4, 3: 1, 4: 2}
        target_leg_number = target_leg_number_by_air_leg_number[leg_in_the_air_number]
        target_leg = self.legs[target_leg_number]
        body_target_point = move_on_a_line(intersection,
                                           target_leg.D,
                                           self.margin)

        return body_target_point

    def body_compensation_for_a_leg(self, leg_num):        
        target = self.target_body_position(leg_num)
        self.logger.info(f'Move. body_compensation_for_a_leg. Target : {target}')
        current_body_x = (self.legs[1].O.x +
                          self.legs[2].O.x +
                          self.legs[3].O.x +
                          self.legs[4].O.x) / 4

        current_body_y = (self.legs[1].O.y +
                          self.legs[2].O.y +
                          self.legs[3].O.y +
                          self.legs[4].O.y) / 4

        self.body_movement(target[0] - current_body_x,
                           target[1] - current_body_y,
                           0)

    def compensated_leg_movement(self, leg_num, leg_delta):
        # moving body to compensate future movement
        self.logger.info(f'Processing leg {leg_num} body_compensation_for_a_leg')
        self.body_compensation_for_a_leg(leg_num)

        self.logger.info(f'Processing leg {leg_num} move_end_point {leg_delta}')
        self.legs[leg_num].move_end_point(*leg_delta)
        self.add_angles_snapshot('endpoint')

    def leg_move_with_compensation(self, leg_num, delta_x, delta_y):
        self.compensated_leg_movement(leg_num, [delta_x, delta_y, self.leg_up_single])
        self.logger.info(f'Processing leg {leg_num} move_end_point {[0, 0, -self.leg_up_single]}')
        self.move_leg_endpoint(leg_num, [0, 0, -self.leg_up_single])
        self.add_angles_snapshot('endpoint')
        #self.compensated_leg_movement(leg_num, [0, 0, -self.leg_up])
    
    def leg_move_custom(self, leg_num, mode, leg_delta=[0, 0, 0], add_snapshot=True):
        if mode == 'touch':
            iterations = 2
            mode = f'touch_{leg_num}'
        else:
            iterations = 1
        for i in range(iterations):
            self.move_leg_endpoint(
                leg_num, 
                [
                    round(leg_delta[0]/iterations, 1), 
                    round(leg_delta[1]/iterations, 1),
                    round(leg_delta[2]/iterations, 1)
                ], 
                mode,
                add_snapshot=add_snapshot
            )
   
    def move_leg_endpoint(self, leg_num, leg_delta, snapshot_type='endpoint', add_snapshot=True):        
        self.legs[leg_num].move_end_point(*leg_delta)
        self.legs_deltas[leg_num] = [x + y for x, y in zip(self.legs_deltas[leg_num], leg_delta)]        
        if add_snapshot:
            self.add_angles_snapshot(snapshot_type)
        print(f'move_leg_endpoint. Leg {leg_num}. D: {self.legs[leg_num].D}')

    def move_leg_endpoint_abs(self, leg_num, leg_delta, snapshot_type='endpoint', add_snapshot=True):
        min_z = min([leg.D.z for leg in self.legs.values()])
        leg = self.legs[leg_num]
        target_x = leg_delta[0]
        if leg_delta[0] is None:
            target_x = leg.D.x
        
        target_y = leg_delta[1]
        if leg_delta[1] is None:
            target_y = leg.D.y

        target_z = leg_delta[2]
        if leg_delta[2] is None:
            target_z = leg.D.z - min_z

        new_delta = [round(target_x - leg.D.x, 1), round(target_y - leg.D.y, 1), round(target_z - leg.D.z + min_z, 1)]
        print(f'Legnum: {leg_num}.\nOriginal delta: {leg_delta}\nNew delta: {new_delta}')
        self.logger.info(f'move_leg_endpoint_abs. Legnum: {leg_num}.\nOriginal delta: {leg_delta}\nNew delta: {new_delta}')
        self.legs[leg_num].move_end_point(*new_delta)
        #self.legs_deltas[leg_num] = [x + y for x, y in zip(self.legs_deltas[leg_num], leg_delta)]        
        if add_snapshot:
            self.add_angles_snapshot(snapshot_type)

    def print_legs_diff(self):
        print(self.legs_deltas)
        #for leg_num, leg in self.legs.items():
        #    print(f'Delta {leg_num} : [{round(leg.D.x - leg.O.x, 2)}, {round(leg.D.y - leg.O.y, 2)}, {round(leg.D.z - leg.O.z, 2)}]')
        
    # 1-legged movements
    
    def move_body_straight(self, delta_x, delta_y, leg_seq=[1, 3, 4, 2]):
        for leg_number in leg_seq:
            self.logger.info(f'Processing leg {leg_number} with compensation')
            self.leg_move_with_compensation(leg_number, delta_x, delta_y)
        self.logger.info(f'Processing body to center')
        self.body_to_center()
    """
    def move_body_straight_(self, delta_x, delta_y, leg_seq=[1, 3, 4, 2]):
        self.body_movement(4, 0, 0)
        self.move_leg_endpoint(3, [delta_x, delta_y, cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(3, [0, 0, -cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(4, [delta_x, delta_y, cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(4, [0, 0, -cfg.fenix["leg_up"][1]])

        self.body_movement(-8, 0, 0)
        self.move_leg_endpoint(1, [delta_x, delta_y, cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(1, [0, 0, -cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(2, [delta_x, delta_y, cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(2, [0, 0, -cfg.fenix["leg_up"][1]])
        
        self.body_to_center()

    def move_body_straight(self, delta_x, delta_y, leg_seq=[1, 3, 4, 2]):
        self.body_movement(-10, 0, 0)
        self.move_leg_endpoint(1, [delta_x, delta_y, cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(1, [0, 0, -cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(2, [delta_x, delta_y, cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(2, [0, 0, -cfg.fenix["leg_up"][1]])

        self.body_movement(18, 0, 0)
        self.move_leg_endpoint(3, [delta_x, delta_y, cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(3, [0, 0, -cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(4, [delta_x, delta_y, cfg.fenix["leg_up"][1]])
        self.move_leg_endpoint(4, [0, 0, -cfg.fenix["leg_up"][1]])
        
        self.body_to_center()
    """
    """
    Two phased moves
    """
    # phased 2-legged movement
    def move_2_legs_phased_13(self, delta_x: int = 0, delta_y: int = 0) -> None:
        self.body_movement(round(delta_x / 2, 1), round(delta_y / 2, 1), 0)
        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(delta_x, delta_y, self.leg_up)
        self.add_angles_snapshot('endpoints')

        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(0, 0, -self.leg_up)
        self.add_angles_snapshot('endpoints')
        
    def move_2_legs_phased_24(self, delta_x: int = 0, delta_y: int = 0) -> None:
        self.body_movement(round(delta_x / 2, 1), round(delta_y / 2, 1), 0)
        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(delta_x, delta_y, self.leg_up)
        self.add_angles_snapshot('endpoints')

        #self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(0, 0, -self.leg_up)
        self.add_angles_snapshot('endpoints')      
    
    
    """
    Two phased moves end
    """
    # 2-legged movements
    def turn_in_move(self, angle):
        #angle = -20
        distance = 7
        leg_up = self.leg_up + 1
        x1, y1, x2, y2, x3, y3, x4, y4 = move_robot_legs(
            self.legs[1].D.x, self.legs[1].D.y,
            self.legs[2].D.x, self.legs[2].D.y,
            self.legs[3].D.x, self.legs[3].D.y,
            self.legs[4].D.x, self.legs[4].D.y,
            angle, distance
        )
        d12 = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        d23 = math.sqrt((x2-x3)**2 + (y2-y3)**2)
        d34 = math.sqrt((x4-x3)**2 + (y4-y3)**2)
        
        x1_delta = x1 - self.legs[1].D.x
        y1_delta = y1 - self.legs[1].D.y

        x2_delta = x2 - self.legs[2].D.x
        y2_delta = y2 - self.legs[2].D.y

        x3_delta = x3 - self.legs[3].D.x
        y3_delta = y3 - self.legs[3].D.y

        x4_delta = x4 - self.legs[4].D.x
        y4_delta = y4 - self.legs[4].D.y

        self.legs[2].move_end_point(x2_delta, y2_delta, leg_up)
        self.legs[4].move_end_point(x4_delta, y4_delta, leg_up)
        self.add_angles_snapshot('endpoints')

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(0, 0, -leg_up)
        self.add_angles_snapshot('endpoints')
        
        self.body_to_center()

        self.legs[1].move_end_point(x1_delta, y1_delta, leg_up)
        self.legs[3].move_end_point(x3_delta, y3_delta, leg_up)
        self.add_angles_snapshot('endpoints')

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(0, 0, -leg_up)
        self.add_angles_snapshot('endpoints')
        
        self.body_to_center(snapshot=False)
        self.turn(-angle, True)

    def move_2_legs(self, delta_y, steps=0):
        self.leg_up = 5
        leg_delta_1 = [0, delta_y, self.leg_up]
        leg_delta_2 = [0, 0, -self.leg_up]
        leg_delta_3 = [0, 2*delta_y, self.leg_up]
        offset_y = 0

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(*leg_delta_1)
        self.add_angles_snapshot('endpoints')
        # self.body_movement(0, -offset_y, 0)

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(*leg_delta_2)

        #self.add_angles_snapshot()
        
        self.body_movement(0, delta_y + offset_y, 0) # it adds snapshot itself

        #self.add_angles_snapshot()
        #self.body_movement(0, delta_y, 0)

        ##########
        if steps > 1:
            for _ in range(steps-1):
                for leg in [self.legs[1], self.legs[3]]:
                    leg.move_end_point(*leg_delta_3)
                self.add_angles_snapshot('endpoints')
                #self.body_movement(0, -offset_y, 0)

                for leg in [self.legs[1], self.legs[3]]:
                    leg.move_end_point(*leg_delta_2)
                
                #self.add_angles_snapshot()

                self.body_movement(0, delta_y + offset_y, 0)

                #self.add_angles_snapshot()
                #self.body_movement(0, delta_y, 0)

                for leg in [self.legs[2], self.legs[4]]:
                    leg.move_end_point(*leg_delta_3)
                self.add_angles_snapshot('endpoints')
                #self.body_movement(0, -offset_y, 0)

                for leg in [self.legs[2], self.legs[4]]:
                    leg.move_end_point(*leg_delta_2)

                #self.add_angles_snapshot()
                self.body_movement(0, delta_y + offset_y, 0)

                #self.add_angles_snapshot()
                #self.body_movement(0, delta_y, 0)

        ##########

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(*leg_delta_1)
        self.add_angles_snapshot('endpoints')

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(*leg_delta_2)
        self.add_angles_snapshot('endpoints')
    
    # body movement not while legs are going down
    def move_2_legs_v2(self, delta_y, steps=0):
        leg_delta_1 = [0, delta_y, self.leg_up]
        leg_delta_2 = [0, 0, -self.leg_up]
        leg_delta_3 = [0, 2*delta_y, self.leg_up]
        offset_y = 0

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(*leg_delta_1)
        self.add_angles_snapshot('endpoints')
        # self.body_movement(0, -offset_y, 0)

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(*leg_delta_2)

        self.add_angles_snapshot('endpoints')
        
        self.body_movement(0, delta_y + offset_y, 0) # it adds snapshot itself

        ##########
        if steps > 1:
            for _ in range(steps-1):
                for leg in [self.legs[1], self.legs[3]]:
                    leg.move_end_point(*leg_delta_3)
                self.add_angles_snapshot('endpoints')

                for leg in [self.legs[1], self.legs[3]]:
                    leg.move_end_point(*leg_delta_2)
                
                self.add_angles_snapshot('endpoints')

                self.body_movement(0, delta_y + offset_y, 0)

                for leg in [self.legs[2], self.legs[4]]:
                    leg.move_end_point(*leg_delta_3)
                self.add_angles_snapshot('endpoints')

                for leg in [self.legs[2], self.legs[4]]:
                    leg.move_end_point(*leg_delta_2)

                self.add_angles_snapshot('endpoints')
                self.body_movement(0, delta_y + offset_y, 0)

        ##########

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(*leg_delta_1)
        self.add_angles_snapshot('endpoints')

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(*leg_delta_2)
        self.add_angles_snapshot('endpoints')

    def climb_20(self):
        self.body_movement(0, 0, 14)
        self.body_movement(-5, 0, 0)
        
        self.leg_movement(1, [-5, 10, 27])
        self.leg_movement(1, [25, -10, 0])
        self.leg_movement(1, [0, 0, -7])
        
        self.leg_movement(2, [-5, -10, 27])
        self.leg_movement(2, [25, 10, 0])
        self.leg_movement(2, [0, 0, -7])
        
        self.body_movement(14, 5, 4)

        self.leg_movement(3, [10, 0, 5])
        self.leg_movement(3, [0, 0, -5])

        self.body_movement(-1, -15, -2)

        self.leg_movement(4, [10, 0, 5])
        self.leg_movement(4, [0, 0, -5])

        self.body_movement(-3, 10, 3)

        self.leg_movement(1, [15, 0, 5])
        self.leg_movement(1, [0, 0, -5])

        self.leg_movement(2, [15, 0, 5])
        self.leg_movement(2, [0, 0, -5])

        self.body_movement(23, 0, -2)
        self.body_movement(0, 0, -3)

        self.leg_movement(3, [15, -5, 25], snapshot=False)
        self.leg_movement(4, [15, 5, 25])

        self.leg_movement(3, [10, 0, 0], snapshot=False)
        self.leg_movement(4, [10, 0, 0])

        self.leg_movement(3, [0, 5, -5], snapshot=False)
        self.leg_movement(4, [0, -5, -5])

        self.body_movement(3, 0, 0)
        self.body_movement(0, 0, 6)

        self.body_to_center()
    
    def climb_2_legs(self, delta_z):
        # legs 1 and 2 up
        self.body_movement(0, 0, delta_z)
        tst_leg_up = 7
        #self.body_movement(-3, 0, 0)
        self.body_movement(-9, 0, 0) # 
        step_length = 8
        #self.legs[1].move_end_point(0, 0, 30) # 
        #self.add_angles_snapshot('endpoint') # 
        #self.legs[1].move_end_point(0, 0, -30) # 
        #self.add_angles_snapshot('endpoint') #

        for leg_number in [1, 2]:
            self.legs[leg_number].move_end_point(0, 0, delta_z + tst_leg_up)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(step_length, 0, 0)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(0, 0, -tst_leg_up)
            self.add_angles_snapshot('endpoint')
        
        #self.body_movement(step_length + 6, 0, 0)
        self.body_movement(step_length + 12, 0, 0) #

        for leg_number in [3, 4]:            
            self.legs[leg_number].move_end_point(step_length, 0, tst_leg_up)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(0, 0, -tst_leg_up)
            self.add_angles_snapshot('endpoint')

        self.body_movement(-3, 0, 0)

        # 3 moves of legs
        self.move_2_legs_phased_24(7, 0)
        self.move_2_legs_phased_13(14, 0)
        self.move_2_legs_phased_24(14, 0)
        self.move_2_legs_phased_13(7, 0)

        # legs 3 and 4 up
        self.body_movement(-3, 0, 0)
        self.turn_move(-13)

        for leg_number in [1, 2]:            
            self.legs[leg_number].move_end_point(step_length, 0, tst_leg_up)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(0, 0, -tst_leg_up)
            self.add_angles_snapshot('endpoint')
        
        self.body_movement(step_length + 6, 0, 0)

        for leg_number in [3, 4]:
            self.legs[leg_number].move_end_point(0, 0, delta_z + tst_leg_up)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(step_length, 0, 0)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(0, 0, -tst_leg_up)
            self.add_angles_snapshot('endpoint')

        self.body_movement(-3, 0, 0)
        
    
    def descend_2_legs(self, delta_z):
        tst_leg_up = 7
        self.body_movement(-3, 0, 0)
        step_length = 8

        for leg_number in [1, 2]:            
            self.legs[leg_number].move_end_point(0, 0, tst_leg_up)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(step_length, 0, 0)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(0, 0, -delta_z-tst_leg_up)
            self.add_angles_snapshot('endpoint')
        
        self.body_movement(step_length + 6, 0, 0)

        for leg_number in [3, 4]:            
            self.legs[leg_number].move_end_point(step_length, 0, tst_leg_up)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(0, 0, -tst_leg_up)
            self.add_angles_snapshot('endpoint')

        self.body_movement(-3, 0, 0)

        # 3 moves of legs
        self.move_2_legs_phased_24(7, 0)
        self.move_2_legs_phased_13(14, 0)
        self.move_2_legs_phased_24(14, 0)
        self.move_2_legs_phased_13(14, 0)
        self.move_2_legs_phased_24(7, 0)

        # legs 3 and 4 up
        self.body_movement(-3, 0, 0)
        self.turn_move(-13)

        for leg_number in [1, 2]:            
            self.legs[leg_number].move_end_point(step_length, 0, tst_leg_up)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(0, 0, -tst_leg_up)
            self.add_angles_snapshot('endpoint')
        
        self.body_movement(step_length + 6, 0, 0)

        for leg_number in [3, 4]:
            self.legs[leg_number].move_end_point(0, 0, tst_leg_up)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(step_length, 0, 0)
            self.add_angles_snapshot('endpoint')
            self.legs[leg_number].move_end_point(0, 0, -delta_z-tst_leg_up)
            self.add_angles_snapshot('endpoint')

        self.body_movement(-3, 0, 0)

        self.body_movement(0, 0, -delta_z)
        
    """
    def climb_2_legs(self, delta_z, steps_arr=[8, 12, 12, 12, 8, 12, 8]): #steps_arr=[8, 16, 16, 6, 6, 8]
        
        steps_arr=[12, 12, 6, 12, 6, 6, 6, 12, 12]
        self.body_movement(0, 0, delta_z)
        positive_delta_z = 0 # for climbing up
        negative_delta_z = 0 # for climbing down
        if delta_z > 0:
            positive_delta_z = delta_z
        else:
            negative_delta_z = delta_z

        #tst_leg_up = round(self.leg_up/2)
        tst_leg_up = 5

        legs_z_up_delta = {1: tst_leg_up, 
                           2: tst_leg_up,
                           3: tst_leg_up,
                           4: tst_leg_up}
        
        legs_z_down_delta = {1: -tst_leg_up, 
                             2: -tst_leg_up,
                             3: -tst_leg_up,
                             4: -tst_leg_up}

        sum_even, sum_odd, sum_body_movement = 0, 0, 0
        self.body_movement(-2, 0, 0)
        for step, value in enumerate(steps_arr):
            current_delta_z_up = {key: value for key, value in legs_z_up_delta.items()}
            current_delta_z_down = {key: value for key, value in legs_z_down_delta.items()}
            
            if step == 0:
                body_movement_value = value
            elif step == len(steps_arr) - 1:
                body_movement_value = 0
            else:
                body_movement_value = round(value/2)
            
            sum_body_movement += body_movement_value
            
            if step == 0:
                # print('Leg with delta z is 4')
                current_delta_z_up[1] += positive_delta_z
                current_delta_z_down[1] += negative_delta_z
            if step == 1:
                # print('Leg with delta z id 1')
                current_delta_z_up[2] += positive_delta_z
                current_delta_z_down[2] += negative_delta_z

            if step == 2:
                self.body_movement(4, 0, 0)
            if step % 2 == 0:
                sum_even += value
                if step >= len(steps_arr) - 2:
                    # print('Leg with delta z is 2')
                    current_delta_z_up[3] += positive_delta_z
                    current_delta_z_down[3] += negative_delta_z
                # print('Moving legs 2, 4')
                legs_to_move = [1, 3]                
            else:
                sum_odd += value
                if step >= len(steps_arr) - 2:
                    # print('Leg with delta z is 3')
                    current_delta_z_up[4] += positive_delta_z
                    current_delta_z_down[4] += negative_delta_z
                legs_to_move = [2, 4]
                # print('Moving legs 1, 3')
            
            # up
            for leg_number in legs_to_move:
                #self.legs[leg_number].move_end_point(0, -2, current_delta_z_up[leg_number])
                self.legs[leg_number].move_end_point(0, 0, current_delta_z_up[leg_number])
            self.add_angles_snapshot('endpoint')

            # forward
            for leg_number in legs_to_move:
                #self.legs[leg_number].move_end_point(0, value + 2, 0)
                self.legs[leg_number].move_end_point(value, 0, 0)
            self.add_angles_snapshot('endpoint')

            # down
            for leg_number in legs_to_move:
                self.legs[leg_number].move_end_point(0, 0, current_delta_z_down[leg_number])
            self.add_angles_snapshot('endpoint')

            self.body_movement(body_movement_value, 0, 0) # it adds snapshot itself
        
        self.body_movement(-2, 0, 0)
        if sum_even != sum_odd or sum_even != sum_body_movement:
            raise Exception(f'Bad step lengths: odd ({sum_odd}) and even ({sum_even}) and body({sum_body_movement}) not equal')
    
        self.current_legs_offset_v += delta_z
    """
    def reposition_legs(self, delta_x, delta_y):
        self.logger.info(f'reposition_legs ({delta_x}, {delta_y})')
        if delta_x == delta_y == 0:
            return None

        self.legs[2].move_end_point(delta_x, -delta_y, self.leg_up)
        self.legs[4].move_end_point(-delta_x, delta_y, self.leg_up)
        self.add_angles_snapshot('endpoints')

        self.legs[2].move_end_point(0, 0, -self.leg_up)
        self.legs[4].move_end_point(0, 0, -self.leg_up)
        self.add_angles_snapshot('endpoints')

        self.legs[1].move_end_point(delta_x, delta_y, self.leg_up)
        self.legs[3].move_end_point(-delta_x, -delta_y, self.leg_up)
        self.add_angles_snapshot('endpoints')

        self.legs[1].move_end_point(0, 0, -self.leg_up)
        self.legs[3].move_end_point(0, 0, -self.leg_up)
        self.add_angles_snapshot('endpoints')

        self.current_legs_offset_h_x += delta_x
        self.current_legs_offset_h_y += delta_y
    """
    def turn_body_and_legs(self, angle_deg):
        angle = math.radians(angle_deg)
        for leg in [self.legs[2], self.legs[4]]:
            x_new, y_new = turn_on_angle(leg.D.x, leg.D.y, angle)
            delta_x = x_new - leg.D.x
            delta_y = y_new - leg.D.y
            leg.move_end_point(delta_x, delta_y, self.leg_up)
        self.add_angles_snapshot()
        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(0, 0, -self.leg_up)
        
        self.add_angles_snapshot()
        for leg in [self.legs[1], self.legs[3]]:
            x_new, y_new = turn_on_angle(leg.D.x, leg.D.y, angle)
            delta_x = x_new - leg.D.x
            delta_y = y_new - leg.D.y
            leg.move_end_point(delta_x, delta_y, self.leg_up)
        self.add_angles_snapshot()
        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(0, 0, -self.leg_up)
        self.add_angles_snapshot()
        self.turn_only_body(angle_deg)
    """
    def turn_move(self, angle_deg):
        self.turn(-angle_deg)
        self.turn(angle_deg, only_body=True)  

    def turn(self, angle_deg, only_body=False):
        angle = math.radians(angle_deg)

        center_x = round((self.legs[2].O.x + self.legs[4].O.x) / 2, 2)
        center_y = round((self.legs[2].O.y + self.legs[4].O.y) / 2, 2)

        for leg in [self.legs[2], self.legs[4]]:
            x_new, y_new = turn_on_angle(center_x, center_y, leg.D.x, leg.D.y, angle)
            delta_x = x_new - leg.D.x
            delta_y = y_new - leg.D.y

            leg.move_end_point(delta_x, delta_y, self.leg_up)

        if not only_body:
            self.add_angles_snapshot()

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(0, 0, -self.leg_up)

        if not only_body:
            self.add_angles_snapshot()

        for leg in [self.legs[1], self.legs[3]]:
            x_new, y_new = turn_on_angle(center_x, center_y, leg.D.x, leg.D.y, angle)
            delta_x = x_new - leg.D.x
            delta_y = y_new - leg.D.y

            leg.move_end_point(delta_x, delta_y, self.leg_up)

        if not only_body:
            self.add_angles_snapshot()

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(0, 0, -self.leg_up)

        self.add_angles_snapshot('endpoint')
        self.side_look_angle += angle

    """
    def old_turn(self, angle_deg, only_body=False):
        angle = math.radians(angle_deg)
        for leg in [self.legs[2], self.legs[4]]:
            x_new, y_new = turn_on_angle(leg.D.x, leg.D.y, angle)
            delta_x = x_new - leg.D.x
            delta_y = y_new - leg.D.y
            leg.move_end_point(delta_x, delta_y, self.leg_up)
        if not only_body:
            self.add_angles_snapshot()
        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(0, 0, -self.leg_up)
        if not only_body:
            self.add_angles_snapshot()
        for leg in [self.legs[1], self.legs[3]]:
            x_new, y_new = turn_on_angle(leg.D.x, leg.D.y, angle)
            delta_x = x_new - leg.D.x
            delta_y = y_new - leg.D.y
            leg.move_end_point(delta_x, delta_y, self.leg_up)
        if not only_body:
            self.add_angles_snapshot()
        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(0, 0, -self.leg_up)
        self.add_angles_snapshot()
    """
    # no leg movements
    """
    def turn_only_body(self, angle_deg):
        angle = math.radians(angle_deg)
        for leg in self.legs.values():
            x_new, y_new = turn_on_angle(leg.O.x, leg.O.y, angle)
            delta_x = x_new - leg.O.x
            delta_y = y_new - leg.O.y
            print(f'[{leg.O.x},{leg.O.y}] -> [{x_new}, {y_new}]')
            leg.move_mount_point(delta_x, delta_y, 0)
        self.add_angles_snapshot()
    """

    def look_on_angle_new(self, up):
        angle = cfg.moves['vertical_look_angle']
        x = cfg.leg["mount_point_offset"] * math.cos(math.radians(angle))
        z = cfg.leg["mount_point_offset"] * math.sin(math.radians(angle))
        x = 3
        z = 2
        print(f'X: {x}, Z: {z}. Height: {self.height}')
        for leg in [self.legs[1], self.legs[2]]:
            if up:
                leg.move_mount_point(0, 0, z)
            else:
                leg.move_mount_point(0, 0, -z)
        for leg in [self.legs[3], self.legs[4]]:
            if up:
                leg.move_mount_point(0, 0, -z)
            else:
                leg.move_mount_point(0, 0, z)

        if up:
            self.body_movement(x, 0, 0)
        else:
            self.body_movement(-x, 0, 0)

        self.add_angles_snapshot('body')

    # looks like this is bugged in cached kinematics, cuz current angle not working well
    def look_on_angle(self, angle):
        current_angle = self.current_angle

        x_current = cfg.leg["mount_point_offset"] * \
                    math.cos(math.radians(current_angle))
        z_current = cfg.leg["mount_point_offset"] * \
                    math.sin(math.radians(current_angle))

        x_target = cfg.leg["mount_point_offset"] * math.cos(math.radians(angle))
        z_target = cfg.leg["mount_point_offset"] * math.sin(math.radians(angle))

        x = x_current - x_target
        z = z_current - z_target

        for leg in [self.legs[1], self.legs[4]]:
            leg.move_mount_point(0, -x, z)
        for leg in [self.legs[2], self.legs[3]]:
            leg.move_mount_point(0, x, -z)

        self.add_angles_snapshot('body')

        self.current_angle = angle

    def kneel(self):
        self.body_movement(-10, 0, 6)
        self.move_leg_endpoint(1, [0, -8, 5])
        self.move_leg_endpoint(1, [-10, 0, 0])
        self.move_leg_endpoint(1, [0, 0, -5])
        self.move_leg_endpoint(2, [0, 8, 5])
        self.move_leg_endpoint(2, [-10, 0, 0])
        self.move_leg_endpoint(2, [0, 0, -5])
        self.body_movement(5, 0, 0)
        self.body_movement(0, 0, -5)

        """
        self.body_movement(0, 0, 5)
        self.body_movement(-5, 0, 0)
        self.move_leg_endpoint(2, [0, 0, 5])
        self.move_leg_endpoint(2, [10, 0, 0])
        self.move_leg_endpoint(2, [0, -8, -5])
        self.move_leg_endpoint(1, [0, 0, 5])
        self.move_leg_endpoint(1, [10, 0, 0])
        self.move_leg_endpoint(1, [0, 8, -5])
        self.body_movement(10, 0, -6)
        """
    # demo part
    def battle_stance(self) -> None:
        self.body_movement(0, -10, -2)
        self.look_on_angle(-10)
    
    def normal_stance(self) -> None:
        self.look_on_angle(10)
        self.body_movement(0, 10, 2)        

    def jump(self) -> None:
        self.body_movement(0, 14, 8)
    
    def demo1(self) -> None:
        self.body_movement(0, 3, 8)
        self.turn(-20, only_body=True)
        self.turn(40, only_body=True)
        self.turn(-20, only_body=True)
        self.body_movement(0, -3, -8)
    
    def demo11(self) -> None:
        self.body_movement(0, 3, 8)
        self.turn(-20, only_body=True)
    
    def demo12(self) -> None:
        self.turn(40, only_body=True)
    
    def demo13(self) -> None:
        self.turn(-20, only_body=True)
        self.body_movement(0, -3, -8)

    def demo2(self) -> None:
        self.body_movement(0, 15, 0)
        self.look_on_angle(-20)
        

    # fun moves
    def hit(self, leg_num):
        x_move = 10
        if leg_num == 2:
            x_move = -10
        self.logger.info(f'Processing leg {leg_num} body_compensation_for_a_leg')
        self.body_compensation_for_a_leg(leg_num)
        self.logger.info(f'Processing leg {leg_num} move_end_point 1')
        self.legs[leg_num].move_end_point(10, -x_move, 5)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 2')
        self.legs[leg_num].move_end_point(13, 0, 10)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 3')
        self.legs[leg_num].move_end_point(-13, 0, -10)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 4')
        self.legs[leg_num].move_end_point(-10, x_move, 0)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 5')
        self.legs[leg_num].move_end_point(0, 0, -5)
        self.add_angles_snapshot('endpoint')
        self.body_to_center()

    def play(self, leg_num=2):
        x_move = 10
        if leg_num == 2:
            x_move = -10
        self.logger.info(f'Processing leg {leg_num} body_compensation_for_a_leg')
        self.body_compensation_for_a_leg(leg_num)
        self.logger.info(f'Processing leg {leg_num} move_end_point 1')
        self.legs[leg_num].move_end_point(10, -x_move, 5)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 2')
        self.legs[leg_num].move_end_point(-15, 2*x_move, 3)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 3')
        self.legs[leg_num].move_end_point(15, -2*x_move, -3)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 4')
        self.legs[leg_num].move_end_point(-10, x_move, 0)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 5')
        self.legs[leg_num].move_end_point(0, 0, -5)
        self.add_angles_snapshot('endpoint')
        self.body_to_center()

    # dance moves
    def opposite_legs_up(self, leg_up, leg_forward):
        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(0, 0, leg_up)
        self.add_angles_snapshot('endpoint')

        self.legs[1].move_end_point(leg_forward, leg_forward, 5)
        self.legs[3].move_end_point(-leg_forward, -leg_forward, 5)

        self.add_angles_snapshot('endpoint')

        self.legs[1].move_end_point(-leg_forward, -leg_forward, -5)
        self.legs[3].move_end_point(leg_forward, leg_forward, -5)

        self.add_angles_snapshot('endpoint')
        
        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(0, 0, -leg_up)
        self.add_angles_snapshot('endpoint')

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(0, 0, leg_up)
        self.add_angles_snapshot('endpoint')
        
        self.legs[2].move_end_point(leg_forward, -leg_forward, 5)
        self.legs[4].move_end_point(-leg_forward, leg_forward, 5)        

        self.add_angles_snapshot('endpoint')

        self.legs[2].move_end_point(-leg_forward, leg_forward, -5)
        self.legs[4].move_end_point(leg_forward, -leg_forward, -5)        

        self.add_angles_snapshot('endpoint')
        
        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(0, 0, -leg_up)
        self.add_angles_snapshot('endpoint')

    # Obstacles processing
    def move_body_up(self, delta_z):
        self.body_movement(0, 0, delta_z)

    def move_body_down(self, delta_z):
        self.body_movement(0, 0, -delta_z)


    def move_according_to_plan(self, plan: List[Move]):
        self.successful_moves = 0
        #print('------------BEFORE--------------')
        #for item in self.D_points_history:
        #    print(item)
        #print('------------BEFORE--------------')
        print(f'Plan: {plan}')
        for move in plan:
            #try:
            if move.command == 'forward':
                self.move_1_legged_for_diff(move)
            elif move.command == 'up':
                self.move_body_up(move.value)
            elif move.command == 'down':
                self.move_body_down(move.value)
            #self.successful_moves += 1
            #except Exception as e:
            #    print(f'Successful moves : {self.successful_moves}')
            #    raise Exception(e)
        #print('------------AFTER--------------')
        #for item in self.D_points_history:
        #    print(item)
        #print('------------AFTER--------------')

    def move_1_legged_for_diff(self, move: Move):
        #for leg_number, deltas in move.target_legs_position.items():
        self.logger.info(f'Move. : {move}')

        self.body_movement(-6, 0, 0)

        for leg_number in [1, 2, 3, 4]:
            if leg_number == 3:
                self.body_movement(12 + diff[0], 0, 0) # diff[0] here is crutch
            self.logger.info(f'Leg {leg_number}')
            deltas = move.target_legs_position[leg_number]
            #self.logger.info(f'Move. Deltas : {deltas}')
            C = self.legs[leg_number].C
            plan_x = deltas[0]
            plan_y = deltas[1]
            plan_z = deltas[2]
            diff = [plan_x - C.x, plan_y - C.y, plan_z - C.z]
            self.logger.info(f'Move. Diff : {diff}')
            self.leg_move_obstacled(leg_number, *diff, move_type=2)

        self.body_to_center()

if __name__ == '__main__':
    angles = [0.6011, 0.6493, -1.4409, -1.5582, -0.4712, 0.2555, -2.2452, 0.4314, -2.3604, 0.067, -2.2996, 0.2346, 2.0043, 0.2178, -2.3248, 0.4691]
    
    # {'leg': 1, 'deltas': [8, 0, 16]} : Moving to [-58.0, 43.74, -29.26, -11.76, -13.97, 27.24, 13.27, 17.51, -8.09, 38.6, 6.51, -0.25, -31.7, 42.83, 11.13, -19.18]
    fk = FenixKinematics(fenix_position=angles)
    fk.move_leg_endpoint(1, [0, 0, -20])
    print(fk.sequence[-1].angles_snapshot)
    angles_2 = [0.777, 0.2513, -1.4451, -0.3602, -0.4671, 0.222, -2.2452, 0.4608, -2.3604, 0.0712, -2.2996, 0.2388, 2.0001, 0.1717, -2.3499, 0.5864]
    print([math.degrees(x) for x in angles_2])

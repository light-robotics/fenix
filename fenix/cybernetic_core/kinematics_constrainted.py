import math
from dataclasses import dataclass
from typing import List
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from configs import config as cfg
from cybernetic_core.cybernetic_utils.angles_constrainted import get_leg_angles, turn_on_angle
from cybernetic_core.cybernetic_utils.lines import Point, LinearFunc, calculate_intersection, move_on_a_line
from cybernetic_core.cybernetic_utils.constraints import ConstraintsChecker
import configs.code_config as code_config
import logging.config


@dataclass
class MoveSnapshot:
    move_type: str
    angles_snapshot: List[float]


class Leg:
    def __init__(self, O: Point, D: Point, leg_tag: str, cc: ConstraintsChecker):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('main_logger')
        self.O = O
        self.D = D
        self.leg_tag = leg_tag
        self.cc = cc
        self.update_angles()

    def update_angles(self):
        self.tetta, self.alpha, self.beta, self.gamma = self.calculate_angles()

    def calculate_angles(self):
        O = self.O
        D = self.D
        tetta = math.atan2(D.y - O.y, D.x - O.x)
        if not self.cc.leg_angles_correct(leg_type=self.leg_tag, tetta=tetta):
            self.logger.info(f'Bad tetta : {tetta}')
            raise Exception(f'Bad tetta : {tetta}')

        A = Point(O.x + cfg.leg["d"] * math.cos(tetta),
                  O.y + cfg.leg["d"] * math.sin(tetta),
                  O.z)

        l = round(math.sqrt((D.x - A.x) ** 2 + (D.y - A.y) ** 2), 2)
        delta_z = round(D.z - O.z, 2)
        self.logger.info(f'Trying l {l} and delta_z {delta_z}')
        alpha, beta, gamma = get_leg_angles(l, delta_z, self.leg_tag, self.cc)
        self.logger.info(f'Success : {math.degrees(alpha)}, {math.degrees(beta)}, {math.degrees(gamma)}')

        Bx = cfg.leg["a"] * math.cos(alpha)
        By = cfg.leg["a"] * math.sin(alpha)
        Cx = Bx + cfg.leg["b"] * math.cos(alpha + beta)
        Cy = By + cfg.leg["b"] * math.sin(alpha + beta)
        Dx = Cx + cfg.leg["c"] * math.cos(alpha + beta + gamma)
        Dy = Cy + cfg.leg["c"] * math.sin(alpha + beta + gamma)
        if abs(Dx - l) > 0.01 or abs(Dy - delta_z) > 0.01:
            self.logger.info(f'WTF')
            print('WTF')

        B_xz = [cfg.leg["a"] * math.cos(alpha),
                cfg.leg["a"] * math.sin(alpha)]
        C_xz = [B_xz[0] + cfg.leg["b"] * math.cos(alpha + beta),
                B_xz[1] + cfg.leg["b"] * math.sin(alpha + beta)]
        D_xz = [C_xz[0] + cfg.leg["c"] * math.cos(alpha + beta + gamma),
                C_xz[1] + cfg.leg["c"] * math.sin(alpha + beta + gamma)]

        D_prev = D
        self.A = A
        self.B = Point(A.x + B_xz[0] * math.cos(tetta),
                       A.y + B_xz[0] * math.sin(tetta),
                       A.z + B_xz[1])
        self.C = Point(A.x + C_xz[0] * math.cos(tetta),
                       A.y + C_xz[0] * math.sin(tetta),
                       A.z + C_xz[1])
        self.D = Point(A.x + D_xz[0] * math.cos(tetta),
                       A.y + D_xz[0] * math.sin(tetta),
                       A.z + D_xz[1])

        if abs(D_prev.x - self.D.x) > 0.01 or \
           abs(D_prev.y - self.D.y) > 0.01 or \
           abs(D_prev.z - self.D.z) > 0.01:
            raise Exception('D_prev far from D. Angles : {0}'
                            .format(([tetta, alpha, beta, gamma])))

        return tetta, alpha, beta, gamma

    def move_mount_point(self, delta_x, delta_y, delta_z):
        self.O.move(delta_x, delta_y, delta_z)
        self.update_angles()
    
    def move_end_point(self, delta_x, delta_y, delta_z):
        self.D.move(delta_x, delta_y, delta_z)
        self.update_angles()

    # means one leg is raised and moves with the body
    # end_delta = 0 means that leg is not moving, else it is also moving somehow
    # wtf something weird here
    def move_both_points(self, delta_x, delta_y, delta_z, end_delta_x, end_delta_y, end_delta_z):
        self.move_point(self.O, delta_x, delta_y, delta_z)
        self.move_point(self.D,
                        delta_x + end_delta_x,
                        delta_y + end_delta_y,
                        delta_z + end_delta_z)
        self.update_angles()


class FenixKinematics:
    def __init__(self, legs_offset_v, legs_offset_h_x, legs_offset_h_y):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('main_logger')
        self.constraints_checker = ConstraintsChecker()

        self.legs_offset_v = legs_offset_v

        self.legs_offset_h_x = legs_offset_h_x
        self.legs_offset_h_y = legs_offset_h_y

        self.current_legs_offset_v = self.legs_offset_v
        self.current_legs_offset_h_x = self.legs_offset_h_x
        self.current_legs_offset_h_y = self.legs_offset_h_y

        self.legs_deltas = {1 : [0, 0, 0], 2 : [0, 0, 0], 3 : [0, 0, 0], 4 : [0, 0, 0]}

        self.current_vertical_angle = 0
        self.current_horizontal_angle = 0
        self.current_body_delta = [0, 0, 0]
        self.margin = cfg.fenix["margin"][1]
        self.leg_up = cfg.fenix["leg_up"][2]
        self.leg_up_single = cfg.fenix["leg_up"][1]
        
        self.legs = self.initiate_legs()

        self.angles_history = []
        self.add_angles_snapshot('init')

    def reset_history(self):
        self.angles_history = []

    def add_angles_snapshot(self, move_type: str = 'unknown'):
        # angles are : gamma1, beta1, alpha1, tetta1, gamma2, beta2, alpha2, tetta2 ...
        # for leg1 tetta = 45 means 0 for servo
        # leg2 tetta = -45, leg3 tetta = -135, leg4 tetta = 135        

        position = []
        for leg_number, leg in self.legs.items():
            # print(f'Fixing gamma : {round(math.degrees(leg.gamma), 2)} -> {round(math.degrees(leg.gamma) - cfg.leg["phi_angle"], 2)}')
            position.append(round(math.degrees(leg.gamma) - cfg.leg["phi_angle"], 2))
            position.append(round(math.degrees(leg.beta), 2))
            position.append(round(math.degrees(leg.alpha), 2))
            tetta = math.degrees(leg.tetta)
            if leg_number == 1:
                tetta -= 90
            if leg_number == 2:
                tetta += 90
            if leg_number == 3:
                tetta += 90
            if leg_number == 4:
                tetta -= 90
            tetta = round(tetta, 2)
            position.append(tetta)
        new_move = MoveSnapshot(move_type, self.convert_angles(position))
        self.angles_history.append(new_move)

    @staticmethod
    def convert_angles(angles):
        out_angles = []
        for i in range(4):
            for j in range(4):
                cur_value = angles[4*i + 3 - j]
                if j == 2:
                    out_angles.append(cur_value * -1)
                elif j == 0:
                    if i in [0, 2]:
                        tetta = round(cur_value + 45, 2)
                    else:
                        tetta = round(cur_value - 45, 2)
                    # -- experimental --
                    if tetta > 70:
                        tetta -= 360
                        print(f'Tetta converted : -360')
                    if tetta < -70:
                        tetta += 360
                        print(f'Tetta converted : +360')
                    # -- experimental --
                    if tetta > 70 or tetta < - 70:
                        raise Exception(f'Wrong tetta angle : {tetta}')
                    out_angles.append(tetta)
                else:
                    out_angles.append(cur_value)

        return out_angles

    @property
    def sequence(self):
        return self.angles_history

    def initiate_legs(self):
        O1 = Point(cfg.leg["mount_point_offset"],
                   cfg.leg["mount_point_offset"],
                   self.legs_offset_v)
        D1 = Point(self.legs_offset_h_x,
                   self.legs_offset_h_y - cfg.start["y_offset_body"],
                   0)
        self.logger.info('Initiating leg 1')
        Leg1 = Leg(O1, D1, 'front_leg', self.constraints_checker)

        O2 = Point(cfg.leg["mount_point_offset"],
                   -cfg.leg["mount_point_offset"],
                   self.legs_offset_v)
        D2 = Point(self.legs_offset_h_x,
                   -self.legs_offset_h_y - cfg.start["y_offset_body"],
                   0)
        self.logger.info('Initiating leg 2')
        Leg2 = Leg(O2, D2, 'rear_leg', self.constraints_checker)

        O3 = Point(-cfg.leg["mount_point_offset"],
                   -cfg.leg["mount_point_offset"],
                   self.legs_offset_v)
        D3 = Point(-self.legs_offset_h_x,
                   -self.legs_offset_h_y - cfg.start["y_offset_body"],
                   0)
        self.logger.info('Initiating leg 3')
        Leg3 = Leg(O3, D3, 'rear_leg', self.constraints_checker)

        O4 = Point(-cfg.leg["mount_point_offset"],
                   cfg.leg["mount_point_offset"],
                   self.legs_offset_v)
        D4 = Point(-self.legs_offset_h_x,
                   self.legs_offset_h_y - cfg.start["y_offset_body"],
                   0)
        self.logger.info('Initiating leg 4')
        Leg4 = Leg(O4, D4, 'front_leg', self.constraints_checker)

        return {1: Leg1, 2: Leg2, 3: Leg3, 4: Leg4}

    ################## MOVEMENTS START HERE ##################
    def leg_movement(self, leg_num, leg_delta):
        leg = self.legs[leg_num]

        leg.move_end_point(leg_delta[0], leg_delta[1], leg_delta[2])
        self.add_angles_snapshot('endpoint')

    def body_movement(self, delta_x, delta_y, delta_z, snapshot=True):
        print(f'Body movement [{delta_x}, {delta_y}, {delta_z}]')
        self.current_body_delta = [x + y for x, y in zip(self.current_body_delta, [delta_x, delta_y, delta_z])]
        print(f'self.current_body_delta : {self.current_body_delta}')
        if delta_x == delta_y == delta_z == 0:
            return

        for leg in self.legs.values():
            leg.move_mount_point(delta_x, delta_y, delta_z)

        if snapshot:
            self.add_angles_snapshot('body')

        self.current_legs_offset_v -= delta_z
    
    def start(self):
        self.body_movement(0, 0, -cfg.start["vertical"] + cfg.start["initial_z_position_delta"])
        self.body_movement(0, 0, cfg.start["vertical"] - cfg.start["initial_z_position_delta"])
    
    def reset(self):
        self.body_to_center()
        delta_z = self.current_body_delta[2]
        self.body_movement(0, 0, -delta_z)

    def end(self):
        self.reset()
        self.body_movement(0, 0, -cfg.start["vertical"] + 
                                  cfg.start["initial_z_position_delta"])
        """
        self.body_to_center()
        delta_z = self.current_body_delta[2]
        self.body_movement(0, 0, -cfg.start["vertical"] + 
                                  cfg.start["initial_z_position_delta"] - 
                                  delta_z)
        """

    def body_to_center(self, delta_y=cfg.start["y_offset_body"], delta_x=0):
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

        self.body_movement(round(avg_d_x - avg_o_x + delta_x, 2),
                           round(avg_d_y - avg_o_y + delta_y, 2),
                           0)

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
        #self.compensated_leg_movement(leg_num, [0, 0, -self.leg_up])

    def move_leg_endpoint(self, leg_num, leg_delta):        
        self.legs[leg_num].move_end_point(*leg_delta)
        self.legs_deltas[leg_num] = [x + y for x, y in zip(self.legs_deltas[leg_num], leg_delta)]        
        self.add_angles_snapshot('endpoint')

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
    Two phased moves
    """
    # phased 2-legged movement
    def move_2_legs_phased_13(self, delta_x: int = 0, delta_y: int = 0) -> None:
        self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(delta_x, delta_y, self.leg_up)
        self.add_angles_snapshot('endpoints')

        self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(0, 0, -self.leg_up)
        self.add_angles_snapshot('endpoints')

        
    def move_2_legs_phased_24(self, delta_x: int = 0, delta_y: int = 0) -> None:        
        self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(delta_x, delta_y, self.leg_up)
        self.add_angles_snapshot('endpoints')

        self.body_movement(round(delta_x / 4, 1), round(delta_y / 4, 1), 0)

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(0, 0, -self.leg_up)
        self.add_angles_snapshot('endpoints')      
    
    
    """
    Two phased moves end
    """
    # 2-legged movements
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

    def climb_2_legs(self, delta_z, steps_arr=[8, 12, 12, 12, 8, 12, 8]): #steps_arr=[8, 16, 16, 6, 6, 8]
        positive_delta_z = 0 # for climbing up
        negative_delta_z = 0 # for climbing down
        if delta_z > 0:
            positive_delta_z = delta_z
        else:
            negative_delta_z = delta_z

        #tst_leg_up = round(self.leg_up/2)
        tst_leg_up = 6 # 6 #5

        legs_z_up_delta = {1: tst_leg_up, 
                           2: tst_leg_up,
                           3: tst_leg_up,
                           4: tst_leg_up}
        
        legs_z_down_delta = {1: -tst_leg_up, 
                             2: -tst_leg_up,
                             3: -tst_leg_up,
                             4: -tst_leg_up}

        sum_even, sum_odd, sum_body_movement = 0, 0, 0
        for step, value in enumerate(steps_arr):
            current_delta_z_up = {key: value for key, value in legs_z_up_delta.items()}
            current_delta_z_down = {key: value for key, value in legs_z_down_delta.items()}
            
            """
            if step == 0 or step == len(steps_arr) - 1:
                body_movement_value = value
            else:
                body_movement_value = round(value/2)
            """
            
            if step == 0:
                body_movement_value = value
            elif step == len(steps_arr) - 1:
                body_movement_value = 0
            else:
                body_movement_value = round(value/2)
            
            sum_body_movement += body_movement_value

            if step == 0:
                # print('Leg with delta z is 4')
                current_delta_z_up[4] += positive_delta_z
                current_delta_z_down[4] += negative_delta_z
            if step == 1:
                # print('Leg with delta z id 1')
                current_delta_z_up[1] += positive_delta_z
                current_delta_z_down[1] += negative_delta_z

            if step % 2 == 0:
                sum_even += value
                if step >= len(steps_arr) - 2:
                    # print('Leg with delta z is 2')
                    current_delta_z_up[2] += positive_delta_z
                    current_delta_z_down[2] += negative_delta_z
                # print('Moving legs 2, 4')
                legs_to_move = [2, 4]                
            else:
                sum_odd += value
                if step >= len(steps_arr) - 2:
                    # print('Leg with delta z is 3')
                    current_delta_z_up[3] += positive_delta_z
                    current_delta_z_down[3] += negative_delta_z
                legs_to_move = [1, 3]
                # print('Moving legs 1, 3')
            
            # up
            for leg_number in legs_to_move:
                #self.legs[leg_number].move_end_point(0, -2, current_delta_z_up[leg_number])
                self.legs[leg_number].move_end_point(0, 0, current_delta_z_up[leg_number])
            self.add_angles_snapshot('endpoint')

            # forward
            for leg_number in legs_to_move:
                #self.legs[leg_number].move_end_point(0, value + 2, 0)
                self.legs[leg_number].move_end_point(0, value, 0)
            self.add_angles_snapshot('endpoint')

            # down
            for leg_number in legs_to_move:
                self.legs[leg_number].move_end_point(0, 0, current_delta_z_down[leg_number])
            self.add_angles_snapshot('endpoint')

            self.body_movement(0, body_movement_value, 0) # it adds snapshot itself
        
        if sum_even != sum_odd or sum_even != sum_body_movement:
            raise Exception(f'Bad step lengths: odd ({sum_odd}) and even ({sum_even}) and body({sum_body_movement}) not equal')
    
        self.current_legs_offset_v += delta_z

    """
    def climb_2_legs_down(self, delta_z, steps_arr=[8, 12, 12, 12, 8, 12, 8]): #steps_arr=[8, 16, 16, 6, 6, 8]
        negative_delta_z = delta_z # for climbing down

        #tst_leg_up = round(self.leg_up/2)
        tst_leg_up = 6 # 6 #5

        legs_z_up_delta = {1: tst_leg_up, 
                           2: tst_leg_up,
                           3: tst_leg_up,
                           4: tst_leg_up}
        
        legs_z_down_delta = {1: -tst_leg_up, 
                             2: -tst_leg_up,
                             3: -tst_leg_up,
                             4: -tst_leg_up}

        sum_even, sum_odd, sum_body_movement = 0, 0, 0
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
                current_delta_z_up[4] += positive_delta_z
                current_delta_z_down[4] -= negative_delta_z
            if step == 1:
                # print('Leg with delta z id 1')
                current_delta_z_up[1] += positive_delta_z
                current_delta_z_down[1] -= negative_delta_z

            if step % 2 == 0:
                sum_even += value
                if step >= len(steps_arr) - 2:
                    # print('Leg with delta z is 2')
                    current_delta_z_up[2] += positive_delta_z
                    current_delta_z_down[2] -= negative_delta_z
                # print('Moving legs 2, 4')
                legs_to_move = [2, 4]                
            else:
                sum_odd += value
                if step >= len(steps_arr) - 2:
                    # print('Leg with delta z is 3')
                    current_delta_z_up[3] += positive_delta_z
                    current_delta_z_down[3] -= negative_delta_z
                legs_to_move = [1, 3]
                # print('Moving legs 1, 3')
            
            # up
            for leg_number in legs_to_move:
                #self.legs[leg_number].move_end_point(0, -2, current_delta_z_up[leg_number])
                self.legs[leg_number].move_end_point(0, 0, current_delta_z_up[leg_number])
            self.add_angles_snapshot('endpoint')

            # forward
            for leg_number in legs_to_move:
                #self.legs[leg_number].move_end_point(0, value + 2, 0)
                self.legs[leg_number].move_end_point(0, value, 0)
            self.add_angles_snapshot('endpoint')

            # down
            for leg_number in legs_to_move:
                self.legs[leg_number].move_end_point(0, 0, current_delta_z_down[leg_number])
            self.add_angles_snapshot('endpoint')

            self.body_movement(0, body_movement_value, 0) # it adds snapshot itself
        
        if sum_even != sum_odd or sum_even != sum_body_movement:
            raise Exception(f'Bad step lengths: odd ({sum_odd}) and even ({sum_even}) and body({sum_body_movement}) not equal')
    
        self.current_legs_offset_v += delta_z
    """

    # 2-legged movements
    """
    def move_2_legs_v2(self, delta_y, steps=0):
        leg_delta_0 =  [0, 0, self.leg_up]
        leg_delta_11 = [0, delta_y, 0]
        leg_delta_12 = [0, 2*delta_y, 0]
        leg_delta_2 =  [0, 0, -self.leg_up]
        
        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(*leg_delta_0)
        self.add_angles_snapshot()
        
        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(*leg_delta_11)
        self.add_angles_snapshot()

        self.body_movement(0, delta_y, 0) # it adds snapshot itself

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(*leg_delta_2)
        self.add_angles_snapshot() ###
                
        ##########
        if steps > 1:
            for _ in range(steps-1):
                for leg in [self.legs[1], self.legs[3]]:
                    leg.move_end_point(*leg_delta_0)
                self.add_angles_snapshot()

                for leg in [self.legs[1], self.legs[3]]:
                    leg.move_end_point(*leg_delta_12)                
                self.body_movement(0, delta_y, 0)

                for leg in [self.legs[1], self.legs[3]]:
                    leg.move_end_point(*leg_delta_2)
                self.add_angles_snapshot() ###

                for leg in [self.legs[2], self.legs[4]]:
                    leg.move_end_point(*leg_delta_0)
                self.add_angles_snapshot()

                for leg in [self.legs[2], self.legs[4]]:
                    leg.move_end_point(*leg_delta_12)                
                self.body_movement(0, delta_y, 0)
                

                for leg in [self.legs[2], self.legs[4]]:
                    leg.move_end_point(*leg_delta_2)
                self.add_angles_snapshot() ###

        ##########

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(*leg_delta_0)
        self.add_angles_snapshot()

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(*leg_delta_11)
        self.add_angles_snapshot()

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(*leg_delta_2)
        self.add_angles_snapshot()
    """
    def reposition_legs(self, delta_x, delta_y):
        print(f'reposition_legs ({delta_x}, {delta_y})')
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
    def look_on_angle(self, angle):
        current_vertical_angle = self.current_vertical_angle

        x_current = cfg.leg["mount_point_offset"] * \
                    math.cos(math.radians(current_vertical_angle))
        z_current = cfg.leg["mount_point_offset"] * \
                    math.sin(math.radians(current_vertical_angle))

        x_target = cfg.leg["mount_point_offset"] * math.cos(math.radians(angle))
        z_target = cfg.leg["mount_point_offset"] * math.sin(math.radians(angle))

        x = x_current - x_target
        z = z_current - z_target

        for leg in [self.legs[1], self.legs[4]]:
            leg.move_mount_point(0, -x, z)
        for leg in [self.legs[2], self.legs[3]]:
            leg.move_mount_point(0, x, -z)

        self.add_angles_snapshot('body')

        self.current_vertical_angle = angle

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
        
    def check_leg(self, leg_num):
        turn_angle = 20 if leg_num == 1 else -20
        move_x = -2 if leg_num == 1 else 2
        move_y = 10
        self.logger.info(f'Processing leg {leg_num} body_compensation_for_a_leg')
        self.body_compensation_for_a_leg(leg_num)
        self.logger.info(f'Processing turn')
        self.turn(turn_angle, only_body=True)
        self.logger.info(f'Processing leg {leg_num} move_end_point 1')
        self.legs[leg_num].move_end_point(0, 0, 8)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 2')
        self.legs[leg_num].move_end_point(0, move_y, 5)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 3')
        self.legs[leg_num].move_end_point(move_x, -move_y, -5)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 5')
        self.legs[leg_num].move_end_point(0, 0, -8)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing turn')
        self.turn(-turn_angle, only_body=True)
        self.body_to_center()

    def check_leg_2(self, leg_num):
        turn_angle = 15 if leg_num == 1 else -15
        #move_x = -2 if leg_num == 1 else 2
        move_x = 0
        move_y = 8
        self.logger.info(f'Processing leg {leg_num} body_compensation_for_a_leg')
        self.body_compensation_for_a_leg(leg_num)
        self.logger.info(f'Processing turn')
        self.turn(turn_angle, only_body=True)
        self.logger.info(f'Processing leg {leg_num} move_end_point 1')
        self.legs[leg_num].move_end_point(0, 0, 8)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 2')
        self.legs[leg_num].move_end_point(0, move_y, 5)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 3')
        self.legs[leg_num].move_end_point(move_x, -move_y, -5)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 5')
        self.legs[leg_num].move_end_point(0, 0, -8)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing turn')
        self.turn(-turn_angle, only_body=True)
        self.body_to_center()

    def hit(self, leg_num):
        assert leg_num in [1, 4]
        move_x = 0
        move_y = 11
        self.logger.info(f'Processing leg {leg_num} body_compensation_for_a_leg')
        self.body_compensation_for_a_leg(leg_num)
        self.logger.info(f'Processing leg {leg_num} move_end_point 1')
        self.legs[leg_num].move_end_point(0, 0, 8)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 2')
        self.legs[leg_num].move_end_point(0, move_y, 5)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 3')
        self.legs[leg_num].move_end_point(move_x, -move_y, -5)
        self.add_angles_snapshot('endpoint')
        self.logger.info(f'Processing leg {leg_num} move_end_point 4')
        self.legs[leg_num].move_end_point(0, 0, -8)
        self.add_angles_snapshot('endpoint')
        self.body_to_center()

    def demo_sequence(self):
        self.demo1()
        self.check_leg(4)
        self.check_leg(1)
        self.battle_stance()

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

if __name__ == '__main__':
    fk = FenixKinematics(10, 10, 10)
    fk.body_movement(0, 0, 5)
    print(fk.sequence)

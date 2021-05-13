import math
import copy
from dataclasses import dataclass
from functools import lru_cache
from typing import List, Dict

from moves import Move


@dataclass
class Point:
    x: float
    y: float
    z: float

@dataclass
class Line:
    p1: Point
    p2: Point

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


def get_angle_by_coords(x1, y1):
    l = math.sqrt(x1 ** 2 + y1 ** 2)
    initial_angle = math.asin(abs(y1) / l)
    if x1 >= 0 and y1 >= 0:
        return initial_angle
    if x1 >= 0 and y1 < 0:
        return 2*math.pi - initial_angle
    if x1 < 0 and y1 >= 0:
        return math.pi - initial_angle
    if x1 < 0 and y1 < 0:
        return initial_angle + math.pi

def turn_on_angle_new(start_x, start_y, x1, y1, angle):
    print(f'x1, y1 : {round(x1, 2)}, {round(y1, 2)}')
    l = math.sqrt((x1 - start_x) ** 2 + (y1 - start_y) ** 2)
    initial_angle = get_angle_by_coords((x1 - start_x), (y1 - start_y))
    result_angle = angle + initial_angle
    print(f'{math.degrees(initial_angle)} -> {math.degrees(result_angle)}')

    return round(start_x + math.cos(result_angle) * l, 2), \
           round(start_y + math.sin(result_angle) * l, 2)

def turn_on_angle(x1, y1, angle):
    print(f'x1, y1 : {round(x1, 2)}, {round(y1, 2)}')
    l = math.sqrt(x1 ** 2 + y1 ** 2)
    initial_angle = get_angle_by_coords(x1, y1)
    result_angle = angle + initial_angle
    print(f'{math.degrees(initial_angle)} -> {math.degrees(result_angle)}')

    return round(math.cos(result_angle) * l, 2), \
           round(math.sin(result_angle) * l, 2)

"""
class MovementPlan:
    # [
    #     {1: [1, 1, 0], 2: [5, 5, 0], 3:[-5, -5, 0], 4:[5, -5, 0]},
    #     {1: [3, 1, 0], 2: [2, 5, 0], 3:[-3, -5, 0], 4:[2, -5, 0]}
    # ]
    def __init__(self):
        self.plan = []
    
    def add_move(self, move: List[Dict]):
        new_move = copy.deepcopy(move)
        self.plan.append(new_move)

    def __iter__(self):
        return self.plan.__iter__()

    def __repr__(self):
        return '\n'.join([str(x) for x in self.plan])
"""
class Leg:
    d = 6.7
    """
    a = 8.7
    b = 6.9
    #c = 13.2
    c = 14.2 # star toe
    """
    
    # version 21 prints
    a = 16.7
    b = 6.9
    c = 20.2

    # version 21.1 prints
    a = 12.7
    b = 6.9
    # c = 17.2 # point toe
    c = 15.5 # crescent toe
    

    def __init__(self, O, D):
        self.O = O
        self.D = D
        self.update_angles()

    def __repr__(self):
        return f'Leg ({self.O} -> {self.D})\n'

    def update_angles(self):
        self.tetta, self.alpha, self.beta, self.gamma = self.calculate_angles()

    def calculate_angles(self):
        O = self.O
        D = self.D
        tetta = math.atan2(D.y - O.y, D.x - O.x)
        A = Point(O.x + self.d * math.cos(tetta),
                  O.y + self.d * math.sin(tetta),
                  O.z)

        l = math.sqrt((D.x - A.x) ** 2 + (D.y - A.y) ** 2)
        delta_z = D.z - O.z

        alpha, beta, gamma = get_leg_angles(l, delta_z)

        Bx = self.a * math.cos(alpha)
        By = self.a * math.sin(alpha)
        Cx = Bx + self.b * math.cos(alpha + beta)
        Cy = By + self.b * math.sin(alpha + beta)
        Dx = Cx + self.c * math.cos(alpha + beta + gamma)
        Dy = Cy + self.c * math.sin(alpha + beta + gamma)
        if abs(Dx - l) > 0.01 or abs(Dy - delta_z) > 0.01:
            print('WTF')

        B_xz = [self.a * math.cos(alpha),
                self.a * math.sin(alpha)]
        C_xz = [B_xz[0] + self.b * math.cos(alpha + beta),
                B_xz[1] + self.b * math.sin(alpha + beta)]
        D_xz = [C_xz[0] + self.c * math.cos(alpha + beta + gamma),
                C_xz[1] + self.c * math.sin(alpha + beta + gamma)]

        D_prev = D
        self.A = A
        self.B = Point(A.x + B_xz[0] * math.cos(tetta),
                       A.y + B_xz[0] * math.sin(tetta),
                       A.z + B_xz[1])
        self.C = Point(A.x + C_xz[0] * math.cos(tetta),
                       A.y + C_xz[0] * math.sin(tetta),
                       A.z + C_xz[1])
        self.D = Point(round(A.x + D_xz[0] * math.cos(tetta), 1),
                       round(A.y + D_xz[0] * math.sin(tetta), 1),
                       round(A.z + D_xz[1], 1))

        if abs(D_prev.x - self.D.x) > 0.01 or \
           abs(D_prev.y - self.D.y) > 0.01 or \
           abs(D_prev.z - self.D.z) > 0.01:
            raise Exception('D_prev far from D. Angles : {0}'
                            .format(([tetta, alpha, beta, gamma])))

        return tetta, alpha, beta, gamma

    @staticmethod
    def move_point(point, delta_x, delta_y, delta_z):
        point.x += delta_x
        point.y += delta_y
        point.z += delta_z

    def move_mount_point(self, delta_x, delta_y, delta_z):
        self.move_point(self.O, delta_x, delta_y, delta_z)
        self.update_angles()

    def move_end_point(self, delta_x, delta_y, delta_z):
        self.move_point(self.D, delta_x, delta_y, delta_z)
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


@lru_cache(maxsize=None)
def get_leg_angles(delta_x, delta_z):
    possible_angles = find_angles(delta_x, delta_z)

    return get_best_angles(possible_angles)


def find_angles(Dx, Dy):
    a, b, c = Leg.a, Leg.b, Leg.c
    results = []
    full_dist = math.sqrt(Dx ** 2 + Dy ** 2)
    if full_dist > a + b + c:
        raise Exception('No decisions. Full distance : {0}'.format(full_dist))

    # from_angle = -45.0
    # to_angle = 45.0
    # angle_step = 1.5
    # for k in np.arange(from_angle, to_angle, angle_step):
    for k in range(-45, 46, 1):
        ksi = math.radians(k)

        Cx = Dx + c * math.cos(math.pi / 2 + ksi)
        Cy = Dy + c * math.sin(math.pi / 2 + ksi)
        dist = math.sqrt(Cx ** 2 + Cy ** 2)

        if dist > a + b or dist < abs(a - b):
            pass
        else:
            alpha1 = math.acos((a ** 2 + dist ** 2 - b ** 2) / (2 * a * dist))
            beta1 = math.acos((a ** 2 + b ** 2 - dist ** 2) / (2 * a * b))
            beta = -1 * (math.pi - beta1)

            alpha2 = math.atan2(Cy, Cx)
            alpha = alpha1 + alpha2

            Bx = a * math.cos(alpha)
            By = a * math.sin(alpha)

            BD = math.sqrt((Dx - Bx) ** 2 + (Dy - By) ** 2)
            angle_C = math.acos((b ** 2 + c ** 2 - BD ** 2) / (2 * b * c))

            for coef in [-1, 1]:
                gamma = coef * (math.pi - angle_C)

                Cx = Bx + b * math.cos(alpha + beta)
                Cy = By + b * math.sin(alpha + beta)
                new_Dx = Cx + c * math.cos(alpha + beta + gamma)
                new_Dy = Cy + c * math.sin(alpha + beta + gamma)
                if abs(new_Dx - Dx) > 0.01 or abs(new_Dy - Dy) > 0.01:
                    continue

                results.append([alpha, beta, gamma])

    return results


def check_angles(angles):
    # wtf check this on Fenix
    alpha = math.degrees(angles[0])
    beta = math.degrees(angles[1])
    gamma = math.degrees(angles[2])

    if alpha < -35 or alpha > 55:
        return False
    if beta < -115 or beta > -20:
        return False
    if gamma < -110 or gamma > 0:
        return False

    mode = 90 # 40
    #mode = 40
    if alpha + beta + gamma < -90 - mode or alpha + beta + gamma > -90 + mode:
        return False

    return True


def get_best_angles(all_angles):
    min_distance = 100000000
    best_angles = None
    min_distance_num = 0

    for item in all_angles:
        if not check_angles(item):
            continue
        cur_distance = get_angles_distance(item)

        if cur_distance <= min_distance:
            min_distance = cur_distance
            best_angles = item[:]

    if min_distance > 0.1:
        min_distance_num += 1
        if min_distance_num > 1:
            # print('best_angles : {0}'.format([math.degrees(x) for x in best_angles]))
            raise Exception('Min distance found : {0}'.format(min_distance))

    if best_angles is None:        
        raise Exception('No angles\n')

    return best_angles


def get_angles_distance(angles):
    # no diff, just distance with perpendicular
    # 100 -> endleg leaning inside
    return (math.degrees(angles[0] + angles[1] + angles[2]) + 90) ** 2


class MovementSequence:
    def __init__(self, legs_offset_h, legs_offset_v):
        self.legs_offset_v = legs_offset_v
        self.legs_offset_h = legs_offset_h

        self.current_legs_offset_v = self.legs_offset_v
        self.current_legs_offset_h = self.legs_offset_h

        self.legs_deltas = {1 : [0, 0, 0], 2 : [0, 0, 0], 3 : [0, 0, 0], 4 : [0, 0, 0]}

        self.current_angle = 0
        self.margin = 3 # 4
        self.leg_up = 6 # two-legged-moves # 6
        self.leg_up_single = 3 # one-legged-move
        self.mode = 90  # needed?
        self.mount_point_offset = 3.8

        self.legs = self.initiate_legs()

        self.angles_history = []
        self.D_points_history = []
        #self.movement_plan = MovementPlan()

        self.add_angles_snapshot()
        

    @property
    def coords(self):
        avg_x, avg_y = 0, 0 
        for _, leg in self.legs.items():
            avg_x += leg.D.x
            avg_y += leg.D.y
        
        return [round(avg_x/4, 2), round(avg_y/4, 2)]

    def get_sequence_length(self):
        sum_diff = 0
        for i in range(len(self.sequence) - 1):
            max_diff = 0
            for a, b in zip(self.sequence[i], self.sequence[i+1]):
                max_diff = max(max_diff, abs(a - b))
            sum_diff += max_diff
        
        return round(sum_diff)

    def reset_history(self):
        self.angles_history = []

    def add_angles_snapshot(self):
        # angles are : gamma1, beta1, alpha1, tetta1, gamma2, beta2, alpha2, tetta2 ...
        # for leg1 tetta = 45 means 0 for servo
        # leg2 tetta = -45, leg3 tetta = -135, leg4 tetta = 135

        position = []        
        for leg_number, leg in self.legs.items():
            position.append(round(math.degrees(leg.gamma), 2))
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
        self.angles_history.append(self.convert_angles(position))

        """
        leg_1_D_copy = copy.deepcopy(self.legs[1].D)
        leg_1_D_copy.z -= self.legs_offset_v

        leg_2_D_copy = copy.deepcopy(self.legs[2].D)
        leg_2_D_copy.z -= self.legs_offset_v

        leg_3_D_copy = copy.deepcopy(self.legs[3].D)
        leg_3_D_copy.z -= self.legs_offset_v

        leg_4_D_copy = copy.deepcopy(self.legs[4].D)
        leg_4_D_copy.z -= self.legs_offset_v
        """
        self.D_points_history.append([self.legs[1].D, self.legs[2].D, self.legs[3].D, self.legs[4].D])

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
                        #print(f'Tetta converted : -360')
                    if tetta < -70:
                        tetta += 360
                        #print(f'Tetta converted : +360')
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
        O1 = Point(self.mount_point_offset,
                   self.mount_point_offset,
                   self.legs_offset_v)
        D1 = Point(self.legs_offset_h,
                   self.legs_offset_h,
                   0)
        Leg1 = Leg(O1, D1)

        O2 = Point(self.mount_point_offset,
                   -self.mount_point_offset,
                   self.legs_offset_v)
        D2 = Point(self.legs_offset_h,
                   -self.legs_offset_h,
                   0)
        Leg2 = Leg(O2, D2)

        O3 = Point(-self.mount_point_offset,
                   -self.mount_point_offset,
                   self.legs_offset_v)
        D3 = Point(-self.legs_offset_h,
                   -self.legs_offset_h,
                   0)
        Leg3 = Leg(O3, D3)

        O4 = Point(-self.mount_point_offset,
                   self.mount_point_offset,
                   self.legs_offset_v)
        D4 = Point(-self.legs_offset_h,
                   self.legs_offset_h,
                   0)
        Leg4 = Leg(O4, D4)

        return {1: Leg1, 2: Leg2, 3: Leg3, 4: Leg4}

    ################## MOVEMENTS START HERE ##################
    def leg_movement(self, leg_num, leg_delta):
        leg = self.legs[leg_num]

        leg.move_end_point(leg_delta[0], leg_delta[1], leg_delta[2])
        self.add_angles_snapshot()

    def body_movement(self, delta_x, delta_y, delta_z, snapshot=True):
        #print(f'Body movement [{delta_x}, {delta_y}, {delta_z}]')
        if delta_x == delta_y == delta_z == 0:
            return

        for leg in self.legs.values():
            leg.move_mount_point(delta_x, delta_y, delta_z)

        if snapshot:
            self.add_angles_snapshot()

        self.current_legs_offset_v -= delta_z

    def body_to_center(self, delta_y=0, delta_x=0):
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
        self.body_compensation_for_a_leg(leg_num)

        self.legs[leg_num].move_end_point(*leg_delta)
        self.add_angles_snapshot()

    def leg_move_with_compensation(self, leg_num, delta_x, delta_y, obstacle_z=0, move_type:int = 1):
        if move_type == 1:
            if obstacle_z >= 0:
                self.compensated_leg_movement(leg_num, [delta_x, delta_y, self.leg_up_single + obstacle_z])
            else:
                self.compensated_leg_movement(leg_num, [delta_x, delta_y, self.leg_up_single])
        else:
            if obstacle_z >= 0:
                self.compensated_leg_movement(leg_num, [0, 0, self.leg_up_single + obstacle_z])
                self.move_leg_endpoint(leg_num, [delta_x, delta_y, 0])
            else:
                self.compensated_leg_movement(leg_num, [delta_x, delta_y, self.leg_up_single])

        if obstacle_z >= 0:
            self.move_leg_endpoint(leg_num, [0, 0, -self.leg_up_single])
        else:
            self.move_leg_endpoint(leg_num, [0, 0, -self.leg_up_single + obstacle_z])
        #self.compensated_leg_movement(leg_num, [0, 0, -self.leg_up])

    def move_leg_endpoint(self, leg_num, leg_delta):        
        self.legs[leg_num].move_end_point(*leg_delta)
        self.legs_deltas[leg_num] = [x + y for x, y in zip(self.legs_deltas[leg_num], leg_delta)]        
        self.add_angles_snapshot()

    def print_legs_diff(self):
        print(self.legs_deltas)
        #for leg_num, leg in self.legs.items():
        #    print(f'Delta {leg_num} : [{round(leg.D.x - leg.O.x, 2)}, {round(leg.D.y - leg.O.y, 2)}, {round(leg.D.z - leg.O.z, 2)}]')
        
    # 1-legged movements
    def move_body_straight(self, delta_x, delta_y, leg_seq=[1, 2, 3, 4]):
        for leg_number in leg_seq:
            self.leg_move_with_compensation(leg_number, delta_x, delta_y)
        self.body_to_center()

    def move_body_up(self, delta_z):
        self.body_movement(0, 0, delta_z)

    def move_body_down(self, delta_z):
        self.body_movement(0, 0, -delta_z)


    def move_according_to_plan(self, plan: List[Move]):
        self.successful_moves = 0
        for move in plan:
            try:
                if move.command == 'forward':
                    self.move_1_legged_for_diff(move)
                elif move.command == 'up':
                    self.move_body_up(move.value)
                elif move.command == 'down':
                    self.move_body_down(move.value)
                self.successful_moves += 1
            except Exception as e:
                print(f'Successful moves : {self.successful_moves}')
                raise Exception(e)

    def move_1_legged_for_diff(self, move: Move):
        for leg_number, deltas in move.target_legs_position.items():
            D = self.legs[leg_number].D
            diff = [deltas[0] - D.x, deltas[1] - D.y, deltas[2] - D.z]
            self.leg_move_with_compensation(leg_number, *diff, move_type=2)
        self.body_to_center()
        
    """
    def move_1_legged_with_a_plan(self, plan:MovementPlan, move_type:int = 1, leg_seq:List[int] = [1, 2, 3, 4]):
        for move in plan:
            #print(f'Trying : {move}')
            #print(f'Current legs : {self.legs}')
            for leg_number, deltas in move.items():
                D = self.legs[leg_number].D
                diff = [deltas[0] - D.x, deltas[1] - D.y, deltas[2] - D.z]
                #print(f'Leg {leg_number} diff = {diff}')
                self.leg_move_with_compensation(leg_number, *diff, move_type)
            self.body_to_center()
    """
    # 2-legged movements
    def move_2_legs(self, delta_y, steps=0):
        leg_delta_1 = [0, delta_y, self.leg_up]
        leg_delta_2 = [0, 0, -self.leg_up]
        leg_delta_3 = [0, 2*delta_y, self.leg_up]
        offset_y = 0

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(*leg_delta_1)
        self.add_angles_snapshot()
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
                self.add_angles_snapshot()
                #self.body_movement(0, -offset_y, 0)

                for leg in [self.legs[1], self.legs[3]]:
                    leg.move_end_point(*leg_delta_2)
                
                #self.add_angles_snapshot()

                self.body_movement(0, delta_y + offset_y, 0)

                #self.add_angles_snapshot()
                #self.body_movement(0, delta_y, 0)

                for leg in [self.legs[2], self.legs[4]]:
                    leg.move_end_point(*leg_delta_3)
                self.add_angles_snapshot()
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
        self.add_angles_snapshot()

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(*leg_delta_2)
        self.add_angles_snapshot()
    
    def climb_2_legs(self, delta_z, steps_arr=[8, 12, 12, 12, 8, 12, 8]): #steps_arr=[8, 16, 16, 6, 6, 8]
        positive_delta_z = 0 # for climbing up
        negative_delta_z = 0 # for climbing down
        if delta_z > 0:
            positive_delta_z = delta_z
        else:
            negative_delta_z = delta_z

        tst_leg_up = round(self.leg_up/2)

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
                current_delta_z_down[4] -= negative_delta_z
            if step == 1:
                # print('Leg with delta z id 1')
                current_delta_z_up[1] += positive_delta_z
                current_delta_z_down[1] -= negative_delta_z

            if step%2 == 0:
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
                self.legs[leg_number].move_end_point(0, 0, current_delta_z_up[leg_number])
            self.add_angles_snapshot()

            # forward
            for leg_number in legs_to_move:
                self.legs[leg_number].move_end_point(0, value, 0)
            self.add_angles_snapshot()

            # down
            for leg_number in legs_to_move:
                self.legs[leg_number].move_end_point(0, 0, current_delta_z_down[leg_number])
            self.add_angles_snapshot()

            self.body_movement(0, body_movement_value, 0) # it adds snapshot itself
        
        if sum_even != sum_odd or sum_even != sum_body_movement:
            raise Exception(f'Bad step lengths: odd ({sum_odd}) and even ({sum_even}) and body({sum_body_movement}) not equal')
    
    # 2-legged movements
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

    def reposition_legs(self, delta_x, delta_y):
        self.legs[2].move_end_point(delta_x, -delta_y, self.leg_up)
        self.legs[4].move_end_point(-delta_x, delta_y, self.leg_up)
        self.add_angles_snapshot()

        self.legs[2].move_end_point(0, 0, -self.leg_up)
        self.legs[4].move_end_point(0, 0, -self.leg_up)
        self.add_angles_snapshot()

        self.legs[1].move_end_point(delta_x, delta_y, self.leg_up)
        self.legs[3].move_end_point(-delta_x, -delta_y, self.leg_up)
        self.add_angles_snapshot()

        self.legs[1].move_end_point(0, 0, -self.leg_up)
        self.legs[3].move_end_point(0, 0, -self.leg_up)
        self.add_angles_snapshot()

        self.current_legs_offset_h += delta_x
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
            x_new, y_new = turn_on_angle_new(center_x, center_y, leg.D.x, leg.D.y, angle)
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
            x_new, y_new = turn_on_angle_new(center_x, center_y, leg.D.x, leg.D.y, angle)
            delta_x = x_new - leg.D.x
            delta_y = y_new - leg.D.y

            leg.move_end_point(delta_x, delta_y, self.leg_up)

        if not only_body:
            self.add_angles_snapshot()

        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(0, 0, -self.leg_up)

        self.add_angles_snapshot()

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
        current_angle = self.current_angle

        x_current = self.mount_point_offset * \
                    math.cos(math.radians(current_angle))
        z_current = self.mount_point_offset * \
                    math.sin(math.radians(current_angle))

        x_target = self.mount_point_offset * math.cos(math.radians(angle))
        z_target = self.mount_point_offset * math.sin(math.radians(angle))

        x = x_current - x_target
        z = z_current - z_target

        for leg in [self.legs[1], self.legs[4]]:
            leg.move_mount_point(0, -x, z)
        for leg in [self.legs[2], self.legs[3]]:
            leg.move_mount_point(0, x, -z)

        self.add_angles_snapshot()

        self.current_angle = angle

    # fun moves
    def hit(self, leg_num):
        self.body_compensation_for_a_leg(leg_num)
        self.legs[leg_num].move_end_point(-8, 0, 10)
        self.add_angles_snapshot()
        self.legs[leg_num].move_end_point(0, 18, 0)
        self.add_angles_snapshot()
        self.legs[leg_num].move_end_point(0, -18, 0)
        self.add_angles_snapshot()
        self.legs[leg_num].move_end_point(8, 0, -10)
        self.add_angles_snapshot()
        self.body_to_center()

    # dance moves
    def opposite_legs_up(self, leg_up, leg_forward):
        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(0, 0, leg_up)
        self.add_angles_snapshot()

        self.legs[1].move_end_point(leg_forward, leg_forward, 5)
        self.legs[3].move_end_point(-leg_forward, -leg_forward, 5)

        self.add_angles_snapshot()

        self.legs[1].move_end_point(-leg_forward, -leg_forward, -5)
        self.legs[3].move_end_point(leg_forward, leg_forward, -5)

        self.add_angles_snapshot()
        
        for leg in [self.legs[1], self.legs[3]]:
            leg.move_end_point(0, 0, -leg_up)
        self.add_angles_snapshot()

        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(0, 0, leg_up)
        self.add_angles_snapshot()
        
        self.legs[2].move_end_point(leg_forward, -leg_forward, 5)
        self.legs[4].move_end_point(-leg_forward, leg_forward, 5)        

        self.add_angles_snapshot()

        self.legs[2].move_end_point(-leg_forward, leg_forward, -5)
        self.legs[4].move_end_point(leg_forward, -leg_forward, -5)        

        self.add_angles_snapshot()
        
        for leg in [self.legs[2], self.legs[4]]:
            leg.move_end_point(0, 0, -leg_up)
        self.add_angles_snapshot()



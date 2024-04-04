import math
from functools import lru_cache, cache
from typing import List

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from cybernetic_core.geometry.lines import Point
from cybernetic_core.cybernetic_utils.constraints import leg_angles_correct, tettas_correct

import configs.config as cfg
import configs.code_config as code_config
import logging.config


class FenixPosition:
    def __init__(self, 
            beta1, alpha1, tetta1,
            beta2, alpha2, tetta2,
            beta3, alpha3, tetta3,
            beta4, alpha4, tetta4,
            logger,
        ):
        self.beta1 = beta1
        self.alpha1 = alpha1
        self.tetta1 = tetta1
        self.beta2 = beta2
        self.alpha2 = alpha2
        self.tetta2 = tetta2
        self.beta3 = beta3
        self.alpha3 = alpha3
        self.tetta3 = tetta3
        self.beta4 = beta4
        self.alpha4 = alpha4
        self.tetta4 = tetta4
        self.logger = logger
        self.angles = [
            self.beta1, self.alpha1, self.tetta1,
            self.beta2, self.alpha2, self.tetta2,
            self.beta3, self.alpha3, self.tetta3,
            self.beta4, self.alpha4, self.tetta4
        ]

    def __hash__(self):
        i = 1
        result = 0
        for item in self.angles:
            result += round(item*10)*i
            i *= 100
        return result

    def check_angles(self):
        if not tettas_correct([self.tetta1, self.tetta2, self.tetta3, self.tetta4], self.logger):
            raise ValueError(f'Bad tettas : {self.tetta1, self.tetta2, self.tetta3, self.tetta4}')

        for index, leg in enumerate([
            [self.alpha1, self.beta1, self.tetta1],
            [self.alpha2, self.beta2, self.tetta2],
            [self.alpha3, self.beta3, self.tetta3],
            [self.alpha4, self.beta4, self.tetta4]]
            ):
            if not leg_angles_correct(leg[0], leg[1], leg[2], logger=self.logger):
                raise ValueError(f'Leg {index+1}. Bad angles:alpha {leg[0]}, beta {leg[1]}, tetta {leg[2]}')


@cache
def convert_to_servo_angles(fp: FenixPosition) -> FenixPosition:
    #print(f'Converted from {fp.angles}')
    beta1 = convert_beta(fp.beta1)
    alpha1 = convert_alpha(fp.alpha1)
    tetta1 = convert_tetta(fp.tetta1, 1)
    beta2 = convert_beta(fp.beta2)
    alpha2 = convert_alpha(fp.alpha2)
    tetta2 = convert_tetta(fp.tetta2, 2)
    beta3 = convert_beta(fp.beta3)
    alpha3 = convert_alpha(fp.alpha3)
    tetta3 = convert_tetta(fp.tetta3, 3)
    beta4 = convert_beta(fp.beta4)
    alpha4 = convert_alpha(fp.alpha4)
    tetta4 = convert_tetta(fp.tetta4, 4)

    new_fp = FenixPosition(beta1, alpha1, tetta1,
            beta2, alpha2, tetta2,
            beta3, alpha3, tetta3,
            beta4, alpha4, tetta4,
            fp.logger,)
   
    #print(f'Converted to {new_fp.angles}')
    new_fp.check_angles()
    return new_fp

@lru_cache(maxsize=None)
def find_angles(Cx, Cy, logger):
    a, b = cfg.leg["a"], cfg.leg["b"]
    dist = math.sqrt(Cx ** 2 + Cy ** 2)
    if dist > a + b:
        raise Exception('No decisions. Full distance : {0}'.format(dist))

    alpha1 = math.acos((a ** 2 + dist ** 2 - b ** 2) / (2 * a * dist))
    beta1 = math.acos((a ** 2 + b ** 2 - dist ** 2) / (2 * a * b))
    beta = math.pi - beta1

    alpha2 = math.atan2(Cy, Cx)
    alpha = alpha1 + alpha2

    logger.info(f'Alpha1: {round(math.degrees(alpha1), 2)}\nAlpha2: {round(math.degrees(alpha2), 2)}\nBeta1: {round(math.degrees(beta1), 2)}')

    return alpha, beta

def calculate_leg_angles(O: Point, C: Point, logger):
    tetta = math.atan2(C.y - O.y, C.x - O.x)

    A = Point(O.x + cfg.leg["d"] * math.cos(tetta),
                O.y + cfg.leg["d"] * math.sin(tetta),
                O.z)
    logger.info(f'{math.degrees(tetta)}, {A}')

    l = round(math.sqrt((C.x - A.x) ** 2 + (C.y - A.y) ** 2), 2)
    delta_z = round(C.z - O.z, 2)
    logger.info(f'Trying l {l} and delta_z {delta_z}')
    alpha, beta = find_angles(l, delta_z, logger)
    logger.info(f'Success : {math.degrees(alpha)}, {math.degrees(beta)}')

    return tetta, alpha, beta

def convert_alpha(alpha: float) -> float:
    alpha_converted = round(math.degrees(alpha), 2)
    return alpha_converted

def convert_beta(beta: float) -> float:
    beta_converted = round(math.degrees(beta) - 90, 2)
    return beta_converted

def convert_tetta(tetta: float, leg_number: int) -> float:
    # virtual model to real servos
    tetta_degrees = math.degrees(tetta)
    
    if leg_number == 1:
        tetta_degrees -= 45
    elif leg_number == 2:
        tetta_degrees += 45
    elif leg_number == 3:
        tetta_degrees += 135
    elif leg_number == 4:
        tetta_degrees -= 135
    
    return round(tetta_degrees, 2)

"""
@cache
def convert_legs_angles(legs_angles: FenixPosition) -> List[float]:
    # input: 16 angles in RADIANS
    # output: 16 converted angles in DEGREES
    angles_converted = FenixPosition(legs_angles)
    angles_converted.convert_to_servo_angles()
    angles_converted.check_angles()

    return angles_converted.angles
"""
def calculate_C_point(O: Point, tetta: float, alpha: float, beta: float) -> Point:
    A = Point(O.x + cfg.leg["d"] * math.cos(tetta),
                O.y + cfg.leg["d"] * math.sin(tetta),
                O.z)
    
    B_xz = [cfg.leg["a"] * math.cos(alpha),
            cfg.leg["a"] * math.sin(alpha)]
    C_xz = [B_xz[0] + cfg.leg["b"] * math.cos(alpha - beta),
            B_xz[1] + cfg.leg["b"] * math.sin(alpha - beta)]

    C = Point(round(A.x + C_xz[0] * math.cos(tetta), 2),
                    round(A.y + C_xz[0] * math.sin(tetta), 2),
                    round(A.z + C_xz[1], 2))

    return C

# ----------------------
# moves for Fenix
def get_angle_by_coords(x1, y1):
    l = math.sqrt(x1 ** 2 + y1 ** 2)
    initial_angle = math.asin(abs(y1) / l)
    if x1 >= 0 and y1 >= 0:
        return initial_angle
    if x1 >= 0 and y1 < 0:
        return 2 * math.pi - initial_angle
    if x1 < 0 and y1 >= 0:
        return math.pi - initial_angle
    if x1 < 0 and y1 < 0:
        return initial_angle + math.pi

def turn_on_angle(start_x, start_y, x1, y1, angle):
    print(f'x1, y1 : {round(x1, 2)}, {round(y1, 2)}')
    l = math.sqrt((x1 - start_x) ** 2 + (y1 - start_y) ** 2)
    initial_angle = get_angle_by_coords((x1 - start_x), (y1 - start_y))
    result_angle = angle + initial_angle
    print(f'{math.degrees(initial_angle)} -> {math.degrees(result_angle)}')

    return round(start_x + math.cos(result_angle) * l, 2), \
           round(start_y + math.sin(result_angle) * l, 2)


if __name__ == '__main__':
    logging.config.dictConfig(code_config.logger_config)
    logger = logging.getLogger('main_logger')

    #print(calculate_leg_angles(Point(x=3.8, y=-3.8, z=0), Point(x=18.0, y=-21.0, z=-11.0), 'rear_leg', logger))
    # [2022-03-09 23:22:21,571][INFO] Trying l 3.46 and delta_z -14.22
    O = Point(x=3.8, y=-3.8, z=0)
    C = Point(x=7.24, y=-3.69, z=-14.22)
    leg_tag = 'rear_leg'
    calculate_leg_angles(O, C, logger)

    # D initial : Point(x=7.24, y=-3.69, z=-14.22)
    # D calcula : Point(x=14.15, y=-3.47, z=-14.22)

    # Moving to [44.75, 41.68, 30.47, 44.75, 41.68, -30.47, 44.75, 41.68, 30.47, 44.75, 41.68, -30.47]
    # Moving to [44.75, 41.68, 30.47, 44.75, 41.68, -30.47, 44.75, 41.68, 30.47, 44.75, 41.68, -30.47]. Move type: init

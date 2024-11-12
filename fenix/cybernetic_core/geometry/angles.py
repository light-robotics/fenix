import math
from functools import lru_cache
from typing import List

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from cybernetic_core.geometry.lines import Point
from cybernetic_core.cybernetic_utils.constraints import leg_angles_correct, tettas_correct

import configs.config as cfg
import configs.code_config as code_config
import logging.config


class AnglesException(Exception):
    pass

class GeometryException(Exception):
    pass

@lru_cache(maxsize=None)
def find_angles(Cx, Cy, logger):
    a, b = cfg.leg["a"], cfg.leg["b"]
    dist = math.sqrt(Cx ** 2 + Cy ** 2)
    if dist > a + b or dist < abs(b - a):
        raise GeometryException('No decisions. Full distance : {0}'.format(dist))

    
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
    #logger.info(f'Trying l {l} and delta_z {delta_z}')
    alpha, beta = find_angles(l, delta_z, logger)
    #logger.info(f'Success : {math.degrees(alpha)}, {math.degrees(beta)}')

    return tetta, alpha, beta

def convert_alpha(alpha: float) -> float:
    alpha_converted = round(math.degrees(alpha), 2)
    return alpha_converted

def convert_alpha_to_kinematic(alpha_deg: float) -> float:
    return round(math.radians(alpha_deg), 4)

def convert_beta(beta: float) -> float:
    beta_converted = round(math.degrees(beta) - 90, 2)
    return beta_converted

def convert_beta_to_kinematic(beta_deg: float) -> float:
    return round(math.radians(beta_deg + 90), 4)

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

def convert_tetta_to_kinematic(tetta_deg: float, leg_number: int) -> float:
    # real servos to virtual model
    if leg_number == 1:
        tetta_deg += 45        
    elif leg_number == 2:
        tetta_deg -= 45
    elif leg_number == 3:
        tetta_deg -= 135
    elif leg_number == 4:
        tetta_deg += 135
    
    tetta_radians = math.radians(tetta_deg)
    
    return round(tetta_radians, 4)

def convert_legs_angles(legs_angles: List[float], logger=None) -> List[float]:
    # input: 16 angles in RADIANS
    # output: 16 converted angles in DEGREES
    # now tetta, alpha, beta one leg after another
    #print(f'Before conversion: {legs_angles}')
    angles_converted = [
        convert_beta(legs_angles[2]),
        convert_alpha(legs_angles[1]),
        convert_tetta(legs_angles[0], 1),
        convert_beta(legs_angles[5]),
        convert_alpha(legs_angles[4]),
        convert_tetta(legs_angles[3], 2),
        convert_beta(legs_angles[8]),
        convert_alpha(legs_angles[7]),
        convert_tetta(legs_angles[6], 3),
        convert_beta(legs_angles[11]),
        convert_alpha(legs_angles[10]),
        convert_tetta(legs_angles[9], 4),
    ]

    #print(f'Converted: {angles_converted}')

    if not tettas_correct([
        angles_converted[2], 
        angles_converted[5], 
        angles_converted[8], 
        angles_converted[11]
        ], 
        logger=logger):
        raise AnglesException('Bad tettas')

    for i in range(4):
        alpha_converted = angles_converted[3*i+1]
        beta_converted = angles_converted[3*i+2]
        tetta_converted = angles_converted[3*i]
        if not leg_angles_correct(
                alpha=alpha_converted,
                beta=beta_converted,
                tetta=tetta_converted,
                logger=logger
            ):
            raise AnglesException(f'Leg {i+1}. Bad angles:alpha {alpha_converted}, beta {beta_converted}, tetta {tetta_converted}')

    return angles_converted

def convert_legs_angles_to_kinematic(legs_angles: List[float]) -> List[float]:
    # input: 16 angles in DEGREES
    # output: 16 converted angles in RADIANS
    # now tetta, alpha, beta one leg after another
    angles_converted = [
        convert_tetta_to_kinematic(legs_angles[2], 1),        
        convert_alpha_to_kinematic(legs_angles[1]),
        convert_beta_to_kinematic(legs_angles[0]),

        convert_tetta_to_kinematic(legs_angles[5], 2),        
        convert_alpha_to_kinematic(legs_angles[4]),
        convert_beta_to_kinematic(legs_angles[3]),

        convert_tetta_to_kinematic(legs_angles[8], 3),
        convert_alpha_to_kinematic(legs_angles[7]),
        convert_beta_to_kinematic(legs_angles[6]),

        convert_tetta_to_kinematic(legs_angles[11], 4),
        convert_alpha_to_kinematic(legs_angles[10]),
        convert_beta_to_kinematic(legs_angles[9]),
    ]

    return angles_converted

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

    angles = [0.7853981633974483, 0.06096254159288539, 1.9076410156604413, -0.7853981633974483, 0.06096254159288539, 1.9076410156604413, -2.356194490192345, 0.06096254159288539, 1.9076410156604413, 2.356194490192345, 0.06096254159288539, 1.9076410156604413]
    angles_converted = convert_legs_angles(angles)
    print(angles_converted)
    angles_converted_back = convert_legs_angles_to_kinematic(angles_converted)
    print(angles_converted_back)

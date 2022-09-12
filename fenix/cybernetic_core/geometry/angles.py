import math
from functools import lru_cache
from typing import List

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from cybernetic_core.geometry.lines import Point
from cybernetic_core.cybernetic_utils.constraints import leg_angles_correct

import configs.config as cfg
import configs.code_config as code_config
import logging.config


@lru_cache(maxsize=None)
def get_leg_angles(delta_x, delta_z, leg_tag, logger):
    possible_angles = find_angles(delta_x, delta_z, leg_tag, logger)

    return get_best_angles(possible_angles)

def find_angles(Dx, Dy, leg_tag, logger):
    a, b, c = cfg.leg["a"], cfg.leg["b"], cfg.leg["c"]
    results = []
    full_dist = math.sqrt(Dx ** 2 + Dy ** 2)
    if full_dist > a + b + c:
        raise Exception('No decisions. Full distance : {0}'.format(full_dist))

    for k in range(cfg.angles["to_surface"]["min"], 
                   cfg.angles["to_surface"]["max"],
                   cfg.angles["to_surface"]["step"]):

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
                    # only one of two coeffs is correct

                if leg_angles_correct(
                    leg_type=leg_tag, 
                    alpha=math.degrees(alpha), 
                    beta=math.degrees(beta), 
                    gamma=math.degrees(gamma),
                    logger=logger
                ):
                    results.append([alpha, beta, gamma])

    return results

def calculate_leg_angles(O: Point, D: Point, leg_tag: str, logger):
    #logger.info(f'[CLA] O: {O}, D: {D}, {leg_tag}')
    tetta = math.atan2(D.y - O.y, D.x - O.x)
    #print(tetta, math.degrees(tetta))
    if not leg_angles_correct(leg_type=leg_tag, tetta=tetta, logger=logger):
        logger.info(f'Bad tetta : {tetta}')
        raise Exception(f'Bad tetta : {tetta}')

    A = Point(O.x + cfg.leg["d"] * math.cos(tetta),
                O.y + cfg.leg["d"] * math.sin(tetta),
                O.z)    

    l = round(math.sqrt((D.x - A.x) ** 2 + (D.y - A.y) ** 2), 2)
    delta_z = round(D.z - O.z, 2)
    logger.info(f'Trying l {l} and delta_z {delta_z}')
    alpha, beta, gamma = get_leg_angles(l, delta_z, leg_tag, logger)
    
    logger.info(f'Success : {math.degrees(alpha)}, {math.degrees(beta)}, {math.degrees(gamma)}')

    D_calculated = calculate_D_point(O, tetta, alpha, beta, gamma)
    #logger.info(f'[CLA] D initial : {D}')
    #logger.info(f'[CLA] D calculated : {D_calculated}')

    if abs(D_calculated.x - D.x) > 0.01 or \
        abs(D_calculated.y - D.y) > 0.01 or \
        abs(D_calculated.z - D.z) > 0.01:
        raise Exception('D_prev far from D. Angles : {0}'
                        .format(([math.degrees(x) for x in [tetta, alpha, beta, gamma]])))

    return tetta, alpha, beta, gamma

def calculate_D_point(O: Point, tetta: float, alpha: float, beta: float, gamma: float) -> Point:
    A = Point(O.x + cfg.leg["d"] * math.cos(tetta),
                O.y + cfg.leg["d"] * math.sin(tetta),
                O.z)
    
    B_xz = [cfg.leg["a"] * math.cos(alpha),
            cfg.leg["a"] * math.sin(alpha)]
    C_xz = [B_xz[0] + cfg.leg["b"] * math.cos(alpha + beta),
            B_xz[1] + cfg.leg["b"] * math.sin(alpha + beta)]
    D_xz = [C_xz[0] + cfg.leg["c"] * math.cos(alpha + beta + gamma),
            C_xz[1] + cfg.leg["c"] * math.sin(alpha + beta + gamma)]

    D = Point(round(A.x + D_xz[0] * math.cos(tetta), 2),
                    round(A.y + D_xz[0] * math.sin(tetta), 2),
                    round(A.z + D_xz[1], 2))

    return D

def convert_gamma(gamma: float) -> float:
    gamma_converted = round(math.degrees(gamma) - cfg.leg["phi_angle"], 2)
    return gamma_converted

def convert_gamma_back(gamma_converted: float) -> float:
    gamma_initial = round(math.radians(gamma_converted + cfg.leg["phi_angle"]), 4)
    return gamma_initial

def convert_alpha(alpha: float) -> float:
    alpha_converted = round(math.degrees(alpha), 2)
    return alpha_converted

def convert_alpha_back(alpha_converted: float) -> float:
    alpha_initial = round(math.radians(alpha_converted), 4)
    return alpha_initial

def convert_beta(beta: float) -> float:
    beta_converted = -round(math.degrees(beta), 2)
    return beta_converted

def convert_beta_back(beta_converted: float) -> float:
    beta_initial = -round(math.radians(beta_converted), 4)
    return beta_initial

def convert_tetta(tetta: float, leg_number: int) -> float:
    # virtual model to real servos
    tetta_degrees = math.degrees(tetta)
    if leg_number == 1:
        tetta_degrees -= 45
    elif leg_number == 2:
        #tetta_degrees -= 135
        tetta_degrees += 45
    elif leg_number == 3:
        tetta_degrees += 135
        #tetta_degrees -= 45
    elif leg_number == 4:
        #tetta_degrees += 45
        tetta_degrees -= 135
    
    return round(tetta_degrees, 2)

def convert_tetta_back(tetta_degrees: float, leg_number: int) -> float:
    # real servos to virtual model    
    if leg_number == 1:
        tetta_degrees += 45
    elif leg_number == 2:
        #tetta_degrees += 135
        tetta_degrees -= 45
    elif leg_number == 3:
        tetta_degrees -= 135
        #tetta_degrees += 45
    elif leg_number == 4:
        #tetta_degrees -= 45
        tetta_degrees += 135
    
    return round(math.radians(tetta_degrees), 4)
    
def convert_legs_angles(legs_angles: List[float]) -> List[float]:
    # input: 16 angles in RADIANS
    # output: 16 converted angles in DEGREES
    # was gamma, beta, alpha, tetta one leg after another
    # now tetta, alpha, beta, gamma one leg after another
    angles_converted = [
        convert_tetta(legs_angles[0], 1),
        convert_alpha(legs_angles[1]),
        convert_beta(legs_angles[2]),
        convert_gamma(legs_angles[3]),
        convert_tetta(legs_angles[4], 2),
        convert_alpha(legs_angles[5]),
        convert_beta(legs_angles[6]),
        convert_gamma(legs_angles[7]),
        convert_tetta(legs_angles[8], 3),
        convert_alpha(legs_angles[9]),
        convert_beta(legs_angles[10]),
        convert_gamma(legs_angles[11]),
        convert_tetta(legs_angles[12], 4),
        convert_alpha(legs_angles[13]),
        convert_beta(legs_angles[14]),
        convert_gamma(legs_angles[15]),
    ]

    return angles_converted

def convert_legs_angles_back(legs_angles_converted: List[float]) -> List[float]:
    # input: 16 angles in RADIANS
    # output: 16 converted angles in DEGREES
    # was gamma, beta, alpha, tetta one leg after another
    # now tetta, alpha, beta, gamma one leg after another
    angles_initial = [
        convert_tetta_back(legs_angles_converted[0], 1),
        convert_alpha_back(legs_angles_converted[1]),
        convert_beta_back(legs_angles_converted[2]),
        convert_gamma_back(legs_angles_converted[3]),
        convert_tetta_back(legs_angles_converted[4], 2),
        convert_alpha_back(legs_angles_converted[5]),
        convert_beta_back(legs_angles_converted[6]),
        convert_gamma_back(legs_angles_converted[7]),
        convert_tetta_back(legs_angles_converted[8], 3),
        convert_alpha_back(legs_angles_converted[9]),
        convert_beta_back(legs_angles_converted[10]),
        convert_gamma_back(legs_angles_converted[11]),
        convert_tetta_back(legs_angles_converted[12], 4),
        convert_alpha_back(legs_angles_converted[13]),
        convert_beta_back(legs_angles_converted[14]),
        convert_gamma_back(legs_angles_converted[15]),
    ]

    return angles_initial

def get_best_angles(all_angles):
    min_distance = 100000
    best_angles = None
    min_distance_num = 0

    for item in all_angles:
        #if not check_angles(item):
        #    continue
        cur_distance = get_angles_distance(item)
        #print([math.degrees(x) for x in item], cur_distance, min_distance)
        if cur_distance <= min_distance:
            min_distance = cur_distance
            best_angles = item[:]

    if min_distance > 0.1:
        min_distance_num += 1
        if min_distance_num > 1:
            # print('best_angles : {0}'.format([math.degrees(x) for x in best_angles]))
            raise Exception('Min distance found : {0}'.format(min_distance))

    if best_angles is None:        
        raise Exception('No angles')

    return best_angles

def get_angles_distance(angles):
    # no diff, just distance with perpendicular
    # 100 -> endleg leaning inside
    #return (math.degrees(angles[0] + angles[1] + angles[2]) + cfg.angles["to_surface"]["ideal"]) ** 2
    return (math.degrees(angles[0] + angles[1] + angles[2]) + cfg.angles["to_surface"]["ideal"]) ** 2


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

"""
O : Point(x=3.8, y=3.8, z=11). D : Point(x=18, y=15, z=0)
[0.6678325892598048, 0.932520500332173, -1.2639702727274524, -1.221893261879673]
[0.6678325892598048, 0.932520500332173, -1.2639702727274524, -1.221893261879673]
O : Point(x=3.8, y=-3.8, z=11). D : Point(x=18, y=-21, z=0)
[-0.8806504408162148, 0.7856728603314223, -1.0753113837445074, -1.0368117081026034]
[-0.8806504408162148, 0.7856728603314223, -1.0753113837445074, -1.0368117081026034]
O : Point(x=-3.8, y=-3.8, z=11). D : Point(x=-18, y=-21, z=0)
[-2.2609422127735783, 0.7856728603314223, -1.0753113837445074, -1.0368117081026034]
[-2.2609422127735783, 0.7856728603314223, -1.0753113837445074, -1.0368117081026034]
O : Point(x=-3.8, y=3.8, z=11). D : Point(x=-18, y=15, z=0)
[2.4737600643299884, 0.932520500332173, -1.2639702727274524, -1.221893261879673]
[2.4737600643299884, 0.932520500332173, -1.2639702727274524, -1.221893261879673]
"""

if __name__ == '__main__':
    logging.config.dictConfig(code_config.logger_config)
    logger = logging.getLogger('main_logger')

    #print(calculate_leg_angles(Point(x=3.8, y=-3.8, z=0), Point(x=18.0, y=-21.0, z=-11.0), 'rear_leg', logger))
    # [2022-03-09 23:22:21,571][INFO] Trying l 3.46 and delta_z -14.22
    O = Point(x=3.8, y=-3.8, z=0)
    D = Point(x=7.24, y=-3.69, z=-14.22)
    leg_tag = 'rear_leg'
    calculate_leg_angles(O, D, leg_tag, logger)

    # D initial : Point(x=7.24, y=-3.69, z=-14.22)
    # D calcula : Point(x=14.15, y=-3.47, z=-14.22)

    """
    initial_angles = [-1.221893261879673, -1.2639702727274524, 0.932520500332173, 0.6678325892598048, 
                      -1.0368117081026034, -1.0753113837445074, 0.7856728603314223, -0.8806504408162148, 
                      -1.0368117081026034, -1.0753113837445074, 0.7856728603314223, -2.2609422127735783, 
                      -1.221893261879673, -1.2639702727274524, 0.932520500332173, 2.4737600643299884]
    # was gamma, beta, alpha, tetta one leg after another
    # now tetta, alpha, beta, gamma one leg after another
    new_initial_angles = [0.6678325892598048, 0.932520500332173, -1.2639702727274524, -1.221893261879673, 
                          -0.8806504408162148, 0.7856728603314223, -1.0753113837445074, -1.0368117081026034, 
                          -2.2609422127735783, 0.7856728603314223, -1.0753113837445074, -1.0368117081026034, 
                          2.4737600643299884, 0.932520500332173, -1.2639702727274524, -1.221893261879673]

    converted_new = convert_legs_angles(new_initial_angles)
    print(converted_new)
    converted_new_back = convert_legs_angles_back(converted_new)
    print(converted_new_back)

    for i in range(len(new_initial_angles)):
        if abs(new_initial_angles[i] - converted_new_back[i]) > 0.001:
            print(f'{i} : {abs(new_initial_angles[i] - converted_new_back[i])}')

    
    print([round(math.degrees(x), 2) for x in initial_angles])
    angles_converted = [-6.74, 53.43, 72.42, -47.510000000000005, -5.46, 45.02, 61.61, -36.9, 5.46, 45.02, 61.61, -36.9, 6.74, 53.43, 72.42, -47.510000000000005]
    print(convert_legs_angles(initial_angles))

    print(-1.221893261879673)
    print(convert_gamma(-1.221893261879673))
    print(convert_gamma_back(convert_gamma(-1.221893261879673)))
    
    O = Point(x=3.8, y=3.8, z=11)
    tetta, alpha, beta, gamma = calculate_leg_angles(O, Point(x=18, y=15, z=0), 'front_leg', logger)
    print(tetta, alpha, beta, gamma)
    D = calculate_D_point(O, tetta, alpha, beta, gamma)
    print(D)
    
    print(calculate_leg_angles(Point(x=3.8, y=-3.8, z=11), Point(x=18, y=-21, z=0), 'rear_leg', logger))
    print(calculate_leg_angles(Point(x=-3.8, y=-3.8, z=11), Point(x=-18, y=-21, z=0), 'rear_leg', logger))
    print(calculate_leg_angles(Point(x=-3.8, y=3.8, z=11), Point(x=-18, y=15, z=0), 'front_leg', logger))
    """
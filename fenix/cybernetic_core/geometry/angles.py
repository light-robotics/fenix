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

class FenixPositionLeg():
    def __init__(self, number, alpha, beta, gamma, tetta):
        self.number = number
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.tetta = tetta

class FenixPosition():
    def __init__(self, 
            leg1_tetta,
            leg1_alpha,
            leg1_beta,
            leg1_gamma,
            leg2_tetta,
            leg2_alpha,
            leg2_beta,
            leg2_gamma,
            leg3_tetta,
            leg3_alpha,
            leg3_beta,
            leg3_gamma,
            leg4_tetta,
            leg4_alpha,
            leg4_beta,
            leg4_gamma
            ):
        leg1 = FenixPositionLeg(1, leg1_alpha, leg1_beta, leg1_gamma, leg1_tetta)
        leg2 = FenixPositionLeg(2, leg2_alpha, leg2_beta, leg2_gamma, leg2_tetta)
        leg3 = FenixPositionLeg(3, leg3_alpha, leg3_beta, leg3_gamma, leg3_tetta)
        leg4 = FenixPositionLeg(4, leg4_alpha, leg4_beta, leg4_gamma, leg4_tetta)
        self.legs = {
            1: leg1, 2: leg2, 3: leg3, 4: leg4
        }

    def to_servo(self):
        return [

        ]

@lru_cache(maxsize=None)
def get_leg_angles(delta_x, delta_z, logger):
    possible_angles = find_angles(delta_x, delta_z, logger)
    best_angles = get_best_angles(possible_angles)
    logger.info(f'Best angles: {[math.degrees(x) for x in best_angles]}')
    return best_angles

def find_angles(Dx, Dy, logger):
    a, b, c = cfg.leg["a"], cfg.leg["b"], cfg.leg["c"]
    results = []
    full_dist = math.sqrt(Dx ** 2 + Dy ** 2)
    if full_dist > a + b + c:
        raise Exception('No decisions. Full distance : {0}'.format(full_dist))

    for k in range(-45, 45, 3):

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
                #print(f'B: {Bx, By}. C: {Cx, Cy}')
                #print(f'alpha: {round(math.degrees(alpha), 2)}, beta: {round(math.degrees(beta), 2)}, gamma: {round(math.degrees(gamma), 2)}')
                if leg_angles_correct(
                    alpha=math.degrees(alpha), 
                    beta=math.degrees(beta), 
                    gamma=math.degrees(gamma),
                    logger=logger
                ):
                    results.append([alpha, beta, gamma])

    return results

def calculate_leg_angles(O: Point, D: Point, logger):
    #print(f'[CLA] O: {O}, D: {D}')
    tetta = math.atan2(D.y - O.y, D.x - O.x)
    #print(tetta, math.degrees(tetta))
    
    A = Point(O.x + cfg.leg["d"] * math.cos(tetta),
                O.y + cfg.leg["d"] * math.sin(tetta),
                O.z)    
    #print(f'A: {A}')
    l = round(math.sqrt((D.x - A.x) ** 2 + (D.y - A.y) ** 2), 2)
    delta_z = round(D.z - O.z, 2)
    logger.info(f'Trying l {l} and delta_z {delta_z}')
    alpha, beta, gamma = get_leg_angles(l, delta_z, logger)

    if not leg_angles_correct(alpha=alpha, beta=beta, gamma=gamma, tetta=tetta, logger=logger):
        logger.info(f'Bad tetta : {tetta}')
        raise Exception(f'Bad tetta : {tetta}')
    
    logger.info(f'Success : {math.degrees(alpha)}, {math.degrees(beta)}, {math.degrees(gamma)}')

    D_calculated = calculate_D_point(O, tetta, alpha, beta, gamma, D)
    #print(f'[CLA] D initial : {D}')
    #print(f'[CLA] D calculated : {D_calculated}')

    
    if (abs(D_calculated.x - D.x) > 0.01) or \
        (abs(D_calculated.y - D.y) > 0.01) or \
        (abs(D_calculated.z - D.z) > 0.01):
        #print(abs(D_calculated.x - D.x), abs(D_calculated.y - D.y), abs(D_calculated.z - D.z))
        raise Exception('D_prev far from D. Angles : {0}'
                        .format(([math.degrees(x) for x in [tetta, alpha, beta, gamma]])))
    
    return tetta, alpha, beta, gamma

def calculate_D_point(
        O: Point, 
        tetta: float, 
        alpha: float, 
        beta: float, 
        gamma: float, 
        D: Point | None = None
    ) -> Point:
    A = Point(O.x + cfg.leg["d"] * math.cos(tetta),
                O.y + cfg.leg["d"] * math.sin(tetta),
                O.z)
    
    B_xz = [cfg.leg["a"] * math.cos(alpha),
            cfg.leg["a"] * math.sin(alpha)]
    C_xz = [B_xz[0] + cfg.leg["b"] * math.cos(alpha + beta),
            B_xz[1] + cfg.leg["b"] * math.sin(alpha + beta)]
    D_xz = [C_xz[0] + cfg.leg["c"] * math.cos(alpha + beta + gamma),
            C_xz[1] + cfg.leg["c"] * math.sin(alpha + beta + gamma)]

    D_calculated = Point(round(A.x + D_xz[0] * math.cos(tetta), 2),
                    round(A.y + D_xz[0] * math.sin(tetta), 2),
                    round(A.z + D_xz[1], 2))
    if D is None:
        return D_calculated
    
    if (abs(D_calculated.x - D.x) > 0.01) or \
        (abs(D_calculated.y - D.y) > 0.01) or \
        (abs(D_calculated.z - D.z) > 0.01):
        return Point(round(A.x - D_xz[0] * math.cos(tetta), 2),
                    round(A.y - D_xz[0] * math.sin(tetta), 2),
                    round(A.z + D_xz[1], 2))
    #print(f'[CDP] A: {A}')
    #print(f'[CDP] Bxz: {B_xz}')
    #print(f'[CDP] Cxz: {C_xz}')
    #print(f'[CDP] Dxz: {D_xz}')
    return D_calculated

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
    """
    print('Angles:', 
          math.degrees(angles[0]),
          math.degrees(angles[1]),
          math.degrees(angles[2]),
          math.degrees(angles[0] + angles[1] + angles[2])
          )
    """
    return (math.degrees(angles[0] + angles[1] + angles[2]) + 90) ** 2

def convert_alpha(alpha: float) -> float:
    alpha_converted = round(math.degrees(alpha), 2)
    return alpha_converted

def convert_alpha_to_kinematic(alpha_deg: float) -> float:
    return round(math.radians(alpha_deg), 4)

def convert_beta(beta: float) -> float:
    return -round(math.degrees(beta) + 90, 2)

def convert_beta_to_kinematic(beta_deg: float) -> float:
    return round(math.radians(-beta_deg - 90), 4)

def convert_gamma(gamma: float) -> float:
    return -round(math.degrees(gamma), 2)

def convert_gamma_to_kinematic(gamma_deg: float) -> float:
    return -round(math.radians(gamma_deg), 4)

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
        convert_gamma(legs_angles[3]),
        convert_beta(legs_angles[2]),
        convert_alpha(legs_angles[1]),
        convert_tetta(legs_angles[0], 1),

        convert_gamma(legs_angles[7]),
        convert_beta(legs_angles[6]),
        convert_alpha(legs_angles[5]),
        convert_tetta(legs_angles[4], 2),

        convert_gamma(legs_angles[11]),
        convert_beta(legs_angles[10]),
        convert_alpha(legs_angles[9]),
        convert_tetta(legs_angles[8], 3),

        convert_gamma(legs_angles[15]),
        convert_beta(legs_angles[14]),
        convert_alpha(legs_angles[13]),
        convert_tetta(legs_angles[12], 4),
    ]
    
    #print(f'Converted: {angles_converted}')

    if not tettas_correct([
        angles_converted[3], 
        angles_converted[7], 
        angles_converted[11], 
        angles_converted[15]
        ], 
        logger=logger):
        raise AnglesException('Bad tettas')

    for i in range(4):
        alpha_converted = angles_converted[4*i+2]
        beta_converted = angles_converted[4*i+1]
        gamma_converted = angles_converted[4*i]
        tetta_converted = angles_converted[4*i+3]
        if not leg_angles_correct(
                alpha=alpha_converted,
                beta=beta_converted,
                gamma=gamma_converted,
                tetta=tetta_converted,
                logger=logger
            ):
            raise AnglesException(f'Leg {i+1}. Bad angles:alpha {alpha_converted}, beta {beta_converted}, gamma {gamma_converted}, tetta {tetta_converted}')

    return angles_converted

def convert_legs_angles_to_kinematic(legs_angles: List[float]) -> List[float]:
    # input: 16 angles in DEGREES
    # output: 16 converted angles in RADIANS
    # now tetta, alpha, beta one leg after another
    #print(f'convert_legs_angles_to_kinematic. Before {legs_angles}')
    angles_converted = [
        convert_tetta_to_kinematic(legs_angles[3], 1),        
        convert_alpha_to_kinematic(legs_angles[2]),
        convert_beta_to_kinematic(legs_angles[1]),
        convert_gamma_to_kinematic(legs_angles[0]),

        convert_tetta_to_kinematic(legs_angles[7], 2),        
        convert_alpha_to_kinematic(legs_angles[6]),
        convert_beta_to_kinematic(legs_angles[5]),
        convert_gamma_to_kinematic(legs_angles[4]),

        convert_tetta_to_kinematic(legs_angles[11], 3),
        convert_alpha_to_kinematic(legs_angles[10]),
        convert_beta_to_kinematic(legs_angles[9]),
        convert_gamma_to_kinematic(legs_angles[8]),

        convert_tetta_to_kinematic(legs_angles[15], 4),
        convert_alpha_to_kinematic(legs_angles[14]),
        convert_beta_to_kinematic(legs_angles[13]),
        convert_gamma_to_kinematic(legs_angles[12]),
    ]
    #print(f'convert_legs_angles_to_kinematic. After {angles_converted}')
    return angles_converted


    
def convert_legs_angles_C(fp_in: FenixPosition, logger=None) -> FenixPosition:
    # input: 16 angles in RADIANS
    # output: 16 converted angles in DEGREES
    # now tetta, alpha, beta one leg after another
    #print(f'Before conversion: {legs_angles}')
    fp = FenixPosition(
        leg1_alpha=convert_alpha(fp_in.legs[1].alpha),
        leg1_beta=convert_beta(fp_in.legs[1].beta),
        leg1_gamma=convert_gamma(fp_in.legs[1].gamma),
        leg1_tetta=convert_tetta(fp_in.legs[1].tetta, 1),

        leg2_alpha=convert_alpha(fp_in.legs[2].alpha),
        leg2_beta=convert_beta(fp_in.legs[2].beta),
        leg2_gamma=convert_gamma(fp_in.legs[2].gamma),
        leg2_tetta=convert_tetta(fp_in.legs[2].tetta, 2),

        leg3_alpha=convert_alpha(fp_in.legs[3].alpha),
        leg3_beta=convert_beta(fp_in.legs[3].beta),
        leg3_gamma=convert_gamma(fp_in.legs[3].gamma),
        leg3_tetta=convert_tetta(fp_in.legs[3].tetta, 3),

        leg4_alpha=convert_alpha(fp_in.legs[4].alpha),
        leg4_beta=convert_beta(fp_in.legs[4].beta),
        leg4_gamma=convert_gamma(fp_in.legs[4].gamma),
        leg4_tetta=convert_tetta(fp_in.legs[4].tetta, 4),
    )    
    #print(f'Converted: {angles_converted}')

    if not tettas_correct([
        fp.legs[1].tetta, 
        fp.legs[2].tetta, 
        fp.legs[3].tetta, 
        fp.legs[4].tetta
        ], 
        logger=logger):
        raise AnglesException('Bad tettas')

    for i in range(4):
        alpha_converted = fp.legs[i+1].alpha
        beta_converted = fp.legs[i+1].beta
        gamma_converted = fp.legs[i+1].gamma
        tetta_converted = fp.legs[i+1].tetta
        if not leg_angles_correct(
                alpha=alpha_converted,
                beta=beta_converted,
                gamma=gamma_converted,
                tetta=tetta_converted,
                logger=logger
            ):
            raise AnglesException(f'Leg {i+1}. Bad angles:alpha {alpha_converted}, beta {beta_converted}, gamma {gamma_converted}, tetta {tetta_converted}')

    return fp

def convert_legs_angles_to_kinematic_C(fp_in: FenixPosition, logger=None) -> FenixPosition:
    # input: 16 angles in DEGREES
    # output: 16 converted angles in RADIANS
    # now tetta, alpha, beta one leg after another
    #print(f'convert_legs_angles_to_kinematic. Before {legs_angles}')
    fp = FenixPosition(
        leg1_alpha=convert_alpha_to_kinematic(fp_in.legs[1].alpha),
        leg1_beta=convert_beta_to_kinematic(fp_in.legs[1].beta),
        leg1_gamma=convert_gamma_to_kinematic(fp_in.legs[1].gamma),
        leg1_tetta=convert_tetta_to_kinematic(fp_in.legs[1].tetta, 1),

        leg2_alpha=convert_alpha_to_kinematic(fp_in.legs[2].alpha),
        leg2_beta=convert_beta_to_kinematic(fp_in.legs[2].beta),
        leg2_gamma=convert_gamma_to_kinematic(fp_in.legs[2].gamma),
        leg2_tetta=convert_tetta_to_kinematic(fp_in.legs[2].tetta, 2),

        leg3_alpha=convert_alpha_to_kinematic(fp_in.legs[3].alpha),
        leg3_beta=convert_beta_to_kinematic(fp_in.legs[3].beta),
        leg3_gamma=convert_gamma_to_kinematic(fp_in.legs[3].gamma),
        leg3_tetta=convert_tetta_to_kinematic(fp_in.legs[3].tetta, 3),

        leg4_alpha=convert_alpha_to_kinematic(fp_in.legs[4].alpha),
        leg4_beta=convert_beta_to_kinematic(fp_in.legs[4].beta),
        leg4_gamma=convert_gamma_to_kinematic(fp_in.legs[4].gamma),
        leg4_tetta=convert_tetta_to_kinematic(fp_in.legs[4].tetta, 4)
    )

    return fp

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

    fp = FenixPosition('physical', 44.51882068166497, 14.398429391637588, -82.79813097435527, -20.637939780612253, -26.762858610560755, 12.719663051904275, -128.64048416277242, 26.401895199628335, -135.24095796267952, 4.079459501331462, -131.7573745682841, 13.68223214772406, 114.59728860411596, 9.837685342396234, -134.63935227779214, 33.598245106471474)
    print(fp.legs[1].alpha)

    """
    angles = [0.7853981633974483, 0.06096254159288539, 1.9076410156604413, -0.7853981633974483, 0.06096254159288539, 1.9076410156604413, -2.356194490192345, 0.06096254159288539, 1.9076410156604413, 2.356194490192345, 0.06096254159288539, 1.9076410156604413]
    angles_converted = convert_legs_angles(angles)
    print(angles_converted)
    angles_converted_back = convert_legs_angles_to_kinematic(angles_converted)
    print(angles_converted_back)
    
    O = Point(x=3.8, y=-3.8, z=0)
    #D = Point(x=9.13, y=-7.01, z=-14)
    for z in range(-20, 10):
        for y in [-5, 0, 5]:
            for x in [9, 13, 17]:
                D = Point(x=x, y=y, z=z)                
                try:
                    a = calculate_leg_angles(O, D, logger)
                    print(f'D: {D}')
                    #print([round(math.degrees(x), 2) for x in a])
                except Exception as e:
                    if 'No angles' not in str(e):
                        print(e)
    #angles = get_leg_angles(0.3, -8.2, logger)
    #print([math.degrees(x) for x in angles])

    # 40.69674025591244 -136.88832870747177 -35.808411548440716 -132.00000000000006
    
    [-12.994616791916506, -13.243320770137336, -137.86707982845593, 31.110400598593166]
    [36.15818543980832, -16.428406089263376, -138.46132915745176, 31.88973524671527]
    [-12.994616791916506, -8.269444188721673, -135.48420122953993, 20.753645418261787]
    [36.15818543980832, -11.648881972170209, -136.48554311597042, 22.13442508814046]
    [-12.994616791916506, -3.7008469377274382, -132.58336145386508, 10.284208391592482]
    [36.15818543980832, -7.210819829356003, -134.0060979992245, 12.216917828580524]
    """
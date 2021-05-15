import math
from functools import lru_cache

import config as cfg


@lru_cache(maxsize=None)
def get_leg_angles(delta_x, delta_z):
    possible_angles = find_angles(delta_x, delta_z)

    return get_best_angles(possible_angles)


def find_angles(Dx, Dy):
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

                results.append([alpha, beta, gamma])

    return results

def check_angles(angles):
    alpha = math.degrees(angles[0])
    beta = math.degrees(angles[1])
    gamma = math.degrees(angles[2])

    if alpha < cfg.angles["alpha"]["min"] or \
        alpha > cfg.angles["alpha"]["max"]:
        return False
    if beta < cfg.angles["beta"]["min"] or \
        beta > cfg.angles["beta"]["max"]:
        return False
    if gamma < cfg.angles["gamma"]["min"] or \
        gamma > cfg.angles["gamma"]["max"]:
        return False

    mode = cfg.mode

    if alpha + beta + gamma < -90 - mode or \
       alpha + beta + gamma > -90 + mode:
        return False

    return True

def get_best_angles(all_angles):
    min_distance = 100000
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
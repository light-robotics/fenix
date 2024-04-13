#!/usr/bin/env python3.9
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import copy
from typing import List, Dict, Callable
import itertools
import logging

from cybernetic_core.kinematics import FenixKinematics as Fenix
from cybernetic_core.obstacles.obstacle import Obstacle, obstacles_from_csv
from cybernetic_core.geometry.lines import convert_points_to_3d_lines
from cybernetic_core.geometry.angles import AnglesException
from cybernetic_core.cybernetic_utils.moves import Move, Attempt


class CollisionException(Exception):
    pass


logging.basicConfig(filename='pathfinding.log', filemode='w', format='%(message)s')

obstacles = obstacles_from_csv()

def generate_movement_plan(fnx: Fenix, plan: List[Move], target_xy: List[int]) -> List[Move]:
    
    legs = {
        1: [fnx.legs[1].C.x,  fnx.legs[1].C.y, fnx.legs[1].C.z],
        2: [fnx.legs[2].C.x,  fnx.legs[2].C.y, fnx.legs[2].C.z],
        3: [fnx.legs[3].C.x,  fnx.legs[3].C.y, fnx.legs[3].C.z],
        4: [fnx.legs[4].C.x,  fnx.legs[4].C.y, fnx.legs[4].C.z]
    }

    movement_plan = []
    movement_plan.append(Move('init', target_legs_position=legs))

    for move in plan:
        if move.command == 'forward':
            step = move.value
        else:
            step = 0

        center_x, center_y = 0, 0
        for leg, values in legs.items():
            if leg in move.legs:
                values[0] += step
                center_x += values[0]
                center_y += values[1]

        center_x = round(center_x/4)
        center_y = round(center_y/4)

        movement_plan.append(Move(move.command, value=move.value, target_legs_position=legs))
        if target_xy[0] <= center_x and target_xy[1] <= center_y:
            return movement_plan

    return None

def adjust_movement_plan_to_obstacles(initial_movement_plan: List[Move], obstacles: List[Obstacle]) -> List[Move]:
    adjusted_plan = []
    for move in initial_movement_plan:
        new_move = copy.deepcopy(move)
        for leg, position in move.target_legs_position.items():
            if leg in move.legs:
                x, y, z = position
                for obs in obstacles:
                    z_delta = obs.touching_the_obstacle(x, y)
                    if z_delta > 0:
                        z = z_delta
            new_move.target_legs_position[leg] = [x, y, z]
        adjusted_plan.append(new_move)
    
    return adjusted_plan

def obstacle_collision(fenix: Fenix, obstacles: List[Obstacle]) -> bool:
    # fenix.D_points_history contains history of points travelled
    # obstacles is a list of obstacles to check
    
    history = convert_points_to_3d_lines(fenix.C_points_history)
    for line in history:
        for obs in obstacles:
            if obs.intersecting_the_obstacle(line):
                print(f'{line}. {obs}')
                return True

    return False

def check_movement_plan(plan: List[Move], target_xy: List[int]) -> int:
    #fnx = Fenix(legs_offset_v=20, legs_offset_h_x=20, legs_offset_h_y=16)
    #fnx = Fenix(legs_offset_v=15, legs_offset_h_x=13, legs_offset_h_y=20)
    fnx = Fenix()   
    try:
        print(f'Checking plan {plan}')
        plan = generate_movement_plan(fnx, plan, target_xy)
        if plan is None:
            logging.warning('None plan')
            return 0
        
        adjusted_movement_plan = adjust_movement_plan_to_obstacles(plan, obstacles)

        fnx.move_according_to_plan(adjusted_movement_plan)

        if obstacle_collision(fnx, obstacles) is True:
            logging.warning('Collision')
            raise CollisionException(f'Collision')

        seq_len = fnx.get_sequence_length()

        return seq_len #, fnx
    except CollisionException:
        print('Collision')
    except AnglesException as e:
        print(e)
    #except Exception as e:
    #    print(str(e))
    #    logging.warning(str(e))
    #    return -1

def get_sequence(plan: List[Move], target_xy: List[int]):
    #fnx = Fenix(legs_offset_v=20, legs_offset_h_x=20, legs_offset_h_y=16)
    fnx = Fenix()
    plan = generate_movement_plan(fnx, plan, target_xy)   
    adjusted_movement_plan = adjust_movement_plan_to_obstacles(plan, obstacles)
    fnx.move_according_to_plan(adjusted_movement_plan)
    return fnx.sequence

def get_lines(plan: List[Move], target_xy: List[int]):
    #fnx = Fenix(legs_offset_v=20, legs_offset_h_x=20, legs_offset_h_y=16)
    fnx = Fenix()
    plan = generate_movement_plan(fnx, plan, target_xy)   
    adjusted_movement_plan = adjust_movement_plan_to_obstacles(plan, obstacles)
    fnx.move_according_to_plan(adjusted_movement_plan)
    if obstacle_collision(fnx, obstacles) is True:
        raise Exception(f'Collision')
    return convert_points_to_3d_lines(fnx.D_points_history)
    
def check_possibilities(check_function: Callable, target: List[int], possible_moves: List[Move], steps: int) -> List[Attempt]:
    good_tries = []
    bad_tries = []
    for i in range(1, steps + 1):
        plan_lists = [possible_moves for _ in range(i)]
        for plan in itertools.product(*plan_lists):            
            plan = list(plan)
            cur_bad_tries = bad_tries[:]
            bad_plan = False
            for subplan in cur_bad_tries:
                if plan[:len(subplan)] == subplan:
                    bad_tries.append(plan)
                    bad_plan = True
                    continue
            logging.warning(f'\nPlan : {plan}. Bad : {bad_plan}')
            #print(f'Bad Plan : {bad_plan}. Plan : {plan}')
            if not bad_plan:
                #print(plan)
                result = check_function(plan, target)
                if result is not None and result >= 0:
                    good_tries.append(Attempt(plan, result))
                    #print(f'Added to good. Result : {result}')
                    if result > 0:
                        logging.warning(f'SUCCESS!!!! Result : {result}')
                    else:
                        logging.warning(f'Added to good. Result : {result}')
                else:
                    bad_tries.append(plan)
                    #print('Added to bad')
                    logging.warning('Added to bad')

    return good_tries

def get_best_sequence(target=[30, 0]):
    possibilities = check_possibilities(check_movement_plan, target, [Move('forward', 12), Move('forward', 8), Move('up', 5)], 5)
    for possibility in possibilities:
        if possibility.result > 0:
            print(possibility)

    from operator import attrgetter
    best_option = min([x for x in possibilities if x.result > 0], key=attrgetter('result'))
    print('----------')
    print(best_option)
    #return get_sequence(best_option.moves, target)

print('-----')
get_best_sequence()
# 40 : Result : 2493|Move[f.10]|Move[f.10]|Move[f.10]|Move[f.10]
# 50 : Result : 3115|Move[f.10]|Move[f.10]|Move[f.10]|Move[f.10]|Move[f.10]
# 60 : Result : 3811|Move[f.10]|Move[f.10]|Move[f.10]|Move[f.10]|Move[f.10]|Move[u.5]|Move[f.10]
# 70 : Plan : [Move[f.10], Move[f.8], Move[f.10], Move[f.10], Move[f.8], Move[f.8], Move[f.8], Move[u.5], Move[f.8]]. Bad : False
# SUCCESS!!!! Result : 4840
"""
fnx = Fenix()
plan = [Move('forward', 8), Move('forward', 8), Move('forward', 8), Move('forward', 8)]
generated_plan = generate_movement_plan(fnx, plan, [30, 0])
adjusted_plan = adjust_movement_plan_to_obstacles(generated_plan, obstacles)
check_movement_plan(adjusted_plan, [30, 0])
"""

#lines = get_lines(plan, [0, 75])
#for line in lines:
#    print(line)
#print(get_sequence(plan, [0, 75]))

"""
fnx = Fenix()
plan = [Move('forward', 10), Move('forward', 10), Move('forward', 10), Move('forward', 10), Move('forward', 8)]
#print(check_movement_plan(plan, [0, 60]))


generated_plan = generate_movement_plan(fnx, plan, [0, 60])
print(generated_plan)
adjusted_plan = adjust_movement_plan_to_obstacles(generated_plan, obstacles)
print('---------')
print(adjusted_plan)
check_movement_plan(adjusted_plan, [0, 60])
"""
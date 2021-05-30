#!/usr/bin/env python3.9

import copy
from typing import List, Dict, Callable
import itertools
import logging

from kinematics import FenixKinematics as Fenix
from obstacle import Obstacle, obstacle_from_csv
from lines import convert_points_to_3d_lines
from moves import Move, Attempt


logging.basicConfig(filename='pathfinding.log', filemode='w', format='%(message)s')

#obstacles = obstacle_from_csv('/fenix/movement/obstacles/simple_obstacle.csv')
#obstacles = obstacle_from_csv('/fenix/movement/obstacles/two_stairs.csv')
obstacles = obstacle_from_csv('/fenix/movement/obstacles/rugged_terrain.csv')
#obstacles = obstacle_from_csv('/fenix/movement/obstacles/rugged_terrain_v2.csv')

def generate_movement_plan(fnx: Fenix, plan: List[Move], target_xy: List[int]) -> List[Move]:
    
    legs = {
        1: [fnx.legs[1].D.x,  fnx.legs[1].D.y, fnx.legs[1].D.z],
        2: [fnx.legs[2].D.x,  fnx.legs[2].D.y, fnx.legs[2].D.z],
        3: [fnx.legs[3].D.x,  fnx.legs[3].D.y, fnx.legs[3].D.z],
        4: [fnx.legs[4].D.x,  fnx.legs[4].D.y, fnx.legs[4].D.z]
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
                values[1] += step
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
    
    history = convert_points_to_3d_lines(fenix.D_points_history)
    for line in history:
        for obs in obstacles:
            if obs.intersecting_the_obstacle(line):
                return True

    return False

def check_movement_plan(plan: List[Move], target_xy: List[int]) -> int:
    fnx = Fenix(legs_offset_v=20, legs_offset_h_x=20, legs_offset_h_y=16)
    #fnx = Fenix(legs_offset_v=15, legs_offset_h_x=13, legs_offset_h_y=20)
        
    try:
        #print(f'Checking plan {plan}')
        plan = generate_movement_plan(fnx, plan, target_xy)
        if plan is None:
            logging.warning('None plan')
            return 0
        
        adjusted_movement_plan = adjust_movement_plan_to_obstacles(plan, obstacles)

        fnx.move_according_to_plan(adjusted_movement_plan)

        if obstacle_collision(fnx, obstacles) is True:
            logging.warning('Collision')
            raise Exception(f'Collision')

        seq_len = fnx.get_sequence_length()

        return seq_len #, fnx
    except Exception as e:
        print(str(e))
        logging.warning(str(e))
        return -1

def get_sequence(plan: List[Move], target_xy: List[int]):
    fnx = Fenix(legs_offset_v=20, legs_offset_h_x=20, legs_offset_h_y=16)
    plan = generate_movement_plan(fnx, plan, target_xy)   
    adjusted_movement_plan = adjust_movement_plan_to_obstacles(plan, obstacles)
    fnx.move_according_to_plan(adjusted_movement_plan)
    return fnx.sequence

def get_lines(plan: List[Move], target_xy: List[int]):
    fnx = Fenix(legs_offset_v=20, legs_offset_h_x=20, legs_offset_h_y=16)
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
                if result >= 0:
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

def get_best_sequence():
    target = [0, 60]
    #possibilities = check_possibilities(check_movement_plan, target, [Move('forward', 9), Move('forward', 6), Move('up', 15)], 10)
    possibilities = check_possibilities(check_movement_plan, target, [Move('forward', 13), Move('forward', 10), Move('forward', 8), Move('up', 5)], 8)
    for possibility in possibilities:
        if possibility.result > 0:
            print(possibility)

    from operator import attrgetter
    best_option = min([x for x in possibilities if x.result > 0], key=attrgetter('result'))
    print('----------')
    print(best_option)
    #return get_sequence(best_option.moves, target)

print('-----')
#print(get_best_sequence())
# 40 : Result : 2493|Move[f.10]|Move[f.10]|Move[f.10]|Move[f.10]
# 50 : Result : 3115|Move[f.10]|Move[f.10]|Move[f.10]|Move[f.10]|Move[f.10]
# 60 : Result : 3811|Move[f.10]|Move[f.10]|Move[f.10]|Move[f.10]|Move[f.10]|Move[u.5]|Move[f.10]
# 70 : Plan : [Move[f.10], Move[f.8], Move[f.10], Move[f.10], Move[f.8], Move[f.8], Move[f.8], Move[u.5], Move[f.8]]. Bad : False
# SUCCESS!!!! Result : 4840


#plan = [Move('forward', 10), Move('forward', 10), Move('forward', 10), Move('forward', 10), Move('forward', 10), Move('up', 5), Move('forward', 10)]
plan = [Move('forward', 10), Move('forward', 8), Move('forward', 13), Move('forward', 10), Move('forward', 10), Move('up', 5), Move('forward', 13), Move('forward', 8), Move('forward', 8)]

#Move[f.10]|Move[f.8]|Move[f.13]|Move[f.10]|Move[f.10]|Move[u.5]|Move[f.13]|Move[f.8]|Move[f.8]

#fnx = Fenix(legs_offset_v=15, legs_offset_h_x=18, legs_offset_h_y=18)

#lines = get_lines(plan, [0, 75])
#for line in lines:
#    print(line)
print(get_sequence(plan, [0, 75]))

"""
fnx = Fenix(legs_offset_v=20, legs_offset_h_x=18, legs_offset_h_y=18)
plan = [Move('forward', 10), Move('forward', 10), Move('forward', 10), Move('forward', 10), Move('forward', 8)]
#print(check_movement_plan(plan, [0, 60]))


generated_plan = generate_movement_plan(fnx, plan, [0, 60])
print(generated_plan)
adjusted_plan = adjust_movement_plan_to_obstacles(generated_plan, obstacles)
print('---------')
print(adjusted_plan)
check_movement_plan(adjusted_plan, [0, 60])
"""
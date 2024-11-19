from typing import List, Tuple
import sys
import os
import math
from joblib import Memory
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from cybernetic_core.kinematics import FenixKinematics
from configs import config as cfg
from configs import code_config
from cybernetic_core.cybernetic_utils.moves import Sequence

#from functools import cache
memory = Memory(code_config.cache_dir, verbose=0)

UP_OR_DOWN_CM   = cfg.moves["up_or_down_cm"]
FORWARD_BODY_CM = cfg.moves["move_body_cm"]
FORWARD_LEGS_1LEG_CM = cfg.moves["forward_body_1_leg_cm"]
FORWARD_LEGS_2LEG_CM = cfg.moves["forward_body_2_leg_cm"]
REPOSITION_CM   = cfg.moves["reposition_cm"]
SIDE_LOOK_ANGLE = cfg.moves["side_look_angle"]
VERTICAL_LOOK_ANGLE = cfg.moves["vertical_look_angle"]


class Move:
    def __init__(self, move_type, values):
        self.move_type = move_type
        self.values = values
    
    def __repr__(self):
        return f'Move({self.move_type}, {self.values})'

#@cache
#@memory.cache
def get_sequence_for_command(command: str, kwargs=None) -> Sequence:
    sequence = []
    
    if command == 'up':
        sequence.append(Move('body_movement', {'deltas': [0, 0, UP_OR_DOWN_CM]}))
    elif command == 'down':
        sequence.append(Move('body_movement', {'deltas': [0, 0, -UP_OR_DOWN_CM]}))
    elif command == 'forward_one_legged_v2':
        for leg in [1, 2, 3, 4]:
            if leg == 1:
                sequence.append(Move('body_movement', {'deltas': [-7, 0, 0]}))
            elif leg == 3:
                sequence.append(Move('body_movement', {'deltas': [16, 0, 0]}))

            if leg in [1, 4]:
                y_diff = -1
            else:
                y_diff = 1
            if leg in [1, 2]:
                x_diff = -0
            else:
                x_diff = 0
            sequence.append(Move('balance', {}))
            sequence.append(Move('endpoint', {'leg': leg, 'deltas': [FORWARD_LEGS_1LEG_CM + x_diff, y_diff, 20]}))
            sequence.append(Move('touch', {'leg': leg}))
            #sequence.append(Move('body_to_center', {}))
            sequence.append(Move('balance', {}))
            sequence.append(Move('balance', {}))
        sequence.append(Move('body_to_center', {}))
    elif command == 'forward_one_legged':
        for leg in [1, 3, 2, 4]:
            
            if leg in [1, 4]:
                y_diff = -3
            else:
                y_diff = 3
            if leg in [1, 2]:
                x_diff = -2
            else:
                x_diff = 2
            sequence.append(Move('body_compensation_for_a_leg', {'leg': leg}))
            sequence.append(Move('endpoint', {'leg': leg, 'deltas': [FORWARD_LEGS_1LEG_CM + x_diff, y_diff, 12]}))
            sequence.append(Move('touch', {'leg': leg}))
            sequence.append(Move('body_to_center', {}))
            sequence.append(Move('balance', {}))
            sequence.append(Move('balance', {}))
        #sequence.append(Move('body_to_center', {}))
    elif command in ['battle_mode', 'sentry_mode', 'walking_mode', 'run_mode']:
        sequence.append(Move('switch_mode', {"mode": command}))
    else:
        print(f'Unknown command')
    
    #print(f'[SG]. Sequence commands: {sequence}')
    return sequence

def get_angles_for_sequence(move: Move, fenix_position: List[int]):
    fk = FenixKinematics(fenix_position=fenix_position)
    # print(f'fenix_position: {fenix_position}')

    if move.move_type == 'body_movement':
        fk.body_movement(*move.values['deltas'])
    elif move.move_type == 'body_to_center':
        fk.body_to_center()
    #elif move.move_type == 'compensated_leg_movement':
    #    fk.compensated_leg_movement(move.values['leg'], move.values['deltas'])
    elif move.move_type == 'body_compensation_for_a_leg':
        fk.body_compensation_for_a_leg(move.values['leg'])
    elif move.move_type == 'endpoint':
        fk.move_leg_endpoint(move.values['leg'], move.values['deltas'])
    elif move.move_type == 'touch':
        leg_num = move.values['leg']
        leg = fk.legs[leg_num]
        global leg1x, leg1y, leg2x, leg2y, leg3x, leg3y, leg4x, leg4y
        delta_x, delta_y = 0, 0
        if int(leg_num) == 1:
            if 'leg1x' not in globals():
                leg1x, leg1y = leg.C.x, leg.C.y
            delta_x, delta_y = round(leg1x - leg.C.x, 1), round(leg1y - leg.C.y, 1)
        elif leg_num == 2:
            if 'leg2x' not in globals():
                leg2x, leg2y = leg.C.x, leg.C.y
            delta_x, delta_y = round(leg2x - leg.C.x, 1), round(leg2y - leg.C.y, 1)
        elif leg_num == 3:
            if 'leg3x' not in globals():
                leg3x, leg3y = leg.C.x, leg.C.y
            delta_x, delta_y = round(leg3x - leg.C.x, 1), round(leg3y - leg.C.y, 1)
        elif leg_num == 4:
            if 'leg4x' not in globals():
                leg4x, leg4y = leg.C.x, leg.C.y
            delta_x, delta_y = round(leg4x - leg.C.x, 1), round(leg4y - leg.C.y, 1)
        
        print("Touch before. leg_num: ", 
              move.values['leg'],
              f'Cx = {leg.C.x}. Cy = {leg.C.y}',
              cfg.modes['walking_mode']['x']
              ,'\n'
              , 'delta:'
              , delta_x
              , delta_y)
        
        delta_x = delta_y = 0
        fk.leg_move_custom(move.values['leg'], 'touch', [-delta_x, -delta_y, -20])
    elif move.move_type == 'balance':
        with open('/fenix/fenix/wrk/gyroaccel_data.txt', "r") as f:
            pitch, roll = f.readline().split(',')
        pitch, roll = float(pitch), float(roll)
        if pitch < -cfg.fenix["balance_offset"] and roll > cfg.fenix["balance_offset"]:
            fk.leg_move_custom(1, 'balance', [0, 0, -10])
        elif pitch > cfg.fenix["balance_offset"] and roll > cfg.fenix["balance_offset"]:
            fk.leg_move_custom(2, 'balance', [0, 0, -10])
        elif pitch > cfg.fenix["balance_offset"] and roll < -cfg.fenix["balance_offset"]:
            fk.leg_move_custom(3, 'balance', [0, 0, -10])
        elif pitch < -cfg.fenix["balance_offset"] and roll < -cfg.fenix["balance_offset"]:
            fk.leg_move_custom(4, 'balance', [0, 0, -10])
    elif move.move_type == 'switch_mode':
        fk.switch_mode(move.values['mode'])

    #print(f'[SG]. Sequence: {fk.sequence[-1]}')
    return fk.sequence[-1]
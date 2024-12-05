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
from cybernetic_core.geometry.angles import FenixPosition

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
    elif command == 'balance':
        sequence.append(Move('balance', {}))
        sequence.append(Move('balance', {}))
        sequence.append(Move('balance', {}))
    elif command == 'forward_one_legged_v2':
        for leg in [1, 2, 3, 4]:
            if leg == 1:
                sequence.append(Move('body_movement', {'deltas': [-7, 0, 0]}))
            elif leg == 3:
                sequence.append(Move('body_movement', {'deltas': [14, 0, 0]}))

            if leg in [1, 4]:
                y_diff = 2
            else:
                y_diff = -2 
            if leg in [1, 2]:
                x_diff = -0
            else:
                x_diff = 0
            sequence.append(Move('balance', {}))
            sequence.append(Move('endpoint', {'leg': leg, 'deltas': [FORWARD_LEGS_1LEG_CM + x_diff, y_diff, 10]}))
            sequence.append(Move('touch', {'leg': leg}))
            #sequence.append(Move('body_to_center', {}))
            sequence.append(Move('balance', {}))
            sequence.append(Move('balance', {}))
        sequence.append(Move('body_to_center', {}))
    #elif command == 'forward_two_legged':
    elif command == 'forward_32':
    #elif command == 'forward_one_legged':        
        sequence.append(Move('endpoints', {'legs': [1, 3], 'deltas': [8, 0, 20]}))
        sequence.append(Move('down_2_legs', {'legs': [1, 3]}))
        sequence.append(Move('down_2_legs', {'legs': [1, 3]}))
        sequence.append(Move('down_2_legs', {'legs': [1, 3]}))
        sequence.append(Move('down_leg_up', {}))
        sequence.append(Move('down_leg_up', {}))
        sequence.append(Move('down_leg_up', {}))
        #sequence.append(Move('body_to_center', {}))
        sequence.append(Move('balance', {}))
        sequence.append(Move('balance', {}))
        sequence.append(Move('body_movement', {'deltas': [4, 0, 0]}))
        sequence.append(Move('endpoints', {'legs': [2, 4], 'deltas': [8, 0, 20]}))
        sequence.append(Move('down_2_legs', {'legs': [2, 4]}))
        sequence.append(Move('down_2_legs', {'legs': [2, 4]}))
        sequence.append(Move('down_2_legs', {'legs': [2, 4]}))
        sequence.append(Move('down_leg_up', {}))
        sequence.append(Move('down_leg_up', {}))
        sequence.append(Move('down_leg_up', {}))
        sequence.append(Move('balance', {}))
        sequence.append(Move('balance', {}))
        sequence.append(Move('body_movement', {'deltas': [4, 0, 0]}))
    elif command == 'forward_one_legged':
        for leg in [3, 4, 1, 2]:
            
            if leg == 1:
                y_diff = -2
            elif leg == 2:
                y_diff = 2
            else:
                y_diff = 0
            #if leg in [1, 4]:
            #    y_diff = -2 # -2
            #else:
            #    y_diff = 2 # 2
            if leg in [1, 2]:
                x_diff = 0 # -1
            else:
                x_diff = 0 # 1

            """
            sequence.append(Move('body_compensation_for_a_leg', {'leg': leg}))
            """
            side_step = 6
            if leg == 3:
                sequence.append(Move('body_movement', {'deltas': [0, side_step, 0]}))
            elif leg == 4:
                sequence.append(Move('body_movement', {'deltas': [0, -side_step, 0]}))
            elif leg == 1:
                sequence.append(Move('body_movement', {'deltas': [-0, -side_step, 0]}))
            elif leg == 2:
                sequence.append(Move('body_movement', {'deltas': [-0, side_step, 0]}))
            
            sequence.append(Move('endpoint_normalized', {'leg': leg, 'deltas': [0, 0, 30]}))
            sequence.append(Move('endpoint', {'leg': leg, 'deltas': [FORWARD_LEGS_1LEG_CM + x_diff, y_diff, 0]}))
            sequence.append(Move('touch', {'leg': leg}))
            #sequence.append(Move('touch', {'leg': leg}))
            #sequence.append(Move('touch', {'leg': leg}))
            sequence.append(Move('body_to_center', {}))
            sequence.append(Move('balance', {}))
            sequence.append(Move('balance', {}))
        
    elif command in ['battle_mode', 'sentry_mode', 'walking_mode', 'run_mode']:
        sequence.append(Move('switch_mode', {"mode": command}))
    else:
        print(f'Unknown command')
    
    #print(f'[SG]. Sequence commands: {sequence}')
    return sequence

def get_angles_for_sequence(move: Move, fenix_position: FenixPosition):
    fk = FenixKinematics(fenix_position=fenix_position)
    print(f'Move: {move.move_type}. {move.values}')
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
    elif move.move_type == 'endpoints':
        fk.move_leg_endpoint(move.values['legs'][0], move.values['deltas'], add_snapshot=False)
        fk.move_leg_endpoint(move.values['legs'][1], move.values['deltas'])
    elif move.move_type == 'endpoint_normalized':
        leg_num = move.values['leg']
        leg = fk.legs[leg_num]
          
        delta = move.values['deltas']
        fk.move_leg_endpoint(move.values['leg'], delta)
    elif move.move_type == 'touch':
        fk.leg_move_custom(move.values['leg'], 'touch', [0, 0, -24])
    elif move.move_type == 'down_2_legs':
        fk.leg_move_custom(move.values['legs'][0], 'touch_2legs', [0, 0, -8], add_snapshot=False)
        fk.leg_move_custom(move.values['legs'][1], 'touch_2legs', [0, 0, -8])
    elif move.move_type == 'down_leg_up':
        with open("/fenix/fenix/wrk/neopixel_command.txt", "r") as f:
            legs_down = f.readline().split(',')[0]
        print(f"down_leg_up. legs_down: {legs_down}")
        if legs_down == '1110':
            fk.leg_move_custom(4, 'touch', [0, 0, -8])
        elif legs_down == '1101':
            fk.leg_move_custom(3, 'touch', [0, 0, -8])
        elif legs_down == '1011':
            fk.leg_move_custom(2, 'touch', [0, 0, -8])
        elif legs_down == '0111':
            fk.leg_move_custom(1, 'touch', [0, 0, -8])
        elif legs_down == '1111':
            print('All down')
        else:
            print('More than one leg up')
    elif move.move_type == 'balance':
        with open('/fenix/fenix/wrk/gyroaccel_data.txt', "r") as f:
            pitch, roll = f.readline().split(',')
        pitch, roll = float(pitch), float(roll)
        if pitch < -cfg.fenix["balance_offset"] and abs(roll) < cfg.fenix["balance_offset"]:
            fk.leg_move_custom(1, 'balance2', [0, 0, -5], add_snapshot=False)
            fk.leg_move_custom(4, 'balance2', [0, 0, -5])
            print('Balance [1, 4]')
        elif pitch > cfg.fenix["balance_offset"] and abs(roll) < cfg.fenix["balance_offset"]:
            fk.leg_move_custom(2, 'balance2', [0, 0, -5], add_snapshot=False)
            fk.leg_move_custom(3, 'balance2', [0, 0, -5])
            print('Balance [2, 3]')
        elif abs(pitch) < cfg.fenix["balance_offset"] and roll < -cfg.fenix["balance_offset"]:
            fk.leg_move_custom(3, 'balance2', [0, 0, -5], add_snapshot=False)
            fk.leg_move_custom(4, 'balance2', [0, 0, -5])
            print('Balance [3, 4]')
        elif abs(pitch) < cfg.fenix["balance_offset"] and roll > cfg.fenix["balance_offset"]:
            fk.leg_move_custom(1, 'balance2', [0, 0, -5], add_snapshot=False)
            fk.leg_move_custom(2, 'balance2', [0, 0, -5])
            print('Balance [1, 2]')
        elif pitch < -cfg.fenix["balance_offset"] and roll > cfg.fenix["balance_offset"]:
            fk.leg_move_custom(1, 'balance1', [0, 0, -5])
            print('Balance [1]')
        elif pitch > cfg.fenix["balance_offset"] and roll > cfg.fenix["balance_offset"]:
            fk.leg_move_custom(2, 'balance1', [0, 0, -5])
            print('Balance [2]')
        elif pitch > cfg.fenix["balance_offset"] and roll < -cfg.fenix["balance_offset"]:
            fk.leg_move_custom(3, 'balance1', [0, 0, -5])
            print('Balance [3]')
        elif pitch < -cfg.fenix["balance_offset"] and roll < -cfg.fenix["balance_offset"]:
            fk.leg_move_custom(4, 'balance1', [0, 0, -5])
            print('Balance [4]')
    elif move.move_type == 'switch_mode':
        fk.switch_mode(move.values['mode'])

    #print(f'[SG]. Sequence: {fk.sequence[-1]}')
    return fk.sequence
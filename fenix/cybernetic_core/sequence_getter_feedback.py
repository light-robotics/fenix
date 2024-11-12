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
    elif command == 'forward_one_legged':
        sequence.append(Move('compensated_leg_movement', {'leg': 1, 'deltas': [0, 0, 15]}))
        sequence.append(Move('touch', {'leg': 1, 'deltas': [0, 0, -15]}))
        sequence.append(Move('body_to_center', {}))
    else:
        print(f'Unknown command')
    
    print(f'[SG]. Sequence commands: {sequence}')
    return sequence

def get_angles_for_sequence(move: Move, fenix_position: List[int]):
    fk = FenixKinematics(fenix_position=fenix_position)
    print(f'fenix_position: {fenix_position}')
    if move.move_type == 'body_movement':
        fk.body_movement(*move.values['deltas'])
    elif move.move_type == 'body_to_center':
        fk.body_to_center()
    elif move.move_type == 'compensated_leg_movement':
        fk.compensated_leg_movement(move.values['leg'], move.values['deltas'])
    elif move.move_type == 'touch':
        fk.leg_move_with_touching(move.values['leg'])

    print(f'[SG]. Sequence: {fk.sequence[-1]}')
    return fk.sequence[-1]
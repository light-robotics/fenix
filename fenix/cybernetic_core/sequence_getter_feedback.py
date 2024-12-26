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
            side_step = 0
            #front_step = 10
            if leg == 3:
                front_step = 9
                sequence.append(Move('body_movement', {'deltas': [front_step, side_step, 0]}))
            elif leg == 4:
                front_step = 10
                sequence.append(Move('body_movement', {'deltas': [front_step, -side_step, 0]}))
            elif leg == 1:
                front_step = 9
                sequence.append(Move('body_movement', {'deltas': [-front_step, -side_step, 0]}))
                sequence.append(Move('body_movement', {'deltas': [0, 0, 6]}))
            elif leg == 2:
                front_step = 9
                sequence.append(Move('body_movement', {'deltas': [-front_step, side_step, 0]}))
            
            #sequence.append(Move('endpoint', {'leg': leg, 'deltas': [0, 0, 27]}))
            #sequence.append(Move('endpoint', {'leg': leg, 'deltas': [-3, 0, 10]}))
            sequence.append(Move('endpoint_absolute', {'leg': leg, 'deltas': [None, None, 24]}))
            #sequence.append(Move('endpoint', {'leg': leg, 'deltas': [FORWARD_LEGS_1LEG_CM, 0, 0]}))
            """
            # new 13-17
            if leg == 3:
                sequence.append(Move('endpoint_absolute', {'leg': leg, 'deltas': [-11.11, -22.31, None]}))
                #Point(x=-11.11, y=-18.31, z=6.73)
            elif leg == 4:
                sequence.append(Move('endpoint_absolute', {'leg': leg, 'deltas': [-14.46, 21.21, None]}))
                #Point(x=-14.46, y=19.21, z=5.88)
            elif leg == 1:
                sequence.append(Move('endpoint_absolute', {'leg': leg, 'deltas': [25.92, 16.56, None]}))
                #Point(x=25.92, y=18.56, z=4.56)
            elif leg == 2:
                sequence.append(Move('endpoint_absolute', {'leg': leg, 'deltas': [23.56, -16.87, None]}))
                #Point(x=23.56, y=-16.87, z=1.12
            
            """
            # 16-18 :
            if leg == 3:
                sequence.append(Move('endpoint_absolute', {'leg': leg, 'deltas': [-11.0, -22.8, None]}))
                #Point(x=-11.51, y=-22.7, z=12.66)
            elif leg == 4:
                sequence.append(Move('endpoint_absolute', {'leg': leg, 'deltas': [-14.5, 22.71, None]}))
                #Point(x=-14.89, y=21.69, z=7.8)
            elif leg == 1:
                sequence.append(Move('endpoint_absolute', {'leg': leg, 'deltas': [29.3, 17.18, None]}))
                #Point(x=25.79, y=16.68, z=2.18)
            elif leg == 2:
                sequence.append(Move('endpoint_absolute', {'leg': leg, 'deltas': [26.9, -20, None]}))
                #Point(x=23.43, y=-17.02, z=3.26)
            
            sequence.append(Move('touch', {'leg': leg}))
            sequence.append(Move('body_to_center', {}))
            sequence.append(Move('balance', {}))
            sequence.append(Move('balance', {}))
            sequence.append(Move('balance', {}))
            sequence.append(Move('balance', {}))
            sequence.append(Move('balance', {}))
            sequence.append(Move('balance', {}))
        
    elif command in ['battle_mode', 'sentry_mode', 'walking_mode', 'run_mode']:
        sequence.append(Move('switch_mode', {"mode": command}))
    else:
        print(f'Unknown command')
    
    #print(f'[SG]. Sequence commands: {sequence}')
    return sequence

def get_angles_for_sequence(move: Move, fenix_position: FenixPosition):
    fk = FenixKinematics(fenix_position=fenix_position, init_snapshot=False)
    print(f'Move: {move.move_type}. {move.values}')
    print('Legs Ds: ', [leg.D for leg in fk.legs.values()])
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
    elif move.move_type == 'endpoint_absolute':
        fk.move_leg_endpoint_abs(move.values['leg'], move.values['deltas'])
    elif move.move_type == 'endpoints':
        fk.move_leg_endpoint(move.values['legs'][0], move.values['deltas'], add_snapshot=False)
        fk.move_leg_endpoint(move.values['legs'][1], move.values['deltas'])
    elif move.move_type == 'endpoint_normalized':
        leg_num = move.values['leg']
        leg = fk.legs[leg_num]
          
        delta = move.values['deltas']
        fk.move_leg_endpoint(move.values['leg'], delta)
    elif move.move_type == 'touch':
        leg = move.values['leg']
        if leg in [1, 4]:
            y_diff = -1
        else:
            y_diff = 1
        fk.leg_move_custom(move.values['leg'], 'touch', [0, 0, -24])
        #fk.move_leg_endpoint(move.values['leg'], [0, 0, -2])
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
        attempts = 0
        ga_read = False
        while not ga_read and attempts < 10:
            try:
                attempts += 1
                with open('/fenix/fenix/wrk/gyroaccel_data.txt', "r") as f:
                    pitch, roll = f.readline().split(',')
                ga_read = True
            except ValueError:
                print('Value error reading pitch and roll')
                continue
            
        pitch, roll = float(pitch), float(roll)
        balance_value = -5

        with open("/fenix/fenix/wrk/neopixel_command.txt", "r") as f:
            legs_down = f.readline().split(',')[0]
        print(f"balance. legs_down: {legs_down}")
        if len(legs_down) != 4:
            legs_down = '1111'
            
        leg1_down, leg2_down, leg3_down, leg4_down = legs_down
        
        if leg1_down == '0':
            fk.leg_move_custom(1, 'balance1', [0, 0, balance_value])
            print(f'{pitch, roll}. Branch 17. Balance [1] {balance_value}')
        elif leg2_down == '0':
            fk.leg_move_custom(2, 'balance1', [0, 0, balance_value])
            print(f'{pitch, roll}. Branch 18. Balance [2] {balance_value}')
        elif leg3_down == '0':
            fk.leg_move_custom(3, 'balance1', [0, 0, balance_value])
            print(f'{pitch, roll}. Branch 19. Balance [3] {balance_value}')
        elif leg4_down == '0':
            fk.leg_move_custom(4, 'balance1', [0, 0, balance_value])
            print(f'{pitch, roll}. Branch 20. Balance [4] {balance_value}')

        elif pitch < -cfg.fenix["balance_offset"] and abs(roll) < cfg.fenix["balance_offset"]:
            #if fk.legs[1].D.z + fk.legs[4].D.z > fk.legs[2].D.z + fk.legs[3].D.z:
                fk.leg_move_custom(1, 'balance2', [0, 0, balance_value], add_snapshot=False)
                fk.leg_move_custom(4, 'balance2', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 1. Balance [1, 4] {balance_value}')
            #else:
            #    fk.leg_move_custom(2, 'balance2', [0, 0, -balance_value], add_snapshot=False)
            #    fk.leg_move_custom(3, 'balance2', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 2. Balance [2, 3] {-balance_value}')
        elif pitch > cfg.fenix["balance_offset"] and abs(roll) < cfg.fenix["balance_offset"]:
            #if fk.legs[2].D.z + fk.legs[3].D.z > fk.legs[1].D.z + fk.legs[4].D.z:
                fk.leg_move_custom(2, 'balance2', [0, 0, balance_value], add_snapshot=False)
                fk.leg_move_custom(3, 'balance2', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 3. Balance [2, 3] {balance_value}')
            #else:
            #    fk.leg_move_custom(1, 'balance2', [0, 0, -balance_value], add_snapshot=False)
            #    fk.leg_move_custom(4, 'balance2', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 4. Balance [1, 4] {-balance_value}')
        elif abs(pitch) < cfg.fenix["balance_offset"] and roll < -cfg.fenix["balance_offset"]:
            #if fk.legs[3].D.z + fk.legs[4].D.z > fk.legs[1].D.z + fk.legs[2].D.z:
                fk.leg_move_custom(3, 'balance2', [0, 0, balance_value], add_snapshot=False)
                fk.leg_move_custom(4, 'balance2', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 5. Balance [3, 4] {balance_value}')
            #else:
            #    fk.leg_move_custom(1, 'balance2', [0, 0, -balance_value], add_snapshot=False)
            #    fk.leg_move_custom(2, 'balance2', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 6. Balance [1, 2] {-balance_value}')
        elif abs(pitch) < cfg.fenix["balance_offset"] and roll > cfg.fenix["balance_offset"]:
            #if fk.legs[1].D.z + fk.legs[2].D.z > fk.legs[3].D.z + fk.legs[4].D.z:
                fk.leg_move_custom(1, 'balance2', [0, 0, balance_value], add_snapshot=False)
                fk.leg_move_custom(2, 'balance2', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 7. Balance [1, 2] {balance_value}')
            #else:
            #    fk.leg_move_custom(3, 'balance2', [0, 0, -balance_value], add_snapshot=False)
            #    fk.leg_move_custom(4, 'balance2', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 8. Balance [3, 4] {-balance_value}')
        elif pitch < -cfg.fenix["balance_offset"] and roll > cfg.fenix["balance_offset"]:
            #if fk.legs[1].D.z > fk.legs[3].D.z:
                fk.leg_move_custom(1, 'balance1', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 9. Balance [1] {balance_value}')
            #else:
            #    fk.leg_move_custom(3, 'balance1', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 10. Balance [3] {-balance_value}')
        elif pitch > cfg.fenix["balance_offset"] and roll > cfg.fenix["balance_offset"]:
            #if fk.legs[2].D.z > fk.legs[4].D.z:
                fk.leg_move_custom(2, 'balance1', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 11. Balance [2] {balance_value}')
            #else:
            #    fk.leg_move_custom(4, 'balance1', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 12. Balance [4] {-balance_value}')
        elif pitch > cfg.fenix["balance_offset"] and roll < -cfg.fenix["balance_offset"]:
            #if fk.legs[3].D.z > fk.legs[1].D.z:
                fk.leg_move_custom(3, 'balance1', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 13. Balance [3] {balance_value}')
            #else:
            #    fk.leg_move_custom(1, 'balance1', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 14. Balance [1] {-balance_value}')
        elif pitch < -cfg.fenix["balance_offset"] and roll < -cfg.fenix["balance_offset"]:
            #if fk.legs[4].D.z > fk.legs[2].D.z:
                fk.leg_move_custom(4, 'balance1', [0, 0, balance_value])
                print(f'{pitch, roll}. Branch 15. Balance [4] {balance_value}')
            #else:
            #    fk.leg_move_custom(2, 'balance1', [0, 0, -balance_value])
            #    print(f'{pitch, roll}. Branch 16. Balance [2] {-balance_value}')
    elif move.move_type == 'switch_mode':
        fk.switch_mode(move.values['mode'])

    #print(f'[SG]. Sequence: {fk.sequence[-1]}')
    return fk.sequence
from typing import List, Tuple
import sys
import os
from joblib import Memory
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cybernetic_core.kinematics import FenixKinematics
from configs import config as cfg
from configs import code_config
from cybernetic_core.cybernetic_utils.moves import Sequence

memory = Memory(code_config.cache_dir, verbose=0)

UP_OR_DOWN_CM   = cfg.moves["up_or_down_cm"]
FORWARD_BODY_CM = cfg.moves["move_body_cm"]
FORWARD_LEGS_1LEG_CM = cfg.moves["forward_body_1_leg_cm"]
FORWARD_LEGS_2LEG_CM = cfg.moves["forward_body_2_leg_cm"]
REPOSITION_CM   = cfg.moves["reposition_cm"]
SIDE_LOOK_ANGLE = cfg.moves["side_look_angle"]
VERTICAL_LOOK_ANGLE = cfg.moves["vertical_look_angle"]


class VirtualFenix():
    """
    An intermediate level of abstraction to apply constraints
    Actually this is only for look angles, cuz other constraints it is easy to check on the fly
    This looks really bad, I need some better decision some day
    """
    def __init__(self, logger):
        self.logger = logger
        self.side_look_angle = 0
        self.vertical_look_angle = 0

    def get_sequence(self, command: str, fenix_position: List[int]):
        if command == 'look_left':
            if self.side_look_angle <= -cfg.limits['side_look_angle']:
                self.logger.info(f'Look left limit reached')
                return None, None
            self.side_look_angle -= SIDE_LOOK_ANGLE
        elif command == 'look_right':
            if self.side_look_angle >= cfg.limits['side_look_angle']:
                self.logger.info(f'Look right limit reached')
                return None, None
            self.side_look_angle += SIDE_LOOK_ANGLE
        elif command == 'look_down':
            if self.vertical_look_angle <= -cfg.limits['vertical_look_angle']:
                self.logger.info(f'Look down limit reached')
                return None, None
            self.vertical_look_angle -= VERTICAL_LOOK_ANGLE
        elif command == 'look_up':
            if self.vertical_look_angle >= cfg.limits['vertical_look_angle']:
                self.logger.info(f'Look up limit reached')
                return None, None
            self.vertical_look_angle += VERTICAL_LOOK_ANGLE
        
        sequence, new_position = get_sequence_for_command_cached(command, fenix_position)
        return sequence, new_position


#@memory.cache
def get_sequence_for_command_cached(command: str, fenix_position: List[int]) -> Sequence:
    fk = FenixKinematics(fenix_position=fenix_position)
    
    if command == 'forward_1':
        # Legs 1 and 3 moved x1
        fk.move_2_legs_phased_13(0, FORWARD_LEGS_2LEG_CM)
    elif command == 'forward_2':
        # Legs 2 and 4 moved x2
        fk.move_2_legs_phased_24(0, 2 * FORWARD_LEGS_2LEG_CM)
    elif command == 'forward_22':
        # Legs 2 and 4 moved x1
        fk.move_2_legs_phased_24(0, FORWARD_LEGS_2LEG_CM)
    elif command == 'forward_3':
        # Legs 1 and 3 moved x2
        fk.move_2_legs_phased_13(0, 2 * FORWARD_LEGS_2LEG_CM)
    elif command == 'forward_32':
        # Legs 1 and 3 moved x1
        fk.move_2_legs_phased_13(0, FORWARD_LEGS_2LEG_CM)
    elif command == 'forward_one_legged':
        fk.move_body_straight(0, FORWARD_LEGS_1LEG_CM)
    elif command in ['battle_mode', 'sentry_mode', 'walking_mode', 'run_mode']:
        fk.switch_mode(command)
    elif command == 'body_forward':
        if fk.body_delta_xy()[1] > cfg.limits["body_forward"]:
            print('Forward body limit reached')
        else:
            fk.body_movement(0, FORWARD_BODY_CM, 0)
    elif command == 'body_backward':
        if fk.body_delta_xy()[1] < -cfg.limits["body_forward"]:
            print('Backward body limit reached')
        else:
            fk.body_movement(0, -FORWARD_BODY_CM, 0)
    elif command == 'body_left':
        if fk.body_delta_xy()[0] < -cfg.limits["body_sideways"]:
            print('Body left limit reached')
        else:
            fk.body_movement(-FORWARD_BODY_CM, 0, 0)
    elif command == 'body_right':
        if fk.body_delta_xy()[0] > cfg.limits["body_sideways"]:
            print('Body right limit reached')
        else:
            fk.body_movement(FORWARD_BODY_CM, 0, 0)
    elif command == 'body_to_center':
        fk.body_to_center()
    elif command == 'up':
        fk.body_movement(0, 0, UP_OR_DOWN_CM)
    elif command == 'down':
        fk.body_movement(0, 0, -UP_OR_DOWN_CM)
    elif command == 'backward_two_legged':
        pass
    elif command == 'backward_one_legged':
        pass
    
    elif command == 'strafe_left_two_legged':
        pass
    elif command == 'strafe_left_one_legged':
        pass
    
    elif command == 'strafe_right_two_legged':
        pass
    elif command == 'strafe_right_one_legged':
        pass    
    
    elif command == 'look_up':
        fk.look_on_angle(-VERTICAL_LOOK_ANGLE) # this should be iterative P.S.: or not    
    elif command == 'look_down':
        fk.look_on_angle(VERTICAL_LOOK_ANGLE) # this should be iterative P.S.: or not
    elif command == 'look_left':
        fk.turn(-12, only_body=True)
    elif command == 'look_right':
        fk.turn(12, only_body=True)
    elif command == 'sight_to_normal':
        fk.look_on_angle(0)
        fk.turn(-fk.side_look_angle, only_body=True)
    elif command == 'turn_left_two_legged':
        fk.turn_move(-25)
    elif command == 'turn_left_one_legged':
        pass    
    elif command == 'turn_right_two_legged':
        fk.turn_move(25)
    elif command == 'turn_right_one_legged':
        pass    
    elif command == 'reposition_x_up':
        fk.reposition_legs(REPOSITION_CM, 0)
    elif command == 'reposition_x_down':
        fk.reposition_legs(-REPOSITION_CM, 0)
    elif command == 'reposition_y_up':
        fk.reposition_legs(0, REPOSITION_CM)
    elif command == 'reposition_y_down':
        fk.reposition_legs(0, -REPOSITION_CM)
    elif command == 'start':
        fk.start()
    elif command == 'end':
        fk.end()
    elif command == 'reset':
        fk.reset()
    elif command == 'hit_1':
        fk.hit(1)
    elif command == 'hit_4':
        fk.hit(4)
    elif command == 'check_leg':
        fk.check_leg_2(1)
    #elif command == 'check_leg_4':
        fk.check_leg_2(4)
    elif command == 'check_leg_2':
        fk.body_movement(0, 0, UP_OR_DOWN_CM)
        fk.demo11()
        fk.demo12()
        fk.demo13()
        fk.check_leg_2(1)
        fk.check_leg_2(4)            
        fk.body_movement(0, -3*FORWARD_BODY_CM, 0)
        fk.body_movement(0, 3*FORWARD_BODY_CM, 0)
        fk.body_movement(0, 0, -UP_OR_DOWN_CM)
    elif command == 'battle_stance':
        fk.battle_stance()
    elif command == 'normal_stance':
        fk.normal_stance()
    elif command == 'jump':
        fk.jump()
    elif command == 'demo1':
        fk.demo1()
    elif command == 'demo2':
        fk.demo2()
    elif command == 'demo11':
        fk.demo11()
    elif command == 'demo12':
        fk.demo12()
    elif command == 'demo13':
        fk.demo13()
    elif command == 'double_back':
        fk.body_movement(0, -3*FORWARD_BODY_CM, 0)
    elif command == 'demo_sequence':
        fk.demo_sequence()
    else:
        print(f'Unknown command')
    
    return fk.sequence, fk.current_position

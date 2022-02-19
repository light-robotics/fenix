from typing import List, Dict
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cybernetic_core.kinematics_constrainted import FenixKinematics
from configs import config as cfg


class SequenceGetter:

    UP_OR_DOWN_CM   = cfg.moves["up_or_down_cm"]
    FORWARD_BODY_CM = cfg.moves["move_body_cm"]
    FORWARD_LEGS_1LEG_CM = cfg.moves["forward_body_1_leg_cm"]
    FORWARD_LEGS_2LEG_CM = cfg.moves["forward_body_2_leg_cm"]
    REPOSITION_CM   = cfg.moves["reposition_cm"]

    def __init__(self, fk: FenixKinematics):
        self.fk = fk
            
    def get_sequence_for_command(self, command: str) -> List[List[float]]:
        self.fk.reset_history()
        
        # forward_two_legged state commands
        if command == 'forward_1':
            # Legs 1 and 3 moved x1
            self.fk.move_2_legs_phased_13(0, self.FORWARD_LEGS_2LEG_CM)
        elif command == 'forward_2':
            # Legs 2 and 4 moved x2
            self.fk.move_2_legs_phased_24(0, 2 * self.FORWARD_LEGS_2LEG_CM)
        elif command == 'forward_22':
            # Legs 2 and 4 moved x1
            self.fk.move_2_legs_phased_24(0, self.FORWARD_LEGS_2LEG_CM)
        elif command == 'forward_3':
            # Legs 1 and 3 moved x2
            self.fk.move_2_legs_phased_13(0, 2 * self.FORWARD_LEGS_2LEG_CM)
        elif command == 'forward_32':
            # Legs 1 and 3 moved x1
            self.fk.move_2_legs_phased_13(0, self.FORWARD_LEGS_2LEG_CM)
        elif command == 'forward_one_legged':
            self.fk.move_body_straight(0, self.FORWARD_LEGS_1LEG_CM)
        elif command == 'body_forward':
            self.fk.body_movement(0, self.FORWARD_BODY_CM, 0)
        elif command == 'backward_two_legged':
            pass
        elif command == 'backward_one_legged':
            pass
        elif command == 'body_backward':
            self.fk.body_movement(0, -self.FORWARD_BODY_CM, 0)
        elif command == 'strafe_left_two_legged':
            pass
        elif command == 'strafe_left_one_legged':
            pass
        elif command == 'body_left':
            self.fk.body_movement(-self.FORWARD_BODY_CM, 0, 0)
        elif command == 'strafe_right_two_legged':
            pass
        elif command == 'strafe_right_one_legged':
            pass
        elif command == 'body_right':
            self.fk.body_movement(self.FORWARD_BODY_CM, 0, 0)
        elif command == 'body_to_center':
            self.fk.body_to_center()
        elif command == 'up':
            self.fk.body_movement(0, 0, self.UP_OR_DOWN_CM)
        elif command == 'look_up':
            self.fk.look_on_angle(-20) # this should be iterative
        elif command == 'down':
            self.fk.body_movement(0, 0, -self.UP_OR_DOWN_CM)
        elif command == 'look_down':
            self.fk.look_on_angle(20) # this should be iterative
        elif command == 'turn_left_two_legged':
            self.fk.turn_move(-25)
        elif command == 'turn_left_one_legged':
            pass
        elif command == 'look_left':
            self.fk.turn(-12, only_body=True)
        elif command == 'turn_right_two_legged':
            self.fk.turn_move(25)
        elif command == 'turn_right_one_legged':
            pass
        elif command == 'look_right':
            self.fk.turn(12, only_body=True)
        elif command == 'sight_to_normal':
            self.fk.look_on_angle(0)
            # should fix left-right sight also
        elif command == 'reposition_x_up':
            self.fk.reposition_legs(self.REPOSITION_CM, 0)
        elif command == 'reposition_x_down':
            self.fk.reposition_legs(-self.REPOSITION_CM, 0)
        elif command == 'reposition_y_up':
            self.fk.reposition_legs(0, self.REPOSITION_CM)
        elif command == 'reposition_y_down':
            self.fk.reposition_legs(0, -self.REPOSITION_CM)
        elif command == 'start':
            self.fk.start()
        elif command == 'end':
            self.fk.end()
        elif command == 'reset':
            self.fk.reset()
        elif command == 'body_forward':
            self.fk.body_movement(0, 5, 0)
        elif command == 'body_backward':
            self.fk.body_movement(0, -5, 0)
        elif command == 'hit_1':
            self.fk.hit(1)
        elif command == 'hit_4':
            self.fk.hit(4)
        elif command == 'check_leg':
            self.fk.check_leg_2(1)
        #elif command == 'check_leg_4':
            self.fk.check_leg_2(4)
        elif command == 'check_leg_2':
            self.fk.body_movement(0, 0, self.UP_OR_DOWN_CM)
            self.fk.demo11()
            self.fk.demo12()
            self.fk.demo13()
            self.fk.check_leg_2(1)
            self.fk.check_leg_2(4)            
            self.fk.body_movement(0, -3*self.FORWARD_BODY_CM, 0)
            self.fk.body_movement(0, 3*self.FORWARD_BODY_CM, 0)
            self.fk.body_movement(0, 0, -self.UP_OR_DOWN_CM)
        elif command == 'battle_stance':
            self.fk.battle_stance()
        elif command == 'normal_stance':
            self.fk.normal_stance()
        elif command == 'jump':
            self.fk.jump()
        elif command == 'demo1':
            self.fk.demo1()
        elif command == 'demo2':
            self.fk.demo2()
        elif command == 'demo11':
            self.fk.demo11()
        elif command == 'demo12':
            self.fk.demo12()
        elif command == 'demo13':
            self.fk.demo13()
        elif command == 'double_back':
            self.fk.body_movement(0, -3*self.FORWARD_BODY_CM, 0)
        elif command == 'demo_sequence':
            self.fk.demo_sequence()
        else:
            print(f'Unknown command')

        return self.fk.sequence

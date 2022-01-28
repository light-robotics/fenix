from typing import List, Dict
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cybernetic_core.kinematics import FenixKinematics
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
            self.fk.body_movement(0, 0, 2)
        elif command == 'end':
            pass
        elif command == 'body_forward':
            self.fk.body_movement(0, 5, 0)
        elif command == 'body_backward':
            self.fk.body_movement(0, -5, 0)
        elif command == 'hit':
            self.fk.hit(1)
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



"""
def calculate_sequence(ms, command):

    command_text = command.split(' ')[0]
    command_value = int(command.split(' ')[1])
    try:
        command_value_2 = int(command.split(' ')[2])
    except:
        command_value_2 = None

    if ms is None:
        ms = FenixKinematics(legs_offset_v=-10, legs_offset_h=15)

    ms.reset_history()
    print(f'Offset V : {ms.current_legs_offset_v}. Offset H : {ms.current_legs_offset_h}')

    if command_text == 'forward2leg' or command_text == 'f2l':
        if command_value_2 is None:
            ms.move_2_legs_v2(command_value)
        else:
            ms.move_2_legs_v2(command_value, command_value_2)
    elif command_text == 'climb2leg' or command_text == 'c2l':
        if command_value > 0:
            target_height = 10 + command_value # 20            
        else:
            target_height = 10
        
        target_width = 18
        
        ms.body_movement(0, 0, ms.current_legs_offset_v + target_height)
        ms.reposition_legs(target_width - ms.current_legs_offset_h, target_width - ms.current_legs_offset_h)

        #ms.climb_2_legs(command_value, steps_arr=[12, 20, 20, 24, 12])
        ms.climb_2_legs(command_value, steps_arr=[12, 20, 16, 12, 16, 12])
    elif command_text == 'run2leg' or command_text == 'r2l':

        target_height = 10        
        target_width = 16
        
        ms.body_movement(0, 0, ms.current_legs_offset_v + target_height)
        ms.reposition_legs(target_width - ms.current_legs_offset_h, target_width - ms.current_legs_offset_h)

        # same movement, but another speed is applied in move module
        if command_value_2 is None:
            ms.move_2_legs_v2(command_value)
        else:
            ms.move_2_legs_v2(command_value, command_value_2)
    elif command_text == 'backward2leg' or command_text == 'b2l':
        if command_value_2 is None:
            ms.move_2_legs(-command_value)
        else:
            ms.move_2_legs(-command_value, command_value_2)
        #ms.move_2_legs(-command_value)
    elif command_text == 'forward' or command_text == 'f':
        ms.move_body_straight(0, command_value, leg_seq=[1, 4, 3, 2])
    elif command_text == 'backward' or command_text == 'b':
        ms.move_body_straight(0, -command_value, leg_seq=[2, 3, 4, 1])
    elif command_text == 'up' or command_text == 'u':
        ms.body_movement(0, 0, command_value)
    elif command_text == 'down' or command_text == 'd':
        ms.body_movement(0, 0, -command_value)
    elif command_text == 'rep' or command_text == 'r':
        if command_value_2 is None:
            command_value_2 = command_value
        ms.reposition_legs(command_value, command_value_2)
    elif command_text == 'lookvert' or command_text == 'lv':
        #if ms.current_angle * command_value > 0:
        #    ms.look_on_angle(0)
        ms.look_on_angle(-command_value)
    elif command_text == 'lookhor' or command_text == 'lh':
        ms.turn(command_value, only_body=True)
    elif command_text == 'turn2leg' or command_text == 't2l':
        ms.turn_move(command_value)
    elif command_text == 'turn' or command_text == 't':
        turn = command_value
        current_turn = 0
        if turn > 0:
            while turn - current_turn > 30:
                print('Turning on 30')
                ms.turn_move(30)
                current_turn += 30

            print(f'Turning on {turn - current_turn}')
            ms.turn_move(turn - current_turn)
        else:
            while turn - current_turn < -30:
                print('Turning on -30')
                ms.turn_move(-30)
                current_turn -= 30

            print(f'Turning on {turn - current_turn}')
            ms.turn_move(turn - current_turn)

    elif command_text == 'comp':
        ms.body_compensation_for_a_leg(command_value)
    elif command_text == 'legup' or command_text == 'lu':
        ms.move_leg_endpoint(command_value, [0, 0, command_value_2])
    elif command_text == 'legforw' or command_text == 'lf':
        ms.move_leg_endpoint(command_value, [0, command_value_2, 0])
    elif command_text == 'legside' or command_text == 'ls':
        ms.move_leg_endpoint(command_value, [command_value_2, 0, 0])
    elif command_text == 'center' or command_text == 'c':
        ms.body_to_center()
    elif command_text == 'start':
        target_height = 12
        target_width = 14
        # current_legs_offset_v is below zero
        ms.body_movement(0, 0, ms.current_legs_offset_v + target_height)
        ms.reposition_legs(target_width - ms.current_legs_offset_h, target_width - ms.current_legs_offset_h)
        #ms.leg_up = 5
    elif command_text == 'end':
        target_height = 3
        target_width = 18
        # current_legs_offset_v is below zero
        ms.reposition_legs(target_width - ms.current_legs_offset_h, target_width - ms.current_legs_offset_h)
        ms.body_movement(0, 0, ms.current_legs_offset_v + target_height)
        #ms.leg_up = 4
    elif command_text == 'dance':
        ms.opposite_legs_up(command_value, command_value_2)
    elif command_text == 'hit':
        ms.hit(command_value)
    else:
        return None
    
    #ms.print_legs_diff()
    return ms.sequence
"""
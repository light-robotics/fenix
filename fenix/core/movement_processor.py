import time
import datetime
import copy
from typing import Optional, Union
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cybernetic_core.kinematics import FenixKinematics
from cybernetic_core.sequence_getter import get_sequence_for_command_cached
import configs.code_config as code_config
import configs.config as config
import logging.config

if not code_config.DEBUG:
    from fenix_hardware.fenix_servos import FenixServos


class MovementProcessor:
    def __init__(self):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('main_logger')
        self.logger.info('==================START==================')

        self.max_processed_command_id = 0
        self.state = '0'
        
        self.fk = FenixKinematics()

        if not code_config.DEBUG:
            self.fs = FenixServos()
        
        self.speed = 500
        self.body_speed = 1000        

        # state is used for multi-phased moves
        # state 0 means start position
        # state f1 means legs 1 and 3 had been moved forward
        # state f2 means legs 2 and 4 had been moved forward

    def read_command(self) -> Optional[Union[str, int]]:        
        with open(code_config.movement_command_file, 'r') as f:
            contents = f.readline().split(',')

        if len(contents) != 3:
            return None

        command_id = int(contents[0])
        command = contents[1].strip()

        # this commands are not excluded even if id is the same
        repeating_commands = [
            'forward_two_legged',
            'backward_two_legged',
            'strafe_left_two_legged',
            'strafe_right_two_legged', 
            'up', 
            'down',
            'look_up',
            'look_down',
            'look_left',
            'turn_right'
            ]        

        if self.max_processed_command_id == 0:
            self.max_processed_command_id = command_id
        elif self.max_processed_command_id == command_id and \
            command not in repeating_commands:
            # command has already been processed
            #print(f'Command {contents} has already been processed')
            return None

        self.max_processed_command_id = command_id
        return command, int(contents[2])

    def execute_command(self, command: str, speed: int) -> None:
        if self.speed != speed:
            if not code_config.DEBUG:
                self.fs.set_speed(speed)
            self.speed = speed
            print(f'Setting speed to {speed}')

        # first we finish movements that are in progress
        if self.state == 'f1':
            if command == 'forward_two_legged':
                print(f'Forward 2. Legs 2 and 4 moved x2. {speed}')
                self.state = 'f2'
                self.run_sequence('forward_2')
            else:
                print(f'Forward 22. Legs 2 and 4 moved x1. {speed}')
                self.state = '0'
                self.run_sequence('forward_22')
        elif self.state == 'f2':
            if command == 'forward_two_legged':
                print(f'Forward 3. Legs 1 and 3 moved x2. {speed}')
                self.state = 'f1'
                self.run_sequence('forward_3')
            else:
                print(f'Forward 32. Legs 1 and 3 moved x1. {speed}')
                self.state = '0'
                self.run_sequence('forward_32')
        
        # then we make the next move
        if self.state == '0':
            if command == 'forward_two_legged':
                print(f'Forward 1. Legs 1 and 3 moved x1. {speed}')
                self.state = 'f1'
                self.run_sequence('forward_1')
            else:
                print(f'Executing command {command}')
                self.logger.info(f'Executing command {command}')
                if command == 'none':
                    time.sleep(0.1)
                else:    
                    self.run_sequence(command)
                        
    def run_sequence(self, command: str) -> None:
        try:            
            self.logger.info(f'MOVE. Trying command {command}')
            before_sequence_time = datetime.datetime.now()
            #import math
            #print(f'self.fk.current_position :\n{[math.degrees(x) for x in self.fk.current_position]}')
            sequence, new_position = get_sequence_for_command_cached(command, self.fk.current_position)
            self.logger.info(f'[TIMING] Sequence calculation took : {datetime.datetime.now() - before_sequence_time}')
            self.fk = FenixKinematics(fenix_position=new_position)
        except Exception as e:
            print(f'MOVE Failed. Could not process command - {str(e)}')
            self.logger.info(f'MOVE Failed. Could not process command - {str(e)}')
            time.sleep(2.0)
            return
        self.logger.info(f'MOVE Started')    
        start_time = datetime.datetime.now()
        prev_angles = None
        for move_snapshot in sequence:
            angles = move_snapshot.angles_snapshot[:]
            #if move_snapshot.move_type == 'body' and self.speed != self.body_speed:
            #    self.fs.set_speed(self.body_speed)
            self.logger.info(f'Moving to {angles}')
            if not code_config.DEBUG:                
                #self.fs.set_servo_values_not_paced(angles) # here issuing command to servos
                self.fs.set_servo_values_not_paced_v2(angles, prev_angles)
                #self.fs.set_servo_values_not_paced_v2(angles)
                #self.fs.set_servo_values_not_paced_v3(angles, prev_angles)
                #self.fs.set_servo_values_paced(angles)
                prev_angles = angles[:]
            else:
                time.sleep(1.0)
        self.logger.info(f'[TIMING] Step took : {datetime.datetime.now() - start_time}')

    def move(self):
        try:
            while True:
                command_read = self.read_command()
                
                if command_read is None:
                    time.sleep(0.1)
                    continue

                command, speed = command_read
                if command == 'exit':
                    break
                
                self.execute_command(command, speed)

        except KeyboardInterrupt:
            print('Movement stopped')

if __name__ == '__main__':
    MP = MovementProcessor()
    MP.move()

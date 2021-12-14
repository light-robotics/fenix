import time
import datetime
import copy
from typing import Optional, Union
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cybernetic_core.kinematics import FenixKinematics
from cybernetic_core.sequence_getter import SequenceGetter
from fenix_hardware.fenix_servos import FenixServos
import configs.code_config as code_config
import configs.config as config
import logging.config


class MovementProcessor:
    def __init__(self):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('main_logger')
        self.logger.info('==================START==================')

        self.max_processed_command_id = 0
        self.state = '0'
        #self.fk = FenixKinematics(legs_offset_v=16, legs_offset_h_x=18, legs_offset_h_y=18)
        self.fk = FenixKinematics(
            legs_offset_v=config.start["vertical"], 
            legs_offset_h_x=config.start["horizontal_x"],
            legs_offset_h_y=config.start["horizontal_y"]
        )
        self.fs = FenixServos()
        self.sg = SequenceGetter(self.fk)
        self.speed = 500
        self.body_speed = 1000        

        # state is used for multi-phased moves
        # state 0 means start position
        # state f1 means legs 1 and 3 had been moved forward
        # state f2 means legs 2 and 4 had been moved forward

    def read_command(self) -> Optional[Union[str, int]]:
        command_file = '//fnx//fenix//wrk//movement_command.txt'
        with open(command_file, 'r') as f:
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
            print(f'Command {contents} has already been processed')
            return None

        self.max_processed_command_id = command_id
        return command, int(contents[2])

    def execute_command(self, command: str, speed: int) -> None:
        if self.speed != speed:
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
            fk_cp = copy.deepcopy(self.fk)
            self.logger.info(f'MOVE. Trying command {command}')
            sequence = self.sg.get_sequence_for_command(command)
        except Exception as e:
            self.logger.info(f'MOVE Failed. Could not process command - {str(e)}')
            self.fk = copy.deepcopy(fk_cp)
            time.sleep(2.0)
            return
        self.logger.info(f'MOVE Started')    
        start_time = datetime.datetime.now()
        for move_snapshot in sequence:
            angles = move_snapshot.angles_snapshot[:]
            #if move_snapshot.move_type == 'body' and self.speed != self.body_speed:
            #    self.fs.set_speed(self.body_speed)
            self.logger.info(f'Moving to {angles}')
            #time.sleep(3)
            self.fs.set_servo_values_paced(angles) # here issuing command to servos
        self.logger.info(f'Step took : {datetime.datetime.now() - start_time}')

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

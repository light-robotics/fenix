import time
import datetime
import pickle
from typing import Callable, Optional, Union
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cybernetic_core.kinematics import FenixKinematics
from cybernetic_core.geometry.angles import convert_legs_angles, convert_legs_angles_to_kinematic
from cybernetic_core.sequence_getter_feedback import get_sequence_for_command, get_angles_for_sequence
from cybernetic_core.geometry.angles import AnglesException
from core.utils.multiphase_moves import CommandsForwarder
import configs.code_config as code_config
import logging.config

if not code_config.DEBUG:
    from fenix_hardware.fenix_servos import FenixServos


class MovementProcessor:
    def __init__(self):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('main_logger')
        self.logger.info('==================START==================')

        self.max_processed_command_id = 0
        
        fk = FenixKinematics()
        self.cf = CommandsForwarder()
        
        self.fenix_position = fk.current_position

        if not code_config.DEBUG:
            self.fs = FenixServos()
        
        self.speed = 400
        self.body_speed = 800

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
            'body_forward',
            'body_backward',
            'body_left',
            'body_right',            
            'turn_right',
            'turn_left'
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

    def execute_command(self, command: str, speed: int, kwargs=None) -> None:
        if self.speed != speed:
            if not code_config.DEBUG:
                self.fs.set_speed(speed)
            self.speed = speed
            print(f'Setting speed to {speed}')

        # first we finish movements that are in progress
        if command in self.cf.moves:
            next_move = self.cf.get_move(command)
            print(f'Executing move: {next_move}')
            self.run_sequence(next_move)
        else:
            if self.cf.current_status:
                next_move = self.cf.get_move(command)
                print(f'Status: {self.cf.current_status}. Executing move: {next_move}')
                self.run_sequence(next_move)
            print(f'Executing move: {command}')
            if command == 'none':
                time.sleep(0.1)
            else:    
                self.run_sequence(command, kwargs)
                        
    def run_sequence(self, command: str, kwargs=None) -> None:        
        self.logger.info(f'[MOVE] Started run_sequence : {datetime.datetime.now()}')
        self.logger.info(f'MOVE. Trying command {command}')
        sequence = get_sequence_for_command(command, kwargs)
            
        self.logger.info(f'[MOVE] Started: {datetime.datetime.now()}')    
        start_time = datetime.datetime.now()

        for move in sequence:
            next_angles = get_angles_for_sequence(move, self.fenix_position)
            angles_snapshot = next_angles.angles_snapshot[:]
            #print(f'angles_snapshot: {angles_snapshot}')

            if next_angles.move_type == 'body':
                self.fs.set_speed(self.body_speed)
            else:
                self.fs.set_speed(self.speed)
                        
            if next_angles.move_type == 'touch':
                self.logger.info('[MP] Using function set_servo_values_touching')
                move_function = self.fs.set_servo_values_touching
            else:
                self.logger.info('[MP] Using function set_servo_values_paced_wo_feedback')
                move_function = self.fs.set_servo_values_paced_wo_feedback

            self.logger.info(f'[MP] Moving to {angles_snapshot}. Move type: {next_angles.move_type}')
            self.logger.info(f'Speed: {self.fs.speed}')
            if not code_config.DEBUG:
                new_angles = move_function(angles_snapshot)
                self.fenix_position = convert_legs_angles_to_kinematic(new_angles[:])
            else:
                time.sleep(1.0)
        self.logger.info(f'[MOVE] finished: {datetime.datetime.now()}')
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

                if command == 'disable_torque':
                    self.fs.disable_torque()
                    
                else:
                    try:
                        self.execute_command(command, speed)
                    except Exception as e:
                        self.fs.disable_torque()
                        time.sleep(1.0)
                        print(e)

        except KeyboardInterrupt:
            print('Movement stopped')

if __name__ == '__main__':
    MP = MovementProcessor()
    MP.move()

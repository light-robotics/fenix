import time
import datetime
import pickle
from typing import Callable, Optional, Union
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cybernetic_core.kinematics import FenixKinematics
from cybernetic_core.geometry.angles import build_position_from_servos, convert_legs_angles_to_kinematic_C
from cybernetic_core.sequence_getter_feedback import get_sequence_for_command, get_angles_for_sequence
from cybernetic_core.geometry.angles import AnglesException, DistanceException
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

    def get_and_move_to_angles(self, move):
        sequence = get_angles_for_sequence(move, self.fenix_position)
        print(f'Inner sequence: {len(sequence)}')
        for next_angles in sequence:
            #print(next_angles)
            angles_snapshot = next_angles.angles_snapshot
            print(f'angles_snapshot: {angles_snapshot}')

            if next_angles.move_type == 'body':
                self.fs.set_speed(1500)
            else:
                self.fs.set_speed(self.speed)
                        
            
            if next_angles.move_type == 'touch_1':
                self.logger.info('[MP] Using function set_servo_values_touching_1')
                move_function = self.fs.set_servo_values_touching_1
                self.fs.set_speed(1000)
            elif next_angles.move_type == 'touch_2':
                self.logger.info('[MP] Using function set_servo_values_touching_2')
                move_function = self.fs.set_servo_values_touching_2
                self.fs.set_speed(1000)
            elif next_angles.move_type == 'touch_3':
                self.logger.info('[MP] Using function set_servo_values_touching_3')
                move_function = self.fs.set_servo_values_touching_3
                self.fs.set_speed(1000)
            elif next_angles.move_type == 'touch_4':
                self.logger.info('[MP] Using function set_servo_values_touching_4')
                move_function = self.fs.set_servo_values_touching_4
                self.fs.set_speed(1000)
            elif next_angles.move_type == 'touch_2legs':
                self.logger.info('[MP] Using function set_servo_values_3leg_touching')
                move_function = self.fs.set_servo_values_3leg_touching
                self.fs.set_speed(1000)
            elif next_angles.move_type == 'balance1':
                self.logger.info('[MP] Using function set_servo_values_balancing_1leg')
                move_function = self.fs.set_servo_values_balancing_1leg
                self.fs.set_speed(3000)
            elif next_angles.move_type == 'balance2':
                self.logger.info('[MP] Using function set_servo_values_balancing_2leg')
                move_function = self.fs.set_servo_values_balancing_2leg
                self.fs.set_speed(3000)
            else:
                self.logger.info('[MP] Using function set_servo_values_paced_wo_feedback')
                move_function = self.fs.set_servo_values_paced_wo_feedback

            self.logger.info(f'[MP] Moving to {angles_snapshot}. Move type: {next_angles.move_type}')
            self.logger.info(f'Speed: {self.fs.speed}')

            new_angles = move_function(angles_snapshot)
            
            print('New position set')
            self.fenix_position = convert_legs_angles_to_kinematic_C(new_angles)
        #print(f'convert_legs_angles_to_kinematic: {self.fenix_position}')



    def run_sequence(self, command: str, kwargs=None) -> bool:        
        self.logger.info(f'[MOVE] Started run_sequence : {datetime.datetime.now()}')
        self.logger.info(f'MOVE. Trying command {command}')
        
        sequence = get_sequence_for_command(command, kwargs)
        #move = sequence[0]
            
        self.logger.info(f'[MOVE] Started: {datetime.datetime.now()}')    
        start_time = datetime.datetime.now()
        print(f'Outer Sequence len: {len(sequence)}')
        for move in sequence:
            print(f'Move: {move}')
            #time.sleep(5)
            command_executed = False
            attempts = 1
            while not command_executed and attempts < 12:
                try:
                    attempts += 1
                    self.get_and_move_to_angles(move)
                    command_executed = True
                    #return result
                except DistanceException as e:
                    print(f'Attempt {attempts}. Execution of command {command} resulted in:\n{e}\nMoving down')
                    down_sequence = get_sequence_for_command('down')
                    self.get_and_move_to_angles(down_sequence[0])
                except AnglesException as e:
                    print(f'Attempt {attempts}. Execution of command {command} resulted in:\n{e}\nMoving up')
                    try:
                        up_sequence = get_sequence_for_command('up')
                        self.get_and_move_to_angles(up_sequence[0])
                    except (AnglesException, DistanceException) as e:
                        print(f'Attempt {attempts}. Execution of command UP resulted in:\n{e}\nMoving down')
                        down_sequence = get_sequence_for_command('down')
                        self.get_and_move_to_angles(down_sequence[0])
            if attempts == 12:
                print('Command failed all attempts. Exiting') 
                return False

        self.logger.info(f'[MOVE] finished: {datetime.datetime.now()}')
        self.logger.info(f'[TIMING] Step took : {datetime.datetime.now() - start_time}')
        return False

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
                    #try:
                        self.execute_command(command, speed)
                    #except Exception as e:
                    #    self.fs.disable_torque()
                    #    time.sleep(1.0)
                    #    print(e)

        except KeyboardInterrupt:
            print('Movement stopped')

if __name__ == '__main__':
    MP = MovementProcessor()
    MP.move()

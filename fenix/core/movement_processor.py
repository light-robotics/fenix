import time
import datetime
import copy
from typing import Callable, Optional, Union
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cybernetic_core.kinematics import FenixKinematics
from cybernetic_core.sequence_getter import VirtualFenix
from core.utils.multiphase_moves import CommandsForwarder
from fenix_hardware.fenix_lidar import FenixLidar
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
        
        fk = FenixKinematics()
        self.vf = VirtualFenix(self.logger)
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

    def execute_command(self, command: str, speed: int) -> None:
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
                self.run_sequence(command)

    def move_function_dispatch(self, command: str) -> Callable:
        if command in ['hit_1', 'hit_4', 'forward_one_legged']:
            self.logger.info('Using function set_servo_values_paced')
            return self.fs.set_servo_values_paced
        elif command in ['forward_1', 'forward_2', 'forward_3', 'forward_22', 'forward_32']:
            self.logger.info('Using function set_servo_values_for_running')
            return self.fs.set_servo_values_for_running
            #return self.fs.set_servo_values_overshoot
        else:
            self.logger.info('Using function set_servo_values_not_paced_v2')
            return self.fs.set_servo_values_not_paced_v2
                        
    def run_sequence(self, command: str) -> None:
        self.logger.info(f'[MOVE] Started run_sequence : {datetime.datetime.now()}')
        try:            
            self.logger.info(f'MOVE. Trying command {command}')
            before_sequence_time = datetime.datetime.now()
            #sequence, new_position = get_sequence_for_command_cached(command, self.fenix_position)
            sequence, new_position = self.vf.get_sequence(command, self.fenix_position)
            
            if sequence is None:
                self.logger.info(f'MOVE. Command aborted')
                return
            self.logger.info(f'[TIMING] Sequence calculation took : {datetime.datetime.now() - before_sequence_time}')
            self.fenix_position = new_position[:]
        except ValueError as e:
            print(f'MOVE Failed. Could not process command - {str(e)}')
            self.logger.info(f'MOVE Failed. Could not process command - {str(e)}')
            time.sleep(0.3)
            return
        
        self.logger.info(f'[MOVE] Started: {datetime.datetime.now()}')    
        start_time = datetime.datetime.now()
        #prev_angles = None
        if not code_config.DEBUG:
            move_function = self.move_function_dispatch(command)

        for move_snapshot in sequence:
            angles = move_snapshot.angles_snapshot[:]
            #if move_snapshot.move_type == 'body' and self.speed != self.body_speed:
            #    self.fs.set_speed(self.body_speed)
            if move_snapshot.move_type == 'body':
                self.fs.set_speed(self.body_speed)
            else:
                self.fs.set_speed(self.speed)
            self.logger.info(f'Moving to {angles}. Move type: {move_snapshot.move_type}')
            self.logger.info(f'Speed: {self.fs.speed}')
            if not code_config.DEBUG:
                move_function(angles)
                #self.fs.set_servo_values_not_paced_v2(angles, prev_angles)
                #self.fs.set_servo_values_paced(angles)
                #prev_angles = angles[:]
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
                elif command == 'enable_torque':
                    self.fs.enable_torque()
                elif command == 'lidar_scan':
                    #for i in range(50):
                        self.fl = FenixLidar()
                        self.fl.current_height = self.vf.get_height(self.fenix_position)
                        self.fl.scan_front()
                        #time.sleep(1)
                else:
                    try:
                        self.execute_command(command, speed)
                    except Exception as e:
                        self.fs.disable_torque()
                        raise Exception

        except KeyboardInterrupt:
            print('Movement stopped')

if __name__ == '__main__':
    MP = MovementProcessor()
    MP.move()

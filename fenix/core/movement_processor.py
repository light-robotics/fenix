import time
import datetime
import pickle
from typing import Callable, Optional, Union
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cybernetic_core.kinematics import FenixKinematics
from cybernetic_core.sequence_getter import VirtualFenix
from cybernetic_core.geometry.angles import AnglesException
from core.utils.multiphase_moves import CommandsForwarder
from fenix_hardware.fenix_tof_cam import FenixTofCamera
from fenix_hardware.fenix_tof_sensor import FenixTofs
from hardware.mpu6050_avg import single_scan
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
        self.ftfs = FenixTofs()
        #self.ftc = FenixTofCamera()
        
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

    def move_function_dispatch(self, command: str) -> Callable:
        if command in ['hit_1', 'hit_2', 'forward_one_legged']:
            self.logger.info('Using function set_servo_values_paced')
            return self.fs.set_servo_values_paced
        elif command in ['forward_1', 'forward_2', 'forward_3', 'forward_22', 'forward_32']:
            self.logger.info('Using function set_servo_values_for_running')
            return self.fs.set_servo_values_for_running
            #return self.fs.set_servo_values_paced
        else:
            self.logger.info('Using function set_servo_values_not_paced_v2')
            #return self.fs.set_servo_values_not_paced_v2
            return self.fs.set_servo_values_paced
                        
    def run_sequence(self, command: str, kwargs) -> None:
        if command == 'overcome_obstacle':
            with open('/fenix/fenix/wrk/obstacles_sequence', 'rb') as f:
                sequence = pickle.load(f)
        else:
            self.logger.info(f'[MOVE] Started run_sequence : {datetime.datetime.now()}')
            try:            
                self.logger.info(f'MOVE. Trying command {command}')
                before_sequence_time = datetime.datetime.now()
                #sequence, new_position = get_sequence_for_command_cached(command, self.fenix_position)
                sequence, new_position = self.vf.get_sequence(command, self.fenix_position, kwargs)
                
                if sequence is None:
                    self.logger.info(f'MOVE. Command aborted')
                    return
                self.logger.info(f'[TIMING] Sequence calculation took : {datetime.datetime.now() - before_sequence_time}')
                self.fenix_position = new_position[:]
            except (ValueError, AnglesException) as e:
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
                elif command == 'tof_scan':
                    self.execute_command("back_8", 1000)
                    legs_zs = self.vf.get_legs_zs(self.fenix_position)
                    print(f'legs_zs: {legs_zs}')
                    self.execute_command(
                        "leg_up_adjusted", 
                        1000, 
                        {
                            "leg_num": 2, 
                            "leg_up": 35 - legs_zs[1]
                        }
                    )
                    time.sleep(2.0)
                    
                    angle = self.vf.get_leg_angle_to_surface(self.fenix_position, 1)
                    data_1 = self.ftfs.calculate_touch(0, angle) + 4
                    print(f'Moving down for {data_1} cm')
                    self.execute_command(f"leg_down_adjusted", 1000, {"leg_num": 2, "leg_down": data_1})

                    self.execute_command(
                        "leg_up_adjusted", 
                        1000, 
                        {
                            "leg_num": 1, 
                            "leg_up": 35 - legs_zs[0]
                        }
                    )
                    time.sleep(2.0)
                    angle = self.vf.get_leg_angle_to_surface(self.fenix_position, 2)
                    data_2 = self.ftfs.calculate_touch(1, angle) + 5
                    print(f'Moving down for {data_2} cm')
                    self.execute_command(f"leg_down_adjusted", 1000, {"leg_num": 1, "leg_down": data_2})
                    self.execute_command("back_legs", 1000)
                    """
                elif command == 'tof_scan':
                    self.execute_command('up_16', 500)
                    self.execute_command('look_down', 500)
                    self.execute_command('look_down', 500)
                    self.execute_command('look_down', 500)
                    time.sleep(1)
                    current_height = round(self.vf.get_height(self.fenix_position))
                    roll = single_scan()
                    #self.ftc.read_depth(current_height, roll)
                    time.sleep(1)
                    self.execute_command('look_up', 500)
                    self.execute_command('look_up', 500)
                    self.execute_command('look_up', 500)
                    self.execute_command('down_16', 1000)
                elif command == 'tof_scan2':
                    self.execute_command('body_forward_8', 500)
                    time.sleep(1)
                    for _ in range(3):
                        current_height = self.vf.get_height(self.fenix_position)
                        #self.ftc.read_depth(current_height)
                        self.execute_command('up_4', 500)
                        time.sleep(1)
                    current_height = self.vf.get_height(self.fenix_position)
                    #self.ftc.read_depth(current_height)
                    time.sleep(1)
                    self.execute_command('down_12', 1000)
                    self.execute_command('body_backward_8', 1000)
                    """
                else:
                    try:
                        self.execute_command(command, speed)
                    except Exception as e:
                        self.fs.disable_torque()
                        print(e)
                        #raise Exception

        except KeyboardInterrupt:
            print('Movement stopped')

if __name__ == '__main__':
    MP = MovementProcessor()
    MP.move()

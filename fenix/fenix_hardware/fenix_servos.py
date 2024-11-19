import time
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.lx16a import LX16A, read_values
import logging
import configs.config as config
import configs.code_config as code_config
import logging.config
logging.config.dictConfig(code_config.logger_config)


class FenixServos:
    def __init__(self):
        self.m1 = LX16A(port='/dev/ttyAMA3') # 5-8   # 1-4
        self.m4 = LX16A(port='/dev/ttyAMA4') # 1-4   # 13-16
        self.speed = 500
        self.min_speed = 700
        self.max_speed = 0 # 130 # 0 is instant, 10000 is very slow
        self.diff_from_target_limit = config.fenix["servos"]["diff_from_target_limit"] # when it's time to start next movement
        self.diff_from_prev_limit = config.fenix["servos"]["diff_from_prev_limit"] # 1.0 # start next movement if we're stuck

        self.logger = logging.getLogger('main_logger')
        
        # 0.16 sec / 60 degrees for 7.4V+
        # 0.18 sec / 60 degrees for 6V+
        # my max speed is for 45 degrees
        # that means that max speed should be 120 for 7.4V+ and 135 for 6V+
        self.servos = [2, 3, 4, 5, 8, 9, 10, 11, 14, 15, 16, 17, 20, 21, 22, 23]

    def print_status(self):
        for i in [2, 3, 4, 5, 8, 9, 10, 11]:
            self.m1.read_values(i)
            time.sleep(0.0002)
        for i in [14, 15, 16, 17, 20, 21, 22, 23]:
            self.m4.read_values(i)
            time.sleep(0.0002)
    
    def set_speed(self, new_speed):
        if new_speed > 10000 or new_speed < self.max_speed:
            raise Exception(f'Invalid speed value {new_speed}. Should be between {self.max_speed} and 10000')
        self.speed = new_speed
        self.logger.info(f'FenixServos. Speed set to {self.speed}')
    
    #@timing
    def get_current_angles(self):
        current_angles = []
        
        for i in [2, 3, 4, 5, 8, 9, 10, 11]:
            current_angles.append(self.m1.read_angle(i))
            time.sleep(0.0002)
        for i in [14, 15, 16, 17, 20, 21, 22, 23]:
            current_angles.append(self.m4.read_angle(i))
            time.sleep(0.0002)

        self.logger.info(f'Read current angles : {current_angles}')
        
        return current_angles
    
    # return 4 angles from one board
    def get_angles_from_board(self, board):
        angles = []
        start_numbers = {self.m1 : 1, self.m2 : 5, self.m3 : 9, self.m4 : 13}
        for j in range(start_numbers[board], start_numbers[board] + 4):
            angles.append(board.read_angle(j))
        return angles

    def angles_are_close(self, target_angles):
        """
        compares self angles to target angles
        if they are different, return false
        """
        current_angles = self.get_current_angles()
        """
        j = 1
        for m in [self.m1, self.m2, self.m3, self.m4]:            
            for _ in range(4):
                current_angles.append(m.readAngle(j))
                j += 1
        print('Current angles :')
        print(current_angles)
        """
        for i in range(16):
            if abs(current_angles[i] - target_angles[i]) > 2:
                print('Angles {0} diff too big. {1}, {2}'.format(i, current_angles[i], target_angles[i]))
                return False

        return True

    def get_board_by_id(self, id):
        if id in [2, 3, 4, 5, 8, 9, 10, 11]:
            return self.m1
        elif id in [14, 15, 16, 17, 20, 21, 22, 23]:
            return self.m4
        else:
            raise ValueError(f'Bad id: {id}')

    def enable_torque(self):
        for id in self.servos:
            self.get_board_by_id(id).enable_torque(id)

    def disable_torque(self):
        for id in self.servos:
            self.get_board_by_id(id).disable_torque(id)

    def set_servo_values(self, angles, rate=0):
        print('Sending values \n{0}'.format(angles))
        #self.get_angles_diff(angles)
        j = 1
        for m in [self.m1, self.m2, self.m3, self.m4]:
            for _ in range(4):
                m.move_servo_to_angle(j, angles[j-1], rate)
                j += 1
    #@timing
    def send_command_to_servos(self, angles, rate):
        j = 0
        for i in [2, 3, 4, 5, 8, 9, 10, 11]:
            self.m1.move_servo_to_angle(i, angles[j], rate)
            time.sleep(0.0002)
            j += 1
        for i in [14, 15, 16, 17, 20, 21, 22, 23]:
            self.m4.move_servo_to_angle(i, angles[j], rate)
            time.sleep(0.0002)
            j += 1
    
    def log_servo_data(self):
        j = 1
        for m in [self.m1, self.m2, self.m3, self.m4]:
            for _ in range(4):
                self.logger.info(read_values(m, j))
                time.sleep(0.0002)
                j += 1
    
    # sends 4 angles to one board
    def send_angles_to_board(self, board, angles, rate):
        start_numbers = {self.m1 : 1, self.m2 : 5, self.m3 : 9, self.m4 : 13}
        for j in range(start_numbers[board], start_numbers[board] + 4):
            board.move_servo_to_angle(j, angles[j-1], rate)

    def set_servo_values_balancing(self, angles):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')

        self.send_command_to_servos(angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {angles}')
        prev_pitch, prev_roll = None, None
        for s in range(20):
            self.logger.info(f'Step {s}')
            
            with open('/fenix/fenix/wrk/gyroaccel_data.txt', "r") as f:
                pitch, roll = f.readline().split(',')
            pitch, roll = float(pitch), float(roll)
            self.logger.info(f"ga_data: {pitch, roll}")
            
            if prev_pitch and abs(prev_pitch) < abs(pitch):
                self.logger.info(f'Body balance moving wrong pitch {prev_pitch, pitch}. Exiting')
                return self.get_current_angles()
            if prev_roll and abs(prev_roll) < abs(roll):
                self.logger.info(f'Body balance moving wrong roll {prev_roll, roll}. Exiting')
                return self.get_current_angles()
            
            if abs(pitch) < config.fenix["balance_offset"] and abs(roll) < config.fenix["balance_offset"]:
                current_angles = self.get_current_angles()
                self.logger.info(f'current angles: {current_angles}')
                self.logger.info(f'Body balanced. Exiting')
                self.send_command_to_servos(current_angles, 0)
                return current_angles
            
            prev_pitch, prev_roll = pitch, roll
            time.sleep(0.1)
        return self.get_current_angles()

    def set_servo_values_touching(self, angles):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')

        self.send_command_to_servos(angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {angles}')
        
        for s in range(50):
            self.logger.info(f'Step {s}')
            
            with open("/fenix/fenix/wrk/neopixel_command.txt", "r") as f:
                legs_down = f.readline().split(',')[0]
            self.logger.info(f"legs_down: {legs_down}")
            if legs_down == '1111':
                current_angles = self.get_current_angles()
                self.logger.info(f'current angles: {current_angles}')
                print(f'All down. Exiting')
                self.send_command_to_servos(current_angles, 0)
                return current_angles
            
            time.sleep(0.03)
        return self.get_current_angles()
            

    #@timing
    def set_servo_values_paced(self, angles):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        prev_angles = self.get_current_angles()

        self.send_command_to_servos(angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {angles}')
        #time.sleep(0.8 * rate / 1000)
        time.sleep(0.05)
        adjustment_done = False
        
        for s in range(50):
            self.logger.info(f'Step {s}')
            #with open("/fenix/fenix/wrk/gyroaccel_data.txt", "r") as f:
            #    ga_data = f.readline()
            #self.logger.info(f"GA_DATA: {ga_data}")
            #time.sleep(0.02)
            
            current_angles = self.get_current_angles()
            self.logger.info(f'current angles: {current_angles}')
            # if diff from prev angles or target angles is small - continue
            diff_from_target = self.get_angles_diff(angles, current_angles)
            diff_from_prev = self.get_angles_diff(current_angles, prev_angles)

            self.logger.info(f'Diff from prev  : {diff_from_prev[0]}')
            self.logger.info(f'Diff from target: {diff_from_target[0]}')
     
            if diff_from_target[1] < self.diff_from_target_limit:                
                self.logger.info(f'Ready to move further')
                break
            
            elif diff_from_prev[1] < self.diff_from_prev_limit and \
                    not adjustment_done:

                if diff_from_target[1] > 2 * self.diff_from_target_limit:
                    print('-----------ALARM-----------')
                    self.logger.info('-----------ALARM-----------')
                
                self.logger.info(f'Command sent : {angles}')
                if diff_from_target[1] > self.diff_from_target_limit * 3:
                    self.logger.info(f'We"re in trouble, too large diff : {diff_from_target[1]}')
                    break
                else:
                    #adjusted_angles = [round(target + (-1.5 * diff if abs(diff) > self.diff_from_target_limit else 0), 1) for target, diff in zip(angles, diff_from_target[0])]
                    adjusted_angles = [round(target + (-1.5 * diff), 1) for target, diff in zip(angles, diff_from_target[0])]
                    
                    self.logger.info(f'Adjusting to : {adjusted_angles}')
                    adjustment_done = True
                    self.send_command_to_servos(adjusted_angles, 0)
                    #time.sleep(0.03)
                    break

            elif diff_from_prev[1] < self.diff_from_prev_limit and \
                    adjustment_done:
                self.logger.info(f'Unreachable. Moving further')
                break

            prev_angles = current_angles[:]
    
    def set_servo_values_paced_wo_adjustment(self, angles):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        
        self.send_command_to_servos(angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {angles}')
        #time.sleep(0.8 * rate / 1000)
        time.sleep(0.05)

        prev_angles = self.get_current_angles()
        for s in range(50):
            self.logger.info(f'Step {s}')
            #time.sleep(0.02)
            
            current_angles = self.get_current_angles()
            self.logger.info(f'current angles: {current_angles}')
            # if diff from prev angles or target angles is small - continue
            diff_from_target = self.get_angles_diff(angles, current_angles)
            diff_from_prev = self.get_angles_diff(current_angles, prev_angles)

            self.logger.info(f'Diff from prev  : {diff_from_prev[0]}')
            self.logger.info(f'Diff from target: {diff_from_target[0]}')
     
            if diff_from_target[1] < self.diff_from_target_limit:                
                self.logger.info(f'Ready to move further')
                break
            
            elif diff_from_prev[1] < self.diff_from_prev_limit:
                self.logger.info(f'Unreachable. Moving further')
                break

            prev_angles = current_angles[:]

    def set_servo_values_paced_wo_feedback_w_adjustment(self, angles):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        
        current_angles = self.get_current_angles()
        self.logger.info(f'current angles: {current_angles}')

        diff_from_target = self.get_angles_diff(angles, current_angles)

        adjusted_angles = [round(target + (0.1 * diff), 1) for target, diff in zip(angles, diff_from_target[0])]

        self.send_command_to_servos(adjusted_angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {adjusted_angles}')
        time.sleep(rate / 1000)
    
    def set_servo_values_paced_wo_feedback(self, angles):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        
        self.send_command_to_servos(angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {angles}')
        time.sleep(rate / 1000)
        return self.get_current_angles()
            
    def set_servo_values_not_paced(self, angles):
        # every command is executed over fixed time (1 sec for speed = 1000)
        self.send_command_to_servos(angles, int(self.speed * 0.9))
        wait_time = max(0, self.speed / 1000 - config.fenix['movement_command_advance_ms'])
        self.logger.info(f'Wait time : {wait_time}, speed : {int(self.speed * 0.9)}')
        time.sleep(wait_time)

    def set_servo_values_not_paced_v2(self, angles, prev_angles=None):
        # every command is executed over a computed time, depending on the angle
        _, max_angle_diff = self.get_angles_diff(angles, prev_angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        wait_time = max(0, rate / 1000 - config.fenix['movement_command_advance_ms'])

        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        self.logger.info(f'Wait time : {wait_time}')
        
        self.send_command_to_servos(angles, rate)
        
        time.sleep(wait_time)
        self.logger.info(f'[DIFF] Diff with target:')
        self.get_angles_diff(angles)

    def set_servo_values_overshoot(self, angles, prev_angles=None):
        angles_diff, max_angle_diff = self.get_angles_diff(angles, prev_angles)
        rate = round(max(self.speed * (1 + config.fenix['movement_overshoot_coefficient']) * max_angle_diff / 45, self.max_speed)) # speed is normalized
        wait_time = max(0, rate / 1000 - config.fenix['movement_command_advance_ms'])

        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        self.logger.info(f'Wait time : {wait_time}')

        adjusted_angles = [round(target + (config.fenix['movement_overshoot_coefficient'] * diff), 1) for target, diff in zip(angles, angles_diff)]

        self.logger.info(f'{angles} ->\n{adjusted_angles}')
        
        self.send_command_to_servos(adjusted_angles, rate)
        
        time.sleep(wait_time)
        self.logger.info(f'[DIFF] Diff from target:')
        self.get_angles_diff(angles)
    
    def set_servo_values_for_running(self, angles, rate=config.speed["run"]):
        wait_time = max(0, rate / 1000 - config.fenix['movement_command_advance_ms'])
        self.logger.info(f'Wait time : {wait_time}')
        
        self.send_command_to_servos(angles, rate)
        
        time.sleep(wait_time)
        self.logger.info(f'[DIFF] Diff from target:')
        self.get_angles_diff(angles)

    def get_angles_diff(self, target_angles, test_angles=None):
        if test_angles is None:
            test_angles = self.get_current_angles()

        angles_diff = []
        for current, target in zip(test_angles, target_angles):
            angles_diff.append(round(current - target, 2))
        max_angle_diff = max([abs(x) for x in angles_diff])
        self.logger.info(f'[DIFF] Max : {max_angle_diff}. Avg : {sum([abs(x) for x in angles_diff])/16}. Sum : {sum([abs(x) for x in angles_diff])}')
        return angles_diff, max_angle_diff


if __name__ == '__main__':
    fnx = FenixServos()

    
    fnx.set_speed(2000)
    sequence = [[65, 16, 19, 0.0, 65, 16, 19, 0.0, 65, 16, 19, 0.0, 65, 16, 19, 0.0]]
    # 19.42853486276713, -74.1493912084663, -65.2791436543008   
    for angles in sequence:     
        fnx.set_servo_values_paced(angles)
    
    fnx.print_status()

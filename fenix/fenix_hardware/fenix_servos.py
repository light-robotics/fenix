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
from copy import deepcopy

from cybernetic_core.geometry.angles import FenixPosition, build_position_from_servos

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
    
    def get_current_angles(self) -> FenixPosition:
        current_position = FenixPosition(
            leg1_gamma=self.m1.read_angle(2),
            leg1_beta=self.m1.read_angle(3),
            leg1_alpha=self.m1.read_angle(4),
            leg1_tetta=self.m1.read_angle(5),

            leg2_gamma=self.m1.read_angle(8),
            leg2_beta=self.m1.read_angle(9),
            leg2_alpha=self.m1.read_angle(10),
            leg2_tetta=self.m1.read_angle(11),

            leg3_gamma=self.m4.read_angle(14),
            leg3_beta=self.m4.read_angle(15),
            leg3_alpha=self.m4.read_angle(16),
            leg3_tetta=self.m4.read_angle(17),

            leg4_gamma=self.m4.read_angle(20),
            leg4_beta=self.m4.read_angle(21),
            leg4_alpha=self.m4.read_angle(22),
            leg4_tetta=self.m4.read_angle(23),
        )
        
        self.logger.info(f'Read current angles : {current_position}')
        
        return current_position

    def get_current_angles_old(self):
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
    
    def send_command_to_servos(self, fp: FenixPosition, rate):
        self.m1.move_servo_to_angle(2, fp.legs[1].gamma, rate)
        self.m1.move_servo_to_angle(3, fp.legs[1].beta, rate)
        self.m1.move_servo_to_angle(4, fp.legs[1].alpha, rate)
        self.m1.move_servo_to_angle(5, fp.legs[1].tetta, rate)

        self.m1.move_servo_to_angle(8, fp.legs[2].gamma, rate)
        self.m1.move_servo_to_angle(9, fp.legs[2].beta, rate)
        self.m1.move_servo_to_angle(10, fp.legs[2].alpha, rate)
        self.m1.move_servo_to_angle(11, fp.legs[2].tetta, rate)

        self.m4.move_servo_to_angle(14, fp.legs[3].gamma, rate)
        self.m4.move_servo_to_angle(15, fp.legs[3].beta, rate)
        self.m4.move_servo_to_angle(16, fp.legs[3].alpha, rate)
        self.m4.move_servo_to_angle(17, fp.legs[3].tetta, rate)

        self.m4.move_servo_to_angle(20, fp.legs[4].gamma, rate)
        self.m4.move_servo_to_angle(21, fp.legs[4].beta, rate)
        self.m4.move_servo_to_angle(22, fp.legs[4].alpha, rate)
        self.m4.move_servo_to_angle(23, fp.legs[4].tetta, rate)

    def send_command_to_servos_old(self, angles, rate):
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

    def set_servo_values_balancing_1leg(self, angles):
        return self.set_servo_values_balancing(angles)
    
    def set_servo_values_balancing_2leg(self, angles):
        return self.set_servo_values_balancing(angles, legs=2)

    def set_servo_values_balancing(self, angles, legs=1):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')

        self.send_command_to_servos(angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {angles}')
        prev_pitch, prev_roll = None, None
        for s in range(20):
            self.logger.info(f'Step {s}')
            
            try:
                with open('/fenix/fenix/wrk/gyroaccel_data.txt', "r") as f:
                    pitch, roll = f.readline().split(',')
            except ValueError as e:
                print(f'Error reading balance:\n{e}')
                continue
            pitch, roll = float(pitch), float(roll)
            self.logger.info(f"ga_data: {pitch, roll}")
            
            if prev_pitch and abs(prev_pitch) <= abs(pitch):
                self.logger.info(f'Body balance moving wrong pitch {prev_pitch, pitch}. Exiting')
                return self.get_current_angles()
            if prev_roll and abs(prev_roll) <= abs(roll):
                self.logger.info(f'Body balance moving wrong roll {prev_roll, roll}. Exiting')
                return self.get_current_angles()
            
            if legs == 1:
                condition = abs(pitch) < config.fenix["balance_offset"] or abs(roll) < config.fenix["balance_offset"]
            elif legs == 2:
                condition = abs(pitch) < config.fenix["balance_offset"] and abs(roll) < config.fenix["balance_offset"]
            
            if condition:
                current_angles = self.get_current_angles()
                self.logger.info(f'current angles: {current_angles}')
                self.logger.info(f'Body balanced. Exiting')
                self.send_command_to_servos(current_angles, 0)
                return current_angles
            
            prev_pitch, prev_roll = pitch, roll
            time.sleep(0.1)
        return self.get_current_angles()

    def set_servo_values_touching_1(self, angles):
        return self.set_servo_values_touching(angles, 1)

    def set_servo_values_touching_2(self, angles):
        return self.set_servo_values_touching(angles, 2)
    
    def set_servo_values_touching_3(self, angles):
        return self.set_servo_values_touching(angles, 3)
    
    def set_servo_values_touching_4(self, angles):
        return self.set_servo_values_touching(angles, 4)

    def set_servo_values_touching(self, angles, legnum):
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
            if len(legs_down) == 4 and legs_down[legnum - 1] == '1':
                current_angles = self.get_current_angles()
                self.logger.info(f'current angles: {current_angles}')
                print(f'{legnum} down. Exiting')
                self.send_command_to_servos(current_angles, 0)
                return current_angles

            time.sleep(0.03)
        return self.get_current_angles()

    def set_servo_values_3leg_touching(self, angles):
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
            if legs_down == '1110' or legs_down == '1101' or \
                legs_down == '1011' or legs_down == '0111':
                current_angles = self.get_current_angles()
                self.logger.info(f'current angles: {current_angles}')
                print(f'3 legs down: {legs_down}. Exiting')
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
                    adjusted_angles = [round(target + (-1.5 * diff), 1) for target, diff in zip(angles.to_servo(), diff_from_target[0])]
                    
                    self.logger.info(f'Adjusting to : {adjusted_angles}')
                    fp_adjusted = build_position_from_servos(adjusted_angles)
                    adjustment_done = True
                    self.send_command_to_servos(fp_adjusted, 0)
                    #time.sleep(0.03)
                    break

            elif diff_from_prev[1] < self.diff_from_prev_limit and \
                    adjustment_done:
                self.logger.info(f'Unreachable. Moving further')
                break

            prev_angles = deepcopy(current_angles)
    
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

    def set_servo_values_not_paced_v2(self, fp: FenixPosition, prev_fp: FenixPosition = None):
        # every command is executed over a computed time, depending on the angle
        _, max_angle_diff = self.get_angles_diff(fp, prev_fp)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        wait_time = max(0, rate / 1000 - config.fenix['movement_command_advance_ms'])

        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        self.logger.info(f'Wait time : {wait_time}')
        
        self.send_command_to_servos(fp, rate)
        
        time.sleep(wait_time)
        self.logger.info(f'[DIFF] Diff with target:')
        self.get_angles_diff(fp)

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

    def get_angles_diff(self, target_position: FenixPosition, test_position: FenixPosition = None):
        if test_position is None:
            test_position = self.get_current_angles()

        angles_diff = []
        for current, target in zip(test_position.to_servo(), target_position.to_servo()):
            angles_diff.append(round(current - target, 2))
        max_angle_diff = max([abs(x) for x in angles_diff])
        self.logger.info(f'[DIFF] Max : {max_angle_diff}. Avg : {sum([abs(x) for x in angles_diff])/16}. Sum : {sum([abs(x) for x in angles_diff])}')
        return angles_diff, max_angle_diff


if __name__ == '__main__':
    fnx = FenixServos()

    """
    fnx.set_speed(2000)
    sequence = [
        [-65.37, 30.75, -34.63, -10.55, -22.04, 37.07, 15.02, 18.02, -15.2, 42.89, 3.69, -0.26, -18.72, 38.35, 13.62, -20.17],
        [44.51882068166497, 14.398429391637588, -82.79813097435527, -20.637939780612253, -26.762858610560755, 12.719663051904275, -128.64048416277242, 26.401895199628335, -135.24095796267952, 4.079459501331462, -131.7573745682841, 13.68223214772406, 114.59728860411596, 9.837685342396234, -134.63935227779214, 33.598245106471474]
    ]
    # 19.42853486276713, -74.1493912084663, -65.2791436543008   
    for angles in sequence:     
        fnx.set_servo_values_paced(angles)
    
    #time.sleep(2)
    fnx.disable_torque()
    """
    fnx.print_status()

import time
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.lx16a import LX16A
import logging
import configs.code_config as code_config
import logging.config
logging.config.dictConfig(code_config.logger_config)


#logging.basicConfig(filename='/fenix/movement/log.log', filemode='w', format='%(asctime)s - %(message)s', level=logging.INFO)


class FenixServos:
    def __init__(self):
        self.m1 = LX16A(Port='/dev/ttyAMA0') # 5-8   # 1-4
        self.m2 = LX16A(Port='/dev/ttyAMA2') # 9-12  # 5-8
        self.m3 = LX16A(Port='/dev/ttyAMA3') # 13-16 # 9-12
        self.m4 = LX16A(Port='/dev/ttyAMA1') # 1-4   # 13-16
        self.speed = 1000
        self.max_speed = 100 # 130 # 0 is instant, 10000 is very slow
        self.diff_from_target_limit = 2.5 # 2.0 # 5.0 # when it's time to start next movement
        self.diff_from_prev_limit = 0.5 # 1.0 # start next movement if we're stuck

        self.logger = logging.getLogger('main_logger')
        
        # 0.16 sec / 60 degrees for 7.4V+
        # 0.18 sec / 60 degrees for 6V+
        # my max speed is for 45 degrees
        # that means that max speed should be 120 for 7.4V+ and 135 for 6V+

    def print_status(self):
        j = 1
        for m in [self.m1, self.m2, self.m3, self.m4]:
            for _ in range(4):
                m.read_values(j)
                j += 1
    
    def set_speed(self, new_speed):
        if new_speed > 10000 or new_speed < self.max_speed:
            raise Exception(f'Invalid speed value {new_speed}. Should be between {self.max_speed} and 10000')
        self.speed = new_speed
        self.logger.info(f'FenixServos. Speed set to {self.speed}')
    
    #@timing
    def get_current_angles(self):
        current_angles = []
        j = 1
        for m in [self.m1, self.m2, self.m3, self.m4]:            
            for _ in range(4):
                current_angles.append(m.read_angle(j))
                time.sleep(0.0002)
                j += 1
        #print('Current angles :')
        #print(current_angles)
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
        j = 1
        for m in [self.m1, self.m2, self.m3, self.m4]:
            for _ in range(4):
                m.move_servo_to_angle(j, angles[j-1], rate)
                time.sleep(0.0002)
                j += 1
    
    # sends 4 angles to one board
    def send_angles_to_board(self, board, angles, rate):
        start_numbers = {self.m1 : 1, self.m2 : 5, self.m3 : 9, self.m4 : 13}
        for j in range(start_numbers[board], start_numbers[board] + 4):
            board.move_servo_to_angle(j, angles[j-1], rate)


    #@timing
    def set_servo_values_paced(self, angles):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        
        self.send_command_to_servos(angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {angles}')
        time.sleep(0.05)
        adjustment_done = False

        prev_angles = self.get_current_angles()
        for _ in range(50):
            time.sleep(0.03)
            
            current_angles = self.get_current_angles()
            self.logger.info(f'current angles: {current_angles}')
            # if diff from prev angles or target angles is small - continue
            diff_from_target = self.get_angles_diff(angles, current_angles)
            diff_from_prev = self.get_angles_diff(current_angles, prev_angles)
            #print(f'Diff from target : {diff_from_target}')
            #print(f'Diff from prev   : {diff_from_prev}')
            #print(f'Time : {datetime.datetime.now()}')
     
            if diff_from_target[1] < self.diff_from_target_limit:
                if diff_from_target[1] > self.diff_from_target_limit + 2:
                    print('-----------ALARM-----------')
                    print(f'Diff from target : {diff_from_target}')
                    print(f'Diff from prev   : {diff_from_prev}')
                    self.logger.info('-----------ALARM-----------')
                    self.logger.info(f'Diff from target : {diff_from_target}')
                    self.logger.info(f'Diff from prev   : {diff_from_prev}')
                #print('Ready to move further')
                self.logger.info(f'Ready to move further')
                break
            
            if diff_from_target[1] > self.diff_from_target_limit and \
                diff_from_prev[1] < self.diff_from_prev_limit and \
                    not adjustment_done:
                self.logger.info(f'Diff from prev  : {diff_from_prev[0]}')
                self.logger.info(f'Diff from target: {diff_from_target[0]}')
                self.logger.info(f'Command sent : {angles}')
                if diff_from_target[1] > self.diff_from_target_limit * 3:
                    self.logger.info(f'We"re in trouble, too large diff : {diff_from_target[1]}')
                    self.logger.info(diff_from_target[0])
                else:
                    adjusted_angles = [round(target + (-diff), 1) for target, diff in zip(angles, diff_from_target[0])]
                    self.logger.info(f'Adjusting to : {adjusted_angles}')
                    adjustment_done = True
                    self.send_command_to_servos(adjusted_angles, 0)
                    time.sleep(0.05)

            prev_angles = current_angles[:]
        
        #print(f'Angles after : {self.get_current_angles()}')
        #time.sleep(0.5)
        #print(f'After sleep : {self.get_current_angles()}\n')
        

    def get_angles_diff(self, target_angles, test_angles=None):
        if test_angles is None:
            test_angles = self.get_current_angles()

        angles_diff = []
        for current, target in zip(test_angles, target_angles):
            angles_diff.append(round(current - target, 2))
        #print('Angles diff : {0}'.format(angles_diff))
        max_angle_diff = max([abs(x) for x in angles_diff])
        #print('Max angle diff : {0}'.format(max_angle_diff))
        return angles_diff, max_angle_diff


if __name__ == '__main__':
    fnx = FenixServos()
        
    fnx.set_speed(500)
    sequence = [[0.0, 60.0, 60.0, -30.0, 0.0, 60.0, 60.0, -30.0, 0.0, 60.0, 60.0, -30.0, 0.0, 60.0, 60.0, -30.0]]
    """
    sequence = [[-8.02, 18.04, 95.45, -12.6, 8.02, 18.04, 95.45, -12.6, -8.02, 18.04, 95.45, -12.6, 8.02, 18.04, 95.45, -12.6], 
    [-7.64, 0.19, 24.61, -63.58, 22.98, 14.27, 73.54, -30.73, -8.72, 16.57, 112.6, -6.97, -9.18, 18.54, 100.81, -7.73], 
    [-7.64, 17.51, 22.76, -82.75, 22.98, 14.27, 73.54, -30.73, -8.72, 16.57, 112.6, -6.97, -9.18, 18.54, 100.81, -7.73],
    [-7.64, 0.19, 24.61, -63.58, 22.98, 14.27, 73.54, -30.73, -8.72, 16.57, 112.6, -6.97, -9.18, 18.54, 100.81, -7.73],
    [-8.02, 18.04, 95.45, -12.6, 8.02, 18.04, 95.45, -12.6, -8.02, 18.04, 95.45, -12.6, 8.02, 18.04, 95.45, -12.6]]
    """
    
    for angles in sequence:     
        fnx.set_servo_values_paced(angles)
    
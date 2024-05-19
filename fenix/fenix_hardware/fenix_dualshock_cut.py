import time
import datetime
from enum import Enum
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.dualshock import DualShock
from fenix_hardware.neopixel_commands_setter import NeopixelCommandsSetter
from core.commands_writer import CommandsWriter
import configs.config as cfg
from configs.modes import NIGHT_MODE


last_write = datetime.datetime.now()
commands_read_delta = 0.2

def pause_writes(f):
    def wrapper(*args):
        global last_write
        current_time = datetime.datetime.now()
        if (current_time - last_write).total_seconds() > commands_read_delta:
            print('Not Skipping', last_write, current_time, f)
            last_write = current_time
            f(*args)
        else:
            print('Skipping', last_write, current_time)
    return wrapper

def remove_noise(f):
    def wrapper(*args):
        if abs(args[1]) > 3000:
            print('Value', args[1])
            f(*args)
        else:
            print('Noise', args[1])
    return wrapper


class FenixModes(Enum):
    WALKING = 1
    RUN     = 2
    SENTRY  = 3
    BATTLE  = 4

class FenixDualShock(DualShock):
    """
    To execute neopixel commands run fenix/run/neopixel_commands_reader.py before running this
    To execute servo commands run fenix/core/movement_processor.py AFTER running this
    """
    def __init__(self):
        self.neopixel = NeopixelCommandsSetter()
        self.connect()
        self.light_on = False
        self.started = False
        self.mode = FenixModes.RUN
        self.command_writer = CommandsWriter()
        self.command_writer.write_command('none', 1000)
        self.left_x, self.left_y, self.right_x, self.right_y = 0, 0, 0, 0

    def connect(self):
        self.neopixel.issue_command('rainbow_blue')
        super().__init__()
        self.neopixel.issue_command('blink_blue')        
        time.sleep(3)
        self.neopixel.issue_command('shutdown')

    def start(self):
        super().listen()

    def on_playstation_button_press(self):
        self.command_writer.write_command('disable_torque', 1000)
    
    def on_options_press(self):
        self.command_writer.write_command('exit', 0)
        time.sleep(0.5)
        self.command_writer.write_command('none', 1000)
    
    def on_share_press(self):
        self.command_writer.write_command('reset', 1000)

    def on_R1_press(self):
        if self.light_on:
            self.light_on = False
            self.neopixel.issue_command('light_off')
            print('Turn the lights off')
        else:
            self.light_on = True
            self.neopixel.issue_command('light_on')
            print('Turn the lights on')        
    
    def on_R2_press(self, value):
        if not self.light_on:
            # -32k to 32k -> 50 -> 255
            value1 = value + 32768
            value2 = int(50 + value1/320)
            print(value2)
            self.neopixel.issue_command('light', value=value2)
            print(f'Flashlight for {value} power')
    
    def on_R2_release(self):
        self.light_on = False
        self.neopixel.issue_command('light_off')
        print('Flashlight off')

    def on_L1_press(self):
        if self.light_on:
            self.light_on = False
            self.neopixel.issue_command('light_off')
            print('Turn the lights off')
        else:
            self.light_on = True
            self.neopixel.issue_command('dipped_headlights')
            print('Turn the dim lights on')   
    
    def on_L2_press(self, value):
        self.light_on = True
        self.neopixel.issue_command('rampage')
        print('Rampage')

    @staticmethod
    def convert_value_to_speed(value):
        """
        max_speed = 100, min_speed = 2000
        abs(value) from 0 to 32768
        """
        
        value = abs(value)
        #if value < 12000:
        #    return 1500 # > 1000 will be ignored for moving
        if value < 20000:
            return 400
        if value < 25000:
            return 300
        if value < 30000:
            return 250
        return 1000
    
    def on_up_arrow_press(self):
        if self.mode in [FenixModes.WALKING, FenixModes.SENTRY]:
            self.command_writer.write_command('up', 500)
        elif self.mode in [FenixModes.RUN]:
            self.command_writer.write_command('forward_two_legged', cfg.speed["run"])

    def on_down_arrow_press(self):
        if self.mode in [FenixModes.WALKING, FenixModes.SENTRY]:
            self.command_writer.write_command('down', 500)
        elif self.mode in [FenixModes.RUN]:
            self.command_writer.write_command('backward_two_legged', cfg.speed["run"])

    def on_right_arrow_press(self):
        if self.mode in [FenixModes.WALKING]:
            self.command_writer.write_command('turn_right_two_legged', 500)
        elif self.mode in [FenixModes.RUN]:
            self.command_writer.write_command('strafe_right_two_legged', cfg.speed["run"])

    def on_left_arrow_press(self):
        if self.mode in [FenixModes.WALKING]:
            self.command_writer.write_command('turn_left_two_legged', 500)
        elif self.mode in [FenixModes.RUN]:
            self.command_writer.write_command('strafe_left_two_legged', cfg.speed["run"])
      
    def on_up_down_arrow_release(self):
        self.command_writer.write_command('none', 500)

    def on_left_right_arrow_release(self):
        self.command_writer.write_command('none', 500)
        
    def on_x_press(self):
        self.mode = FenixModes.BATTLE
        if not NIGHT_MODE:
            self.neopixel.issue_command('steady', color='purple')
        print('Switched mode to BATTLE')

    def on_triangle_press(self):
        self.mode = FenixModes.RUN
        if not NIGHT_MODE:
            self.neopixel.issue_command('steady', color='red')
        print('Switched mode to RUN')

    def on_circle_press(self):
        self.mode = FenixModes.SENTRY
        if not NIGHT_MODE:
            self.neopixel.issue_command('steady', color='cyan')
        print('Switched mode to SENTRY')

    def on_square_press(self):
        self.mode = FenixModes.WALKING
        if not NIGHT_MODE:
            self.neopixel.issue_command('steady', color='blue')
        print('Switched mode to WALKING')

if __name__ == '__main__':
    FenixDualShock().start()

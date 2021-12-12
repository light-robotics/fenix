import time
from enum import Enum
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.dualshock import DualShock
from fenix_hardware.neopixel_commands_setter import NeopixelCommandsSetter
from core.commands_writer import CommandsWriter


class FenixModes(Enum):
    ONE_LEGGED = 1
    TWO_LEGGED = 2
    STATIONARY = 3

class FenixDualShock(DualShock):
    def __init__(self):
        self.neopixel = NeopixelCommandsSetter()
        self.connect()
        self.light_on = False
        self.started = False
        self.mode = FenixModes.TWO_LEGGED
        self.command_writer = CommandsWriter()
        self.command_writer.write_command('none', 1000)

    def connect(self):
        self.neopixel.issue_command('rainbow_blue')
        super().__init__()
        self.neopixel.issue_command('blink_blue')        
        time.sleep(3)
        self.neopixel.issue_command('shutdown')

    def start(self):
        super().listen()

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
            self.neopixel.issue_command('light', value2)
            print(f'Flashlight for {value} power')
    
    def on_R2_release(self):
        self.light_on = False
        self.neopixel.issue_command('light_off')
        print('Flashlight off')
    
    @staticmethod
    def convert_value_to_speed(value):
        """
        max_speed = 100, min_speed = 2000
        abs(value) from 0 to 32768
        """
        value = abs(value)
        if value < 4000:
            return 2000
        if value < 12000:
            return 1000
        if value < 20000:
            return 500
        if value < 25000:
            return 300
        if value < 30000:
            return 250
        return 150

    def on_L3_up(self, value):
        if self.mode == FenixModes.TWO_LEGGED:
            self.command_writer.write_command('forward_two_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.ONE_LEGGED:
            self.command_writer.write_command('forward_one_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.STATIONARY:
            self.command_writer.write_command('body_forward', 1000)
    
    def on_L3_down(self, value):
        if self.mode == FenixModes.TWO_LEGGED:
            self.command_writer.write_command('backward_two_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.ONE_LEGGED:
            self.command_writer.write_command('backward_one_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.STATIONARY:
            self.command_writer.write_command('body_backward', 1000)

    def on_L3_left(self, value):
        if self.mode == FenixModes.TWO_LEGGED:
            self.command_writer.write_command('strafe_left_two_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.ONE_LEGGED:
            self.command_writer.write_command('strafe_left_one_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.STATIONARY:
            self.command_writer.write_command('body_left', 1000)

    def on_L3_right(self, value):
        if self.mode == FenixModes.TWO_LEGGED:
            self.command_writer.write_command('strafe_right_two_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.ONE_LEGGED:
            self.command_writer.write_command('strafe_right_one_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.STATIONARY:
            self.command_writer.write_command('body_right', 1000)
    
    def on_L3_press(self):
        if self.mode == FenixModes.STATIONARY:
            self.command_writer.write_command('body_to_center', 1000)
    
    def on_L3_y_at_rest(self):
        self.command_writer.write_command('none', 1000)

    def on_R3_up(self, value):
        if self.mode in [FenixModes.ONE_LEGGED, FenixModes.TWO_LEGGED]:
            self.command_writer.write_command('up', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.STATIONARY:
            self.command_writer.write_command('look_up', 300)
    
    def on_R3_down(self, value):
        if self.mode in [FenixModes.ONE_LEGGED, FenixModes.TWO_LEGGED]:
            self.command_writer.write_command('down', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.STATIONARY:
            self.command_writer.write_command('look_down', 300)
    
    def on_R3_left(self, value):
        if self.mode == FenixModes.TWO_LEGGED:
            self.command_writer.write_command('turn_left_two_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.ONE_LEGGED:
            self.command_writer.write_command('turn_left_one_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.STATIONARY:
            self.command_writer.write_command('look_left', 1000)
    
    def on_R3_right(self, value):
        if self.mode == FenixModes.TWO_LEGGED:
            self.command_writer.write_command('turn_right_two_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.ONE_LEGGED:
            self.command_writer.write_command('turn_right_one_legged', self.convert_value_to_speed(value))
        elif self.mode == FenixModes.STATIONARY:
            self.command_writer.write_command('look_right', 1000)

    def on_R3_press(self):
        if self.mode == FenixModes.STATIONARY:
            self.command_writer.write_command('sight_to_normal', 1000)
    
    def on_R3_y_at_rest(self):
        self.command_writer.write_command('none', 1000)

    def on_right_arrow_press(self):
        self.command_writer.write_command('reposition_x_up', 500)

    def on_left_arrow_press(self):
        self.command_writer.write_command('reposition_x_down', 500)
      
    def on_up_arrow_press(self):
        self.command_writer.write_command('reposition_y_up', 500)

    def on_down_arrow_press(self):
        self.command_writer.write_command('reposition_y_down', 500)
    
    def on_x_press(self):
        if self.started:
            self.command_writer.write_command('exit', 0)
        else:
            self.started = True
            self.command_writer.write_command('start', 1000)

    def on_triangle_press(self):
        self.mode = FenixModes.TWO_LEGGED
        print('Switched mode to TWO_LEGGED')

    def on_circle_press(self):
        self.mode = FenixModes.STATIONARY
        print('Switched mode to STATIONARY')

    def on_square_press(self):
        self.mode = FenixModes.ONE_LEGGED
        print('Switched mode to ONE_LEGGED')

if __name__ == '__main__':
    FenixDualShock().start()

import time
import datetime
from enum import Enum
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.dualsense import DualSense
from fenix_hardware.neopixel_commands_setter import NeopixelCommandsSetter
from core.commands_writer import CommandsWriter
import configs.config as cfg
from configs.modes import NIGHT_MODE


class FenixModes(Enum):
    WALKING = 1
    RUN     = 2
    SENTRY  = 3
    BATTLE  = 4

class FenixDualSense(DualSense):
    """
    To execute neopixel commands run fenix/run/neopixel_commands_reader.py before running this
    To execute servo commands run fenix/core/movement_processor.py AFTER running this
    """
    def __init__(self):
        self.neopixel = NeopixelCommandsSetter()
        self.command_writer = CommandsWriter()
        self.command_writer.write_command('none', 1000)
        self.connect()
        self.light_on = False
        self.started = False
        self.mode = FenixModes.WALKING
        self.left_x, self.left_y, self.right_x, self.right_y = 0, 0, 0, 0

    def connect(self):
        self.neopixel.issue_command('rainbow_blue')
        super().__init__()
        self.neopixel.issue_command('blink_blue')        
        time.sleep(3)
        self.neopixel.issue_command('shutdown')
    
    def on_cross_btn_pressed(self):
        print('External class X pressed')

if __name__ == '__main__':
    fds = FenixDualSense()
    while fds.is_running:
        time.sleep(0.1)

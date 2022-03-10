# Should be run with sudo, else Neopixel is not working
import time
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.neopixel import Neopixel
from configs import code_config


class NeopixelCommandsReader():

    command_file = code_config.neopixel_command_file

    def __init__(self):
        self.neopixel = Neopixel()

    def read(self):
        prev_command = None
        while True:
            try:
                with open(self.command_file, 'r') as f:
                    command = f.readline()
                
                if prev_command == command:
                    time.sleep(0.1)
                    continue

                contents = command.split(',')                
                
                if len(contents) != 3:
                    time.sleep(0.1)
                    continue
                
                mode, color, brightness = contents
                self.neopixel.activate_mode(mode, color, int(brightness))
                prev_command = command
                time.sleep(0.1)
            except KeyboardInterrupt:
                self.neopixel.shutdown()
                break

if __name__ == '__main__':
    NeopixelCommandsReader().read()

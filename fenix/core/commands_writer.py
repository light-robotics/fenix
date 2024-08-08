import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from configs import code_config
from configs import config as cfg


class CommandsWriter:
    symbols = {
        'w' : 'forward',
        's' : 'backward',
        'r' : 'up',
        'f' : 'down',
        'a' : 'turn_left',
        'd' : 'turn_right',
        'q' : 'strafe_left',
        'e' : 'strafe_right',
        'aw': 'turn_forward_left',
        'dw': 'turn_forward_right',
        'wq': 'forward_left',
        'we': 'forward_right',
        'x' : 'start',
        'z' : 'exit',
        '`' : 'none',
        '1' : 'disable_torque',
        't' : 'look_up',
        'g' : 'look_down'
    }

    def __init__(self):
        self.command_file = code_config.movement_command_file
        self.command_id = 1
        self.prev_command = 'none'
        self.prev_speed = 1000

    def write_command(self, command: str, speed: int) -> None:
        if self.prev_command == command and self.prev_speed == speed:
            return
        if speed > 2000:
            print(f'Ignoring command {command}, {speed}')
        else:
            self.prev_command = command
            self.prev_speed = speed
            print(f'writing {command}, {speed} to command file')
            with open(self.command_file, 'w') as f:
                f.write(f'{self.command_id},{command},{speed}')
                self.command_id += 1
    
    def initiate_local_writer(self) -> None:
        try:
            while True:
                symbol = input('Enter command:\n')
                command = self.symbols.get(symbol, 'none')
                speed = 500
                if command in [
                    'forward_two_legged',
                    'backward_two_legged',
                    'strafe_right_two_legged',
                    'strafe_left_two_legged',
                    ]:
                    speed = cfg.speed["run"]
                self.write_command(command, speed)
                if command == 'exit':
                    break

        except KeyboardInterrupt:
            self.write_command('exit')

if __name__ == '__main__':
    CommandsWriter().initiate_local_writer()

from configs import code_config


class CommandsWriter:
    symbols = {
        'w' : 'forward',
        's' : 'backward',
        'r' : 'up',
        't' : 'down',
        'a' : 'turn_left',
        'd' : 'turn_right',
        'q' : 'strafe_left',
        'e' : 'strafe_right',
        'aw': 'turn_forward_left',
        'dw': 'turn_forward_right',
        'wq': 'forward_left',
        'we': 'forward_right',
        'x' : 'start',
        'z' : 'exit'
    }

    def __init__(self):
        self.command_file = code_config.movement_command_file
        self.command_id = 1

    def write_command(self, command: str, speed: int) -> None:
        if speed > 1000:
            print(f'Ignoring command {command}, {speed}')
        else:
            print(f'writing {command}, {speed} to command file')
            with open(self.command_file, 'w') as f:
                f.write(f'{self.command_id},{command},{speed}')
                self.command_id += 1
    
    def initiate_local_writer(self) -> None:
        try:
            while True:
                symbol = input('Enter command:\n')
                command = self.symbols.get(symbol, 'none')
                self.write_command(command, 500)
                if command == 'exit':
                    break

        except KeyboardInterrupt:
            self.write_command('exit')

if __name__ == '__main__':
    CommandsWriter().initiate_local_writer()

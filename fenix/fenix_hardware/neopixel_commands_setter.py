from configs import code_config


class NeopixelCommandsSetter:
    neopixel_command_file = code_config.neopixel_command_file
    
    def __init__(self):
        pass

    def write_command(self, command: str):
        with open(self.neopixel_command_file, 'w') as f:
            f.write(command)
    
    def issue_command(self, command_in: str, color: str = '', value: int = 0):
        commands_mapper = {
            'light_on'          : 'flashlight,white,255',
            'light_off'         : 'steady,white,0',
            'light'             : f'flashlight,white,{value}',
            'dipped_headlights' : f'flashlight,dim-white,150',
            'rainbow_blue'      : 'rainbow,blue,255',
            'blink_blue'        : 'blink,blue,255',
            'shutdown'          : 'shutdown,white,0',
            'activation'        : 'activation,blue,120',
            'rampage'           : 'steady,red,255',
            'steady'            : f'steady,{color},255',
            'running_diodes'    : f'running_diodes,{color},255'
        }
        if command_in.isnumeric():
            command_out = f'{command_in},white,255'
        else:
            command_out = commands_mapper[command_in]
        
        self.write_command(command_out)

class NeopixelCommandsSetter:
    neopixel_command_file = '//fnx//fenix//wrk//neopixel_command.txt'
    
    def __init__(self):
        pass

    def write_command(self, command: str):
        with open(self.neopixel_command_file, 'w') as f:
            f.write(command)
    
    def issue_command(self, command_in: str, value: int = 0):
        commands_mapper = {
            'light_on'     : 'flashlight,white,255',
            'light_off'    : 'steady,white,0',
            'light'        : f'flashlight,white,{value}',
            'rainbow_blue' : 'rainbow,blue,255',
            'blink_blue'   : 'blink,blue,255',
            'shutdown'     : 'shutdown,white,0'
        }
        command_out = commands_mapper[command_in]
        
        self.write_command(command_out)

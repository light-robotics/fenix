import subprocess
import os
import tempfile
from time import sleep

from dualsense_controller import DualSenseController


class DualSense():
    mac_address = '10:18:49:4B:97:4F'
    device_address = '/dev/input/js0'

    def __init__(self):
        self.connect_ds5()   
        # list availabe devices and throw exception when tzhere is no device detected
        device_infos = DualSenseController.enumerate_devices()
        if len(device_infos) < 1:
            raise Exception('No DualSense Controller available.')

        # flag, which keeps program alive
        self.is_running = True

        # create an instance, use fiŕst available device
        self.controller = DualSenseController()
        #self.init_listeners()
        self.controller.activate()
        self.controller.on_error(self.on_error)

    def connect_ds5(self):
        status = subprocess.call(f'ls {self.device_address} 2>/dev/null', shell=True)
        while status == 2:
            print(f"Controller not connected. Status {status}. Please hold PS and share for 5 seconds")

            sleep(6)        
            scriptFile = tempfile.NamedTemporaryFile(delete=True)
            with open(scriptFile.name, 'w') as f:
                f.write('#!/bin/bash\n')
                f.write('bluetoothctl << EOF\n')
                f.write(f'pair {self.mac_address}\n')
                f.write(f'connect {self.mac_address}\n')
                f.write(f'trust {self.mac_address}\n')
                f.write('EOF')
            os.chmod(scriptFile.name, 0o777)
            scriptFile.file.close()
            subprocess.call(scriptFile.name, shell=True)
            sleep(1)
            status = subprocess.call(f'ls {self.device_address} 2>/dev/null', shell=True)
        print('Controller connected')
        sleep(1)

    # switches the keep alive flag, which stops the below loop
    def stop(self):
        self.is_running = False
        self.controller.deactivate()

    def on_error(self, error):
        print(f'Opps! an error occured: {error}')
        self.stop()

if __name__ == '__main__':
    ds = DualSense()    
    while ds.is_running:
        sleep(0.001)
    
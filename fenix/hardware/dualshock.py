import subprocess
import time
import os
import tempfile
from pyPS4Controller.controller import Controller

class DualShock(Controller):
    """
    Before running this you have to manually connect your device, make it trusted,
    find its mac address and path for its input device
    I was using this tutorial: https://salamwaddah.com/blog/connecting-ps4-controller-to-raspberry-pi-via-bluetooth
    And some information from here: https://pypi.org/project/pyPS4Controller/
    override all the commands you need, like on_x_press or on_R2_release
    """
    mac_address = 'A4:53:85:81:D5:6A'
    device_address = '/dev/input/js0'

    def __init__(self):
        self.connect_ds4()        
        Controller.__init__(self, interface=self.device_address, connecting_using_ds4drv=False)

    def connect_ds4(self):
        status = subprocess.call(f'ls {self.device_address} 2>/dev/null', shell=True)
        while status == 2:
            print(f"Controller not connected. Status {status}. Please hold PS and share for 5 seconds")

            time.sleep(6)        
            scriptFile = tempfile.NamedTemporaryFile(delete=True)
            with open(scriptFile.name, 'w') as f:
                f.write('#!/bin/bash\n')
                f.write('bluetoothctl << EOF\n')
                f.write(f'connect {self.mac_address}\n')
                f.write('EOF')
            os.chmod(scriptFile.name, 0o777)
            scriptFile.file.close()
            subprocess.call(scriptFile.name, shell=True)
            time.sleep(1)
            status = subprocess.call(f'ls {self.device_address} 2>/dev/null', shell=True)
        print('Controller connected')
        time.sleep(1)


if __name__ == '__main__':
    controller = DualShock()
    controller.listen()

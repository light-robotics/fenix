import sys
import os
from datetime import datetime


sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.fnx_vl531x import FnxVL51L1X
from fenix_hardware.neopixel_commands_setter import NeopixelCommandsSetter

tof = FnxVL51L1X()
neopixel = NeopixelCommandsSetter()

tof_arr = []
for i in range(5):
    tof_arr.append(tof.get_data())

avg_arr = sum(tof_arr)/len(tof_arr)
print(f'AVG: {avg_arr}')

while True:
    tof_data = tof.get_data()
    print(f'{datetime.now()}. TOF: {tof_data}')
    if tof_data < avg_arr:
        neopixel.issue_command('light_on')
    else:
        neopixel.issue_command('light_off')

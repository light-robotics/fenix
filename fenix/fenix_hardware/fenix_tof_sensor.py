
import sys
import os
import math

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.fnx_vl531x import FnxVL51L1X

height = 115
offset = 20

def calculate_target(angle):
    angle *= -1
    if angle >= 90:
        print(f'Value: {math.tan(math.radians(angle))}')
        return height + offset / math.tan(math.radians(angle))
    print(f'Value2: {math.tan(math.radians(angle - 90))}')
    return height - offset * math.tan(math.radians(angle - 90))

class FenixTofs:
    def __init__(self):
        self.tofs: list[FnxVL51L1X] = [
            FnxVL51L1X(i2c_bus=1, i2c_address=0x29),
            FnxVL51L1X(i2c_bus=1, i2c_address=0x2a)
        ]

    def get_data(self, sensor_num):
        return self.tofs[sensor_num].get_averaged_data() #get_data()
    
    def calculate_touch(self, sensor_num, angle):
        target = calculate_target(angle)
        data = self.get_data(sensor_num)
        print(f'Angle: {angle}. Target: {target}. Data: {data}')
        return round((data - 115)/10)


if __name__ == '__main__':
    tof1 = FnxVL51L1X(i2c_bus=1, i2c_address=0x29)
    tof2 = FnxVL51L1X(i2c_bus=1, i2c_address=0x2a)

    fts = FenixTofs([tof1, tof2])
    for i in range(3):
        print(fts.get_data(0))
        print(fts.get_data(1))

#!/usr/bin/env python

import time
import datetime

from VL53L1X import VL53L1X


class FnxVL51L1X(VL53L1X):
    coef = 0.91 # 0.88
    def __init__(self, i2c_bus=1, i2c_address=0x29):
        super().__init__(i2c_bus, i2c_address)
        self.open()
        self.start_ranging(1)
        self.get_distance() # first scan is garbage
    
    def get_data(self):
        for _ in range(3):
            data = round(self.get_distance()*self.coef, 1)
            if data > 0:
                return data
        return None

    def get_averaged_data(self):
        data = []
        for _ in range(3):
            data.append(self.get_data())
        data.sort()
        print(f'Scanned data: {data}')
        return data[1]
    
    def __del__(self):
        self.stop_ranging()

if __name__ == '__main__':
    change_address = False
    fnx_tof = FnxVL51L1X(i2c_address=0x2A)
    for i in range(3):
        print(datetime.datetime.now())
        print(fnx_tof.get_data())


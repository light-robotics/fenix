#!/usr/bin/env python

import time
import datetime

from VL53L1X import VL53L1X


class FnxVL51L1X(VL53L1X):
    def __init__(self, i2c_bus=1, i2c_address=0x29):
        super().__init__(i2c_bus, i2c_address)
        self.open()
        self.start_ranging(1)
        self.get_distance() # first scan is garbage
    
    def get_data(self):
        return round(self.get_distance()*0.92, 1)
    
    def __del__(self):
        self.stop_ranging()

if __name__ == '__main__':
    fnx_tof = FnxVL51L1X()
    for i in range(3):
        print(datetime.datetime.now())
        print(fnx_tof.get_data())

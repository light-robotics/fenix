from ina219 import INA219
from ina219 import DeviceRangeError
import time

class FNX_INA219:
    SHUNT_OHMS = 0.1

    def __init__(self):
        self.ina1 = INA219(self.SHUNT_OHMS, address=0x40)
        self.ina1.configure()
        self.ina2 = INA219(self.SHUNT_OHMS, address=0x41)
        self.ina2.configure()
        self.ina3 = INA219(self.SHUNT_OHMS, address=0x44)
        self.ina3.configure()
        self.ina4 = INA219(self.SHUNT_OHMS, address=0x45)
        self.ina4.configure()
    
    def read(self):
        try:
            return {
                    1 : round(self.ina1.current(), 2),
                    2 : round(self.ina2.current(), 2),
                    3 : round(self.ina3.current(), 2),
                    4 : round(self.ina4.current(), 2)
            }
        except DeviceRangeError as e:
            # Current out of device range with specified shunt resistor
            return(e)
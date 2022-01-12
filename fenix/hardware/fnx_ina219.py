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
        result = {}
        for i, ina in zip([1, 2, 3, 4], [self.ina1, self.ina2, self.ina3, self.ina4]):
            try:
                result[i] = round(ina.current(), 2)
            except DeviceRangeError as e:
            # Current out of device range with specified shunt resistor
                result[i] = 'over 9000'
        return result        

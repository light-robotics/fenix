# Should be run with sudo, else Neopixel is not working
import time
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.neopixel import Neopixel


# @reboot sudo /fenix/venv/bin/python /fenix/fenix/run/startup_neopixel.py &
if __name__ == '__main__':
    time.sleep(10.0)
    neopixel = Neopixel()
    neopixel.activate_mode('steady', 'green', 100)
    time.sleep(10.0)
    neopixel.shutdown()

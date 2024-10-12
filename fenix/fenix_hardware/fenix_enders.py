import RPi.GPIO as GPIO
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from fenix_hardware.neopixel_commands_setter import NeopixelCommandsSetter
import time


def button_changed_left(button):
    global left_down
    if GPIO.input(button) == GPIO.LOW:
        print("Left leg down.")
        left_down = True
    else:
        print("Left leg up.")
        left_down = False

def button_changed_right(button):
    global right_down
    if GPIO.input(button) == GPIO.LOW:
        print("Right leg down.")
        right_down = True
    else:
        print("Right leg up.")
        right_down = False


class FenixEnders():
    BUTTON_LEFT = 24
    BUTTON_RIGHT = 23

    def __init__(self):
        self.neopixel = NeopixelCommandsSetter()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.BUTTON_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.BUTTON_LEFT, GPIO.BOTH, callback=button_changed_left, bouncetime=10)

        GPIO.setup(self.BUTTON_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.BUTTON_RIGHT, GPIO.BOTH, callback=button_changed_right, bouncetime=10)

    def run(self):
        global left_down, right_down
        print("Press CTRL-C to exit.")
        try:
            while True:
                if left_down and right_down:
                    self.neopixel.issue_command('steady', color='purple')
                elif left_down:
                    self.neopixel.issue_command('steady', color='red')
                elif right_down:
                    self.neopixel.issue_command('steady', color='blue')
                else:
                    self.neopixel.issue_command('shutdown')
                time.sleep(0.01)

        finally:
            GPIO.cleanup()

if __name__ == '__main__':
    global left_down, right_down
    left_down, right_down = False, False
    fe = FenixEnders().run()

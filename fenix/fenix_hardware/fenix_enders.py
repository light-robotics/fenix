import RPi.GPIO as GPIO
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from fenix_hardware.neopixel_commands_setter import NeopixelCommandsSetter
import time


def button_changed_front_left(button):
    global front_left_down
    if GPIO.input(button) == GPIO.LOW:
        print("Left leg down.")
        front_left_down = True
    else:
        print("Left leg up.")
        front_left_down = False

def button_changed_front_right(button):
    global front_right_down
    if GPIO.input(button) == GPIO.LOW:
        print("Right leg down.")
        front_right_down = True
    else:
        print("Right leg up.")
        front_right_down = False

def button_changed_back_right(button):
    global back_right_down
    if GPIO.input(button) == GPIO.LOW:
        print("Btn3 leg down.")
        back_right_down = True
    else:
        print("Btn3 leg up.")
        back_right_down = False

def button_changed_back_left(button):
    global back_left_down
    if GPIO.input(button) == GPIO.LOW:
        print("Btn4 leg down.")
        back_left_down = True
    else:
        print("Btn4 leg up.")
        back_left_down = False


class FenixEnders():
    FRONT_LEFT = 24
    FRONT_RIGHT = 23
    BACK_RIGHT = 27
    BACK_LEFT = 22

    def __init__(self):
        self.neopixel = NeopixelCommandsSetter()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.FRONT_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.FRONT_LEFT, GPIO.BOTH, callback=button_changed_front_left, bouncetime=10)

        GPIO.setup(self.FRONT_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.FRONT_RIGHT, GPIO.BOTH, callback=button_changed_front_right, bouncetime=10)

        GPIO.setup(self.BACK_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.BACK_RIGHT, GPIO.BOTH, callback=button_changed_back_right, bouncetime=10)

        GPIO.setup(self.BACK_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.BACK_LEFT, GPIO.BOTH, callback=button_changed_back_left, bouncetime=10)

    def run(self):
        global front_left_down, right_down
        prev_value = None
        print("Press CTRL-C to exit.")
        try:
            while True:
                result_legs = ''
                if front_left_down:
                    result_legs += '1'
                else:
                    result_legs += '0'

                if front_right_down:
                    result_legs += '1'
                else:
                    result_legs += '0'

                if back_right_down:
                    result_legs += '1'
                else:
                    result_legs += '0'

                if back_left_down:
                    result_legs += '1'
                else:
                    result_legs += '0'

                if result_legs != prev_value:
                    self.neopixel.issue_command(result_legs)
                prev_value = result_legs
                time.sleep(0.01)

        finally:
            GPIO.cleanup()

if __name__ == '__main__':
    global front_left_down, front_right_down, back_right_down, back_left_down
    front_left_down, front_right_down, back_right_down, back_left_down = False, False, False, False
    fe = FenixEnders().run()

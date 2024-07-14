"""
A version of neopixel led strip for Fenix quadruped robot
This file written by Light Robotics (light.robotics.2020@gmail.com)
Used library written by Tony DiCola (tony@tonydicola.com)
!!! Script should be run with sudo !!!
"""

import argparse
import time
from rpi_ws281x import PixelStrip, Color

class Neopixel:
    # LED strip configuration:
    LED_COUNT = 18        # Number of LED pixels.
    LED_PIN = 18          # GPIO pin connected to the pixels (18 uses PWM!).
    # LED_PIN = 10        # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
    LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
    LED_DMA = 10          # DMA channel to use for generating signal (try 10)
    LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
    LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
    LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

    colors = {
        'red'    : Color(255, 0, 0),
        'green'  : Color(0, 255, 0),
        'blue'   : Color(0, 0, 255),
        'white'  : Color(255, 255, 255),
        'none'   : Color(0, 0, 0),

        'yellow' : Color(204, 102, 0),        
        'cyan'   : Color(0, 255, 255),
        'purple' : Color(218, 112, 214),
        'orange' : Color(255, 140, 0),
        'redfox' : Color(195, 88, 23),
        'midred' : Color(148, 56, 58),
        'test'   : Color(130, 70, 50),

        'dim-white'    : Color(125, 125, 125),
        'dim-blue'     : Color(0, 0, 125),
        'white-yellow' : Color(255, 227, 200),
    }

    def __init__(self):
        self.strip = PixelStrip(self.LED_COUNT, 
                                self.LED_PIN, 
                                self.LED_FREQ_HZ, 
                                self.LED_DMA, 
                                self.LED_INVERT, 
                                self.LED_BRIGHTNESS, 
                                self.LED_CHANNEL)

        # Intialize the library (must be called once before other functions).
        self.strip.begin()        

    def activate_mode(self, mode: str, color_str: str, brightness: int) -> None:
        print(f'Activating mode {mode} - {color_str} - {brightness}')
        color = self.colors[color_str]
        #self.strip.setBrightness(brigthness)

        if mode == 'shutdown':
            self.shutdown()
        
        if mode == 'steady':
            self.steady_color(color, brightness)
        if mode == 'blink':
            self.blink(color, brightness)
        if mode == 'flashlight':
            self.flashlight(color, brightness)
        if mode == 'rainbow':
            self.rainbow(brightness)
        if mode == 'theater':
            self.theater_chase(color, brightness)
        if mode == 'rainbow_cycle':
            self.rainbow_cycle(brightness, wait_ms=2, iterations=20)
        if mode == 'theater_rainbow':
            self.theater_chase_rainbow(brightness)
        if mode == 'activation':
            self.activation(color, brightness)
        if mode == 'running_diodes':
            self.running_diodes(color, brightness)

    def mode_cycle(self, mode: str, color_str: str, brightness: int) -> None:
        while True:
            self.activate_mode(mode, color_str, brightness)
            time.sleep(1.0)
    
    def shutdown(self):
        self.strip.setBrightness(0)
        self.strip.show()

    def steady_color(self, color: Color, brightness: int = 255, wait_ms: int = 0) -> None:
        # wait_ms introduces delay between pixels turning on
        self.strip.setBrightness(brightness)
        #print(f'Setting brightness to {brightness}')

        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            #print(f'Pixel {i} color set to {color}')
            self.strip.show()
            time.sleep(wait_ms / 1000.0)
    
    def blink(self, color: Color, brightness: int = 255, wait_ms: int = 10) -> None:
        self.steady_color(color)
        self.strip.setBrightness(brightness)
        #print(f'Setting brightness to {brightness}')

        for j in range(255):
            self.strip.setBrightness(255-j)
            self.strip.show()
            time.sleep(wait_ms / 1000.0)
        for j in range(255):
            self.strip.setBrightness(j)
            self.strip.show()
            time.sleep(wait_ms / 1000.0)        
    
    def flashlight(self, color: Color = colors['white'], brightness: int = 255) -> None:
        self.strip.setBrightness(brightness)
        #print(f'Setting brightness to {brightness}')

        for i in range(0, self.LED_COUNT):
            if i < 8: # first 6 are front flashlight
                self.strip.setPixelColor(i, color)
            else:
                self.strip.setPixelColor(i, self.colors['none'])
        self.strip.show()

    def activation(self, color: Color = colors['white'], brightness: int = 255) -> None:
        self.steady_color(self.colors['none'], 0)
        self.strip.setBrightness(0)
        #print(f'Setting brightness to 0')
        time.sleep(0.3)

        self.strip.setBrightness(brightness)
        #print(f'Setting brightness to {brightness}')
        for pixels in [[3, 4], [2, 5], [1, 6], [0, 7]]:
            self.strip.setPixelColor(pixels[0], color)
            self.strip.setPixelColor(pixels[1], color)
            self.strip.show()
            time.sleep(0.7)        

        time.sleep(1.0)

        for pixels in [[8, 17], [9, 16], [10, 15], [11, 14], [12, 13]]:
            self.strip.setPixelColor(pixels[0], color)
            self.strip.setPixelColor(pixels[1], color)
            self.strip.show()
            time.sleep(0.2)
        
        for pixels in [[3, 4], [2, 5], [1, 6], [0, 7], [8, 17], [9, 16], [10, 15], [11, 14], [12, 13]]:
            self.strip.setPixelColor(pixels[0], self.colors['none'])
            self.strip.setPixelColor(pixels[1], self.colors['none'])
            self.strip.show()
            time.sleep(0.1)
    
    def running_diodes(self, 
            color: Color = colors['white'], 
            brightness: int = 255,
            num_cycles: int = 10,
            wait_time: float = 0.05,
        ) -> None:
        self.steady_color(self.colors['none'], 0)
        self.strip.setBrightness(0)
        #print(f'Setting brightness to 0')
        time.sleep(0.3)

        diodes_pairs = [
            [3, 4], [2, 5], [1, 6], [0, 7], [8, 17], [9, 16], [10, 15], [11, 14], [12, 13]
        ]

        self.strip.setBrightness(brightness)
        #print(f'Setting brightness to {brightness}')
        for i in range(num_cycles):
            for index in range(len(diodes_pairs) + 1):
                if index < len(diodes_pairs):
                    self.strip.setPixelColor(diodes_pairs[index][0], color)
                    self.strip.setPixelColor(diodes_pairs[index][1], color)
                if index > 0:
                    self.strip.setPixelColor(diodes_pairs[index-1][0], self.colors['none'])
                    self.strip.setPixelColor(diodes_pairs[index-1][1], self.colors['none'])
                self.strip.show()
                time.sleep(wait_time)        

    def rainbow(self, brightness: int = 255, wait_ms: int = 20, iterations: int = 2) -> None:
        """Draw rainbow that fades across all pixels at once."""
        self.strip.setBrightness(brightness)
        #print(f'Setting brightness to {brightness}')

        for j in range(256 * iterations):
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, self._wheel((i + j) & 255))
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    @staticmethod
    def _wheel(pos: int) -> Color:
        """Generate rainbow colors across 0-255 positions."""
        
        if pos < 85:
            return Color(pos * 3, 255 - pos * 3, 0)
        elif pos < 170:
            pos -= 85
            return Color(255 - pos * 3, 0, pos * 3)
        else:
            pos -= 170
            return Color(0, pos * 3, 255 - pos * 3)
    
    def theater_chase(self, color: Color, brightness: int = 255, wait_ms:int = 50, iterations:int = 10) -> None:
        """Movie theater light style chaser animation."""
        self.strip.setBrightness(brightness)
        #print(f'Setting brightness to {brightness}')

        for _ in range(iterations):
            for q in range(3):
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i + q, color)
                self.strip.show()
                time.sleep(wait_ms / 1000.0)
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i + q, 0)

    def rainbow_cycle(self, brightness: int = 255, wait_ms: int = 20, iterations: int = 5) -> None:
        """Draw rainbow that uniformly distributes itself across all pixels."""
        self.strip.setBrightness(brightness)
        print(f'Setting brightness to {brightness}')

        for j in range(256 * iterations):
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, self._wheel(
                    (int(i * 256 / self.strip.numPixels()) + j) & 255))
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def theater_chase_rainbow(self, brightness: int = 255, wait_ms: int = 50) -> None:
        """Rainbow movie theater light style chaser animation."""
        self.strip.setBrightness(brightness)
        print(f'Setting brightness to {brightness}')

        for j in range(256):
            for q in range(3):
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i + q, self._wheel((i + j) % 255))
                self.strip.show()
                time.sleep(wait_ms / 1000.0)
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i + q, 0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Neopixel strip. Script should be run with sudo')
    parser.add_argument('-m', dest='mode', type=str, default='steady', 
                        help='steady, blink, flashlight, rainbow, theater, rainbow_cycle, theater_rainbow')
    parser.add_argument('-c', dest='color', type=str, default='white', help='for example, white or red')
    parser.add_argument('-b', dest='brightness', type=int, default=255, help='50 -> 255')
    args = parser.parse_args()
    assert 50 <= args.brightness <= 255

    fenix_neopixel = Neopixel()
    try:
        fenix_neopixel.mode_cycle(args.mode, args.color, args.brightness)
    except KeyboardInterrupt:
        fenix_neopixel.shutdown()
    
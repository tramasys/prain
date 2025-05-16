#import RPi.GPIO as GPIO
from utils.gpio_compat import GPIO
import time

class PWMBuzzer:
    _FREQ = {
        'F#4': 370,
        'A#4': 466,
        'C#5': 554,
        'E5': 659,
    }

    def __init__(self, pin: int = 12, duty: int = 50) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        self._pin = pin
        self._duty = duty
        self._pwm = GPIO.PWM(pin, 1)
        self._pwm.stop()

    def play_goal(self) -> None:
        for f, d in [(880, .15), (1100, .15), (1320, .30)]:
            self._pwm.ChangeFrequency(f)
            self._pwm.start(self._duty)
            time.sleep(d)
            self._pwm.stop()
            time.sleep(.05)

    def play_rick_intro(self) -> None:
        seq = [
            ('F#4', .15), ('A#4', .15), ('C#5', .15), ('A#4', .15),
            ('F#4', .15), ('F#4', .15),
            ('F#4', .15), ('A#4', .15), ('C#5', .15), ('A#4', .15),
            ('F#4', .15), ('E5',  .30),
        ]
        for n, d in seq:
            self._pwm.ChangeFrequency(self._FREQ[n])
            self._pwm.start(self._duty)
            time.sleep(d)
            self._pwm.stop()
            time.sleep(.05)

    def stop(self) -> None:
        self._pwm.stop()
        GPIO.cleanup(self._pin)

#import RPi.GPIO as GPIO
from utils.gpio_compat import GPIO
import time
import math

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

    def play_meow(self, total: float = 0.4) -> None:
        step = 0.01
        steps = int(total / step)
        phase1 = int(steps * 0.3)  # Initial drop (30% of time)
        phase2 = int(steps * 0.5)  # Vibrato middle (50% of time)
        phase3 = steps - phase1 - phase2  # Final rise (20% of time)
        self._pwm.start(self._duty)

        # Phase 1: Quick drop from 1000 Hz to 600 Hz
        for i in range(phase1):
            f = 1000 - (400 * i / phase1)
            self._pwm.ChangeFrequency(f)
            time.sleep(step)

        # Phase 2: Vibrato around 600 Hz (±50 Hz, 10 Hz oscillation)
        for i in range(phase2):
            f = 600 + 50 * math.sin(10 * 2 * math.pi * i / phase2)
            self._pwm.ChangeFrequency(f)
            time.sleep(step)

        # Phase 3: Quick rise to 800 Hz
        for i in range(phase3):
            f = 600 + (200 * i / phase3)
            self._pwm.ChangeFrequency(f)
            time.sleep(step)

        self._pwm.stop()

    def stop(self) -> None:
        self._pwm.stop()
        GPIO.cleanup(self._pin)

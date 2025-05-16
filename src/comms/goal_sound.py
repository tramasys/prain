import time
from typing import Sequence, Tuple
import RPi.GPIO as GPIO

class PWMBuzzer:
    def __init__(self, pin: int = 18, *, numbering: int = GPIO.BCM) -> None:
        GPIO.setmode(numbering)
        self._pin = pin
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, 1)
        self._started = False

    Tone = Tuple[int, float] # (frequency [Hz], duration [s])

    def play(self, tones: Sequence[Tone], duty: float = 50.0) -> None:
        """Play *tones* = [(f1, t1), …] using specified duty-cycle %."""
        if not tones:
            return

        if not self._started:
            self._pwm.start(duty)
            self._started = True

        for freq, dur in tones:
            self._pwm.ChangeDutyCycle(duty)
            self._pwm.ChangeFrequency(freq)
            time.sleep(dur)

        self._pwm.ChangeDutyCycle(0)

    def play_goal(self) -> None:
        self.play([(4000, 2)])

    def stop(self) -> None:
        self._pwm.stop()
        GPIO.cleanup(self._pin)

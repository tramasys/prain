#import RPi.GPIO as GPIO
from utils.gpio_compat import GPIO

class TargetDetector:
    """
        ┌─────────┬──────────┬──────────┐
        │ target  │ pin11     │ pin25    │
        ├─────────┼──────────┼──────────┤
        │   a     │    1      │    0     │
        │   b     │    1      │    1     │
        │   c     │    0      │    1     │
        └─────────┴──────────┴──────────┘
    """

    _MAP = {
        (1, 0): "A",
        (1, 1): "B",
        (0, 1): "C",
    }

    def __init__(
        self,
        pin11: int = 11,
        pin25: int = 25,
        *,
        numbering: int = GPIO.BCM,
        pull: int = GPIO.PUD_DOWN,
    ):
        GPIO.setmode(numbering)
        self._p11 = pin11
        self._p25 = pin25
        GPIO.setup(self._p11, GPIO.IN, pull_up_down=pull)
        GPIO.setup(self._p25, GPIO.IN, pull_up_down=pull)

    def detect(self) -> str:
        key = (GPIO.input(self._p11), GPIO.input(self._p25))
        return self._MAP.get(key, "unknown")

    def cleanup(self) -> None:
        GPIO.cleanup((self._p11, self._p25))

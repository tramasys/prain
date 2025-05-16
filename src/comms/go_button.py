# go_button.py – revised
import time
import RPi.GPIO as GPIO
from typing import Final, Optional


class GoButton:
    """Blocking interface for the push-button on BCM GPIO 16."""

    _PIN: Final[int] = 16
    _PUD: Final[int] = GPIO.PUD_UP

    def __init__(self, bounce_ms: int = 50) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._PIN, GPIO.IN, pull_up_down=self._PUD)
        self._bounce = bounce_ms

    def wait_for_press(self, timeout: Optional[float] = None) -> None:
        """
        Block until button pressed (falling edge).
        `timeout` in seconds; --> raises TimeoutError if expired.
        """
        timeout_ms = None if timeout is None else int(timeout * 1000)

        try:
            ch = GPIO.wait_for_edge(
                self._PIN,
                GPIO.FALLING,
                bouncetime=self._bounce,
                timeout=timeout_ms,
            )
            if ch is None and timeout is not None:
                raise TimeoutError("GO button timeout")
        except RuntimeError:
            # occasional RPi.GPIO bug – fall back to polling
            start = time.monotonic()
            while GPIO.input(self._PIN):
                if timeout is not None and time.monotonic() - start >= timeout:
                    raise TimeoutError("GO button timeout (polling fallback)")
                time.sleep(0.005)

    def is_pressed(self) -> bool:
        """Return current button state (active-low)."""
        return not GPIO.input(self._PIN)

    def close(self) -> None:
        GPIO.cleanup(self._PIN)

    # context-manager support
    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

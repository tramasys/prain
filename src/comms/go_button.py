# go_button.py
import RPi.GPIO as GPIO
from typing import Final


class GoButton:
    """Blocking interface for the start-driving push-button on GPIO 16 (BCM)."""

    _PIN: Final[int] = 16          # BCM numbering
    _PUD: Final[int] = GPIO.PUD_UP # use internal pull-up; switch pulls pin low when pressed

    def __init__(self, bounce_ms: int = 50) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._PIN, GPIO.IN, pull_up_down=self._PUD)
        self._bounce = bounce_ms

    def wait_for_press(self) -> None:
        """
        Block until the button is pressed (first FALLING edge).

        Raises
        ------
        RuntimeError
            If GPIO is not set up correctly or interrupted.
        """
        GPIO.wait_for_edge(self._PIN, GPIO.FALLING, bouncetime=self._bounce)

    # optional resource management -------------------------------------------
    def close(self) -> None:               # call manually if not using context manager
        GPIO.cleanup(self._PIN)

    def __enter__(self):                   # allows ``with GoButton() as btn:``
        return self

    def __exit__(self, *_):                # always clean up
        self.close()

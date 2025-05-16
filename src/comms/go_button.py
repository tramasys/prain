import time
import RPi.GPIO as GPIO

class GoButton:
    """
    Physical "GO" push-button (header pin 16 → BCM 23).

    Typical use:
        btn = GoButton()          # set up GPIO
        btn.wait_for_press()      # blocks until pressed
        ...
        btn.cleanup()             # free GPIO lines
    """

    def __init__(self, pin: int = 23, pull: int = GPIO.PUD_DOWN) -> None:
        GPIO.setmode(GPIO.BCM)
        self._pin = pin
        GPIO.setup(pin, GPIO.IN, pull_up_down=pull)

    def is_pressed(self) -> bool:
        return GPIO.input(self._pin) == GPIO.HIGH

    def wait_for_press(self, debounce: float = 0.05) -> None:
        GPIO.wait_for_edge(self._pin, GPIO.RISING)
        time.sleep(debounce)

    def cleanup(self) -> None:
        GPIO.cleanup(self._pin)

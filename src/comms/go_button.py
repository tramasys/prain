# utils/go_button.py  (replace previous version)
import time
import RPi.GPIO as GPIO


class GoButton:
    """
    Blocking / non-blocking access to the “GO” push-button on pin 16.

    Parameters
    ----------
    pin : int
        BCM pin number (physical hdr-pin 16 == BCM 23).
    active_high : bool, default True
        • True  → button connects pin to 3V3 → logic HIGH when pressed
        • False → button connects pin to GND → logic LOW  when pressed
          (typical wiring with internal pull-up).
    """

    def __init__(self, pin: int = 23, *, active_high: bool = True) -> None:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        self._pin         = pin
        self._active_high = active_high
        pull = GPIO.PUD_DOWN if active_high else GPIO.PUD_UP

        GPIO.setup(pin, GPIO.IN, pull_up_down=pull)

    # ─────────────────────────────────────────────────────────────
    def is_pressed(self) -> bool:     # non-blocking
        return GPIO.input(self._pin) == (GPIO.HIGH if self._active_high else GPIO.LOW)

    def wait_for_press(
        self,
        *,
        debounce: float = 0.05,
        poll: float = 0.01,
    ) -> None:                        # blocking
        edge = GPIO.RISING if self._active_high else GPIO.FALLING
        try:
            GPIO.wait_for_edge(self._pin, edge)
        except RuntimeError:          # fall back to polling
            while not self.is_pressed():
                time.sleep(poll)
        time.sleep(debounce)          # debounce

    # ─────────────────────────────────────────────────────────────
    def cleanup(self) -> None:
        GPIO.cleanup(self._pin)

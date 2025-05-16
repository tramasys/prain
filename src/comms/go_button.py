# utils/go_button.py
import time
import RPi.GPIO as GPIO


class GoButton:
    """GO push-button on header-pin 16 (BCM 23)."""

    def __init__(
        self,
        pin: int = 23,               # BCM 23  ⇔  physical pin 16
        *,
        numbering: int = GPIO.BCM,
        pull: int = GPIO.PUD_DOWN,   # change to PUD_UP if your wiring is active-low
    ):
        GPIO.setwarnings(False)
        GPIO.setmode(numbering)
        self._pin = pin
        GPIO.setup(pin, GPIO.IN, pull_up_down=pull)

    # ────────────────────────────────────────────────────────────────
    def is_pressed(self) -> bool:          # non-blocking
        return GPIO.input(self._pin) == GPIO.HIGH

    def wait_for_press(
        self,
        *,
        debounce: float = 0.05,            # seconds
        poll: float = 0.01,                # polling interval if fallback is used
    ) -> None:
        """
        Block until a rising edge is detected.
        Falls back to polling if edge-detection fails.
        """
        try:
            GPIO.wait_for_edge(self._pin, GPIO.RISING)
            time.sleep(debounce)           # debounce
        except RuntimeError:
            while not self.is_pressed():
                time.sleep(poll)
            time.sleep(debounce)

    # ────────────────────────────────────────────────────────────────
    def cleanup(self) -> None:
        GPIO.cleanup(self._pin)

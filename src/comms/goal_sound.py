import RPi.GPIO as GPIO
import time

class PWMBuzzer:
    def __init__(self, pin: int = 12, duty: int = 50) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        self._pin = pin
        self._duty = duty
        self._pwm = GPIO.PWM(pin, 1)
        self._pwm.stop()

    def play_goal(self) -> None:
        pattern = [
            (880, 0.15),
            (1100, 0.15),
            (1320, 0.30),
        ]
        for freq, dur in pattern:
            self._pwm.ChangeFrequency(freq)
            self._pwm.start(self._duty)
            time.sleep(dur)
            self._pwm.stop()
            time.sleep(0.05)              # small gap between tones

    def stop(self) -> None:
        """Stops any ongoing tone and releases the GPIO pin."""
        self._pwm.stop()
        GPIO.cleanup(self._pin)

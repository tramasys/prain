import RPi.GPIO as GPIO

class GPIOLamp:
    def __init__(self, pin: int = 17, active_high: bool = True) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW if active_high else GPIO.HIGH)
        self._pin = pin
        self._on_level = GPIO.HIGH if active_high else GPIO.LOW
        self._off_level = GPIO.LOW if active_high else GPIO.HIGH

    def on(self) -> None:
        GPIO.output(self._pin, self._on_level)

    def off(self) -> None:
        GPIO.output(self._pin, self._off_level)

    def cleanup(self) -> None:
        self.off()
        GPIO.cleanup(self._pin)

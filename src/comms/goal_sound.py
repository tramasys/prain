import RPi.GPIO as GPIO
import time

class PWMBuzzer:
    _NOTE_FREQ = {     # Hz (equal-temperament A4=440 Hz)
        'A4': 440,
        'B4': 494,
        'C#5': 554,
        'D5': 587,
        'E5': 659,
        'F#5': 740,
    }

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
            time.sleep(0.05)

    def play_rickroll(self) -> None:
        melody = [
            ('A4', 0.18), ('B4', 0.18), ('D5', 0.18), ('B4', 0.18),
            ('F#5', 0.18), ('F#5', 0.18), ('E5', 0.30),

            ('A4', 0.18), ('B4', 0.18), ('D5', 0.18), ('B4', 0.18),
            ('E5', 0.18), ('E5', 0.18), ('D5', 0.18), ('C#5', 0.18),
            ('B4', 0.30),
        ]

        for note, dur in melody:
            self._pwm.ChangeFrequency(self._NOTE_FREQ[note])
            self._pwm.start(self._duty)
            time.sleep(dur)
            self._pwm.stop()
            time.sleep(0.05)

    def stop(self) -> None:
        self._pwm.stop()
        GPIO.cleanup(self._pin)

from gpiozero import Button
from typing import Optional, Final

class GoButton:
    _PIN: Final[int] = 16

    def __init__(self) -> None:
        self._btn = Button(self._PIN, pull_up=None, bounce_time=None, active_state=True)

    def wait_for_press(self, timeout: Optional[float] = None) -> None:
        if not self._btn.wait_for_press(timeout):
            raise TimeoutError("GO button timeout")

    def is_pressed(self) -> bool:
        return self._btn.is_pressed

    def close(self) -> None:
        self._btn.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

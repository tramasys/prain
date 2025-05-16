from gpiozero import Button
from typing import Optional, Final

class GoButton:
    """Start-driving button on BCM 16, active-low with internal pull-up."""

    _PIN: Final[int] = 16

    def __init__(self, bounce_s: float = 0.05) -> None:
        self._btn = Button(self._PIN, pull_up=None, bounce_time=None, active_state=True)

    def wait_for_press(self, timeout: Optional[float] = None) -> None:
        """
        Block until the button is pressed.

        Parameters
        ----------
        timeout : float | None
            Seconds to wait; None → wait indefinitely.
        Raises
        ------
        TimeoutError
            If `timeout` elapses with no button press.
        """
        if not self._btn.wait_for_press(timeout):
            raise TimeoutError("GO button timeout")

    def is_pressed(self) -> bool:
        """Return True while the button is physically held down."""
        return self._btn.is_pressed

    # ---------------------------------------------------------------- cleanup
    def close(self) -> None:
        self._btn.close()

    # context-manager support
    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

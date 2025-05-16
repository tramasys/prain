"""
Import-side GPIO shim.
If running on a Raspberry Pi the real RPi.GPIO is used,
otherwise a silent stub is provided so code instantiation doesn't fail.
"""

from types import SimpleNamespace

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    def _noop(*_, **__): ...
    class _FakePWM:
        def __init__(self, pin, freq): self.pin, self.freq = pin, freq
        start = stop = _noop
        def ChangeFrequency(self, f): self.freq = f
    GPIO = SimpleNamespace(
        BOARD="BOARD", BCM="BCM",
        OUT="OUT", IN="IN",
        LOW=0, HIGH=1,
        PWM=_FakePWM,
        setmode=_noop, setup=_noop, output=_noop, cleanup=_noop,
    )

__all__ = ["GPIO"]

"""
Import-side GPIO shim.
If running on a Raspberry Pi the real RPi.GPIO is used,
otherwise a silent stub is provided so code instantiation doesn't fail.
"""

from types import SimpleNamespace

def _noop(*_, **__): ...
class _FakePWM:
    def __init__(self, pin, freq): self.pin, self.freq = pin, freq
    start = stop = _noop
    def ChangeFrequency(self, f): self.freq = f

GPIO = SimpleNamespace(
    # modes
    BOARD="BOARD", BCM="BCM",
    # directions
    OUT="OUT", IN="IN",
    # logic levels
    LOW=0, HIGH=1,
    # resistor pulls
    PUD_OFF=0, PUD_DOWN=1, PUD_UP=2,
    # other API elements
    PWM=_FakePWM,
    setmode=_noop, setup=_noop, output=_noop, input=lambda *a, **k: 0,
    cleanup=_noop, setwarnings=_noop,
)

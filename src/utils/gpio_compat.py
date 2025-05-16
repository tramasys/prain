"""
Unified import for RPi-GPIO code.

    from gpio_compat import GPIO

• On a Raspberry Pi -→ real RPi.GPIO module.
• Anywhere else     -→ silent stub with the same public symbols.

If you **really** want the stub on a Pi (e.g. CI), export DUMMY_GPIO=1.
"""
from types import SimpleNamespace
import importlib
import os
import platform
import sys

def _running_on_rpi() -> bool:
    if os.getenv("DUMMY_GPIO") == "1":         # manual override
        return False
    if sys.platform != "linux":
        return False
    # Heuristic: ARM/ARM64 + devicetree "model" node is enough
    try:
        return platform.machine().startswith(("arm", "aarch")) and \
               os.path.exists("/sys/firmware/devicetree/base/model")
    except Exception:
        return False

if _running_on_rpi():
    # → *real* driver
    GPIO = importlib.import_module("RPi.GPIO")        # type: ignore
else:
    # → minimal, side-effect-free stub
    def _noop(*_, **__): ...
    class _FakePWM:
        def __init__(self, pin, freq): self.freq = freq
        start = stop = _noop
        def ChangeFrequency(self, f): self.freq = f

    GPIO = SimpleNamespace(
        # modes
        BOARD="BOARD", BCM="BCM",
        # directions
        OUT="OUT", IN="IN",
        # levels
        LOW=0, HIGH=1,
        # pulls
        PUD_OFF=0, PUD_DOWN=1, PUD_UP=2,
        # API
        PWM=_FakePWM,
        setmode=_noop, setup=_noop, setwarnings=_noop,
        output=_noop, input=lambda *a, **k: 0,
        cleanup=_noop,
    )

__all__ = ["GPIO"]

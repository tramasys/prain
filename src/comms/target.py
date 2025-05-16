# Reads a 2-bit hardware selector (physical switch) via the kernels
# /sys/class/gpio interface — no external Python GPIO libs required.
#
# Encoding:
#   ┌─────────┬──────────┬──────────┐
#   │ target  │ 11 (X)   │ 25 (Y)   │
#   ├─────────┼──────────┼──────────┤
#   │   a     │   1      │   0      │
#   │   b     │   1      │   1      │
#   │   c     │   0      │   1      │
#   └─────────┴──────────┴──────────┘
# Default BCM lines are 11 and 25

from __future__ import annotations
import os
import time
from contextlib import suppress

_SYSFS = "/sys/class/gpio"

def _ensure_exported(num: int) -> str:
    path = f"{_SYSFS}/gpio{num}"
    if not os.path.exists(path):
        with open(f"{_SYSFS}/export", "w") as fd:
            fd.write(f"{num}")
        for _ in range(10):
            if os.path.exists(path):
                break
            time.sleep(0.01)
    with open(f"{path}/direction", "w") as fd:
        fd.write("in")
    return path

class _InPin:
    def __init__(self, bcm: int):
        self._bcm = bcm
        self._path = _ensure_exported(bcm)

    def read(self) -> bool:
        with open(f"{self._path}/value") as fd:
            return fd.read(1) == "1"

    def cleanup(self) -> None:
        with suppress(FileNotFoundError):
            with open(f"{_SYSFS}/unexport", "w") as fd:
                fd.write(f"{self._bcm}")


class TargetDetector:
    _MAP = {
        (True,  False): "A",
        (True,  True):  "B",
        (False, True):  "C",
    }

    def __init__(self, pin_x: int = 11, pin_y: int = 25):
        self._x = _InPin(pin_x)
        self._y = _InPin(pin_y)

    def detect(self) -> str:
        key = (self._x.read(), self._y.read())
        return self._MAP.get(key, "unknown")

    def cleanup(self) -> None:
        self._x.cleanup()
        self._y.cleanup()

import argparse
from typing import NamedTuple

class CliArgs(NamedTuple):
    subprogram: str | None
    port: str
    baudrate: int

def parse_args() -> CliArgs:
    parser = argparse.ArgumentParser(description="Super mega awesome car controller")
    parser.add_argument("-s", "--subprogram", choices=["dtest", "etest"], help="Run a subprogram instead of the main loop")
    parser.add_argument("-p", "--port", default="/dev/ttyS0", help="UART port (default: /dev/ttyS0)")
    parser.add_argument("-b", "--baudrate", type=int, default=115200, help="UART baudrate (default: 115200)")
    args = parser.parse_args()

    return CliArgs(
        subprogram=args.subprogram,
        port=args.port,
        baudrate=args.baudrate
    )

import argparse
from typing import NamedTuple

class CliArgs(NamedTuple):
    subprogram: str | None
    port: str
    baudrate: int
    host: str
    server_port: int

def parse_args() -> CliArgs:
    parser = argparse.ArgumentParser(description="Self-driving car high-level controller")
    parser.add_argument("-s", "--subprogram", choices=["dtest", "etest", "rcserver", "rcclient"], help="Run a subprogram")
    parser.add_argument("-u", "--uart", default="/dev/ttyS0", help="UART port (default: /dev/ttyS0)")
    parser.add_argument("-b", "--baudrate", type=int, default=115200, help="UART baudrate (default: 115200)")
    parser.add_argument("--host", default="localhost", help="Server host for rcclient (default: localhost)")
    parser.add_argument("--port", type=int, default=5000, help="Server port for rcclient (default: 5000)")
    args = parser.parse_args()

    return CliArgs(
        subprogram=args.subprogram,
        port=args.port,
        baudrate=args.baudrate,
        host=args.host,
        server_port=args.server_port
    )

import argparse
from typing import NamedTuple

class CliArgs(NamedTuple):
    subprogram: str | None
    uart: str
    uart_baudrate: int
    lidar: int
    host: str
    server_port: int

def parse_args() -> CliArgs:
    parser = argparse.ArgumentParser(description="Self-driving car high-level controller")
    parser.add_argument("-s", "--subprogram", choices=["dtest", "etest", "ldtest", "rcserver", "rcclient"], help="Run a subprogram")
    parser.add_argument("-u", "--uart", default="/dev/ttyS0", help="UART port (default: /dev/ttyS0)")
    parser.add_argument("-ub", "--uart-baudrate", type=int, default=115200, help="UART baudrate (default: 115200)")
    parser.add_argument("-l", "--lidar", type=int, default=1, help="Lidar bus number (default: 1)")
    parser.add_argument("--host", default="localhost", help="Server host for rcclient (default: localhost)")
    parser.add_argument("--port", type=int, default=5000, help="Server port for rcclient (default: 5000)")
    args = parser.parse_args()

    return CliArgs(
        subprogram=args.subprogram,
        uart=args.uart,
        uart_baudrate=args.uart_baudrate,
        lidar=args.lidar,
        host=args.host,
        server_port=args.port
    )

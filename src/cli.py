import argparse
from typing import NamedTuple

class CliArgs(NamedTuple):
    target: str
    subprogram: str | None
    uart: str
    uart_baudrate: int
    lidar: int
    lidar_address: int
    host: str
    port: int

def parse_args() -> CliArgs:
    parser = argparse.ArgumentParser(description="Self-driving car high-level controller")
    parser.add_argument("-t", "--target", default="A", help="Target node (default: A)")
    parser.add_argument("-s", "--subprogram", choices=["dtest", "etest", "ldtest", "rcserver", "rcclient", "stest", "switchtest", "soundtest", "gotest", "lamptest", "sweeptest"], help="Run a subprogram")
    parser.add_argument("-u", "--uart", default="/dev/ttyAMA0", help="UART port (default: /dev/ttyAMA0)")
    parser.add_argument("-ub", "--uart-baudrate", type=int, default=115200, help="UART baudrate (default: 115200)")
    parser.add_argument("-l", "--lidar", type=int, default=1, help="Lidar bus number (default: 1)")
    parser.add_argument("-la", "--lidar-address", type=int, default=0x10, help="Lidar I2C address (default: 0x10)")
    parser.add_argument("--host", default="localhost", help="Server host for rcclient (default: localhost)")
    parser.add_argument("--port", type=int, default=5000, help="Server port for rcclient (default: 5000)")
    args = parser.parse_args()

    return CliArgs(
        target=args.target,
        subprogram=args.subprogram,
        uart=args.uart,
        uart_baudrate=args.uart_baudrate,
        lidar=args.lidar,
        lidar_address=args.lidar_address,
        host=args.host,
        port=args.port
    )

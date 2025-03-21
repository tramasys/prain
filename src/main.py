import time
from comms.uart import UartInterface
from utils.logging import setup_logging
from sub.dtest import decode_test
from sub.etest import encode_test
from sub.rc import rc_server
from sub.rcclient import rc_client
from cli import parse_args

def main_loop(uart: UartInterface) -> None:
    logger = setup_logging()
    logger.info("STARTING main control loop")

    try:
        while True:
            logger.info("Running main loop iteration")
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("SHUTTING DOWN")
        uart.close()

def main():
    args = parse_args()

    subprogram_handlers = {
        "dtest": lambda: decode_test(uart),
        "etest": lambda: encode_test(uart),
        "rcserver": lambda: rc_server(uart, args.host, args.server_port),
        None: lambda: main_loop(uart),
    }

    if args.subprogram == "rcclient":
        rc_client(host=args.host, port=args.server_port)
        return

    uart = UartInterface(args.port, baudrate=args.baudrate)
    handler = subprogram_handlers.get(args.subprogram, subprogram_handlers[None])
    handler()

if __name__ == "__main__":
    main()

import time
from comms.uart import UartInterface
from utils.logging import setup_logging
from sub.dtest import decode_test
from sub.etest import encode_test
from cli import parse_args

def main_loop(uart: UartInterface) -> None:
    logger = setup_logging()
    logger.info("STARTING main control loop")

    try:
        while True:
            # Placeholder
            logger.info("Running main loop iteration")
            time.sleep(0.1)

    except KeyboardInterrupt:
        logger.info("SHUTTING DOWN")
        uart.close()

def main():
    args = parse_args()
    uart = UartInterface(args.port, baudrate=args.baudrate)

    if args.subprogram == "dtest":
        decode_test(uart)
    elif args.subprogram == "etest":
        encode_test(uart)
    else:
        main_loop(uart)

if __name__ == "__main__":
    main()

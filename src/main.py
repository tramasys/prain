import time

from utils.logging import setup_logging
from sub.dtest import decode_test
from sub.etest import encode_test
from sub.ldtest import lidar_test
from sub.rc import rc_server
from sub.rcclient import rc_client
from cli import parse_args
from comms.uart import UartInterface
from core.controller import HighLevelController

def main_loop(args) -> None:
    logger = setup_logging()
    logger.info("STARTING main control loop")

    controller = HighLevelController(
        uart_port=args.uart,
        uart_baudrate=args.uart_baudrate,
        lidar_bus=args.lidar,
        lidar_address=args.lidar_address,
        target_node="A",
        logger=logger,
    )

    controller.start()

    try:
        while True:
            logger.info("Keeping main loop iteration alive ...")
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("SHUTTING DOWN")
    finally:
        controller.stop()

def main():
    args = parse_args()

    def run_dtest():
        uart = UartInterface(args.uart, args.uart_baudrate)
        decode_test(uart)
        uart.close()

    def run_etest():
        uart = UartInterface(args.uart, args.uart_baudrate)
        encode_test(uart)
        uart.close()

    def run_rcserver():
        uart = UartInterface(args.uart, args.uart_baudrate)
        rc_server(uart, args.host, args.port)
        uart.close()

    subprogram_handlers = {
        "dtest":    run_dtest,
        "etest":    run_etest,
        "rcserver": run_rcserver,
        "rcclient": lambda: rc_client(args.host, args.port),
        "ldtest":   lambda: lidar_test(args.lidar, args.lidar_address),
        None:       lambda: main_loop(args),
    }

    handler = subprogram_handlers.get(args.subprogram, subprogram_handlers[None])
    handler()

if __name__ == "__main__":
    main()

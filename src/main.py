import time
from utils.logging import setup_logging
from sub.dtest import decode_test
from sub.etest import encode_test
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
        lidar_port=args.lidar,
        lidar_baudrate=args.lidar_baudrate,
    )

    controller.start()

    try:
        while True:
            logger.info("Running main loop iteration")
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("SHUTTING DOWN")
    finally:
        controller.stop()

def main():
    args = parse_args()

    def run_dtest():
        uart = UartInterface(args.uart, baudrate=args.uart_baudrate)
        decode_test(uart)
        uart.close()

    def run_etest():
        uart = UartInterface(args.uart, baudrate=args.uart_baudrate)
        encode_test(uart)
        uart.close()

    def run_rcserver():
        uart = UartInterface(args.uart, baudrate=args.uart_baudrate)
        rc_server(uart, args.host, args.server_port)
        uart.close()

    subprogram_handlers = {
        "dtest":    run_dtest,
        "etest":    run_etest,
        "rcserver": run_rcserver,
        None:       lambda: main_loop(args),
    }

    if args.subprogram == "rcclient":
        rc_client(host=args.host, port=args.server_port)
        return

    handler = subprogram_handlers.get(args.subprogram, subprogram_handlers[None])
    handler()

if __name__ == "__main__":
    main()

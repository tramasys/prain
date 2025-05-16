import time

from utils.logging import setup_logging
from sub.dtest import decode_test
from sub.etest import encode_test
from sub.ldtest import lidar_test
from sub.rc import rc_server
from sub.rcclient import rc_client
from sub.stest import send_test
from sub.switchtest import target_switch_test
from sub.soundtest import test_sound
from sub.gotest import test_go_button
from sub.lamptest import test_lamp
from sub.sweeptest import test_sweep
from cli import parse_args
from comms.uart import UartInterface
from core.controller import HighLevelController
from comms.target import TargetDetector
from comms.go_button import GoButton

def main_loop(args) -> None:
    logger = setup_logging()
    logger.info("[MAIN] starting main control loop")

    detector = TargetDetector()
    target   = detector.detect()
    logger.info(f"[MAIN] Target: {target}")
    detector.cleanup()

    go_btn = GoButton()
    logger.info("[MAIN] waiting for GO button ...")
    go_btn.wait_for_press() # blocks execution until the button is pressed
    logger.info("[MAIN] GO button pressed - starting vehicle")

    controller = HighLevelController(
        uart_port=args.uart,
        uart_baudrate=args.uart_baudrate,
        lidar_bus=args.lidar,
        lidar_address=args.lidar_address,
        target_node=target,
        logger=logger,
    )

    controller.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("[MAIN] shutting down main control loop")
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

    def run_stest():
        uart = UartInterface(args.uart, args.uart_baudrate)
        send_test(uart)
        uart.close()

    def run_sweeptest():
        uart = UartInterface(args.uart, args.uart_baudrate)
        test_sweep(uart)
        uart.close()

    subprogram_handlers = {
        "dtest":        run_dtest,
        "etest":        run_etest,
        "stest":        run_stest,
        "sweeptest":    run_sweeptest,
        "lamptest":     lambda: test_lamp(),
        "gotest":       lambda: test_go_button(),
        "soundtest":    lambda: test_sound(),
        "switchtest":   lambda: target_switch_test(),
        "rcserver":     lambda: rc_server(args.uart, args.uart_baudrate, args.host, args.port),
        "rcclient":     lambda: rc_client(args.host, args.port),
        "ldtest":       lambda: lidar_test(args.lidar, args.lidar_address),
        None:           lambda: main_loop(args),
    }

    handler = subprogram_handlers.get(args.subprogram, subprogram_handlers[None])
    handler()

if __name__ == "__main__":
    main()

import time
from comms.uart import UartInterface
from utils.logging import setup_logging
from sub.dtest import decode_loop

def main():
    logger = setup_logging()
    logger.info("STARTING")

    uart = UartInterface("/dev/ttyS0", baudrate=115200)
    decode_loop(uart)

""""
    try:
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        logger.info("SHUTTING DOWN")
        uart.close()
"""

if __name__ == "__main__":
    main()

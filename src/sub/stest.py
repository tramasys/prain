import time
from prain_uart import *
from comms.uart import UartInterface

def send_test(uart: UartInterface) -> None:
    print(f"[TX-TEST] → {uart.port} @ {uart.baudrate} baud")

    count = 0
    try:
        while True:
            print(f"[TX-TEST] sending frame {count}")
            count += 1
            uart.send_frame(encode_move(Address.MOTION_CTRL, 10))
            time.sleep(5)
    except KeyboardInterrupt:
        pass

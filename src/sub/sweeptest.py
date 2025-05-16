import time
from comms.uart import UartInterface
from comms.manager import UartManager
from prain_uart import *

def test_sweep(uart_manager: UartManager) -> None:
    print(f"Starting sweep test!")

    test_frames = [
        encode_turn(Address.MOTION_CTRL, -100),
        encode_turn(Address.MOTION_CTRL, 200),
        encode_turn(Address.MOTION_CTRL, -100),
    ]

    try:
        for frame in test_frames:
            print(f"Sending frame: {frame.raw:016x}")
            uart_manager.send_frame(frame)
            #time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping encode test")
    finally:
        uart_manager.stop()

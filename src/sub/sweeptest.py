import time
from comms.uart import UartInterface
from prain_uart import *

def encode_test(uart_interface: UartInterface) -> None:
    print(f"Starting sweep test on {uart_interface.port} at {uart_interface.baudrate} baud")

    test_frames = [
        encode_turn(Address.MOTION_CTRL, -50),
        encode_turn(Address.MOTION_CTRL, 100),
        encode_turn(Address.MOTION_CTRL, -50),
    ]

    try:
        for frame in test_frames:
            print(f"\nSending frame: {frame.raw:016x}")
            uart_interface.send_frame(frame)
            time.sleep(0.01)

        print("Encode test complete")
    except KeyboardInterrupt:
        print("\nStopping encode test")
    finally:
        uart_interface.close()

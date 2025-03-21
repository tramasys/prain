import time
from comms.uart import UartInterface
from prain_uart import *

def encode_test(uart_interface: UartInterface) -> None:
    print(f"Starting UART encode test on {uart_interface.port} at {uart_interface.baudrate} baud")

    test_frames = [
        encode_move(Address.MOTION_CTRL, 100),
        encode_reverse(Address.MOTION_CTRL, 200),
        encode_turn(Address.MOTION_CTRL, 90),
        encode_stop(Address.MOTION_CTRL),
        encode_info(Address.MOTION_CTRL, InfoFlag.BATTERY),
        encode_ping(Address.MOTION_CTRL, 42),
        encode_pong(Address.MOTION_CTRL, 42),
        encode_error(Address.MOTION_CTRL, ErrorCode.INVALID_CRC),
        encode_poll(Address.MOTION_CTRL, PollId.DISTANCE),
        encode_response(Address.MOTION_CTRL, PollId.DISTANCE, 420),
        encode_crane(Address.MOTION_CTRL, CraneFlag.UP),
    ]

    try:
        for frame in test_frames:
            print(f"\nSending frame: {frame.raw:016x}")
            uart_interface.send_frame(frame)
            time.sleep(0.1)

        print("Encode test complete")
    except KeyboardInterrupt:
        print("\nStopping encode test")
    finally:
        uart_interface.close()

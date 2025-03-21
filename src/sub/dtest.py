import time
from comms.uart import UartInterface
from prain_uart import *

def print_frame(frame: Frame) -> None:
    decoder = Decoder(frame)
    print(f"Address: {decoder.address.name} ({decoder.address.value})")
    print(f"Command: {decoder.command.name} ({decoder.command.value})")
    print(f"CRC valid: {decoder.verify_crc()}")

    try:
        params = decoder.get_params()
        if isinstance(params, MoveParams):
            print(f"Params: Move distance={params.distance}")
        elif isinstance(params, ReverseParams):
            print(f"Params: Reverse distance={params.distance}")
        elif isinstance(params, TurnParams):
            print(f"Params: Turn angle={params.angle}")
        elif isinstance(params, InfoParams):
            print(f"Params: Info flag={params.flag}")
        elif isinstance(params, EmptyParams):
            print("Params: None (Empty)")
        elif isinstance(params, ErrorParams):
            print(f"Params: Error code={params.error_code}")
        elif isinstance(params, PingParams):
            print(f"Params: Ping id={params.id}")
        elif isinstance(params, PongParams):
            print(f"Params: Pong id={params.id}")
        elif isinstance(params, PollParams):
            print(f"Params: Poll id={params.poll_id}")
        elif isinstance(params, ResponseParams):
            print(f"Params: Response poll_id={params.poll_id}, data={params.data}")
        elif isinstance(params, CraneParams):
            print(f"Params: Crane flag={params.flag}")
        else:
            print("Params: Unknown type")
    except ValueError as e:
        print(f"Error decoding params: {e}")

def decode_test(uart_interface: UartInterface) -> None:
    print(f"Starting UART decode loop on {uart_interface.port} at {uart_interface.baudrate} baud")

    try:
        while True:
            frame = uart_interface.receive_frame()
            if frame is not None:
                print("\nReceived frame:")
                print_frame(frame)
            else:
                print(".", end="", flush=True)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping decode loop")
        uart_interface.close()

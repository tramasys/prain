from comms.uart import UartInterface
from .server import start_server

def rc_server(uart_interface: UartInterface, host: str, port: int) -> None:
    print(f"Starting RC server mode on {uart_interface.port} at {uart_interface.baudrate} baud")
    server_thread = start_server(uart_interface, host, port)

    try:
        server_thread.join()
    except KeyboardInterrupt:
        print("\nStopping RC server mode")
        uart_interface.close()

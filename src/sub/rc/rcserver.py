from .server import start_server

def rc_server(uart_port: str, uart_baud: int, host: str, port: int) -> None:
    print(f"Starting RC server mode on {uart_port} at {uart_baud} baud")

    server_thread = start_server(
        host=host,
        port=port,
        uart_port=uart_port,
        uart_baud=uart_baud
    )

    try:
        server_thread.join()
    except KeyboardInterrupt:
        print("\nStopping RC server mode")

import threading
import socket
from typing import Optional
from comms.uart import UartInterface
from .commands import execute_command

def start_server(uart: UartInterface, host: str = "localhost", port: int = 5000) -> threading.Thread:

    def server_loop():
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind((host, port))
            server.listen(1)
            print(f"Remote control server listening on {host}:{port}")

            while True:
                conn, addr = server.accept()
                with conn:
                    print(f"Connected by {addr}")
                    data = conn.recv(1024).decode().strip()
                    if data:
                        result = execute_command(uart, data)
                        conn.sendall(result.encode())

    thread = threading.Thread(target=server_loop, daemon=True)
    thread.start()
    return thread

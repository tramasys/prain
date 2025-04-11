import threading
import queue
import socket
from comms.manager import UartManager
from .commands import execute_command

received_queue = queue.Queue()

def logger_loop(uart_manager: UartManager) -> None:
    """Continuously read frames from UartManager's rx_queue, store them in received_queue."""
    while True:
        frame = uart_manager.rx_queue.get()
        text = f"LOW-LEVEL RX: raw=0x{frame.raw:016X}"
        frame_hex = f"Ox{frame.raw:016X}"
        frame_bin = f"0b{frame.raw:064b}"
        print(frame_hex)
        print(frame_bin)
        received_queue.put(text)

def start_server(
    host: str = "localhost",
    port: int = 5000,
    uart_port: str = "/dev/ttyS0",
    uart_baud: int = 115200
) -> threading.Thread:
    uart_manager = UartManager(uart_port, uart_baud)
    uart_manager.start()

    # Start a thread that drains frames from UartManager's rx_queue
    t_logger = threading.Thread(target=logger_loop, args=(uart_manager,), daemon=True)
    t_logger.start()

    def server_loop():
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind((host, port))
            server.listen(1)
            print(f"Remote control server listening on {host}:{port}")

            while True:
                conn, addr = server.accept()
                with conn:
                    data = conn.recv(1024).decode().strip()
                    if not data:
                        continue

                    if data == "get_received":
                        logs = []
                        while not received_queue.empty():
                            logs.append(received_queue.get())
                        if logs:
                            response = "\n".join(logs)
                        else:
                            response = ""
                        conn.sendall(response.encode())
                    else:
                        result = execute_command(uart_manager, data)
                        conn.sendall(result.encode())

    t_server = threading.Thread(target=server_loop, daemon=True)
    t_server.start()
    return t_server

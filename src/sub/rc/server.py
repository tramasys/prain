import threading
import queue
import socket
from comms.manager import UartManager
from .commands import execute_command
from prain_uart import *

received_queue = queue.Queue()

def frame_to_text(frame):
    d = Decoder(frame)

    addr = d.address.name
    cmd  = d.command.name

    try:
        p = d.get_params()
    except ValueError:
        return f"{addr} {cmd} (CRC error)"

    if isinstance(p, MoveParams):
        return f"{addr} MOVE {p.distance} cm"
    if isinstance(p, ReverseParams):
        return f"{addr} REVERSE {p.distance} cm"
    if isinstance(p, TurnParams):
        return f"{addr} TURN {p.angle}°"
    if isinstance(p, InfoParams):
        return f"{addr} INFO flag={p.flag}"
    if isinstance(p, PingParams):
        return f"{addr} PING id={p.id}"
    if isinstance(p, PongParams):
        return f"{addr} PONG id={p.id}"
    if isinstance(p, ErrorParams):
        return f"{addr} ERROR code={p.error_code}"
    if isinstance(p, PollParams):
        return f"{addr} POLL id={p.poll_id}"
    if isinstance(p, ResponseParams):
        return f"{addr} RESPONSE id={p.poll_id} data={p.data}"
    if isinstance(p, EmptyParams):
        return f"{addr} {cmd}"

    return f"{addr} {cmd}"

def logger_loop(uart_manager: UartManager) -> None:
    """Continuously read frames from UartManager's rx_queue, store them in received_queue."""
    while True:
        try:
            print("[LOGGER_LOOP] waiting for frame")
            frame = uart_manager.rx_queue.get()
            print(f"[LOGGER_LOOP] got frame 0x{frame.raw:016X}")
            received_queue.put(frame_to_text(frame))
        except queue.Empty:
            print("[LOGGER_LOOP] empty ...")

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

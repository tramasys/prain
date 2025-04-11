import threading
import time
from queue import Queue, Empty
from typing import Optional

from .uart import UartInterface
from prain_uart import Frame

class UartManager:
    def __init__(self, port: str, baudrate: int = 115200):
        self._uart = UartInterface(port, baudrate)
        self._stop_event = threading.Event()

        self.rx_queue: Queue[Frame] = Queue()
        self.tx_queue: Queue[Frame] = Queue()

        self._reader_thread: Optional[threading.Thread] = None
        self._writer_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._stop_event.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._reader_thread.start()
        self._writer_thread.start()

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            frame = self._uart.receive_frame()
            if frame is not None:
                #print(f"LOW-LEVEL RX: raw=0x{frame.raw:016X}")
                self.rx_queue.put(frame)

    def _writer_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                frame = self.tx_queue.get(timeout=0.1)
                self._uart.send_frame(frame)
            except Empty:
                pass  # No frame in queue during this cycle

    def send_frame(self, frame: Frame) -> None:
        self.tx_queue.put(frame)

    def stop(self) -> None:
        self._stop_event.set()
        if self._reader_thread is not None:
            self._reader_thread.join()
        if self._writer_thread is not None:
            self._writer_thread.join()
        self._uart.close()

import threading
from queue import Queue, Empty
from typing import Optional
import time

from comms.uart import UartInterface
from utils.helper import frame_debug
from prain_uart import Frame, Decoder, Command, InfoFlag, PollId

class UartManager:
    def __init__(self, port: str, baudrate: int = 115200):
        self._uart = UartInterface(port, baudrate)
        self._stop_event = threading.Event()

        self.rx_queue: Queue[Frame] = Queue()
        self.tx_queue: Queue[Frame] = Queue()
        self.ack_queue: Queue[Frame] = Queue()
        self.line_poll_queue: Queue[Frame] = Queue()

        self._reader_thread: Optional[threading.Thread] = None
        self._writer_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._stop_event.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._reader_thread.start()
        self._writer_thread.start()

    def _reader_loop(self) -> None:
        """
        Reads frames from UART and puts them into the appropriate queue.
        This is our "mail sorter".
        """
        while not self._stop_event.is_set():
            try:
                frame = self._uart.receive_frame()
                if frame:
                    try:
                        decoder = Decoder(frame)
                        params = decoder.get_params()
                        if decoder.command == Command.INFO and params.flag == InfoFlag.ACK.value:
                            self.ack_queue.put(frame)
                        elif (decoder.command == Command.INFO and params.flag == InfoFlag.LOST_LINE.value) or \
                            (decoder.command == Command.RESPONSE and params.poll_id == PollId.LINE_SENSOR.value):
                            self.line_poll_queue.put(frame)
                        else:
                            self.rx_queue.put(frame)
                    except Exception as e:
                        self.rx_queue.put(frame)
                    
                    # print(f"[MANAGER]: _reader_loop received frame!")
                    # frame_debug(frame)

            except Exception as e:
                print(f"[MANAGER]: _reader_loop encountered an error: {e}")

    def _writer_loop(self):
        while not self._stop_event.is_set() or not self.tx_queue.empty():
            try:
                frame = self.tx_queue.get(timeout=0.05)
                self._uart.send_frame(frame)
                # print(f"[MANAGER]: _writer_loop sent frame!")
                # frame_debug(frame)
            except Empty:
                continue
            except Exception as e:
                print(f"[MANAGER]: _writer_loop encountered an error: {e}")



    def send_frame(self, frame: Frame) -> None:
        self.tx_queue.put(frame)

    def stop(self) -> None:
        self._stop_event.set()
        if self._reader_thread is not None:
            self._reader_thread.join()
        if self._writer_thread is not None:
            self._writer_thread.join()
        self._uart.close()
        
    def clear_ack_queue(self) -> None:
        """
        Clears the ack queue.
        """
        while not self.ack_queue.empty():
            try:
                self.ack_queue.get_nowait()
            except Empty:
                break
            
    def clear_line_poll_queue(self) -> None:
        """
        Clears the line poll queue.
        """
        while not self.line_poll_queue.empty():
            try:
                self.line_poll_queue.get_nowait()
            except Empty:
                break

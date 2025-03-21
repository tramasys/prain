import serial
from typing import Optional
from prain_uart import Frame

class UartInterface:
    def __init__(self, port: str, baudrate: int = 115200):
        self.serial = serial.Serial(port, baudrate, timeout=1)

    def port(self) -> str:
        return self.serial.portstr

    def baudrate(self) -> int:
        return self.serial.baudrate

    def send_frame(self, frame: Frame) -> None:
        self.serial.write(frame.raw.to_bytes(8, "little"))

    def receive_frame(self) -> Optional[Frame]:
        data = self.serial.read(8)
        if len(data) == 8:
            raw = int.from_bytes(data, "little")
            return Frame.set_raw(raw)
        return None

    def close(self) -> None:
        self.serial.close()

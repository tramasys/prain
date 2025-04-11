import serial
from typing import Optional
from prain_uart import Frame

class UartInterface:
    def __init__(self, port: str, baudrate: int = 115200):
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            rtscts=False,
            xonxoff=False,
            timeout=1
        )

    @property
    def port(self) -> str:
        return self.serial.portstr

    @property
    def baudrate(self) -> int:
        return self.serial.baudrate

    def send_frame(self, frame: Frame) -> None:
        self.serial.write(frame.raw.to_bytes(8, "little"))

    def receive_frame(self) -> Optional[Frame]:
        data = self.serial.read(8)

        if len(data) == 8:
            raw = int.from_bytes(data, "little")
            print(f"UART-IFACE: 0b{raw:064b}")
            print(f"UART-IFACE: 0x{raw:016X}")
            frame = Frame()
            return frame.set_raw(raw)

        return None

    def close(self) -> None:
        self.serial.close()

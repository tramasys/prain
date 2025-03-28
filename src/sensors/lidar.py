import threading
import time
import serial

class LidarSensor:
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        self._port = port
        self._baudrate = baudrate
        self._stop = False
        self._thread = None

        self._ser = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            timeout=0.1
        )

        self._latest_scan = None

    def start(self):
        self._stop = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        partial_buffer = b""

        while not self._stop:
            try:
                data = self._ser.read(128)

                if data:
                    partial_buffer += data
                    parsed_scans, leftover = self.parse_lidar_data(partial_buffer)
                    partial_buffer = leftover

                    if parsed_scans:
                        self._latest_scan = parsed_scans[-1]

            except serial.SerialException as e:
                print(f"LidarSensor serial error: {e}")
                self._stop = True
            except Exception as e:
                print(f"LidarSensor unexpected error: {e}")

            time.sleep(0.05)

    def parse_lidar_data(self, raw_bytes: bytes):
        parsed_scans = []
        leftover = raw_bytes

        SCAN_SIZE = 10
        while len(leftover) >= SCAN_SIZE:
            chunk = leftover[:SCAN_SIZE]
            leftover = leftover[SCAN_SIZE:]
            scan_data = chunk
            parsed_scans.append(scan_data)

        return parsed_scans, leftover

    def get_latest_scan(self):
        return self._latest_scan

    def stop(self):
        self._stop = True
        if self._thread is not None:
            self._thread.join()

        if self._ser.is_open:
            self._ser.close()

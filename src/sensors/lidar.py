import threading
import time

class LidarSensor:
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        self._port = port
        self._baudrate = baudrate
        self._stop = False
        self._thread = None

        self._latest_scan = None

    def start(self):
        self._stop = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while not self._stop:
            # Pseudocode:
            # data = read_from_lidar(self._port, self._baudrate)
            # parse that data into a structured form
            # self._latest_scan = ...

            time.sleep(0.05)

    def get_latest_scan(self):
        return self._latest_scan

    def stop(self):
        self._stop = True
        if self._thread is not None:
            self._thread.join()

import threading
import time
import cv2

class CameraSensor:
    def __init__(self, device_index=0):
        self.cap = cv2.VideoCapture(device_index)
        self._stop = False
        self._latest_frame = None
        self._thread = None

    def start(self):
        self._stop = False
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        while not self._stop:
            ret, frame = self.cap.read()
            if ret:
                self._latest_frame = frame
            time.sleep(0.01)

    def get_latest_frame(self):
        return self._latest_frame

    def get_data(self):
        return ""

    def stop(self):
        self._stop = True
        if self._thread:
            self._thread.join()
        self.cap.release()

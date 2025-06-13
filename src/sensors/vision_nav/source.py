import cv2
import logging
from abc import ABC, abstractmethod
import threading
import time
import numpy as np
try:
    from picamera2 import Picamera2
except ModuleNotFoundError:
    logging.getLogger().warning('Picamera2 only exists on Raspberry Pi')

class VideoSource(ABC):  # Interface
    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def get_next_frame(self):
        pass

    @abstractmethod
    def get_main_frame(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def get_info(self):
        pass  # width, height, fps

class FileSource(VideoSource):
    def __init__(self, video_path):
        self.__video_path = video_path
        self.__file = cv2.VideoCapture(video_path)
        self.__log = logging.getLogger(self.__class__.__name__)

    def start(self):
        self.__log.info(f"Processing video file: {self.__video_path}")

    def get_next_frame(self):
        ret, frame = self.__file.read()
        if not ret:
            self.__log.info("End of video file reached.")
            return None
        frame = cv2.resize(frame, (320, 320))
        return frame

    def get_main_frame(self):
        return self.get_next_frame()

    def stop(self):
        self.__file.release()
        self.__log.info("Video file processing finished.")

    def get_info(self):
        return int(self.__file.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.__file.get(cv2.CAP_PROP_FRAME_HEIGHT)), self.__file.get(cv2.CAP_PROP_FPS)

class CameraSource(VideoSource):
    def __init__(self, resolution=(1024, 1024), fps=20.0):
        self.__resolution = resolution
        self.__fps = fps
        self.__frame_lock = threading.Lock()
        self.__latest_frame_main = None
        self.__latest_frame_lores = None

        self.__log = logging.getLogger(self.__class__.__name__)
        try:
            self.__camera = Picamera2()
            self.__camera.video_configuration.controls.FrameRate = self.__fps
            self.__camera.configure(
                self.__camera.create_video_configuration(
                    main={"size": self.__resolution, "format": "YUV420"},
                    lores={"size": (320, 320), "format": "YUV420"}
                )
            )
        except Exception as e:
            self.__log.error(f'Picamera2 not instantiated! {e}')

    def __frame_callback(self, request):
        with self.__frame_lock:
            self.__latest_frame_main = request.make_array("main")
            self.__latest_frame_lores = request.make_array("lores")

    def start(self):
        self.__camera.post_callback = self.__frame_callback
        self.__camera.start()
        self.__log.info("PiCamera2 started.")
        if self.__wait_for_startup():
            time.sleep(0.5)
            try:
                self.__camera.set_controls({"AfMode": 1, "AfTrigger": 0})
                self.__log.info("Autofokus ausgelöst (AfMode=1, AfTrigger=0)")
            except Exception as e:
                self.__log.warning(f"Autofokus-Initialisierung fehlgeschlagen: {e}")
            return
        self.stop()
        raise RuntimeError("Camera failed to deliver frame in time.")

    def get_next_frame(self):
        with self.__frame_lock:
            if self.__latest_frame_lores is not None:
                frame = self.__latest_frame_lores.copy()
                return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)
        return None

    def get_main_frame(self):
        with self.__frame_lock:
            if self.__latest_frame_main is not None:
                frame = self.__latest_frame_main.copy()
                return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)
        return None

    def stop(self):
        self.__camera.stop()
        self.__log.info("PiCamera2 stopped.")

    def get_info(self):
        return self.__resolution[0], self.__resolution[1], self.__fps

    def __wait_for_startup(self, timeout=5.0, check_interval=0.05):
        start_time = time.time()
        self.__log.info(f"Waiting for camera to deliver first frame (timeout = {timeout}s)...")
        while time.time() - start_time < timeout:
            with self.__frame_lock:
                if self.__latest_frame_lores is not None and isinstance(self.__latest_frame_lores, np.ndarray):
                    if self.__latest_frame_lores.size > 0:
                        self.__log.info("Camera frame received.")
                        return True
            time.sleep(check_interval)
        self.__log.error("Camera did not return a valid frame within timeout.")
        return False

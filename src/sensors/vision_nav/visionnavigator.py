import cv2
import threading
import logging
import numpy as np
import subprocess
import os
import time
import psutil
from queue import Queue, Empty, Full

from sensors.vision_nav.detector import Detector
from sensors.vision_nav.source import VideoSource, CameraSource, FileSource
from sensors.vision_nav.node import Node
from sensors.vision_nav.letter_ocr import detect_letter

class VisionNavigator:
    def __init__(self, video_source: VideoSource = CameraSource(), output_path='output.mp4', debug=False, goal_node='A'):
        self.__running = False
        self.__video_source = video_source
        self.__output_path = output_path
        self.__debug = debug
        self.__log = logging.getLogger(self.__class__.__name__)
        _, _, self.__fps = self.__video_source.get_info()

        self.__capture_queue = Queue(maxsize=20)
        self.__write_queue = Queue(maxsize=20)
        self.__node_stack = []
        self.__goal_node = goal_node
        self.__goal_node_detected = False
        self.__latest_main = None

        self.__writer_thread = None
        self.__capture_thread = None
        self.__process_thread = None
        self.__stopped_event = threading.Event()

    def __capture_and_write_loop(self):
        self.__capture_loop()
        self.__writer_loop()

    def start(self):
        self.__running = True
        self.__video_source.start()

        test_frame = self.__video_source.get_next_frame()
        if test_frame is None:
            raise RuntimeError("Kein Frame erhalten - Kamera-Start prüfen")
        frame_height, frame_width = test_frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.__out = cv2.VideoWriter(self.__output_path, fourcc, self.__fps, (frame_width, frame_height))
        if not self.__out.isOpened():
            self.__log.error("VideoWriter konnte nicht geöffnet werden!")

        self.__capture_thread = threading.Thread(target=self.__capture_loop)
        self.__process_thread = threading.Thread(target=self.__process_loop)
        if self.__debug: self.__writer_thread = threading.Thread(target=self.__writer_loop)

        self.__capture_thread.start()
        self.__process_thread.start()
        if self.__debug: self.__writer_thread.start()

    def __capture_loop(self):
        delay = 1.0 / self.__fps
        next_capture_time = time.time()

        while self.__running:
            now = time.time()
            sleep_time = next_capture_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Wenn wir im Rückstand sind, sofort weitermachen – aber nicht stapeln
                next_capture_time = now

            next_capture_time += delay

            frame_lores = self.__video_source.get_next_frame()
            frame_main = self.__video_source.get_main_frame()

            if frame_lores is None or frame_main is None:
                self.__log.warning("Kein gültiger Frame – Capture wird beendet.")
                # self.__running = False
                threading.Thread(target=self.stop, daemon=True).start()
                break

            self.__latest_main = frame_main

            try:
                self.__capture_queue.put((frame_lores, frame_main), timeout=0.01)
            except Full:
                try:
                    self.__capture_queue.get_nowait()  # Ältesten rauswerfen
                    self.__log.warning("Ältesten Frame entfernt – aktueller bevorzugt.")
                    self.__capture_queue.put_nowait((frame_lores, frame_main))
                except Empty:
                    self.__log.warning("Unerwartet: Queue war doch leer.")

    def __process_loop(self):
        detector = Detector(np.zeros((640, 640, 3), dtype=np.uint8))
        node = Node()
        frame_count = 0
        start_time = time.time()
        image_for_goal_node_detection = None

        while self.__running or not self.__capture_queue.empty():
            try:
                item = self.__capture_queue.get(timeout=1)
            except Empty:
                continue

            try:
                frame_lores, frame_main = item

                detector.update_frame(frame_lores)
                processed_frame, edges = detector.get_edges()
                if not image_for_goal_node_detection and edges:
                    image_for_goal_node_detection = processed_frame
                    letter, _ = detect_letter(image_for_goal_node_detection)
                    if letter and letter == self.__goal_node:
                        self.__set_goal_node_detected()

                if edges:
                    node.extend_edge_candidates(edges)

                if Detector.distinct_node_detected() and node.has_edge_candidates():
                    edge_angles = node.process()
                    self.__log.info(f'Node: {edge_angles}')
                    self.__node_stack.append(edge_angles)
                    node = Node()
                    image_for_goal_node_detection = None

                if self.__debug:
                    try:
                        self.__write_queue.put(processed_frame, timeout=0.1)
                    except Full:
                        self.__log.warning("Write-Queue voll – Frame verworfen.")

                    frame_count += 1
                    if frame_count % 50 == 0:
                        elapsed = time.time() - start_time
                        self.__log.info(f"FPS Ø: {frame_count / elapsed:.2f}")
                        mem = psutil.virtual_memory()
                        self.__log.info(f"RAM usage: {mem.percent}%")
            except Exception as e:
                self.__log.warning(f"Fehler in process_loop: {e}")
            finally:
                self.__capture_queue.task_done()  # <-- nur sicher, weil `get()` erfolgreich war

    def __writer_loop(self):
        while self.__running or not self.__write_queue.empty():
            try:
                frame = self.__write_queue.get(timeout=1)
                self.__out.write(frame)
                self.__write_queue.task_done()
                self.__log.debug("Frame written and task done.")
            except Empty:
                # Wenn running bereits False ist, warte einfach auf weitere Elemente
                if not self.__running:
                    time.sleep(0.1)
            except Exception as e:
                self.__log.warning(f"Writer thread error: {e}")

    def stop(self):
        self.__running = False

        if self.__debug: self.__write_queue.join()
        self.__capture_queue.join()

        self.__capture_thread.join()
        self.__process_thread.join()
        if self.__debug: self.__writer_thread.join()

        if self.__debug: self.cleanup()
        self.__stopped_event.set()

    def cleanup(self):
        self.__video_source.stop()

        if self.__out:
            self.__out.release()
            self.__log.info("VideoWriter released")

        if self.__debug: self.reencode_video()

    def get_data(self):
        try:
            data = self.__node_stack.pop()
        except IndexError:
            data = []
        return data
    
    def get_goal_node_reached(self) -> bool:
        return self.__goal_node_detected

    def is_running(self):
        return self.__running

    def reencode_video(self):
        fixed_output_path = f'fixed_{self.__output_path}'
        command = [
            "ffmpeg", "-y", "-loglevel", "error", "-err_detect", "ignore_err",
            "-i", self.__output_path,
            "-vf", f"setpts={2.0}*PTS",
            "-c:v", "libx264", "-preset", "ultrafast", "-crf", "23",
            "-c:a", "aac", "-b:a", "128k", fixed_output_path
        ]

        try:
            subprocess.run(command, check=True)
            self.__log.info(f"Video re-encoded: {fixed_output_path}")
            os.remove(self.__output_path)
            os.rename(fixed_output_path, self.__output_path)
            self.__log.info("Output file replaced with re-encoded version.")
        except subprocess.CalledProcessError as e:
            self.__log.error(f"FFmpeg failed: {e}")
        except OSError as e:
            self.__log.error(f"File error: {e}")

    def run_timed(self, seconds):
        if isinstance(self.__video_source, FileSource):
            self.__log.warning('"run_timed" is not supported for FileSource')
            return
        stop_timer = threading.Timer(seconds, self.stop)
        stop_timer.start()
        self.start()

    def start_with_keypress_to_stop(self):
        self.__stopped_event.clear()
        threading.Thread(target=self.__wait_for_keypress, daemon=True).start()
        self.start()
        self.__stopped_event.wait()

    def __wait_for_keypress(self):
        self.__log.info("Press ENTER to stop recording...")
        input()
        self.stop()
    
    def __set_goal_node_detected(self):
        self.__goal_node_detected = True

from queue import Empty
import threading
import time
import logging

from comms.manager import UartManager
from sensors.camera import CameraSensor
from sensors.vision_nav.visionnavigator import VisionNavigator
# from sensors.vision_nav.source import CameraSource
from sensors.lidar import LidarSensor
from brain.planner import PathPlanner
from brain.graph import Graph
from prain_uart import *

class HighLevelController:
    def __init__(self, uart_port: str, uart_baudrate: int, lidar_bus: int, lidar_address: int, target_node: str, logger: logging.Logger):
        self.uart_manager = UartManager(uart_port, uart_baudrate)

        # self.camera = CameraSensor(device_index=0)
        self.camera = VisionNavigator()
        self.lidar = LidarSensor(bus=lidar_bus, address=lidar_address)

        self.graph = Graph()
        self.planner = PathPlanner(graph=self.graph, target_node=target_node, logger=logger)
        self.logger = logger

        self.keep_running = True
        self._decision_thread = None

    def start(self):
        self.uart_manager.start()
        self.camera.start()
        self.lidar.start()

        self._decision_thread = threading.Thread(target=self._main_loop, daemon=True)
        self._decision_thread.start()

    def _main_loop(self):
        while self.keep_running:
            sensor_data = {
                "camera-angles": self.camera.get_data(),
                "lidar": self.lidar.get_data(),
            }

            inbound_data = []
            while True:
                try:
                    frame = self.uart_manager.rx_queue.get_nowait()
                except Empty:
                    break

                decoder = Decoder(frame)
                if decoder.verify_crc():
                    inbound_data.append((decoder.command, decoder.get_params()))
                    self.logger.debug(f"Received valid frame: cmd={decoder.command.name}, params={decoder.get_params()}")
                else:
                    self.logger.warning(f"Invalid CRC for frame: addr={frame.addr}, cmd={frame.cmd}")

            command, current_node = self.planner.next_action(sensor_data, inbound_data)
            if command is not None:
                self.uart_manager.send_frame(command)

                decoder = Decoder(command)
                if decoder.command.name == 'TURN':
                    time.sleep(1)

            time.sleep(0.05)

    def stop(self):
        self.keep_running = False
        if self._decision_thread:
            self._decision_thread.join()

        self.uart_manager.stop()
        self.camera.stop()
        self.lidar.stop()

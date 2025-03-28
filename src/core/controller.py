import threading
import time
from queue import Queue

from comms.manager import UartManager
from sensors.camera import CameraSensor
from sensors.lidar import LidarSensor
from brain.planner import PathPlanner

class HighLevelController:
    def __init__(self, uart_port: str, uart_baudrate: int, lidar_port: str = "/dev/ttyUSB1", lidar_baudrate: int = 115200):
        self.uart_manager = UartManager(uart_port, uart_baudrate)

        self.camera = CameraSensor(device_index=0)
        self.lidar = LidarSensor(port=lidar_port, baudrate=lidar_baudrate)

        self.planner = PathPlanner(graph={})
        self.current_node = None

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
            # 1) get sensor data
            frame = self.camera.get_latest_frame()
            lidar_data = self.lidar.get_latest_scan()
            sensor_data = {
                "frame": frame,
                "lidar": lidar_data,
            }

            # 2) process inbound uart
            while not self.uart_manager.rx_queue.empty():
                inbound_frame = self.uart_manager.rx_queue.get()
                # todo

            # 3) decide next action
            next_command = self.planner.next_action(self.current_node, sensor_data)
            if next_command is not None:
                self.uart_manager.send_frame(next_command)

            time.sleep(0.1)

    def stop(self):
        self.keep_running = False
        if self._decision_thread:
            self._decision_thread.join()

        self.uart_manager.stop()
        self.camera.stop()
        self.lidar.stop()

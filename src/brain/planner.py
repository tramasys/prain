from prain_uart import *

class PathPlanner:
    def __init__(self, graph=None):
        self.graph = graph if graph else {}
        self.current_node = None

    def next_action(self, current_node, sensor_data) -> Frame | None:
        barrier_detected = sensor_data.get("barrier_detected", False)
        if barrier_detected:
            return encode_stop(Address.MOTION_CTRL)
        else:
            return encode_move(Address.MOTION_CTRL)

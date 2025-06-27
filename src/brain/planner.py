from enum import Enum
import logging
import random
import numpy as np
import math
import networkx as nx
import time
import os
import uuid
import cv2
import json

from prain_uart import *
from brain.graph import Graph
from sensors.vision_nav.letter_ocr import detect_letter
from comms.goal_sound import PWMBuzzer
from comms.manager import UartManager
from sensors.lidar import LidarSensor
from sensors.vision_nav.visionnavigator import VisionNavigator, Node
from queue import Empty

class NavState(Enum):
    TRAVELING_EDGE      = 1
    WAITING_FOR_ANGLES  = 2
    RETRY_CAPTURE       = 9
    ARRIVED_AT_NODE     = 3
    DECIDING_NEXT_ANGLE = 4
    CHECK_NEXT_ANGLE    = 5
    GOAL_REACHED        = 6
    BLOCKED             = 7
    BEGIN               = 8

class PathPlanner:
    def __init__(
        self,
        target_node: str,
        logger: logging.Logger = logging.Logger(__name__),
        manager: UartManager | None = None,
        lidar: LidarSensor | None = None,
        camera: VisionNavigator | None = None,
        debug: bool=False
    ):

        self.logger = logger
        self.nxgraph = self._load_graph_from_json('src/brain/graph_config/graph_data.json')
        self.node_stack = []
        self.current_node = None
        self.target_node = target_node
        self.debug = debug
        self.buzzer = PWMBuzzer()

        self.state = NavState.BEGIN
        self.logger.info(f"Initialized planner at node {self.current_node} with target {self.target_node}")

        self.current_orientation = 0
        self.angles              = None
        self.last_chosen_angle   = None
        self.last_travelled_distance = 0
        self.estimated_goal_coordinates = {
            "A": (self.nxgraph.nodes["A"]["x"], self.nxgraph.nodes["A"]["y"]), 
            "B": (self.nxgraph.nodes["B"]["x"], self.nxgraph.nodes["B"]["y"]), 
            "C": (self.nxgraph.nodes["C"]["x"], self.nxgraph.nodes["C"]["y"])
        } # in mm (x, y)
        self.target_node_coordinates = self.estimated_goal_coordinates[target_node]
        self.current_position = (self.nxgraph.nodes["S"]["x"], self.nxgraph.nodes["S"]["y"]) # S coordinates of S (x, y)
        self.node_id_counter = 1
        self.first_node_reached = False
        self.node_detected_signal = False
        self.angles_locked = False

        self.uart_manager = manager
        self.lidar = lidar
        self.camera = camera

        self.wait_for_angles_start_time = None
        self.wait_for_angles_timeout = 3.0
        
        # Scoring constants
        self.SECTION_BOOST = 3
        self.WRONG_DIRECTION_PENALTY = 0.5
        self.DIRECTION_BOOST = 3
        self.RETURN_BOOST = 2
        self.FURTHER_AWAY_PENALTY = -2

    def next_action(self, sensor_data: dict, inbound_data) -> tuple[Frame | None, str]:
        angles = sensor_data.get("camera-angles", [])
        if angles and not self.angles_locked:
            self.angles = angles
            self.angles_locked = True

        lidar = sensor_data.get("lidar", (None, None, None))
        dist_cm, flux, _ = lidar
        
        self.logger.debug(f"[PLANNER] next_action called with persisted angles: {self.angles}, lidar: {lidar}")
        self._process_inbound_data(inbound_data)

        match self.state:
            
            case NavState.BEGIN:
                self.state = NavState.TRAVELING_EDGE
                return encode_move(Address.MOTION_CTRL, 0), self.current_node

            case NavState.TRAVELING_EDGE:
                if self.node_detected_signal:
                    if self.angles:
                        self.logger.info(f"[PLANNER] Node detected with angles: {self.angles}. Stopping and proceeding.")
                        self.state = NavState.ARRIVED_AT_NODE
                        return None, self.current_node
                    else:
                        self.logger.info("[PLANNER] Node detected, but no angles yet. Starting wait timer...")
                        self.state = NavState.WAITING_FOR_ANGLES
                        self.wait_for_angles_start_time = time.time()
                return None, self.current_node

            case NavState.WAITING_FOR_ANGLES:
                if self.angles:
                    self.logger.info(f"[PLANNER] Angles received: {self.angles}. Proceeding.")
                    self.state = NavState.ARRIVED_AT_NODE
                    return None, self.current_node

                if time.time() - self.wait_for_angles_start_time > self.wait_for_angles_timeout:
                    self.state = NavState.RETRY_CAPTURE
                
                return None, self.current_node

            case NavState.RETRY_CAPTURE:
                self.logger.info("PLANNER] Performing retry sequence...")

                new_angles = self._take_and_process_snapshot()

                if new_angles:
                    self.logger.info(f"PLANNER] Retry successful. New angles: {new_angles}")
                    self.angles = new_angles
                    self.state = NavState.ARRIVED_AT_NODE
                else:
                    self.logger.error("PLANNER] Retry failed to produce angles. Entering BLOCKED state.")
                    self.state = NavState.BLOCKED
                    return encode_stop(Address.MOTION_CTRL), self.current_node
                
                return None, self.current_node

            case NavState.ARRIVED_AT_NODE:
                time.sleep(0.5)
                self.node_detected_signal = False
                self.logger.info("[PLANNER] State: ARRIVED_AT_NODE")

                if len(self.node_stack) == 0:
                    # This should only happen at the very beginning
                    self.node_stack.append("S")
                    self.current_node = "S"
                    self.logger.info("PLANNER] Initialized at START node 'S'.")
                    self.logger.info(f"[PLANNER] Current node: {self.current_node}")
                else:
                    # We have arrived at a new node. Infer which one it is.
                    self._infer_and_update_current_node()
                    self.logger.info(f"[PLANNER] Current node: {self.current_node}")

                    
                if self.state == NavState.BLOCKED:
                    return encode_stop(Address.MOTION_CTRL), self.current_node

                self.logger.info(f"PLANNER] Node stack is now: {self.node_stack}")
                
                if self.current_node == self.target_node:
                    img = self.capture_img()
                    self.goal_reached(img)
                    self.state = NavState.GOAL_REACHED
                    self.buzzer.play_goal()
                    return encode_stop(Address.MOTION_CTRL), self.current_node
                
                self.state = NavState.DECIDING_NEXT_ANGLE
                return None, self.current_node

            case NavState.DECIDING_NEXT_ANGLE:
                self.logger.info("[PLANNER] DECIDING_NEXT_ANGLE state reached")

                if not self.angles:
                    self.state = NavState.BLOCKED
                    return encode_stop(Address.MOTION_CTRL), self.current_node
                
                angle_choice = self._choose_best_direction(self.angles)
                if angle_choice is None:
                    self.state = NavState.BLOCKED
                    return encode_stop(Address.MOTION_CTRL), self.current_node

                self.logger.info(f"[PLANNER] Chose angle: {angle_choice} from {self.angles}")
                turn_amount = (360 - angle_choice if angle_choice > 180 else -angle_choice) * 10

                self.last_turn_amount = turn_amount

                self._update_orientation(angle_choice)
                self.last_chosen_angle = angle_choice
                self.state = NavState.CHECK_NEXT_ANGLE
                self.logger.info(f'turn_amount: {turn_amount}, current orientation: {self.current_orientation}')

                return encode_turn(Address.MOTION_CTRL, turn_amount), self.current_node

            case NavState.CHECK_NEXT_ANGLE:                
                line_found = self.poll_line()
                if not line_found:
                    if self.last_chosen_angle in self.angles:
                        self.angles.remove(self.last_chosen_angle)
                    self.logger.info(f"[PLANNER] No line found after turning {self.last_chosen_angle}°")
                    self.last_chosen_angle = None
                    self.state = NavState.DECIDING_NEXT_ANGLE
                    return encode_turn(Address.MOTION_CTRL, -self.last_turn_amount), self.current_node
                
                self.logger.info(f"DIST: {dist_cm} cm, last angle: {self.last_chosen_angle}")
                
                sweep_dic = {
                    "1": ( "-40", False),
                    "2": ( "20", False),
                    "3": ( "20", False),
                    "4": ( "20", False),
                    "5": ( "20", False),
                    "6": ( "-40", False)
                }
                
                rel_turn = 0
                
                for sweep_id, (angle, is_done) in sweep_dic.items():
                    turn_amount = int(angle)
                    self.logger.info(f"TURN {turn_amount}")
                    rel_turn += turn_amount
                    ack_received = self.turn(turn_amount, 0.3)
                    if not ack_received:
                        self.logger.warning(f"[PLANNER] Turn {turn_amount}° not acknowledged")
                    dist, fl, _ = self.lidar.get_data()
                    self.logger.info(f"[PLANNER] Lidar data after turn {sweep_id}: dist={dist} mm, flux={fl}")
                
                    if dist >= 45 and dist <= 205 and fl > 2000:
                        self.turn(-rel_turn, 2)
                        self.logger.info("PYLON ALERT!!!!!")
                        if self.last_chosen_angle in self.angles:
                            self.angles.remove(self.last_chosen_angle)
                        self.last_chosen_angle = None
                        self.state = NavState.DECIDING_NEXT_ANGLE
                        return encode_turn(Address.MOTION_CTRL, -self.last_turn_amount), self.current_node
                
                self.logger.info("[PLANNER] CHECK_NEXT_ANGLE state reached")
                self.state = NavState.TRAVELING_EDGE
                self.angles = None
                self.angles_locked = False
                return encode_move(Address.MOTION_CTRL, 0), self.current_node

            case NavState.GOAL_REACHED:
                self.logger.info("[PLANNER] GOAL_REACHED state reached")
                return encode_stop(Address.MOTION_CTRL), self.current_node

            case NavState.BLOCKED:
                self.logger.info(f"[PLANNER] BLOCKED at {self.current_node}")
                return encode_stop(Address.MOTION_CTRL), self.current_node

            case _:
                self.logger.warning("[PLANNER] Unhandled state")
                return encode_stop(Address.MOTION_CTRL), self.current_node

    def _get_previous_node(self) -> str:
        """Peek last visited node without removing it."""
        if self.node_stack:
            return self.node_stack[-1]
        return None
    
    def _simulate_step_from(self, position, angle_deg, step_length):
        """
        Converts a step in a given angle (camera frame: 0° = up, clockwise) to Cartesian.
        """
        angle_rad = math.radians((90 - angle_deg) % 360)
        dx = step_length * math.cos(angle_rad)
        dy = step_length * math.sin(angle_rad)
        return (position[0] + dx, position[1] + dy)

    def _distance_to_target(self, pos):
        return math.sqrt((pos[0] - self.target_node_coordinates[0])**2 + (pos[1] - self.target_node_coordinates[1])**2)

    # def _choose_best_direction(self, detected_angles, step_length=500) -> int:
    #     best_angle = None
    #     min_distance = float('inf')
    #     for angle in detected_angles:
    #         orientation_adjusted_angle = int((angle + self.current_orientation) % 360)
    #         new_pos = self._simulate_step_from(self.current_position, orientation_adjusted_angle, step_length)
    #         dist = self._distance_to_target(new_pos)
    #         self.logger.debug(f"[PLANNER] Angle {angle}° → Step to {new_pos} → Distance to goal: {dist:.2f} mm")
            
    #         if dist < min_distance:
    #             min_distance = dist
    #             best_angle = angle
        
    #     self.logger.info(f"[PLANNER] chosen angle: {best_angle}")
    #     return best_angle
    
    def _update_orientation(self, angle_choice) -> int:
        self.current_orientation = int((self.current_orientation + angle_choice) % 360)
        
    def _update_position(self):
        angle_rad = math.radians((90 - self.current_orientation) % 360)
        dx = self.last_travelled_distance * math.cos(angle_rad)
        dy = self.last_travelled_distance * math.sin(angle_rad)
        x, y = self.current_position
        self.current_position = (x + dx, y + dy)
        self.logger.debug(f"[PLANNER] Updated position to {self.current_position} after moving {self.last_travelled_distance} mm at {self.current_orientation}°")
        
    def _resolve_node_by_position(self, tolerance=200, skip_node=None) -> str | None:
        for node, pos in self.nxgraph.nodes.data('position'):
            if node == skip_node:
                continue
            dist = math.hypot(self.current_position[0] - pos[0], self.current_position[1] - pos[1])
            if dist <= tolerance:
                return node
        return None
    
    def _is_close_to_target(self, threshold: float = 400.0) -> bool:
        """
        Returns True if the current position is within `threshold` mm of the target node coordinates.
        """
        tx, ty = self.target_node_coordinates
        cx, cy = self.current_position
        distance = math.hypot(cx - tx, cy - ty)

        self.logger.debug(f"[PLANNER] Distance to target node {self.target_node}: {distance:.2f} mm")
        
        is_beyond = False
        match self.target_node:
            case "A": is_beyond = (cx >= tx and cy >= ty)
            case "B": is_beyond = (tx - threshold <= cx <= tx + threshold and cy >= ty)
            case _: is_beyond = (cx <= tx and cy >= ty)

        is_in_threshold = distance <= threshold
        return is_in_threshold or is_beyond

        
    def _process_inbound_data(self, inbound_data):
        for cmd, params in inbound_data:
            print(f"[PLANNER] Processing inbound data: {cmd.name} with params: {params}")
            match cmd:
                case Command.RESPONSE:
                    self._handle_response(params)
                case Command.INFO:
                    self._handle_info(params)
                case _:
                    self.logger.warning(f"Unhandled frame: {cmd.name} with {params}")
                    
    def _handle_response(self, params: ResponseParams):
        if isinstance(params, ResponseParams) and params.poll_id == PollId.DISTANCE.value:
            self.logger.info(f"[PLANNER] Last travelled distance was: {params.data}")
            self.last_travelled_distance = params.data
        else:
            self.logger.warning(f"[PLANNER] RESPONSE ignored – unexpected poll_id: {params.poll_id}")
            
    def _handle_info(self, params: InfoParams):
        if params.flag == InfoFlag.NODE_DETECTED.value:
            self.logger.info(f'[PLANNER] Node detected from Motion Controller')
            self.node_detected_signal = True
        if params.flag == InfoFlag.MOTION_DONE.value:
            self.logger.info(f'[PLANNER] Motion done from Motion Controller')

    def _generate_new_id(self) -> str:
        node_id = f"n{self.node_id_counter}"
        self.node_id_counter += 1
        return node_id
    
    def goal_reached(self, img) -> bool:
        """
        Returns True if the target node has been reached.
        """
        self.logger.debug(f"[PLANNER] Checking if goal node {self.target_node} is reached...")
        letter, _ = detect_letter(img)
        self.logger.debug(f"[PLANNER] Detected letter: {letter}")
        if letter == self.target_node:
            self.logger.info(f"[PLANNER] Goal node {self.target_node} reached with letter {letter} at position {self.current_position}")
            return True
        self.logger.info(f"[PLANNER] Goal node {self.target_node} not reached yet. Continue navigating...")
        return False
    
    def turn(self, angle: int, timeout: float) -> bool:
        """
        Dreht den Roboter und wartet auf den Abschluss der Bewegung.
        """
        self.logger.info(f"Sende TURN-Befehl ({angle} Zehntelgrad).")
        try:
            self.uart_manager.clear_ack_queue()
            frame = encode_turn(Address.MOTION_CTRL, angle)
            self.uart_manager.send_frame(frame)
            acknowledged = self.await_acknowledgement()
            if acknowledged:
                time.sleep(timeout)
                return True
            # KORREKT: Warte auf TURN_DONE Bestätigung
            return False
        except Exception as e:
            self.logger.error(f"Fehler beim Senden des TURN-Befehls: {e}")
            return False

    def move(self, distance: int, timeout: float) -> bool:
        """
        Bewegt den Roboter und wartet auf den Abschluss der Bewegung.
        """
        self.logger.info(f"Sende MOVE-Befehl ({distance} mm).")
        try:
            self.uart_manager.clear_ack_queue()
            frame = encode_move(Address.MOTION_CTRL, distance)
            self.uart_manager.send_frame(frame)
            acknowledged = self.await_acknowledgement()
            if acknowledged:
                time.sleep(timeout)
                return True
            return False
        except Exception as e:
            self.logger.error(f"Fehler beim Senden des MOVE-Befehls: {e}")
            return False
        
    def poll_line(self, timeout: float = 2.0) -> bool:
        """
        Polls the line sensor to check if the robot is on a line.
        Returns True if the robot is on a line, False otherwise.
        """
        try:
            self.uart_manager.clear_line_poll_queue()
            self.uart_manager.clear_ack_queue()
            frame = encode_poll(Address.MOTION_CTRL, PollId.LINE_SENSOR)
            self.uart_manager.send_frame(frame)
            acknowledged = self.await_acknowledgement()
            if not acknowledged:
                return False
            success = self.await_line_poll()
            time.sleep(1)
            return success
        except Exception as e:
            self.logger.error(f"Fehler beim Senden des MOVE-Befehls: {e}")
            return False

    @property
    def current_graph(self) -> nx.Graph:
        return self.nxgraph

    @property
    def current_positions(self) -> dict:
        return self.node_positions

    def save_img(self, img, directory="images") -> str | None:
        """
        Saves the given image to the specified directory with a unique filename.
        Returns the path to the saved image or None if saving failed.
        """
        if not os.path.exists(directory):
            os.makedirs(directory)

        filename = f"{directory}/node_{self.current_node}_{uuid.uuid4()}.png"
        try:
            cv2.imwrite(filename, img)
            self.logger.info(f"Image saved as {filename}")
            return filename
        except Exception as e:
            self.logger.error(f"Failed to save image: {e}")
            return None

    def capture_img(self, distance: int = 350) -> np.ndarray | None:
        """
        Captures the current best node image and saves it.
        """
        self.logger.info(f"[PLANNER] Ich werde JETZT move({distance}) aufrufen!")

        moved_back = self.move(-distance, 4.0)
        img = self.camera.capture_img()
        moved_forward = self.move(distance, 4.0)
        self.save_img(img)
        return img
    
    def await_acknowledgement(self, timeout: float = 7.0) -> bool:
        """
        Wartet auf einen Bestätigungs-Frame aus der dedizierten ack_queue.
        Diese Version behandelt den Fall, dass das ACK bereits angekommen ist,
        bevor das Warten beginnt.
        """
        self.logger.info(f"Warte auf Bestätigung: 'InfoFlag.ACK' (Timeout: {timeout}s)")
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                inbound_frame = self.uart_manager.ack_queue.get_nowait()
                self.logger.info(f"Bestätigung 'InfoFlag.ACK' empfangen. Erfolg!")
                return True

            except Empty:
                time.sleep(0.05)
                continue
            except Exception as e:
                self.logger.error(f"Unerwarteter Fehler beim Holen aus der ack_queue: {e}")
                break 

        self.logger.warning(f"Timeout! 'InfoFlag.ACK' nicht innerhalb von {timeout} Sekunden empfangen.")
        return False
    
    def await_line_poll(self, timeout: float = 2.0) -> bool:
        """
        Wartet auf eine Antwort vom Line Sensor Poll.
        Returns True if the robot is on a line, False otherwise.
        """
        self.logger.info(f"Warte auf Line Poll (Timeout: {timeout}s)")
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                inbound_frame = self.uart_manager.line_poll_queue.get_nowait()
                decoder = Decoder(inbound_frame)
                params = decoder.get_params()
                if decoder.command == Command.RESPONSE and params.poll_id == PollId.LINE_SENSOR.value:
                    self.logger.info("Line sensor poll received. Success!")
                    return True
            except Empty:
                time.sleep(0.05)
                continue
            except Exception as e:
                self.logger.error(f"Unerwarteter Fehler beim Holen aus der line_poll_queue: {e}")
                break 

        self.logger.warning(f"Timeout! Line sensor poll nicht innerhalb von {timeout} Sekunden empfangen.")
        return False
    
    def _take_and_process_snapshot(self) -> list[int]:
        """
        Nimmt ein einzelnes Bild auf und verarbeitet es, um Winkel zu finden.
        Diese Funktion ist für den Wiederholungsversuch gedacht.
        """
        self.logger.info("Taking and processing a snapshot for angles...")
        
        # Wichtig: Wir verwenden hier die capture_img des VisionNavigators,
        # die KEINE Roboterbewegung auslöst.
        img = self.capture_img()
        if img is None:
            self.logger.warning("Snapshot failed, no image captured.")
            return None

        # Wir verwenden die existierende Logik aus dem Detector, um Kanten zu finden
        _, edges, _ = self.camera.process_single_image(img)
        
        node = Node()
        node.extend_edge_candidates(edges)
        angles = node.process()
        
        self.logger.info(f"Snapshot processed, detected angles: {angles}")
        return angles
    
    def _load_graph_from_json(self, file_path: str) -> nx.Graph:
        """Loads the graph from the specified JSON file."""
        # This function looks correct, but let's ensure coordinates are numeric.
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.logger.error(f"CRITICAL: Failed to load graph from {file_path}: {e}")
            return nx.Graph()

        G = nx.Graph()
        for node_id, attrs in data.get('nodes', {}).items():
            # Ensure coordinates are numbers and apply any scaling factor if necessary
            # Assuming your JSON has grid units and you want to work in mm
            scale_factor = 400 
            x = float(attrs.get('x', 0)) * scale_factor
            y = float(attrs.get('y', 0)) * scale_factor
            G.add_node(node_id, x=x, y=y)

        for edge in data.get('edges', []):
            if 'from' in edge and 'to' in edge:
                G.add_edge(edge['from'], edge['to'])
        
        self.logger.info(f"Graph loaded successfully with {G.number_of_nodes()} nodes and {G.number_of_edges()} edges.")
        return G

    def get_node_coordinates(self, node_id: str):
        # Greift auf die Daten zu, die im Graphen selbst gespeichert sind
        return self.nxgraph.nodes[node_id]['x'], self.nxgraph.nodes[node_id]['y']
    
    def _infer_and_update_current_node(self, tolerance_degrees: float = 10.0):
        """
        Infers the current node based on the last known node and the travel angle.
        This is the heart of the graph-based localization.
        """
        if not self.node_stack:
            self.state = NavState.BLOCKED
            self.logger.error("Cannot infer node, node stack is empty!")
            return

        previous_node_id = self.node_stack[-1]
        prev_coords = self.nxgraph.nodes[previous_node_id]

        # The absolute direction the robot was traveling in
        travel_direction = self.current_orientation

        best_match_node = None
        smallest_angle_diff = float('inf')

        for neighbor_id in self.nxgraph.neighbors(previous_node_id):
            neighbor_coords = self.nxgraph.nodes[neighbor_id]

            # Calculate the true world angle from the previous node to this neighbor
            dx = neighbor_coords['x'] - prev_coords['x']
            dy = neighbor_coords['y'] - prev_coords['y']
            
            # atan2(dy, dx) gives angle from +X axis (East). We convert to our system (0° is +Y).
            expected_direction_rad = math.atan2(dy, dx)
            expected_direction_deg = (450 - math.degrees(expected_direction_rad)) % 360

            # Calculate the smallest difference between the robot's travel direction and the expected direction
            diff = 180 - abs(abs(travel_direction - expected_direction_deg) - 180)

            self.logger.debug(f"Inferring from '{previous_node_id}': Travel_dir={travel_direction:.1f}°. Neighbor '{neighbor_id}' expected_dir={expected_direction_deg:.1f}°. Diff={diff:.1f}°")

            if diff < smallest_angle_diff:
                smallest_angle_diff = diff
                best_match_node = neighbor_id

        if best_match_node and smallest_angle_diff <= tolerance_degrees:
            self.current_node = best_match_node
            self.node_stack.append(self.current_node)
            self.logger.info(f"SUCCESS: Inferred arrival at node '{self.current_node}'. (Angle diff: {smallest_angle_diff:.1f}°)")
        else:
            self.state = NavState.BLOCKED
            self.logger.error(f"FAILURE: Could not infer destination. Best match '{best_match_node}' was off by {smallest_angle_diff:.1f}°. Blocking.")
            
    def _choose_best_direction(self, detected_angles: list[int]) -> int | None:
        """
        Chooses the best camera angle by finding the shortest path on the graph
        and matching the angle to the next node on that path.
        """
        # 1. Find the shortest path to the final target
        try:
            path = nx.shortest_path(self.nxgraph, source=self.current_node, target=self.target_node)
            if len(path) < 2:
                self.logger.warning(f"Already at target or no path found. Path: {path}")
                return None
            next_hop_node = path[1]
            self.logger.info(f"Path to target: {path}. Next hop is '{next_hop_node}'.")
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            self.logger.error(f"No path found from '{self.current_node}' to '{self.target_node}'. Blocking.")
            return None

        # 2. Calculate the absolute world angle to the next hop node
        current_coords = self.nxgraph.nodes[self.current_node]
        next_hop_coords = self.nxgraph.nodes[next_hop_node]
        dx = next_hop_coords['x'] - current_coords['x']
        dy = next_hop_coords['y'] - current_coords['y']
        
        target_direction_rad = math.atan2(dy, dx)
        absolute_target_angle = (450 - math.degrees(target_direction_rad)) % 360

        # 3. Convert this to a relative angle for the robot
        relative_target_angle = (absolute_target_angle - self.current_orientation) % 360

        # 4. Find the detected camera angle that is closest to our ideal relative angle
        best_angle_choice = None
        smallest_diff = float('inf')
        self.logger.debug(f"Robot at {self.current_orientation:.1f}°. Needs to face {absolute_target_angle:.1f}°. Relative turn needed: {relative_target_angle:.1f}°")

        for angle in detected_angles:
            diff = 180 - abs(abs(angle - relative_target_angle) - 180)
            if diff < smallest_diff:
                smallest_diff = diff
                best_angle_choice = angle

        if best_angle_choice is not None:
             self.logger.info(f"Best camera angle for next hop is {best_angle_choice}° (diff to ideal: {smallest_diff:.1f}°)")
        else:
             self.logger.error("No angle found that matches path.")

        return best_angle_choice
    
    def print_graph(self):
        print("=== Graph Summary ===")
        print(f"Nodes ({self.nxgraph.number_of_nodes()}):")
        for node, data in self.nxgraph.nodes(data=True):
            pos = f"x={data.get('x', '?')}, y={data.get('y', '?')}"
            print(f"  {node}: {pos}")

        print(f"\nEdges ({self.nxgraph.number_of_edges()}):")
        for u, v in self.nxgraph.edges():
            print(f"  {u} -- {v}")

        
        
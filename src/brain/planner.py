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

from prain_uart import *
from brain.graph import Graph
from sensors.vision_nav.letter_ocr import detect_letter
from comms.goal_sound import PWMBuzzer
from comms.manager import UartManager
from sensors.lidar import LidarSensor
from sensors.vision_nav.visionnavigator import VisionNavigator
from queue import Empty

class NavState(Enum):
    TRAVELING_EDGE      = 1
    ARRIVED_AT_NODE     = 3
    DECIDING_NEXT_ANGLE = 4
    CHECK_NEXT_ANGLE    = 5
    GOAL_REACHED        = 6
    BLOCKED             = 7
    BEGIN               = 8

class PathPlanner:
    def __init__(
        self,
        graph: Graph,
        target_node: str,
        logger: logging.Logger,
        manager: UartManager | None = None,
        lidar: LidarSensor | None = None,
        camera: VisionNavigator | None = None,
        debug: bool=False
    ):

        self.graph = graph
        self.nxgraph = nx.Graph()
        self.node_stack = []
        self.current_node = "S"
        self.target_node = target_node
        self.logger = logger
        self.debug = debug
        self.buzzer = PWMBuzzer()

        self.state = NavState.BEGIN
        self.logger.info(f"Initialized planner at node {self.current_node} with target {self.target_node}")

        self.current_orientation = 0
        self.angles              = None
        self.last_chosen_angle   = None
        self.last_travelled_distance = 0
        self.estimated_goal_coordinates = {"A": (3118, 2700), "B": (1559, 3600), "C": (0, 2700)} # in mm (x, y)
        self.target_node_coordinates = self.estimated_goal_coordinates[target_node]
        self.current_position = (1559, 0) # S coordinates of S (x, y)
        self.nxgraph.add_node(self.current_node, position=self.current_position)
        self.node_id_counter = 1
        self.first_node_reached = False
        self.best_node_image = None

        self.uart_manager = manager
        self.lidar = lidar
        self.camera = camera

        # Scoring constants
        self.SECTION_BOOST = 3
        self.WRONG_DIRECTION_PENALTY = 0.5
        self.DIRECTION_BOOST = 3
        self.RETURN_BOOST = 2
        self.FURTHER_AWAY_PENALTY = -2

    def next_action(self, sensor_data: dict, inbound_data) -> tuple[Frame | None, str]:
        angles = sensor_data.get("camera-angles", [])
        if angles:
            self.angles = angles

        lidar = sensor_data.get("lidar", (None, None, None))
        dist_cm, flux, _ = lidar
        
        best_node_image = sensor_data.get("best-node-image", None)
        if best_node_image is not None:
            self.best_node_image = best_node_image

        self.logger.info(f"[PLANNER] next_action called with persisted angles: {self.angles}, lidar: {lidar}")
        self._process_inbound_data(inbound_data)

        match self.state:
            
            case NavState.BEGIN:
                self.logger.debug("[PLANNER] BEGIN state reached")
                self.state = NavState.TRAVELING_EDGE
                return encode_move(Address.MOTION_CTRL, 0), self.current_node

            case NavState.TRAVELING_EDGE:
                self.logger.debug("[PLANNER] TRAVELING_EDGE state reached")

                if self.angles:
                    self.state = NavState.ARRIVED_AT_NODE
                    return None, self.current_node

                return None, self.current_node

            case NavState.ARRIVED_AT_NODE:
                time.sleep(2)
                
                img = self.capture_img()
                if img is None:
                    self.logger.warning("[PLANNER] No image captured.")
                    return None, self.current_node
                
                # Goal-Check
                if self.goal_reached(img):
                    self.state = NavState.GOAL_REACHED
                    self.buzzer.play_goal()
                    self.logger.info("[PLANNER] Goal reached")
                    return encode_stop(Address.MOTION_CTRL), self.current_node
                
                self.logger.info("[PLANNER] ARRIVED_AT_NODE reached")

                # --- 1) Entry-Stub: beim ersten Mal nur das Flag setzen und S in den Stack packen ---
                if not self.first_node_reached:
                    self.first_node_reached = True
                    # Startknoten S bleibt bei (1732, 0)
                    self.node_stack.append(self.current_node)
                    self.logger.info("[PLANNER] Entry at S abgeschlossen – weiterhin in DECIDING_NEXT_ANGLE")
                    self.state = NavState.DECIDING_NEXT_ANGLE
                    return None, self.current_node

                # --- 2) Ab jetzt: echte Position aktualisieren und neuen Knoten anlegen ---
                # berechne new position
                self._update_position()
                self.logger.debug(f"[PLANNER] New position: {self.current_position}")

                # neuen Knoten erzeugen
                new_node = self._generate_new_id()
                self.current_node = new_node
                self.nxgraph.add_node(new_node, position=self.current_position)
                self.logger.info(f"[PLANNER] Created node '{new_node}' at {self.current_position}")

                # Edge vom vorherigen zum aktuellen Knoten
                prev = self.node_stack[-1]
                self.nxgraph.add_edge(prev, new_node, weight=self.last_travelled_distance)
                self.logger.debug(f"[PLANNER] Added edge {prev}→{new_node} ({self.last_travelled_distance} mm)")

                # Stack & weiter zum Angle-Deciding
                self.node_stack.append(new_node)
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
                self.logger.info(f"DIST: {dist_cm} cm, last angle: {self.last_chosen_angle}")
                
                time.sleep(1)
                
                sweep_dic = {
                    "1": ( "-100", False),
                    "2": ( "50", False),
                    "3": ( "50", False),
                    "4": ( "50", False),
                    "5": ( "50", False),
                    "6": ( "-100", False)
                }
                
                rel_turn = 0
                
                for sweep_id, (angle, is_done) in sweep_dic.items():
                    turn_amount = int(angle)
                    self.logger.info(f"TURN {turn_amount}")
                    rel_turn += turn_amount
                    self.turn(turn_amount)
                    dist, fl, _ = self.lidar.get_data()
                
                    if dist >= 45 and dist <= 205 and fl > 2000:
                        self.turn(-rel_turn)
                        self.logger.info("PYLON ALERT!!!!!")
                        self.angles.remove(self.last_chosen_angle)
                        self.state = NavState.DECIDING_NEXT_ANGLE
                        return encode_turn(Address.MOTION_CTRL, -self.last_turn_amount), self.current_node
                
                self.logger.info("[PLANNER] CHECK_NEXT_ANGLE state reached")
                self.state = NavState.TRAVELING_EDGE
                self.angles = None
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

    def _choose_best_direction(self, detected_angles, step_length=500) -> int:
        best_angle = None
        min_distance = float('inf')
        for angle in detected_angles:
            orientation_adjusted_angle = int((angle + self.current_orientation) % 360)
            new_pos = self._simulate_step_from(self.current_position, orientation_adjusted_angle, step_length)
            dist = self._distance_to_target(new_pos)
            self.logger.debug(f"[PLANNER] Angle {angle}° → Step to {new_pos} → Distance to goal: {dist:.2f} mm")
            
            if dist < min_distance:
                min_distance = dist
                best_angle = angle
        
        self.logger.info(f"[PLANNER] chosen angle: {best_angle}")
        return best_angle
    
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

        return distance <= threshold

        
    def _process_inbound_data(self, inbound_data):
        for cmd, params in inbound_data:
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
        if params.flag == InfoFlag.NODE_DETECTED:
            self.logger.info(f'[PLANNER] Node detected from Motion Controller')
        if params.flag == InfoFlag.MOTION_DONE:
            self.logger.info(f'[PLANNER] Motion done from Motion Controller')

    def _generate_new_id(self) -> str:
        node_id = f"n{self.node_id_counter}"
        self.node_id_counter += 1
        return node_id
    
    def goal_reached(self, img) -> bool:
        """
        Returns True if the target node has been reached.
        """
        if self.best_node_image is None:
            return False
        
        self.logger.debug(f"[PLANNER] Checking if goal node {self.target_node} is reached...")
        letter, _ = detect_letter(img)
        self.logger.debug(f"[PLANNER] Detected letter: {letter}")
        if letter == self.target_node:
            self.logger.info(f"[PLANNER] Goal node {self.target_node} reached with letter {letter} at position {self.current_position}")
            return True
        self.logger.info(f"[PLANNER] Goal node {self.target_node} not reached yet. Continue navigating...")
        return False
    
    def turn(self, angle: int) -> bool:
        """
        Dreht den Roboter und wartet auf den Abschluss der Bewegung.
        """
        self.logger.info(f"Sende TURN-Befehl ({angle} Zehntelgrad).")
        try:
            frame = encode_turn(Address.MOTION_CTRL, angle)
            self.uart_manager.send_frame(frame)
            # KORREKT: Warte auf TURN_DONE Bestätigung
            return self.await_acknowledgement(InfoFlag.TURN_DONE)
        except Exception as e:
            self.logger.error(f"Fehler beim Senden des TURN-Befehls: {e}")
            return False

    def move(self, distance: int) -> bool:
        """
        Bewegt den Roboter und wartet auf den Abschluss der Bewegung.
        """
        # DIESES LOGGING HAT GEFEHLT:
        self.logger.info(f"Sende MOVE-Befehl ({distance} mm).")
        try:
            frame = encode_move(Address.MOTION_CTRL, distance)
            self.uart_manager.send_frame(frame)
            # KORREKT: Warte auf MOTION_DONE Bestätigung statt time.sleep()
            return self.await_acknowledgement(InfoFlag.MOTION_DONE)
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
        moved_back = self.move(-350)
        img = self.camera.capture_img()
        moved_forward = self.move(350)
        self.save_img(img)
        return img
    
    def await_acknowledgement(self, timeout: float = 10.0) -> bool:
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                inbound_frame = self.uart_manager.rx_queue.get_nowait()
                
                try:
                    decoder = Decoder(inbound_frame)
                    if decoder.command == Command.INFO:
                        params = decoder.get_params()

                        if isinstance(params, InfoParams) and params.flag == InfoFlag.ACK:
                            return True
                        else:
                            self.logger.debug(f"Received INFO frame, but not the one we're waiting for. Got: {params.flag.name}")

                except Exception as e:
                    self.logger.error(f"Error decoding incoming frame while waiting for ack: {e}")

            except Empty:
                time.sleep(0.05)
                continue
            except Exception as e:
                self.logger.error(f"Error while waiting for acknowledgement: {e}")
                return False

        self.logger.warning(f"Timeout! Did not receive '{InfoFlag.ACK.name}' within {timeout} seconds.")
        return False

from enum import Enum
import logging
import random
import numpy as np
import math
import networkx as nx
import time

from prain_uart import *
from brain.graph import Graph
from sensors.vision_nav.letter_ocr import detect_letter
from comms.goal_sound import PWMBuzzer
from comms.manager import UartManager
from sensors.lidar import LidarSensor

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

        self.uart_manager = manager
        self.lidar = lidar

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
        goal_node_reached = sensor_data.get("goal-node-reached", False)

        self.logger.info(f"[PLANNER] next_action called with persisted angles: {self.angles}, lidar: {lidar}")
        self._process_inbound_data(inbound_data)
        print(f'Goal node reached: {goal_node_reached}')

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

                # Goal-Check
                if goal_node_reached:
                    self.state = NavState.GOAL_REACHED
                    self.buzzer.play_goal()
                    self.logger.info("[PLANNER] Goal reached")
                    return encode_stop(Address.MOTION_CTRL), self.current_node

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
                    frame = encode_turn(Address.MOTION_CTRL, turn_amount)
                    self.uart_manager.send_frame(frame)                                
                    time.sleep(1)
                    dist, fl, _ = self.lidar.get_data()
                
                    if dist >= 45 and dist <= 205 and fl > 2000:
                        equalising_frame = encode_turn(Address.MOTION_CTRL, -rel_turn)
                        self.uart_manager.send_frame(equalising_frame)
                        time.sleep(1)
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


    # def _sort_angles_by_pathfinding(self, angles: list[int]) -> list[int]:
    #     """Sort angles based on section scoring and hop distance to target_node."""
    #     target_section = self.graph.get_node_section(self.target_node)
    #     current_section = self.graph.get_node_section(self.current_node)

    #     angle_scores = []
    #     for angle in angles:
    #         potential_node = self._infer_potential_node(self.current_node, angle)
    #         if potential_node and potential_node not in self.visited_nodes:
    #             score = self._score_node(potential_node, target_section, current_section)
    #             angle_scores.append((angle, score))
    #         else:
    #             angle_scores.append((angle, -float('inf')))

    #     angle_scores.sort(key=lambda x: x[1], reverse=True)
    #     sorted_angles = [angle for angle, _ in angle_scores]

    #     self.logger.debug(f"Sorted angles at {self.current_node}: {sorted_angles}")
    #     return sorted_angles

    # def _get_random_angle(self, angles: list[int]) -> int:
    #     return random.choice(angles)

    # def _score_node(self, next_node: str, target_section: str, current_section: str) -> float:
    #     """Score a node based on section and hop distance."""
    #     next_section = self.graph.get_node_section(next_node)

    #     hop_distance_to_target = self._estimate_hop_distance(next_node, self.target_node)
    #     hop_distance_from_current = self._estimate_hop_distance(self.current_node, self.target_node)
    #     direction_score = 1.0 if hop_distance_to_target < hop_distance_from_current else -1.0

    #     target_distance = self._get_section_distance(next_section, target_section)
    #     current_distance = self._get_section_distance(current_section, target_section)

    #     score = 0
    #     if next_section == target_section:
    #         score += self.SECTION_BOOST
    #     if target_distance < current_distance:
    #         score += self.RETURN_BOOST
    #     elif target_distance > current_distance:
    #         score += self.FURTHER_AWAY_PENALTY

    #     if target_distance == current_distance:
    #         directional_score = self.DIRECTION_BOOST if direction_score > 0 else self.WRONG_DIRECTION_PENALTY
    #         score += directional_score

    #     return score

    # def _estimate_hop_distance(self, start_node: str, target_node: str) -> int:
    #     """Estimate hop distance to target_node using BFS."""
    #     if start_node == target_node:
    #         return 0
    #     visited = {start_node}
    #     queue = [(start_node, 0)]  # (node, distance)
    #     while queue:
    #         node, dist = queue.pop(0)
    #         for neighbor, data in self.graph.get_neighbors(node).items():
    #             if data.get("traversable") and neighbor not in visited and not self.graph.is_node_blocked(neighbor):
    #                 if neighbor == target_node:
    #                     return dist + 1
    #                 visited.add(neighbor)
    #                 queue.append((neighbor, dist + 1))
    #     return float('inf')

    # def _get_section_distance(self, from_section: str, to_section: str) -> int:
    #     """Calculate distance between sections."""
    #     sections = ["left", "middle", "right"]
    #     return abs(sections.index(from_section) - sections.index(to_section))
    
    # def _get_section_from_position(self, position: tuple[int, int]) -> str:
    #     x, _ = position
    #     if x < 866:
    #         return "left"
    #     elif x < 2598:
    #         return "middle"
    #     return "right"

    # def _infer_next_node(self, current_node: str, angle: int) -> str:
    #     """Infer the next node after moving along an angle."""
    #     neighbors = self.graph.get_neighbors(current_node)
    #     viable_neighbors = [n for n, d in neighbors.items() if d.get("traversable") and n not in self.visited_nodes]

    #     if not viable_neighbors:
    #         self.logger.debug(f"No viable neighbors from {current_node}, staying put")
    #         return current_node

    #     chosen = min(viable_neighbors, key=lambda n: self._estimate_hop_distance(n, self.target_node), default=current_node)
    #     self.logger.debug(f"Inferred next node from {current_node} with angle {angle}: {chosen}")
    #     return chosen

    # def _infer_potential_node(self, current_node: str, angle: int) -> str | None:
    #     """Infer a potential next node for scoring, without direct angle mapping."""
    #     neighbors = self.graph.get_neighbors(current_node)
    #     viable_neighbors = [n for n, d in neighbors.items() if d.get("traversable") and n not in self.visited_nodes]
    #     if not viable_neighbors:
    #         return None
    #     return min(viable_neighbors, key=lambda n: self._estimate_hop_distance(n, self.target_node), default=None)

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

    def _generate_new_id(self) -> str:
        node_id = f"n{self.node_id_counter}"
        self.node_id_counter += 1
        return node_id

    @property
    def current_graph(self) -> nx.Graph:
        return self.nxgraph

    @property
    def current_positions(self) -> dict:
        return self.node_positions





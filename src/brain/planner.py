from enum import Enum
import logging

from prain_uart import *
from brain.graph import Graph

class NavState(Enum):
    TRAVELING_EDGE              = 1
    ARRIVED_AT_POTENTIAL_NODE   = 2
    ARRIVED_AT_NODE             = 3
    DECIDING_NEXT_ANGLE         = 4
    CHECK_NEXT_ANGLE            = 5
    GOAL_REACHED                = 6
    BLOCKED                     = 7

class PathPlanner:
    def __init__(self, graph: Graph, target_node: str, logger: logging.Logger):
        self.graph = graph
        self.current_node = "S"
        self.target_node = target_node
        self.logger = logger

        self.state = NavState.ARRIVED_AT_NODE
        self.logger.info(f"Initialized planner at node {self.current_node} with target {self.target_node}")

        self.visited_nodes    = set()
        self.current_orientation = 0
        self.node_orientation    = 0
        self.angles_from_camera  = []
        self.current_angle_index = 0
        self.last_chosen_angle   = None

        # Scoring constants
        self.SECTION_BOOST = 3
        self.WRONG_DIRECTION_PENALTY = 0.5
        self.DIRECTION_BOOST = 3
        self.RETURN_BOOST = 2
        self.FURTHER_AWAY_PENALTY = -2

    def next_action(self, sensor_data: dict) -> tuple[Frame | None, str]:
        """
        sensor_data["camera"] => {
          "node_detected": bool,
          "potential_node_detected": bool,
          "angles": list[int]
        }
        sensor_data["lidar"] => (dist_cm, flux, temp)
        """
        camera = sensor_data.get("camera", {})
        lidar  = sensor_data.get("lidar", (None, None, None))
        dist_cm, _, _ = lidar

        node_detected           = camera.get("node_detected", False)
        potential_node_detected = camera.get("potential_node_detected", False)
        angles                  = camera.get("angles", [])

        # ---------------- TRAVELING_EDGE ----------------
        if self.state == NavState.TRAVELING_EDGE:
            self.logger.debug(f"Traveling from {self.current_node}")

            if node_detected:
                self.current_node = self._infer_next_node(self.current_node, self.last_chosen_angle)
                self.state = NavState.ARRIVED_AT_NODE
                self.logger.info(f"Arrived at node {self.current_node}")
                return encode_stop(Address.MOTION_CTRL), self.current_node

            elif potential_node_detected:
                self.state = NavState.ARRIVED_AT_POTENTIAL_NODE
                self.logger.debug("Detected potential node")
                return encode_stop(Address.MOTION_CTRL), self.current_node

            return None, self.current_node

        # ---------- ARRIVED_AT_POTENTIAL_NODE ----------
        elif self.state == NavState.ARRIVED_AT_POTENTIAL_NODE:
            if node_detected:
                self.current_node = self._infer_next_node(self.current_node, self.last_chosen_angle)
                self.state = NavState.ARRIVED_AT_NODE

                self.logger.info(f"Confirmed arrival at node {self.current_node}")

            return None, self.current_node

        # --------------- ARRIVED_AT_NODE ---------------
        elif self.state == NavState.ARRIVED_AT_NODE:
            if self.current_node == self.target_node:
                self.state = NavState.GOAL_REACHED

                self.logger.info("Goal reached!")
                return encode_stop(Address.MOTION_CTRL), self.current_node

            self.angles_from_camera = self._sort_angles_by_pathfinding(angles)
            self.current_angle_index = 0
            self.node_orientation = self.current_orientation
            self.visited_nodes.add(self.current_node)
            self.state = NavState.DECIDING_NEXT_ANGLE

            self.logger.info(f"Arrived at {self.current_node}, detected angles: {self.angles_from_camera}")
            return None, self.current_node

        # ------------ DECIDING_NEXT_ANGLE --------------
        elif self.state == NavState.DECIDING_NEXT_ANGLE:
            if not self.angles_from_camera:
                self.state = NavState.BLOCKED

                self.logger.warning(f"No angles available at {self.current_node}, marking as blocked")
                return encode_stop(Address.MOTION_CTRL), self.current_node

            if self.current_angle_index >= len(self.angles_from_camera):
                self.state = NavState.BLOCKED

                self.logger.warning(f"All angles tried at {self.current_node}, marking as blocked")
                return encode_stop(Address.MOTION_CTRL), self.current_node

            angle_choice = self.angles_from_camera[self.current_angle_index]
            turn_amount = angle_choice - self.node_orientation
            self.current_orientation = angle_choice
            self.last_chosen_angle = angle_choice
            self.state = NavState.CHECK_NEXT_ANGLE

            self.logger.debug(f"Turning to angle {angle_choice} (turn amount: {turn_amount})")
            return encode_turn(Address.MOTION_CTRL, turn_amount), self.current_node

        # -------------- CHECK_NEXT_ANGLE ---------------
        elif self.state == NavState.CHECK_NEXT_ANGLE:
            if dist_cm is not None and dist_cm <= 200:
                revert_turn = self.node_orientation - self.current_orientation
                self.current_orientation = self.node_orientation
                self.current_angle_index += 1
                self.state = NavState.DECIDING_NEXT_ANGLE

                self.logger.warning(f"Angle {self.last_chosen_angle} blocked (dist: {dist_cm}cm), reverting")
                return encode_turn(Address.MOTION_CTRL, revert_turn), self.current_node

            else:
                self.state = NavState.TRAVELING_EDGE
                self.logger.info(f"Angle {self.last_chosen_angle} clear (dist: {dist_cm}cm), moving forward")
                return encode_move(Address.MOTION_CTRL, 0), self.current_node

        # ---------------- GOAL_REACHED -----------------
        elif self.state == NavState.GOAL_REACHED:
            self.logger.debug("Remaining stopped at goal")
            return encode_stop(Address.MOTION_CTRL), self.current_node

        # ---------------- BLOCKED ----------------------
        elif self.state == NavState.BLOCKED:
            self.logger.debug(f"Stopped at blocked node {self.current_node}")
            return encode_stop(Address.MOTION_CTRL), self.current_node

        return None, self.current_node

    def _sort_angles_by_pathfinding(self, angles: list[int]) -> list[int]:
        """Sort angles based on section scoring and hop distance to target_node."""

        target_section = self.graph.get_node_section(self.target_node)
        current_section = self.graph.get_node_section(self.current_node)

        # Score each angle based on potential next nodes
        angle_scores = []
        for angle in angles:
            # Infer potential next node (best guess without direct mapping)
            potential_node = self._infer_potential_node(self.current_node, angle)
            if potential_node and potential_node not in self.visited_nodes:
                score = self._score_node(potential_node, target_section, current_section)
                angle_scores.append((angle, score))
            else:
                # Penalize angles leading to visited or unknown nodes
                angle_scores.append((angle, -float('inf')))

        # Sort by score (highest first) and return just the angles
        angle_scores.sort(key=lambda x: x[1], reverse=True)
        sorted_angles = [angle for angle, _ in angle_scores]

        self.logger.debug(f"Sorted angles at {self.current_node}: {sorted_angles}")
        return sorted_angles

    def _score_node(self, next_node: str, target_section: str, current_section: str) -> float:
        """Score a node based on section and hop distance."""

        next_section = self.graph.get_node_section(next_node)

        # Hop distance as a proxy for direction
        hop_distance_to_target = self._estimate_hop_distance(next_node, self.target_node)
        hop_distance_from_current = self._estimate_hop_distance(self.current_node, self.target_node)
        direction_score = 1.0 if hop_distance_to_target < hop_distance_from_current else -1.0

        # Section scoring
        target_distance = self._get_section_distance(next_section, target_section)
        current_distance = self._get_section_distance(current_section, target_section)

        score = 0
        if next_section == target_section:
            score += self.SECTION_BOOST
        if target_distance < current_distance:
            score += self.RETURN_BOOST
        elif target_distance > current_distance:
            score += self.FURTHER_AWAY_PENALTY

        if target_distance == current_distance:
            directional_score = self.DIRECTION_BOOST if direction_score > 0 else self.WRONG_DIRECTION_PENALTY
            score += directional_score

        return score

    def _estimate_hop_distance(self, start_node: str, target_node: str) -> int:
        """Estimate hop distance to target_node using BFS."""

        if start_node == target_node:
            return 0
        visited = {start_node}
        queue = [(start_node, 0)] # (node, distance)
        while queue:
            node, dist = queue.pop(0)
            for neighbor, traversable in self.graph.get_neighbors(node).items():
                if traversable and neighbor not in visited and not self.graph.is_node_blocked(neighbor):
                    if neighbor == target_node:
                        return dist + 1
                    visited.add(neighbor)
                    queue.append((neighbor, dist + 1))
        return float('inf')

    def _get_section_distance(self, from_section: str, to_section: str) -> int:
        """Calculate distance between sections."""

        sections = ["left", "middle", "right"]
        return abs(sections.index(from_section) - sections.index(to_section))

    def _infer_next_node(self, current_node: str, angle: int) -> str:
        """Infer the next node after moving along an angle."""

        neighbors = self.graph.get_neighbors(current_node)
        viable_neighbors = [n for n, t in neighbors.items() if t and n not in self.visited_nodes]

        if not viable_neighbors:
            self.logger.debug(f"No viable neighbors from {current_node}, staying put")
            return current_node

        chosen = min(viable_neighbors, key=lambda n: self._estimate_hop_distance(n, self.target_node), default=current_node)
        self.logger.debug(f"Inferred next node from {current_node} with angle {angle}: {chosen}")
        return chosen

    def _infer_potential_node(self, current_node: str, angle: int) -> str | None:
        """Infer a potential next node for scoring, without direct angle mapping."""

        neighbors = self.graph.get_neighbors(current_node)
        viable_neighbors = [n for n, t in neighbors.items() if t and n not in self.visited_nodes]
        if not viable_neighbors:
            return None
        return min(viable_neighbors, key=lambda n: self._estimate_hop_distance(n, self.target_node), default=None)

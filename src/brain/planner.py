from enum import Enum
from prain_uart import *
from brain.graph import Graph

class NavState(Enum):
    TRAVELING_EDGE = 1
    ARRIVED_AT_POTENTIAL_NODE = 2
    ARRIVED_AT_NODE = 3
    DECIDING_NEXT_EDGE = 4
    GOAL_REACHED = 5

class PathPlanner:
    def __init__(self, graph: Graph, target_node: str):
        self.graph = graph
        self.current_node = "S"
        self.target_node = target_node
        self.state = NavState.ARRIVED_AT_NODE
        self.available_angles = []
        self.blocked_angles = []
        self.visited_nodes = set()
        self.visited_edges = set()  # Track edges as "from-to" strings

    def next_action(self, sensor_data: dict) -> tuple[Frame | None, str]:
        camera = sensor_data["camera"]  # {potential_node_detected, node_detected, angles}
        lidar = sensor_data["lidar"]
        dist_cm, _, _ = lidar if lidar else (None, None, None)

        if self.state == NavState.TRAVELING_EDGE:
            if camera["node_detected"]:
                self.state = NavState.ARRIVED_AT_NODE
                return encode_stop(Address.MOTION_CTRL), self.current_node
            elif camera["potential_node_detected"]:
                self.state = NavState.ARRIVED_AT_POTENTIAL_NODE
                return encode_stop(Address.MOTION_CTRL), self.current_node
            return None, self.current_node

        elif self.state == NavState.ARRIVED_AT_POTENTIAL_NODE:
            if camera["node_detected"]:
                self.state = NavState.ARRIVED_AT_NODE
            return None, self.current_node

        elif self.state == NavState.ARRIVED_AT_NODE:
            if self.current_node == self.target_node:
                self.state = NavState.GOAL_REACHED
                return encode_stop(Address.MOTION_CTRL), self.current_node

            self.available_angles = camera["angles"]
            self.blocked_angles = []

            self._update_graph(self.current_node, self.available_angles, self.blocked_angles)
            self.visited_nodes.add(self.current_node)
            self.state = NavState.DECIDING_NEXT_EDGE
            return None, self.current_node

        elif self.state == NavState.DECIDING_NEXT_EDGE:
            next_move = self._plan_next_step(self.current_node, self.target_node, dist_cm)
            if not next_move:
                return encode_stop(Address.MOTION_CTRL), self.current_node

            next_node, angle = next_move
            edge_id = f"{self.current_node}-{next_node}"
            self.visited_edges.add(edge_id)
            self.current_node = next_node
            self.state = NavState.TRAVELING_EDGE
            return encode_move(Address.MOTION_CTRL, angle), self.current_node

        elif self.state == NavState.GOAL_REACHED:
            return encode_stop(Address.MOTION_CTRL), self.current_node

        return None, self.current_node

    def _update_graph(self, current_node: str, angles: list, blocked_angles: list):
        neighbors = self.graph.get_neighbors(current_node)
        for angle in angles:
            next_node = self._angle_to_node(current_node, angle)
            if next_node:
                traversable = angle not in blocked_angles
                self.graph.set_edge_traversable(current_node, next_node, traversable)

    def _plan_next_step(self, current_node: str, target_node: str, dist_cm: float | None) -> tuple[str, int] | None:
        possible_edges = self._get_possible_edges(current_node, target_node)
        if not possible_edges:
            return None

        for edge in possible_edges:
            next_node = edge[1]  # (from, to, angle) tuple
            # Pylon detection: if distance ≤ 200cm, assume a pylon might be present
            if dist_cm is not None and dist_cm <= 200:  # Anything ≤ 200cm could be a pylon
                self.graph.set_node_blocked(next_node, True)
            else:  # > 200cm means the node is free
                self.graph.set_node_blocked(next_node, False)

        viable_moves = [
            (next_node, angle) for _, next_node, angle in possible_edges
            if not self.graph.is_node_blocked(next_node) and next_node not in self.visited_nodes
        ]
        return viable_moves[0] if viable_moves else None

    def _get_possible_edges(self, current_node: str, target_node: str) -> list[tuple[str, str, int]]:
        # Scoring adjustments from JS
        SECTION_BOOST = 3
        WRONG_DIRECTION_PENALTY = 0.5
        DIRECTION_BOOST = 3
        RETURN_BOOST = 2
        FURTHER_AWAY_PENALTY = -2

        target_section = self.graph.get_node_section(target_node)
        current_section = self.graph.get_node_section(current_node)

        neighbors = self.graph.get_neighbors(current_node)
        possible_edges = []
        for next_node, traversable in neighbors.items():
            if not traversable or f"{current_node}-{next_node}" in self.visited_edges:
                continue
            angle = self._node_to_angle(current_node, next_node)
            possible_edges.append((current_node, next_node, angle))

        possible_edges.sort(key=lambda edge: self._score_edge(edge, target_node, target_section, current_section,
                                                              SECTION_BOOST, WRONG_DIRECTION_PENALTY, DIRECTION_BOOST,
                                                              RETURN_BOOST, FURTHER_AWAY_PENALTY), reverse=True)
        return possible_edges

    def _score_edge(self, edge: tuple[str, str, int], target_node: str, target_section: str, current_section: str,
                    SECTION_BOOST: float, WRONG_DIRECTION_PENALTY: float, DIRECTION_BOOST: float,
                    RETURN_BOOST: float, FURTHER_AWAY_PENALTY: float) -> float:
        _, next_node, _ = edge
        next_section = self.graph.get_node_section(next_node)

        # Hop distance as a proxy for direction
        hop_distance_to_target = self._estimate_hop_distance(next_node, target_node)
        hop_distance_from_current = self._estimate_hop_distance(self.current_node, target_node)
        direction_score = 1.0 if hop_distance_to_target < hop_distance_from_current else -1.0

        # Section scoring
        target_distance = self._get_section_distance(next_section, target_section)
        current_distance = self._get_section_distance(current_section, target_section)

        score = 0
        if next_section == target_section:
            score += SECTION_BOOST
        if target_distance < current_distance:
            score += RETURN_BOOST
        elif target_distance > current_distance:
            score += FURTHER_AWAY_PENALTY

        if target_distance == current_distance:
            directional_score = DIRECTION_BOOST if direction_score > 0 else WRONG_DIRECTION_PENALTY
            score += directional_score

        return score

    def _estimate_hop_distance(self, start_node: str, target_node: str) -> int:
        if start_node == target_node:
            return 0
        visited = {start_node}
        queue = [(start_node, 0)]  # (node, distance)
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
        sections = ["left", "middle", "right"]
        return abs(sections.index(from_section) - sections.index(to_section))

    def _angle_to_node(self, current_node: str, angle: int) -> str | None:
        neighbors = self.graph.get_neighbors(current_node)
        for node in neighbors:
            if str(angle) in node:
                return node
        return None

    def _node_to_angle(self, current_node: str, next_node: str) -> int:
        try:
            return int(next_node.split("_")[-1]) if "_" in next_node else 0
        except (ValueError, IndexError):
            return 0

class Graph:
    def __init__(self):
        """
        Create the default adjacency dict for the known nodes and edges.
        Keys are node IDs (strings); values are dicts mapping neighbor -> bool (traversable?).
        """
        self._graph = {
            "S": {
                "1": {"traversable": True, "distance": float("inf")},
                "2": {"traversable": True, "distance": float("inf")},
                "3": {"traversable": True, "distance": float("inf")}
            },
            "1": {
                "S": {"traversable": True, "distance": float("inf")},
                "2": {"traversable": True, "distance": float("inf")},
                "A": {"traversable": True, "distance": float("inf")}
            },
            "2": {
                "S": {"traversable": True, "distance": float("inf")},
                "1": {"traversable": True, "distance": float("inf")},
                "3": {"traversable": True, "distance": float("inf")},
                "4": {"traversable": True, "distance": float("inf")},
                "A": {"traversable": True, "distance": float("inf")}
            },
            "3": {
                "S": {"traversable": True, "distance": float("inf")},
                "2": {"traversable": True, "distance": float("inf")},
                "4": {"traversable": True, "distance": float("inf")},
                "C": {"traversable": True, "distance": float("inf")}
            },
            "4": {
                "2": {"traversable": True, "distance": float("inf")},
                "3": {"traversable": True, "distance": float("inf")},
                "A": {"traversable": True, "distance": float("inf")},
                "B": {"traversable": True, "distance": float("inf")},
                "C": {"traversable": True, "distance": float("inf")}
            },
            "A": {
                "1": {"traversable": True, "distance": float("inf")},
                "2": {"traversable": True, "distance": float("inf")},
                "4": {"traversable": True, "distance": float("inf")},
                "B": {"traversable": True, "distance": float("inf")}
            },
            "B": {
                "A": {"traversable": True, "distance": float("inf")},
                "4": {"traversable": True, "distance": float("inf")},
                "C": {"traversable": True, "distance": float("inf")}
            },
            "C": {
                "3": {"traversable": True, "distance": float("inf")},
                "4": {"traversable": True, "distance": float("inf")},
                "B": {"traversable": True, "distance": float("inf")}
            }
        }
        self._goal_vectors = {"A": True, "B": True, "C": True}
        self._blocked_nodes = {node: False for node in self._graph}
        self.node_sections = {
            "S": "middle", "1": "right", "2": "middle", "3": "left",
            "4": "middle", "A": "right", "B": "middle", "C": "left"
        }

    def set_edge_traversable(self, node1: str, node2: str, traversable: bool) -> None:
        """Mark the edge node1 <-> node2 as traversable or blocked."""
        if node1 in self._graph and node2 in self._graph[node1]:
            self._graph[node1][node2]["traversable"] = traversable
        if node2 in self._graph and node1 in self._graph[node2]:
            self._graph[node2][node1]["traversable"] = traversable

    def is_edge_traversable(self, node1: str, node2: str) -> bool:
        """Return True if node1 <-> node2 is currently traversable."""
        if node1 in self._graph and node2 in self._graph[node1]:
            return self._graph[node1][node2].get("traversable", False)
        return False
    
    def set_edge_distance(self, node1: str, node2: str, distance: float) -> None:
        if node1 in self._graph and node2 in self._graph[node1]:
            self._graph[node1][node2]["distance"] = distance
        if node2 in self._graph and node1 in self._graph[node2]:
            self._graph[node2][node1]["distance"] = distance  # symmetrisch

    def get_edge_distance(self, node1: str, node2: str) -> float:
        if node1 in self._graph and node2 in self._graph[node1]:
            return self._graph[node1][node2].get("distance", float("inf"))
        return float("inf")

    def remove_edge(self, node1: str, node2: str) -> None:
        """Completely remove the edge from the graph."""
        if node1 in self._graph and node2 in self._graph[node1]:
            del self._graph[node1][node2]
        if node2 in self._graph and node1 in self._graph[node2]:
            del self._graph[node2][node1]

    def get_neighbors(self, node: str) -> dict[str, bool]:
        """Return a dict of neighbor -> traversable-boolean for the given node."""
        return self._graph.get(node, {})

    def nodes(self):
        """Return a list of all node IDs in the graph."""
        return list(self._graph.keys())

    def get_adjacency(self) -> dict[str, dict[str, bool]]:
        """Return the entire adjacency structure."""
        return self._graph

    def set_node_blocked(self, node: str, blocked: bool) -> None:
        """Mark a node as blocked (e.g., by a pylon) or unblocked."""
        if node in self._blocked_nodes:
            self._blocked_nodes[node] = blocked

    def is_node_blocked(self, node: str) -> bool:
        """Return True if the node is blocked (e.g., by a pylon)."""
        return self._blocked_nodes.get(node, False)

    def get_node_section(self, node: str) -> str:
        """Return the section ('left', 'middle', 'right') for a given node."""
        return self.node_sections.get(node, "middle")  # Default to "middle" if unknown

    def __repr__(self):
        return f"Graph(adjacency={self._graph}, blocked_nodes={self._blocked_nodes}, sections={self.node_sections})"

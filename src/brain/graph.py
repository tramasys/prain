# In prain/src/brain/graph.py

import json
from typing import Dict, List, Any, Optional

class Graph:
    """
    Repräsentiert den statischen Navigationsgraphen des Roboters.
    Diese Klasse lädt die Graphendaten aus einer JSON-Datei und stellt
    eine Schnittstelle zur Abfrage von Knoten, Kanten und deren Eigenschaften bereit.
    """
    def __init__(self, file_path: str):
        """
        Initialisiert und lädt den Graphen aus der angegebenen JSON-Datei.
        
        Args:
            file_path (str): Der Pfad zur graph_data.json Datei.
        """
        self.file_path = file_path
        self._graph_data = self._load_from_json()
        self._validate_graph_data()
        
        # Adjazenzliste für einfachen Zugriff auf Nachbarn
        self._adjacency_list = self._build_adjacency_list()

    def _load_from_json(self) -> Dict[str, Any]:
        """Lädt die Graphendaten aus der JSON-Datei."""
        try:
            with open(self.file_path, 'r') as f:
                data = json.load(f)
                return data
        except FileNotFoundError:
            print(f"ERROR: Graph file not found at {self.file_path}")
            return {}
        except json.JSONDecodeError:
            print(f"ERROR: Could not decode JSON from {self.file_path}")
            return {}

    def _validate_graph_data(self):
        """Stellt sicher, dass die geladenen Daten 'nodes' und 'edges' enthalten."""
        if 'nodes' not in self._graph_data or 'edges' not in self._graph_data:
            raise ValueError("Invalid graph file: Must contain 'nodes' and 'edges' keys.")

    def _build_adjacency_list(self) -> Dict[str, Dict[str, Any]]:
        """Erstellt eine Adjazenzliste aus den Kantendaten für schnellen Zugriff."""
        adj = {node['id']: {} for node in self._graph_data.get('nodes', [])}
        for edge in self._graph_data.get('edges', []):
            u, v = edge['source'], edge['target']
            # Füge Kante in beide Richtungen hinzu, falls nicht gerichtet
            adj[u][v] = edge
            adj[v][u] = edge # Annahme: Kanten sind bidirektional
        return adj

    def get_adjacency(self) -> Dict[str, Dict[str, Any]]:
        """Gibt die komplette Adjazenzliste des Graphen zurück."""
        return self._adjacency_list
        
    def get_nodes(self) -> List[Dict[str, Any]]:
        """Gibt eine Liste aller Knoten im Graphen zurück."""
        return self._graph_data.get('nodes', [])

    def get_node(self, node_id: str) -> Optional[Dict[str, Any]]:
        """Gibt die Daten für einen spezifischen Knoten zurück."""
        for node in self.get_nodes():
            if node['id'] == node_id:
                return node
        return None

    def get_neighbors(self, node_id: str) -> List[str]:
        """Gibt eine Liste der IDs der Nachbarknoten zurück."""
        return list(self._adjacency_list.get(node_id, {}).keys())

    def is_edge_traversable(self, source_node: str, target_node: str) -> bool:
        """
        Prüft, ob eine Kante existiert und traversierbar ist.
        Hier könnte zukünftig Logik für blockierte Kanten rein.
        """
        if source_node not in self._adjacency_list:
            return False
        return target_node in self._adjacency_list[source_node]
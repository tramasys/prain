from sensors.vision_nav.edge import EdgeCandidate
from sensors.vision_nav.edge import Edge
import numpy as np
from sklearn.cluster import DBSCAN

class Node:  
    def __init__(self) -> None:
        self.__edge_candidates: list[EdgeCandidate] = []
        self.__edges: list[Edge] = []
        
    def add_edge_candidates(self, edge_candidate: EdgeCandidate) -> None:
        self.__edge_candidates.append(edge_candidate)
        
    def extend_edge_candidates(self, edge_candidates: list[EdgeCandidate]) -> None:
        self.__edge_candidates.extend(edge_candidates)
        
    def has_edge_candidates(self): return len(self.__edge_candidates) > 0
    
    def add_edge(self, edge: Edge): self.__edges.append(edge)
        
    def extend_edges(self, edges: list[Edge]): self.__edges.extend(edges)
        
    def get_edges(self): return self.__edges
    
    def __str__(self):
        return str(self.__edges)
    
    def process(self):
        return [edge.get_angle() for edge in self.__cluster_edges_dbscan()]
    
    def __cluster_edges_dbscan(self, eps=15.0):
        angles = np.array([[edge_candidate.get_angle()] for edge_candidate in self.__edge_candidates])
        clustering = DBSCAN(eps=eps, min_samples=1, metric='euclidean').fit(angles)
        
        labels = clustering.labels_
        clustered = {}

        for edge, label in zip(self.__edge_candidates, labels):
            clustered.setdefault(label, []).append(edge)

        has_origin_angle = False
        for group in clustered.values():
            avg_angle = int(np.mean([e.get_angle() for e in group]))
            
            if self.is_source_angle(avg_angle, 0, 10):
                if has_origin_angle:
                    continue  # Skip if we already added an origin edge
                avg_angle = 180
                has_origin_angle = True
            
            self.add_edge(Edge(avg_angle))
            
        return self.__edges
    
    def is_source_angle(self, angle: float, reference: float, angle_thresh=10) -> bool:
        diff = abs((angle - reference + 180) % 360 - 180)
        return diff <= angle_thresh
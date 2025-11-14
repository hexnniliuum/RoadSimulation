"""
Road Network Loader
Loads and parses Unity-exported road network JSON files.
"""

import json
import numpy as np


class RoadNetwork:
    """
    Represents a complete road network exported from Unity.
    Provides methods to access and query network data.
    """
    
    def __init__(self, json_path):
        """
        Load road network from JSON file.
        
        Args:
            json_path: Path to the exported JSON file from Unity
        """
        with open(json_path, 'r') as f:
            self.data = json.load(f)
        
        # Create lookup dictionaries for fast access
        self.nodes = {n['ID']: n for n in self.data['Nodes']}
        self.roads = {r['ID']: r for r in self.data['Roads']}
        self.lanes = {l['ID']: l for l in self.data['Lanes']}
        self.bridges = self.data['Bridges']
        self.turns = {t['ID']: t for t in self.data['TurnCurves']}
        
        print(f"Loaded road network (Seed: {self.data['RandomSeed']})")
        print(f"  Nodes: {len(self.nodes)}")
        print(f"  Roads: {len(self.roads)}")
        print(f"  Lanes: {len(self.lanes)}")
        print(f"  Bridges: {len(self.bridges)}")
        print(f"  Turn Curves: {len(self.turns)}")
    
    def get_node_position(self, node_id):
        """
        Get 2D position of a node on XZ plane.
        
        Args:
            node_id: Node ID
            
        Returns:
            Tuple (x, z) of node position
        """
        node = self.nodes[node_id]
        return (node['X'], node['Z'])
    
    def get_node_position_3d(self, node_id):
        """
        Get 3D position of a node.
        
        Args:
            node_id: Node ID
            
        Returns:
            Tuple (x, y, z) of node position
        """
        node = self.nodes[node_id]
        return (node['X'], node['Y'], node['Z'])
    
    def get_lane_path(self, lane_id, num_points=50):
        """
        Get array of points along a lane, including bridge elevations.
        
        Args:
            lane_id: Lane ID
            num_points: Number of points to sample along the lane
            
        Returns:
            numpy array of shape (num_points, 3) with (x, z, y) coordinates
        """
        lane = self.lanes[lane_id]
        
        # Linear interpolation between start and end
        t = np.linspace(0, 1, num_points)
        x = lane['StartX'] + t * (lane['EndX'] - lane['StartX'])
        z = lane['StartZ'] + t * (lane['EndZ'] - lane['StartZ'])
        y = lane['StartY'] + t * (lane['EndY'] - lane['StartY'])
        
        # Apply bridge elevations if present
        if lane['BridgeIndices'] and len(lane['BridgeIndices']) > 0:
            distances = t * lane['Length']
            
            # Convert lane distance to road distance based on direction
            if lane['IsReversed']:
                road_distances = lane['RoadLength'] - lane['StopLineDistanceAtStart'] - distances
            else:
                road_distances = lane['StopLineDistanceAtStart'] + distances
            
            # Apply each bridge's elevation
            for bridge_idx in lane['BridgeIndices']:
                bridge = self.bridges[bridge_idx]
                
                for i, road_dist in enumerate(road_distances):
                    if bridge['StartDistance'] <= road_dist <= bridge['EndDistance']:
                        # Parabolic curve: 4 * t * (1-t)
                        bridge_progress = (road_dist - bridge['StartDistance']) / \
                                        (bridge['EndDistance'] - bridge['StartDistance'])
                        curve_value = 4.0 * bridge_progress * (1.0 - bridge_progress)
                        y[i] += curve_value * bridge['PeakHeight']
        
        return np.column_stack([x, z, y])
    
    def get_turn_curve_path(self, turn_id, num_points=20):
        """
        Get Bézier curve path for a turn.
        
        Args:
            turn_id: Turn curve ID
            num_points: Number of points to sample along the curve
            
        Returns:
            numpy array of shape (num_points, 3) with (x, z, y) coordinates
        """
        turn = self.turns[turn_id]
        
        # Quadratic Bézier: P(t) = (1-t)^2 * P0 + 2(1-t)t * P1 + t^2 * P2
        t = np.linspace(0, 1, num_points)
        
        P0 = np.array([turn['StartX'], turn['StartZ'], turn['StartY']])
        P1 = np.array([turn['ControlX'], turn['ControlZ'], turn['ControlY']])
        P2 = np.array([turn['EndX'], turn['EndZ'], turn['EndY']])
        
        # Vectorized Bézier calculation
        points = (1-t[:, np.newaxis])**2 * P0 + \
                 2 * (1-t[:, np.newaxis]) * t[:, np.newaxis] * P1 + \
                 t[:, np.newaxis]**2 * P2
        
        return points
    
    def get_all_lane_ids(self):
        """Get list of all lane IDs."""
        return list(self.lanes.keys())
    
    def get_all_node_ids(self):
        """Get list of all node IDs."""
        return list(self.nodes.keys())
    
    def get_roads_at_node(self, node_id):
        """
        Get all roads connected to a node.
        
        Args:
            node_id: Node ID
            
        Returns:
            List of road dictionaries
        """
        node = self.nodes[node_id]
        return [self.roads[rid] for rid in node['ConnectedRoadIDs']]
    
    def get_network_bounds(self):
        """
        Get bounding box of the network.
        
        Returns:
            Dictionary with keys: 'x_min', 'x_max', 'z_min', 'z_max'
        """
        all_x = [node['X'] for node in self.nodes.values()]
        all_z = [node['Z'] for node in self.nodes.values()]
        
        return {
            'x_min': min(all_x),
            'x_max': max(all_x),
            'z_min': min(all_z),
            'z_max': max(all_z)
        }
    
    def find_lane_by_nodes(self, start_node_id, end_node_id):
        """
        Find lane connecting two nodes.
        
        Args:
            start_node_id: Starting node ID
            end_node_id: Ending node ID
            
        Returns:
            Lane dictionary or None if not found
        """
        for lane in self.lanes.values():
            if lane['StartNodeID'] == start_node_id and lane['EndNodeID'] == end_node_id:
                return lane
        return None


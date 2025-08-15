#!/usr/bin/env python3
"""
Pure Pursuit Controller for Autonomous Vehicle Simulation
Uses road data from link_set.json to implement path following
"""

import json
import math
import numpy as np
from typing import List, Tuple, Optional, Dict, Any

class PurePursuitController:
    """
    Pure Pursuit Controller for autonomous vehicle navigation
    """
    
    def __init__(self, link_set_file: str, lookahead_distance: float = 5.0):
        """
        Initialize the controller
        
        Args:
            link_set_file: Path to link_set.json file
            lookahead_distance: Lookahead distance in meters
        """
        self.lookahead_distance = lookahead_distance
        self.links = self._load_road_data(link_set_file)
        self.current_link_idx = 0
        self.current_waypoint_idx = 0
        
        # Vehicle parameters (you can adjust these)
        self.wheelbase = 2.7  # meters (typical car wheelbase)
        self.max_steering_angle = math.radians(30)  # maximum steering angle
        self.max_speed = 20.0  # maximum speed in m/s
        
    def _load_road_data(self, file_path: str) -> List[Dict[str, Any]]:
        """Load road data from JSON file"""
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            print(f"Loaded {len(data)} road links")
            return data
        except Exception as e:
            print(f"Error loading road data: {e}")
            return []
    
    def get_current_road_link(self) -> Optional[Dict[str, Any]]:
        """Get current road link data"""
        if 0 <= self.current_link_idx < len(self.links):
            return self.links[self.current_link_idx]
        return None
    
    def get_waypoints(self, link_idx: int) -> List[List[float]]:
        """Get waypoints for a specific road link"""
        if 0 <= link_idx < len(self.links):
            return self.links[link_idx]["points"]
        return []
    
    def calculate_distance(self, point1: List[float], point2: List[float]) -> float:
        """Calculate Euclidean distance between two 3D points"""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(point1, point2)))
    
    def find_closest_waypoint(self, vehicle_pos: List[float], waypoints: List[List[float]]) -> Tuple[int, float]:
        """
        Find the closest waypoint to the vehicle
        
        Returns:
            Tuple of (waypoint_index, distance)
        """
        min_distance = float('inf')
        closest_idx = 0
        
        for i, waypoint in enumerate(waypoints):
            distance = self.calculate_distance(vehicle_pos, waypoint)
            if distance < min_distance:
                min_distance = distance
                closest_idx = i
        
        return closest_idx, min_distance
    
    def find_lookahead_waypoint(self, vehicle_pos: List[float], waypoints: List[List[float]]) -> Optional[List[float]]:
        """
        Find the waypoint that is lookahead_distance ahead of the vehicle
        
        Returns:
            Lookahead waypoint or None if not found
        """
        if not waypoints:
            return None
        
        # Find closest waypoint
        closest_idx, _ = self.find_closest_waypoint(vehicle_pos, waypoints)
        
        # Look ahead from the closest waypoint
        for i in range(closest_idx, len(waypoints)):
            distance = self.calculate_distance(vehicle_pos, waypoints[i])
            if distance >= self.lookahead_distance:
                return waypoints[i]
        
        # If no waypoint is far enough, return the last one
        return waypoints[-1]
    
    def calculate_steering_angle(self, vehicle_pos: List[float], vehicle_heading: float, 
                                target_waypoint: List[float]) -> float:
        """
        Calculate steering angle using pure pursuit geometry
        
        Args:
            vehicle_pos: Current vehicle position [x, y, z]
            vehicle_heading: Current vehicle heading in radians
            target_waypoint: Target waypoint [x, y, z]
        
        Returns:
            Steering angle in radians
        """
        # Calculate lateral distance (cross-track error)
        dx = target_waypoint[0] - vehicle_pos[0]
        dy = target_waypoint[1] - vehicle_pos[1]
        
        # Calculate angle to target
        alpha = math.atan2(dy, dx) - vehicle_heading
        
        # Normalize angle to [-pi, pi]
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi
        
        # Pure pursuit steering formula
        steering_angle = math.atan2(2 * self.wheelbase * math.sin(alpha), 
                                   self.lookahead_distance)
        
        # Limit steering angle
        steering_angle = max(-self.max_steering_angle, 
                           min(self.max_steering_angle, steering_angle))
        
        return steering_angle
    
    def calculate_target_speed(self, current_link: Dict[str, Any]) -> float:
        """Calculate target speed based on road properties"""
        max_speed_str = current_link.get("max_speed", "20")
        try:
            max_speed = float(max_speed_str)
            # Convert to m/s if it's in km/h (assuming km/h)
            if max_speed < 50:  # Likely in m/s
                return min(max_speed, self.max_speed)
            else:  # Likely in km/h
                return min(max_speed / 3.6, self.max_speed)
        except ValueError:
            return self.max_speed
    
    def update_path(self, vehicle_pos: List[float], vehicle_heading: float) -> Tuple[float, float]:
        """
        Main update function for the controller
        
        Args:
            vehicle_pos: Current vehicle position [x, y, z]
            vehicle_heading: Current vehicle heading in radians
        
        Returns:
            Tuple of (steering_angle, target_speed)
        """
        current_link = self.get_current_road_link()
        if not current_link:
            print("No current road link available")
            return 0.0, 0.0
        
        waypoints = self.get_waypoints(self.current_link_idx)
        if not waypoints:
            print("No waypoints available for current road link")
            return 0.0, 0.0
        
        # Find lookahead waypoint
        lookahead_waypoint = self.find_lookahead_waypoint(vehicle_pos, waypoints)
        if not lookahead_waypoint:
            print("No lookahead waypoint found")
            return 0.0, 0.0
        
        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(vehicle_pos, vehicle_heading, lookahead_waypoint)
        
        # Calculate target speed
        target_speed = self.calculate_target_speed(current_link)
        
        # Check if we need to move to next road link
        self._check_link_transition(vehicle_pos, waypoints)
        
        return steering_angle, target_speed
    
    def _check_link_transition(self, vehicle_pos: List[float], waypoints: List[List[float]]):
        """Check if vehicle should transition to next road link"""
        if not waypoints:
            return
        
        # Check if we're close to the end of current road link
        last_waypoint = waypoints[-1]
        distance_to_end = self.calculate_distance(vehicle_pos, last_waypoint)
        
        if distance_to_end < self.lookahead_distance:
            # Move to next road link
            self.current_link_idx += 1
            self.current_waypoint_idx = 0
            print(f"Transitioning to road link {self.current_link_idx}")
            
            # Reset to first link if we've reached the end
            if self.current_link_idx >= len(self.links):
                self.current_link_idx = 0
                print("Completed route, restarting from beginning")

def main():
    """Example usage of the Pure Pursuit Controller"""
    
    # Initialize controller
    controller = PurePursuitController("link_set.json", lookahead_distance=5.0)
    
    # Example vehicle state
    vehicle_pos = [158.0, 189.0, 146.0]  # Starting position
    vehicle_heading = 0.0  # radians
    
    print("Pure Pursuit Controller initialized!")
    print(f"Total road links: {len(controller.links)}")
    
    # Simulate a few control updates
    for i in range(5):
        steering_angle, target_speed = controller.update_path(vehicle_pos, vehicle_heading)
        
        print(f"Step {i+1}:")
        print(f"  Vehicle Position: {vehicle_pos}")
        print(f"  Steering Angle: {math.degrees(steering_angle):.2f}°")
        print(f"  Target Speed: {target_speed:.2f} m/s")
        print(f"  Current Road Link: {controller.current_link_idx}")
        print()
        
        # Simulate vehicle movement (simplified)
        # In real implementation, this would come from vehicle sensors
        vehicle_pos[0] += 1.0  # Move forward
        vehicle_pos[1] += 0.1  # Slight lateral movement

if __name__ == "__main__":
    main()

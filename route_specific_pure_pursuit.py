#!/usr/bin/env python3
"""
Route-Specific Pure Pursuit Controller
경로: A2229B000001 → A2229B000023 → A2229B000013
"""

import json
import math
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional, Dict

class RouteSpecificPurePursuit:
    """
    특정 경로(A2229B000001 → A2229B000023 → A2229B000013)를 위한 Pure Pursuit 컨트롤러
    """
    
    def __init__(self, link_set_file: str, lookahead_distance: float = 5.0):
        """
        Initialize controller for specific route
        
        Args:
            link_set_file: Path to link_set.json
            lookahead_distance: Lookahead distance in meters
        """
        self.lookahead_distance = lookahead_distance
        self.route_links = ['A2229B000001', 'A2229B000023', 'A2229B000013']
        self.current_link_index = 0
        
        # Vehicle parameters
        self.wheelbase = 2.7  # meters
        self.max_steering_angle = math.radians(30)  # 30 degrees
        self.max_speed = 20.0  # m/s
        
        # Load and process route data
        self.route_data = self._load_route_data(link_set_file)
        self.waypoints = self._create_continuous_waypoints()
        self.current_waypoint_index = 0
        
        print(f"✅ Route loaded: {len(self.waypoints)} total waypoints")
        
    def _load_route_data(self, file_path: str) -> List[Dict]:
        """Load specific route links from JSON file"""
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            
            route_data = []
            for link_id in self.route_links:
                for link in data:
                    if link['idx'] == link_id:
                        route_data.append(link)
                        break
            
            print(f"✅ Loaded {len(route_data)} route links")
            return route_data
            
        except Exception as e:
            print(f"❌ Error loading route data: {e}")
            return []
    
    def _create_continuous_waypoints(self) -> List[List[float]]:
        """Create continuous waypoint list from all route links"""
        all_waypoints = []
        
        for i, link in enumerate(self.route_data):
            waypoints = link['points']
            
            if i == 0:
                # First link: add all waypoints
                all_waypoints.extend(waypoints)
            else:
                # Subsequent links: skip first waypoint to avoid duplication
                all_waypoints.extend(waypoints[1:])
        
        print(f"📍 Created continuous path with {len(all_waypoints)} waypoints")
        return all_waypoints
    
    def calculate_distance(self, point1: List[float], point2: List[float]) -> float:
        """Calculate Euclidean distance between two points"""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    def find_closest_waypoint(self, vehicle_pos: List[float]) -> int:
        """Find index of closest waypoint to vehicle"""
        min_distance = float('inf')
        closest_index = self.current_waypoint_index
        
        # Search in a window around current position for efficiency
        search_start = max(0, self.current_waypoint_index - 10)
        search_end = min(len(self.waypoints), self.current_waypoint_index + 50)
        
        for i in range(search_start, search_end):
            distance = self.calculate_distance(vehicle_pos, self.waypoints[i])
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        return closest_index
    
    def find_lookahead_waypoint(self, vehicle_pos: List[float]) -> Tuple[Optional[List[float]], int]:
        """
        Find lookahead waypoint
        
        Returns:
            Tuple of (lookahead_waypoint, waypoint_index)
        """
        closest_idx = self.find_closest_waypoint(vehicle_pos)
        
        # Look ahead from closest waypoint
        for i in range(closest_idx, len(self.waypoints)):
            distance = self.calculate_distance(vehicle_pos, self.waypoints[i])
            if distance >= self.lookahead_distance:
                return self.waypoints[i], i
        
        # If no waypoint is far enough, return the last one
        if self.waypoints:
            return self.waypoints[-1], len(self.waypoints) - 1
        
        return None, -1
    
    def calculate_steering_angle(self, vehicle_pos: List[float], vehicle_heading: float, 
                                target_waypoint: List[float]) -> float:
        """Calculate steering angle using pure pursuit geometry"""
        # Vector from vehicle to target
        dx = target_waypoint[0] - vehicle_pos[0]
        dy = target_waypoint[1] - vehicle_pos[1]
        
        # Angle to target relative to vehicle heading
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
        return max(-self.max_steering_angle, 
                  min(self.max_steering_angle, steering_angle))
    
    def get_current_speed_limit(self) -> float:
        """Get speed limit for current link"""
        if self.current_link_index < len(self.route_data):
            max_speed_str = self.route_data[self.current_link_index].get('max_speed', '20')
            try:
                max_speed = float(max_speed_str)
                if max_speed > 50:  # Convert km/h to m/s
                    max_speed = max_speed / 3.6
                return min(max_speed, self.max_speed)
            except ValueError:
                pass
        return self.max_speed
    
    def update_current_link(self, waypoint_index: int):
        """Update current link based on waypoint progress"""
        # Calculate cumulative waypoint counts
        cumulative_counts = [0]
        for link in self.route_data:
            cumulative_counts.append(cumulative_counts[-1] + len(link['points']))
        
        # Determine current link
        for i, count in enumerate(cumulative_counts[1:]):
            if waypoint_index < count:
                self.current_link_index = i
                break
    
    def update(self, vehicle_pos: List[float], vehicle_heading: float) -> Tuple[float, float, Dict]:
        """
        Main update function
        
        Args:
            vehicle_pos: Current vehicle position [x, y, z]
            vehicle_heading: Current vehicle heading in radians
            
        Returns:
            Tuple of (steering_angle, target_speed, debug_info)
        """
        if not self.waypoints:
            return 0.0, 0.0, {"error": "No waypoints available"}
        
        # Find lookahead waypoint
        lookahead_waypoint, waypoint_idx = self.find_lookahead_waypoint(vehicle_pos)
        
        if lookahead_waypoint is None:
            return 0.0, 0.0, {"error": "No lookahead waypoint found"}
        
        # Update current position tracking
        self.current_waypoint_index = waypoint_idx
        self.update_current_link(waypoint_idx)
        
        # Calculate control commands
        steering_angle = self.calculate_steering_angle(vehicle_pos, vehicle_heading, lookahead_waypoint)
        target_speed = self.get_current_speed_limit()
        
        # Calculate progress
        progress_percent = (waypoint_idx / len(self.waypoints)) * 100
        
        # Debug information
        debug_info = {
            "lookahead_waypoint": lookahead_waypoint,
            "waypoint_index": waypoint_idx,
            "current_link": self.route_links[self.current_link_index],
            "current_link_index": self.current_link_index,
            "progress_percent": progress_percent,
            "total_waypoints": len(self.waypoints),
            "distance_to_lookahead": self.calculate_distance(vehicle_pos, lookahead_waypoint)
        }
        
        return steering_angle, target_speed, debug_info
    
    def get_route_info(self) -> Dict:
        """Get complete route information"""
        total_distance = sum([link['link_length'] for link in self.route_data])
        
        return {
            "route_links": self.route_links,
            "total_distance": total_distance,
            "total_waypoints": len(self.waypoints),
            "link_details": [
                {
                    "id": link['idx'],
                    "distance": link['link_length'],
                    "waypoints": len(link['points']),
                    "max_speed": link['max_speed']
                }
                for link in self.route_data
            ]
        }
    
    def visualize_route(self, vehicle_pos: Optional[List[float]] = None, 
                       show_waypoints: bool = True, save_fig: bool = False):
        """Visualize the route and vehicle position"""
        fig, ax = plt.subplots(figsize=(15, 10))
        
        # Plot route waypoints
        x_coords = [wp[0] for wp in self.waypoints]
        y_coords = [wp[1] for wp in self.waypoints]
        
        ax.plot(x_coords, y_coords, 'b-', linewidth=2, label='Route Path')
        
        if show_waypoints:
            ax.scatter(x_coords[::10], y_coords[::10], c='lightblue', s=20, alpha=0.6, label='Waypoints')
        
        # Mark link boundaries
        cumulative_counts = [0]
        for link in self.route_data:
            cumulative_counts.append(cumulative_counts[-1] + len(link['points']))
        
        colors = ['green', 'orange', 'red']
        for i, (link, count) in enumerate(zip(self.route_data, cumulative_counts[1:])):
            if count <= len(self.waypoints):
                wp = self.waypoints[min(count-1, len(self.waypoints)-1)]
                ax.scatter(wp[0], wp[1], c=colors[i], s=100, marker='s', 
                          label=f'Link {i+1} End: {link["idx"]}')
        
        # Mark start and end
        if self.waypoints:
            start = self.waypoints[0]
            end = self.waypoints[-1]
            ax.scatter(start[0], start[1], c='green', s=200, marker='o', 
                      label='Start', edgecolor='black', linewidth=2)
            ax.scatter(end[0], end[1], c='red', s=200, marker='X', 
                      label='End', edgecolor='black', linewidth=2)
        
        # Plot vehicle position if provided
        if vehicle_pos:
            ax.scatter(vehicle_pos[0], vehicle_pos[1], c='purple', s=150, marker='^', 
                      label='Vehicle', edgecolor='black', linewidth=2)
            
            # Show lookahead circle
            circle = plt.Circle((vehicle_pos[0], vehicle_pos[1]), self.lookahead_distance, 
                              fill=False, color='purple', linestyle='--', alpha=0.7)
            ax.add_patch(circle)
        
        ax.set_xlabel('X Coordinate (m)')
        ax.set_ylabel('Y Coordinate (m)')
        ax.set_title('Route: A2229B000001 → A2229B000023 → A2229B000013')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        plt.tight_layout()
        
        if save_fig:
            plt.savefig('route_visualization.png', dpi=300, bbox_inches='tight')
            print("📸 Route visualization saved as 'route_visualization.png'")
        
        plt.show()

def main():
    """Demo of the route-specific pure pursuit controller"""
    
    print("🛣️  Route-Specific Pure Pursuit Controller Demo")
    print("="*60)
    print("Route: A2229B000001 → A2229B000023 → A2229B000013")
    print("="*60)
    
    # Initialize controller
    controller = RouteSpecificPurePursuit("link_set.json", lookahead_distance=8.0)
    
    # Get route information
    route_info = controller.get_route_info()
    print(f"\n📊 Route Information:")
    print(f"   Total Distance: {route_info['total_distance']:.2f} m")
    print(f"   Total Waypoints: {route_info['total_waypoints']}")
    print(f"   Number of Links: {len(route_info['route_links'])}")
    
    for i, detail in enumerate(route_info['link_details']):
        print(f"   Link {i+1}: {detail['id']}")
        print(f"     Distance: {detail['distance']:.2f}m")
        print(f"     Waypoints: {detail['waypoints']}")
        print(f"     Max Speed: {detail['max_speed']}")
    
    # Simulate vehicle following the route
    print(f"\n🚗 Vehicle Simulation:")
    print("-" * 40)
    
    # Start at the first waypoint
    vehicle_pos = controller.waypoints[0].copy()
    vehicle_heading = 0.0  # Initial heading
    
    simulation_steps = 20
    step_size = 2.0  # meters per step
    
    for step in range(simulation_steps):
        # Update controller
        steering_angle, target_speed, debug_info = controller.update(vehicle_pos, vehicle_heading)
        
        print(f"Step {step+1:2d}:")
        print(f"  Position: ({vehicle_pos[0]:.1f}, {vehicle_pos[1]:.1f})")
        print(f"  Steering: {math.degrees(steering_angle):+6.2f}°")
        print(f"  Speed: {target_speed:.1f} m/s")
        print(f"  Progress: {debug_info['progress_percent']:.1f}%")
        print(f"  Current Link: {debug_info['current_link']}")
        print(f"  Waypoint: {debug_info['waypoint_index']}/{debug_info['total_waypoints']}")
        print()
        
        # Simple vehicle movement simulation
        # Update heading based on steering (simplified model)
        vehicle_heading += steering_angle * 0.1
        
        # Move vehicle forward
        vehicle_pos[0] += step_size * math.cos(vehicle_heading)
        vehicle_pos[1] += step_size * math.sin(vehicle_heading)
        
        # Check if completed
        if debug_info['waypoint_index'] >= len(controller.waypoints) - 1:
            print("🏁 Route completed!")
            break
    
    # Visualize the route
    print("\n📊 Generating route visualization...")
    controller.visualize_route(vehicle_pos=vehicle_pos, save_fig=True)
    
    print("\n✅ Demo completed successfully!")

if __name__ == "__main__":
    main()

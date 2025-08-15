#!/usr/bin/env python3
"""
Test script for Pure Pursuit Controller
Demonstrates path following with road data visualization
"""

import matplotlib.pyplot as plt
import numpy as np
from pure_pursuit_controller import PurePursuitController
import json

def visualize_road_network(controller, max_links=5):
    """Visualize the road network (first few links for clarity)"""
    
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Plot first few road links
    for i in range(min(max_links, len(controller.links))):
        waypoints = controller.get_waypoints(i)
        if waypoints:
            # Extract x, y coordinates
            x_coords = [point[0] for point in waypoints]
            y_coords = [point[1] for point in waypoints]
            
            # Plot road link
            ax.plot(x_coords, y_coords, 'b-', linewidth=2, label=f'Road Link {i}')
            
            # Plot waypoints
            ax.scatter(x_coords, y_coords, c='red', s=20, alpha=0.7)
            
            # Mark start and end
            ax.scatter(x_coords[0], y_coords[0], c='green', s=100, marker='o', label=f'Start {i}')
            ax.scatter(x_coords[-1], y_coords[-1], c='red', s=100, marker='s', label=f'End {i}')
    
    ax.set_xlabel('X Coordinate (m)')
    ax.set_ylabel('Y Coordinate (m)')
    ax.set_title('Road Network Visualization')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    plt.show()

def test_controller_basic():
    """Test basic controller functionality"""
    
    print("=== Testing Pure Pursuit Controller ===\n")
    
    # Initialize controller
    controller = PurePursuitController("link_set.json", lookahead_distance=5.0)
    
    if not controller.links:
        print("Failed to load road data!")
        return
    
    print(f"✅ Loaded {len(controller.links)} road links")
    
    # Test first road link
    first_link = controller.get_current_road_link()
    if first_link:
        print(f"✅ First road link: {first_link['idx']}")
        print(f"   From: {first_link['from_node_idx']}")
        print(f"   To: {first_link['to_node_idx']}")
        print(f"   Waypoints: {len(first_link['points'])}")
        print(f"   Max speed: {first_link['max_speed']}")
        print(f"   Length: {first_link['link_length']:.2f} m")
    
    # Test waypoint finding
    waypoints = controller.get_waypoints(0)
    if waypoints:
        print(f"\n✅ First road link has {len(waypoints)} waypoints")
        print(f"   First waypoint: {waypoints[0]}")
        print(f"   Last waypoint: {waypoints[-1]}")
    
    print("\n=== Basic Tests Completed ===\n")

def test_pure_pursuit_logic():
    """Test pure pursuit steering calculations"""
    
    print("=== Testing Pure Pursuit Logic ===\n")
    
    controller = PurePursuitController("link_set.json", lookahead_distance=5.0)
    
    # Test vehicle at different positions
    test_positions = [
        [158.0, 189.0, 146.0],  # Near start of first road
        [155.0, 187.0, 146.0],  # Middle of first road
        [151.0, 184.0, 146.0],  # Near end of first road
    ]
    
    vehicle_heading = 0.0  # radians
    
    for i, pos in enumerate(test_positions):
        print(f"Test {i+1}: Vehicle at {pos}")
        
        # Get current road link waypoints
        waypoints = controller.get_waypoints(controller.current_link_idx)
        if waypoints:
            # Find lookahead waypoint
            lookahead_waypoint = controller.find_lookahead_waypoint(pos, waypoints)
            if lookahead_waypoint:
                # Calculate steering angle
                steering_angle = controller.calculate_steering_angle(pos, vehicle_heading, lookahead_waypoint)
                
                print(f"   Lookahead waypoint: {lookahead_waypoint}")
                print(f"   Steering angle: {np.degrees(steering_angle):.2f}°")
                
                # Calculate distance to lookahead point
                distance = controller.calculate_distance(pos, lookahead_waypoint)
                print(f"   Distance to lookahead: {distance:.2f} m")
            else:
                print("   No lookahead waypoint found")
        print()
    
    print("=== Pure Pursuit Logic Tests Completed ===\n")

def test_speed_control():
    """Test speed control based on road properties"""
    
    print("=== Testing Speed Control ===\n")
    
    controller = PurePursuitController("link_set.json", lookahead_distance=5.0)
    
    # Test speed calculation for different road links
    for i in range(min(5, len(controller.links))):
        link = controller.links[i]
        target_speed = controller.calculate_target_speed(link)
        
        print(f"Road Link {i}:")
        print(f"   ID: {link['idx']}")
        print(f"   Max Speed: {link['max_speed']}")
        print(f"   Target Speed: {target_speed:.2f} m/s")
        print(f"   Road Type: {link['road_type']}")
        print()
    
    print("=== Speed Control Tests Completed ===\n")

def main():
    """Run all tests"""
    
    print("🚗 Pure Pursuit Controller Test Suite")
    print("=" * 50)
    
    try:
        # Run basic tests
        test_controller_basic()
        
        # Test pure pursuit logic
        test_pure_pursuit_logic()
        
        # Test speed control
        test_speed_control()
        
        # Visualize road network
        print("=== Visualizing Road Network ===")
        controller = PurePursuitController("link_set.json", lookahead_distance=5.0)
        visualize_road_network(controller, max_links=3)
        
        print("\n🎉 All tests completed successfully!")
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()

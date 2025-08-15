#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Course Configuration File
Green Path Driving Test Course

This file contains all the configurable parameters for the driving test course.
Modify the waypoint ranges and parameters below to match your specific path file.

How to customize:
1. Load your path file in a path visualization tool
2. Identify the waypoint numbers for each section start and end
3. Update the course_sections dictionary below
4. Adjust speed and steering parameters as needed
5. Test and fine-tune the parameters
"""

# Speed Configuration (km/h or appropriate units)
class SpeedConfig:
    DEFAULT_SPEED = 19.0        # Normal driving speed
    SLOW_SPEED = 10.0          # Slow speed for turns and complex maneuvers
    PARKING_SPEED = 5.0        # Very slow speed for parking
    RAMP_SPEED = 24.0         # Higher speed for uphill sections
    FAST_SPEED = 40.0         # High speed for acceleration test
    STOP_SPEED = 0.0          # Complete stop

# Steering Configuration
class SteeringConfig:
    DEFAULT_LFD = 18           # Default look-ahead distance
    TURN_LFD = 8              # Look-ahead for 90° turns
    TIGHT_TURN_LFD = 5        # Look-ahead for very tight turns
    PARKING_LFD = 3           # Look-ahead for parking maneuvers
    STRAIGHT_LFD = 25         # Look-ahead for straight sections
    STEERING_OFFSET = 0.015   # Base steering offset
    STEERING_MULTIPLIER = 1.0  # Steering sensitivity multiplier

# Course Section Waypoint Ranges
# Format: "section_name": (start_waypoint, end_waypoint)
# IMPORTANT: These ranges must be customized to match your actual path file!
class CourseConfig:
    """
    Course section definitions - CUSTOMIZE THESE TO MATCH YOUR PATH FILE
    
    To find the correct waypoint numbers:
    1. Use rviz or similar tool to visualize your path
    2. Identify waypoint numbers at section boundaries
    3. Update the ranges below accordingly
    """
    
    COURSE_SECTIONS = {
        # Starting area and initial straight
        "start": (0, 50),
        "initial_straight": (51, 100),
        
        # First left turn (90°)
        "first_left_turn": (101, 150),
        
        # Ramp section (uphill/downhill with mandatory stop)
        "ramp_approach": (151, 200),
        "ramp_section": (201, 300),      # Main ramp with stop point
        "ramp_exit": (301, 350),
        
        # Second left turn (90°)
        "second_left_turn": (351, 400),
        
        # T-Course section (consecutive 90° turns)
        "t_course_approach": (401, 450),
        "t_course": (451, 550),          # Main T-course area
        "t_course_exit": (551, 600),
        
        # Traffic light intersection
        "traffic_light_approach": (601, 650),
        "traffic_light": (651, 700),     # Traffic light stop area
        
        # S-Curve section (curved course)
        "s_curve_approach": (701, 750),
        "s_curve": (751, 850),           # Main S-curve area
        "s_curve_exit": (851, 900),
        
        # First right turn (90°)
        "first_right_turn": (901, 950),
        
        # Parallel parking section
        "parking_approach": (951, 1000),
        "parallel_parking": (1001, 1100), # Main parking area
        "parking_exit": (1101, 1150),
        
        # Second right turn (90°)
        "second_right_turn": (1151, 1200),
        
        # Final sections
        "final_straight": (1201, 1300),
        "final_left_turn": (1301, 1350),
        "finish": (1351, 1400)
    }

# Mission-Specific Parameters
class MissionConfig:
    """Configuration for specific missions"""
    
    # Ramp Mission
    RAMP_STOP_PERCENTAGE = 0.7        # Stop at 70% through ramp section
    RAMP_STOP_DURATION = 3.0          # Stop duration in seconds
    
    # Traffic Light Mission
    TRAFFIC_LIGHT_THRESHOLD = 500     # Green vs Red count threshold
    
    # S-Curve Mission
    S_CURVE_GPS_THRESHOLD = 1.0       # GPS signal threshold for camera mode
    S_CURVE_YAW_LOWER = -80           # Lower yaw limit for steering adjustment
    S_CURVE_YAW_UPPER = -20           # Upper yaw limit for steering adjustment
    S_CURVE_CRITICAL_YAW = -25        # Critical yaw for speed reduction
    
    # Dynamic Obstacle Detection
    OBSTACLE_DISTANCE_THRESHOLD = 7.0  # Distance threshold for obstacle detection
    OBSTACLE_LATERAL_THRESHOLD = 3.0   # Lateral threshold for obstacle detection
    
    # Parking Mission
    PARKING_PRECISION_DISTANCE = 10   # Last N waypoints for precision parking

# Path Configuration
class PathConfig:
    """Path file configuration"""
    
    DEFAULT_PATH_NAME = 'green_path_test'  # Default path file name (without .txt)
    PATH_DIRECTORY = 'path_maker'          # Path files directory
    
    # Alternative path names for different scenarios
    ALTERNATIVE_PATHS = {
        'test_course': 'green_path_test',
        'practice_course': 'practice_path',
        'competition_course': 'competition_path'
    }

# ROS Topics Configuration
class TopicConfig:
    """ROS topic names"""
    
    # Publishers
    GLOBAL_PATH_TOPIC = '/global_path'
    CTRL_CMD_TOPIC = '/ctrl_cmd'
    MISSION_TOPIC = '/mission'
    WAYPOINT_TOPIC = '/waypoint'
    
    # Subscribers
    GPS_TOPIC = '/gps'
    IMU_TOPIC = '/imu'
    TRAFFIC_LIGHT_TOPIC = '/traffic_light'
    CAMERA_STEER_TOPIC = '/cam_steer'
    DYNAMIC_OBSTACLE_TOPIC = '/dy_obs_info'
    
    # Services
    MORAI_EVENT_SERVICE = '/Service_MoraiEventCmd'

# Debugging and Logging Configuration
class DebugConfig:
    """Debug and logging configuration"""
    
    ENABLE_SECTION_LOGGING = True      # Enable section transition logging
    ENABLE_WAYPOINT_LOGGING = False    # Enable detailed waypoint logging
    ENABLE_MISSION_LOGGING = True      # Enable mission status logging
    LOG_FREQUENCY = 10                 # Log every N iterations

# Validation Functions
def validate_course_sections():
    """Validate that course sections don't overlap and are in order"""
    sections = CourseConfig.COURSE_SECTIONS
    prev_end = -1
    
    for section_name, (start, end) in sections.items():
        if start <= prev_end:
            print(f"WARNING: Section '{section_name}' overlaps with previous section!")
            print(f"  Start: {start}, Previous end: {prev_end}")
        
        if start >= end:
            print(f"ERROR: Section '{section_name}' has invalid range: ({start}, {end})")
        
        prev_end = end
    
    print("Course section validation completed.")

def print_course_summary():
    """Print a summary of the course configuration"""
    sections = CourseConfig.COURSE_SECTIONS
    total_waypoints = max([end for _, (_, end) in sections.items()])
    
    print("=" * 60)
    print("DRIVING TEST COURSE CONFIGURATION SUMMARY")
    print("=" * 60)
    print(f"Total waypoints: {total_waypoints}")
    print(f"Number of sections: {len(sections)}")
    print()
    
    for i, (section_name, (start, end)) in enumerate(sections.items(), 1):
        waypoint_count = end - start + 1
        print(f"{i:2d}. {section_name.upper().replace('_', ' '):<25} "
              f"Waypoints: {start:4d}-{end:4d} ({waypoint_count:3d} points)")
    
    print("=" * 60)

if __name__ == '__main__':
    """Run validation and print summary when executed directly"""
    print_course_summary()
    validate_course_sections()

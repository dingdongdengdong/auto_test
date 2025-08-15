#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Example Usage of the Driving Test Template

This file demonstrates how to use the driving test template
and customize it for your specific course requirements.
"""

from driving_test_template import DrivingTestTemplate, MissionType, ActionType
from course_config import CourseConfig, SpeedConfig, SteeringConfig
import rospy

class CustomDrivingTest(DrivingTestTemplate):
    """
    Example of how to extend the driving test template
    for custom course requirements
    """
    
    def __init__(self):
        # You can override configuration before calling parent init
        self.custom_speed_profile = {
            "slow_sections": [3, 5, 8, 10],  # Section numbers that need slow speed
            "fast_sections": [2, 6, 11]      # Section numbers that can go fast
        }
        
        # Call parent constructor
        super().__init__()
    
    def execute_custom_obstacle_avoidance(self, waypoint):
        """
        Example of adding a custom mission for obstacle avoidance
        """
        if self.dynamic_obstacle_detected:
            print(f"[CUSTOM OBSTACLE] Detected at waypoint {waypoint}")
            
            # Custom obstacle avoidance logic
            if self.dy_obs_info[0] < 3.0:  # Very close obstacle
                self.brake()
                self.emergency_mode()
                return True
            elif self.dy_obs_info[0] < 5.0:  # Moderate distance
                self.motor_msg = 5.0  # Slow down significantly
                self.setServoMsgWithLfd(10)  # Increase look-ahead for smoother path
                return True
        
        return False  # No obstacle detected
    
    def execute_mission_by_waypoint(self, current_waypoint):
        """
        Override the main mission execution to add custom behavior
        """
        # First check for custom obstacle avoidance
        if self.execute_custom_obstacle_avoidance(current_waypoint):
            return  # Skip normal mission if handling obstacle
        
        # Apply custom speed profile
        current_section = self.get_current_section(current_waypoint)
        section_number = list(self.course_sections.keys()).index(current_section)
        
        if section_number in self.custom_speed_profile["slow_sections"]:
            base_speed = 8.0  # Slower than normal
        elif section_number in self.custom_speed_profile["fast_sections"]:
            base_speed = 25.0  # Faster than normal
        else:
            base_speed = 19.0  # Normal speed
        
        # Call parent method for standard behavior
        super().execute_mission_by_waypoint(current_waypoint)
        
        # Apply speed override if not already set by specific mission
        if hasattr(self, 'motor_msg') and self.motor_msg == 19.0:
            self.motor_msg = base_speed
    
    def execute_custom_parking_mission(self, waypoint):
        """
        Example of a more detailed parallel parking implementation
        """
        section_start, section_end = self.course_sections["parallel_parking"]
        progress = (waypoint - section_start) / (section_end - section_start)
        
        if progress < 0.3:  # Approach phase
            self.motor_msg = 3.0
            self.setServoMsgWithLfd(15)
            print("[CUSTOM PARKING] Approach phase")
            
        elif progress < 0.6:  # Alignment phase
            self.motor_msg = 2.0
            self.setServoMsgWithLfd(5)
            self.servo_msg *= 1.5
            print("[CUSTOM PARKING] Alignment phase")
            
        elif progress < 0.9:  # Backing phase
            self.motor_msg = 1.5
            self.rear_mode()  # Switch to reverse
            self.servo_msg *= -1  # Reverse steering
            print("[CUSTOM PARKING] Backing phase")
            
        else:  # Exit phase
            self.motor_msg = 2.0
            self.forward_mode()  # Back to forward
            self.setServoMsgWithLfd(8)
            print("[CUSTOM PARKING] Exit phase")

def example_configuration_override():
    """
    Example of how to override course configuration for testing
    """
    # Save original configuration
    original_sections = CourseConfig.COURSE_SECTIONS.copy()
    
    # Override for testing with smaller waypoint ranges
    CourseConfig.COURSE_SECTIONS = {
        "start": (0, 20),
        "initial_straight": (21, 50),
        "first_left_turn": (51, 80),
        "test_section": (81, 120),
        "finish": (121, 150)
    }
    
    print("Configuration overridden for testing:")
    for section, (start, end) in CourseConfig.COURSE_SECTIONS.items():
        print(f"  {section}: waypoints {start}-{end}")
    
    # Restore original configuration
    CourseConfig.COURSE_SECTIONS = original_sections

def example_mission_state_monitoring():
    """
    Example of how to monitor mission completion status
    """
    # This would be called from within your driving test class
    def check_mission_progress(self):
        completed_missions = [mission for mission, status in self.mission_completed.items() if status]
        total_missions = len(self.mission_completed)
        completion_percentage = (len(completed_missions) / total_missions) * 100
        
        print(f"Mission Progress: {completion_percentage:.1f}% ({len(completed_missions)}/{total_missions})")
        print(f"Completed: {', '.join(completed_missions)}")
        
        remaining_missions = [mission for mission, status in self.mission_completed.items() if not status]
        if remaining_missions:
            print(f"Remaining: {', '.join(remaining_missions)}")

def example_custom_waypoint_actions():
    """
    Example of creating a custom waypoint-action mapping
    """
    # Custom action definitions for specific waypoints
    custom_actions = {
        # Waypoint: (action, parameters)
        50: (ActionType.PREPARE_TURN_LEFT, {"speed": 8, "lfd": 10}),
        100: (ActionType.ENTER_RAMP_SECTION, {"speed": 22, "stop_duration": 4}),
        200: (ActionType.CHECK_TRAFFIC_LIGHT, {"threshold": 600}),
        300: (ActionType.ENTER_S_CURVE, {"use_camera": True}),
        400: (ActionType.START_PARALLEL_PARKING, {"precision_mode": True}),
    }
    
    def execute_custom_action(self, waypoint):
        """Execute custom action if defined for current waypoint"""
        if waypoint in custom_actions:
            action, params = custom_actions[waypoint]
            print(f"[CUSTOM ACTION] Executing {action} at waypoint {waypoint}")
            
            # Execute based on action type
            if action == ActionType.PREPARE_TURN_LEFT:
                self.motor_msg = params["speed"]
                self.setServoMsgWithLfd(params["lfd"])
                self.drive_left_signal()
                
            elif action == ActionType.ENTER_RAMP_SECTION:
                self.motor_msg = params["speed"]
                # Custom ramp logic here
                
            # Add more action handlers as needed
            
            return True
        return False

if __name__ == '__main__':
    """
    Example of different ways to run the driving test
    """
    print("Driving Test Template Usage Examples")
    print("=" * 50)
    
    # Example 1: Basic usage
    print("\n1. Basic Template Usage:")
    print("   rosrun pure_pursuit driving_test_template.py")
    
    # Example 2: Configuration override
    print("\n2. Configuration Override Example:")
    example_configuration_override()
    
    # Example 3: Custom class usage
    print("\n3. Custom Extended Class:")
    print("   Use CustomDrivingTest class for advanced features")
    
    # Example 4: Mission monitoring
    print("\n4. Mission State Monitoring:")
    print("   Available in the CustomDrivingTest class")
    
    # Example 5: Custom actions
    print("\n5. Custom Waypoint Actions:")
    print("   See example_custom_waypoint_actions() function")
    
    print("\nTo run the custom driving test:")
    print("   python3 example_usage.py --run")
    
    # Actually run if requested
    import sys
    if "--run" in sys.argv:
        try:
            rospy.init_node('custom_driving_test_example', anonymous=True)
            custom_test = CustomDrivingTest()
        except rospy.ROSInterruptException:
            pass

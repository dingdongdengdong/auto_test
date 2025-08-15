#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Autonomous Driving Test Template
Based on Green Path Scenario

This template implements a complete driving test course with the following sections:
1. Start → Initial Straight
2. First Left Turn (90°)
3. Ramp Section (Uphill/Downhill)
4. Second Left Turn (90°)
5. T-Course (Right-angle turns)
6. Traffic Light Intersection
7. S-Curve (Curved course)
8. First Right Turn (90°)
9. Parallel Parking Section
10. Second Right Turn (90°)
11. Final Straight
12. Final Left Turn (90°)
13. End
"""

import warnings
import sys, os
import rospy
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Bool, String, Int64MultiArray, Float64MultiArray, Int64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, transform
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, GPSMessage, CtrlCmd, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils import pathReader, findLocalPath, purePursuit
import tf
import time
from math import *

warnings.simplefilter(action='ignore', category=FutureWarning)

# Course Configuration
DEFAULT_LFD = 18
DEFAULT_SPEED = 19.0
SLOW_SPEED = 10.0
PARKING_SPEED = 5.0

# Mission Types
class MissionType:
    START = "START"
    STRAIGHT = "STRAIGHT"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    RAMP_UP = "RAMP_UP"
    RAMP_DOWN = "RAMP_DOWN"
    T_COURSE = "T_COURSE"
    TRAFFIC_LIGHT = "TRAFFIC_LIGHT"
    S_CURVE = "S_CURVE"
    PARALLEL_PARKING = "PARALLEL_PARKING"
    END = "END"

# Action Definitions
class ActionType:
    CONTINUE_STRAIGHT = "CONTINUE_STRAIGHT"
    PREPARE_TURN_LEFT = "PREPARE_TURN_LEFT"
    EXECUTE_TURN_LEFT = "EXECUTE_TURN_LEFT"
    PREPARE_TURN_RIGHT = "PREPARE_TURN_RIGHT"
    EXECUTE_TURN_RIGHT = "EXECUTE_TURN_RIGHT"
    ENTER_RAMP_SECTION = "ENTER_RAMP_SECTION"
    EXIT_RAMP_SECTION = "EXIT_RAMP_SECTION"
    ENTER_T_COURSE = "ENTER_T_COURSE"
    EXIT_T_COURSE = "EXIT_T_COURSE"
    CHECK_TRAFFIC_LIGHT = "CHECK_TRAFFIC_LIGHT"
    ENTER_S_CURVE = "ENTER_S_CURVE"
    EXIT_S_CURVE = "EXIT_S_CURVE"
    START_PARALLEL_PARKING = "START_PARALLEL_PARKING"
    COMPLETE_PARALLEL_PARKING = "COMPLETE_PARALLEL_PARKING"
    FINISH_COURSE = "FINISH_COURSE"

class DrivingTestTemplate:
    def __init__(self):
        rospy.init_node('driving_test_template', anonymous=True)
        
        # Path Configuration
        self.path_name = 'green_path_test'
        
        # Mission State Variables
        self.current_mission = MissionType.START
        self.mission_completed = {
            MissionType.START: False,
            MissionType.RAMP_UP: False,
            MissionType.RAMP_DOWN: False,
            MissionType.T_COURSE: False,
            MissionType.TRAFFIC_LIGHT: False,
            MissionType.S_CURVE: False,
            MissionType.PARALLEL_PARKING: False,
        }
        
        # Course Section Waypoint Ranges (to be customized based on your path)
        self.course_sections = {
            # Section: (start_waypoint, end_waypoint)
            "start": (0, 50),
            "initial_straight": (51, 100),
            "first_left_turn": (101, 150),
            "ramp_approach": (151, 200),
            "ramp_section": (201, 300),
            "ramp_exit": (301, 350),
            "second_left_turn": (351, 400),
            "t_course_approach": (401, 450),
            "t_course": (451, 550),
            "t_course_exit": (551, 600),
            "traffic_light_approach": (601, 650),
            "traffic_light": (651, 700),
            "s_curve_approach": (701, 750),
            "s_curve": (751, 850),
            "s_curve_exit": (851, 900),
            "first_right_turn": (901, 950),
            "parking_approach": (951, 1000),
            "parallel_parking": (1001, 1100),
            "parking_exit": (1101, 1150),
            "second_right_turn": (1151, 1200),
            "final_straight": (1201, 1300),
            "final_left_turn": (1301, 1350),
            "finish": (1351, 1400)
        }
        
        # Publishers
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.mission_pub = rospy.Publisher('/mission', String, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/waypoint', Int64, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.ImuCB)
        rospy.Subscriber("/traffic_light", Int64MultiArray, self.trafficCB)
        rospy.Subscriber("/cam_steer", CtrlCmd, self.laneCB)
        rospy.Subscriber("/dy_obs_info", Float64MultiArray, self.dyObsCB)
        
        # Vehicle State Variables
        self.ctrl_cmd_msg = CtrlCmd()
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id = '/base_link'
        
        # GPS and Position
        self.latitude = 0.0
        self.longitude = 0.0
        self.x, self.y = 0.0, 0.0
        self.yaw = 0.0
        
        # Control Variables
        self.motor_msg = DEFAULT_SPEED
        self.servo_msg = 0.0
        self.brake_msg = 0.0
        self.steering_offset = 0.015
        
        # Mission Specific Variables
        self.traffic_signal = 0
        self.green_count = 0
        self.red_count = 0
        self.dynamic_obstacle_detected = False
        self.camera_servo_msg = 0.0
        self.dy_obs_info = [0, 0, 0, 0]
        
        # Transform Broadcaster
        self.br = tf.TransformBroadcaster()
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        # Service Setup
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.req_service = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
        self.req = EventInfo()
        
        # Initialize Systems
        self.forward_mode()
        
        # Path and Pure Pursuit Setup
        path_reader = pathReader('path_maker')
        self.pure_pursuit = purePursuit()
        self.global_path = path_reader.read_txt(self.path_name + ".txt")
        
        # Start Main Loop
        self.run_driving_test()
    
    def run_driving_test(self):
        """Main driving test execution loop"""
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            # Publish global path
            self.global_path_pub.publish(self.global_path)
            
            # Get local path and current waypoint
            local_path, current_waypoint = findLocalPath(self.global_path, self.odom_msg)
            self.convertLL2UTM()
            
            # Update pure pursuit
            self.pure_pursuit.getPath(local_path)
            self.pure_pursuit.getEgoStatus(self.odom_msg, self.motor_msg, False)
            self.steering, target_x, target_y = self.pure_pursuit.steering_angle(DEFAULT_LFD)
            
            # Publish current waypoint
            self.waypoint_pub.publish(current_waypoint)
            
            # Execute mission based on current waypoint
            self.execute_mission_by_waypoint(current_waypoint)
            
            # Publish control commands
            self.publishCtrlCmd()
            
            # Publish current mission status
            self.mission_pub.publish(self.current_mission)
            
            rate.sleep()
    
    def execute_mission_by_waypoint(self, current_waypoint):
        """Execute appropriate mission based on current waypoint"""
        
        # Determine current section
        current_section = self.get_current_section(current_waypoint)
        
        # Reset default values
        self.motor_msg = DEFAULT_SPEED
        self.servo_msg = self.steering * self.steering_offset
        self.brake_msg = 0.0
        
        # Execute mission based on current section
        if current_section == "start":
            self.execute_start_mission(current_waypoint)
            
        elif current_section == "initial_straight":
            self.execute_initial_straight_mission(current_waypoint)
            
        elif current_section == "first_left_turn":
            self.execute_first_left_turn_mission(current_waypoint)
            
        elif current_section == "ramp_approach":
            self.execute_ramp_approach_mission(current_waypoint)
            
        elif current_section == "ramp_section":
            self.execute_ramp_section_mission(current_waypoint)
            
        elif current_section == "ramp_exit":
            self.execute_ramp_exit_mission(current_waypoint)
            
        elif current_section == "second_left_turn":
            self.execute_second_left_turn_mission(current_waypoint)
            
        elif current_section == "t_course_approach":
            self.execute_t_course_approach_mission(current_waypoint)
            
        elif current_section == "t_course":
            self.execute_t_course_mission(current_waypoint)
            
        elif current_section == "t_course_exit":
            self.execute_t_course_exit_mission(current_waypoint)
            
        elif current_section == "traffic_light_approach":
            self.execute_traffic_light_approach_mission(current_waypoint)
            
        elif current_section == "traffic_light":
            self.execute_traffic_light_mission(current_waypoint)
            
        elif current_section == "s_curve_approach":
            self.execute_s_curve_approach_mission(current_waypoint)
            
        elif current_section == "s_curve":
            self.execute_s_curve_mission(current_waypoint)
            
        elif current_section == "s_curve_exit":
            self.execute_s_curve_exit_mission(current_waypoint)
            
        elif current_section == "first_right_turn":
            self.execute_first_right_turn_mission(current_waypoint)
            
        elif current_section == "parking_approach":
            self.execute_parking_approach_mission(current_waypoint)
            
        elif current_section == "parallel_parking":
            self.execute_parallel_parking_mission(current_waypoint)
            
        elif current_section == "parking_exit":
            self.execute_parking_exit_mission(current_waypoint)
            
        elif current_section == "second_right_turn":
            self.execute_second_right_turn_mission(current_waypoint)
            
        elif current_section == "final_straight":
            self.execute_final_straight_mission(current_waypoint)
            
        elif current_section == "final_left_turn":
            self.execute_final_left_turn_mission(current_waypoint)
            
        elif current_section == "finish":
            self.execute_finish_mission(current_waypoint)
    
    def get_current_section(self, waypoint):
        """Determine which section the vehicle is currently in"""
        for section, (start, end) in self.course_sections.items():
            if start <= waypoint <= end:
                return section
        return "unknown"
    
    # Mission Execution Methods
    def execute_start_mission(self, waypoint):
        """Execute start mission"""
        self.current_mission = MissionType.START
        self.motor_msg = SLOW_SPEED
        self.setServoMsgWithLfd(DEFAULT_LFD)
        print(f"[START] Waypoint: {waypoint}")
    
    def execute_initial_straight_mission(self, waypoint):
        """Execute initial straight driving"""
        self.current_mission = MissionType.STRAIGHT
        self.motor_msg = DEFAULT_SPEED
        self.setServoMsgWithLfd(DEFAULT_LFD)
        print(f"[INITIAL_STRAIGHT] Waypoint: {waypoint}")
    
    def execute_first_left_turn_mission(self, waypoint):
        """Execute first left turn (90°)"""
        self.current_mission = MissionType.TURN_LEFT
        self.motor_msg = SLOW_SPEED
        self.setServoMsgWithLfd(8)  # Smaller LFD for tight turns
        self.drive_left_signal()
        print(f"[FIRST_LEFT_TURN] Waypoint: {waypoint}")
    
    def execute_ramp_approach_mission(self, waypoint):
        """Execute ramp approach"""
        self.current_mission = MissionType.RAMP_UP
        self.motor_msg = DEFAULT_SPEED
        self.setServoMsgWithLfd(DEFAULT_LFD)
        print(f"[RAMP_APPROACH] Waypoint: {waypoint}")
    
    def execute_ramp_section_mission(self, waypoint):
        """Execute ramp section (uphill/downhill with stop)"""
        self.current_mission = MissionType.RAMP_UP
        
        # Check if this is the stopping point in the ramp
        section_start, section_end = self.course_sections["ramp_section"]
        stop_point = section_start + int((section_end - section_start) * 0.7)  # Stop at 70% through ramp
        
        if waypoint < stop_point:
            # Going uphill - increase power
            self.motor_msg = DEFAULT_SPEED + 5
            self.setServoMsgWithLfd(DEFAULT_LFD)
        elif waypoint == stop_point and not self.mission_completed[MissionType.RAMP_UP]:
            # Stop at the designated point
            self.brake()
            self.mission_completed[MissionType.RAMP_UP] = True
            rospy.sleep(3)  # Stop for 3 seconds
            print("[RAMP_SECTION] Completed stop mission")
        else:
            # Continue downhill
            self.motor_msg = SLOW_SPEED
            self.setServoMsgWithLfd(DEFAULT_LFD)
        
        print(f"[RAMP_SECTION] Waypoint: {waypoint}")
    
    def execute_ramp_exit_mission(self, waypoint):
        """Execute ramp exit"""
        self.current_mission = MissionType.RAMP_DOWN
        self.motor_msg = DEFAULT_SPEED
        self.setServoMsgWithLfd(DEFAULT_LFD)
        print(f"[RAMP_EXIT] Waypoint: {waypoint}")
    
    def execute_second_left_turn_mission(self, waypoint):
        """Execute second left turn (90°)"""
        self.current_mission = MissionType.TURN_LEFT
        self.motor_msg = SLOW_SPEED
        self.setServoMsgWithLfd(8)
        self.drive_left_signal()
        print(f"[SECOND_LEFT_TURN] Waypoint: {waypoint}")
    
    def execute_t_course_approach_mission(self, waypoint):
        """Execute T-course approach"""
        self.current_mission = MissionType.T_COURSE
        self.motor_msg = SLOW_SPEED
        self.setServoMsgWithLfd(12)
        print(f"[T_COURSE_APPROACH] Waypoint: {waypoint}")
    
    def execute_t_course_mission(self, waypoint):
        """Execute T-course (consecutive 90° turns in narrow space)"""
        self.current_mission = MissionType.T_COURSE
        self.motor_msg = PARKING_SPEED  # Very slow for precision
        self.setServoMsgWithLfd(5)  # Very small LFD for tight maneuvering
        self.servo_msg *= 1.5  # Increase steering sensitivity
        print(f"[T_COURSE] Waypoint: {waypoint}")
    
    def execute_t_course_exit_mission(self, waypoint):
        """Execute T-course exit"""
        self.current_mission = MissionType.T_COURSE
        self.motor_msg = SLOW_SPEED
        self.setServoMsgWithLfd(12)
        self.mission_completed[MissionType.T_COURSE] = True
        print(f"[T_COURSE_EXIT] Waypoint: {waypoint}")
    
    def execute_traffic_light_approach_mission(self, waypoint):
        """Execute traffic light approach"""
        self.current_mission = MissionType.TRAFFIC_LIGHT
        self.motor_msg = SLOW_SPEED
        self.setServoMsgWithLfd(DEFAULT_LFD)
        print(f"[TRAFFIC_LIGHT_APPROACH] Waypoint: {waypoint}")
    
    def execute_traffic_light_mission(self, waypoint):
        """Execute traffic light mission"""
        self.current_mission = MissionType.TRAFFIC_LIGHT
        
        # Check traffic light status
        if self.green_count - self.red_count < 500:  # Red light detected
            print(f"[TRAFFIC_LIGHT] RED LIGHT - STOPPING at waypoint: {waypoint}")
            self.brake()
        else:  # Green light
            print(f"[TRAFFIC_LIGHT] GREEN LIGHT - PROCEEDING at waypoint: {waypoint}")
            self.motor_msg = DEFAULT_SPEED
            self.setServoMsgWithLfd(DEFAULT_LFD)
            self.mission_completed[MissionType.TRAFFIC_LIGHT] = True
    
    def execute_s_curve_approach_mission(self, waypoint):
        """Execute S-curve approach"""
        self.current_mission = MissionType.S_CURVE
        self.motor_msg = SLOW_SPEED
        self.setServoMsgWithLfd(12)
        print(f"[S_CURVE_APPROACH] Waypoint: {waypoint}")
    
    def execute_s_curve_mission(self, waypoint):
        """Execute S-curve (curved course navigation)"""
        self.current_mission = MissionType.S_CURVE
        
        # Use camera steering if available in GPS shadow areas
        if self.original_latitude <= 1.0 and self.original_longitude <= 1.0:
            print("[S_CURVE] Using camera steering in GPS shadow area")
            self.motor_msg = 15 - min(7, abs(self.camera_servo_msg * 20))
            self.servo_msg = self.camera_servo_msg
            
            # Adjust for specific yaw conditions
            if self.yaw < -25:
                self.motor_msg = 6
            if -80 < self.yaw < -20:
                self.servo_msg = self.camera_servo_msg * 2
        else:
            # Normal S-curve navigation
            self.motor_msg = SLOW_SPEED
            self.setServoMsgWithLfd(6)
        
        print(f"[S_CURVE] Waypoint: {waypoint}, Yaw: {self.yaw}")
    
    def execute_s_curve_exit_mission(self, waypoint):
        """Execute S-curve exit"""
        self.current_mission = MissionType.S_CURVE
        
        # Gradual return to normal driving after S-curve
        if -80 < self.yaw < -20:
            self.servo_msg = self.camera_servo_msg * 2
            self.motor_msg = 6
            self.setServoMsgWithLfd(2)
        else:
            self.motor_msg = SLOW_SPEED
            self.setServoMsgWithLfd(12)
            self.mission_completed[MissionType.S_CURVE] = True
        
        print(f"[S_CURVE_EXIT] Waypoint: {waypoint}")
    
    def execute_first_right_turn_mission(self, waypoint):
        """Execute first right turn (90°)"""
        self.current_mission = MissionType.TURN_RIGHT
        self.motor_msg = SLOW_SPEED
        self.setServoMsgWithLfd(8)
        self.drive_right_signal()
        print(f"[FIRST_RIGHT_TURN] Waypoint: {waypoint}")
    
    def execute_parking_approach_mission(self, waypoint):
        """Execute parallel parking approach"""
        self.current_mission = MissionType.PARALLEL_PARKING
        self.motor_msg = PARKING_SPEED
        self.setServoMsgWithLfd(DEFAULT_LFD)
        print(f"[PARKING_APPROACH] Waypoint: {waypoint}")
    
    def execute_parallel_parking_mission(self, waypoint):
        """Execute parallel parking maneuver"""
        self.current_mission = MissionType.PARALLEL_PARKING
        
        # This is a simplified parallel parking - in reality, you would need
        # multiple path segments for: approach, reverse, align, exit
        self.motor_msg = PARKING_SPEED
        self.setServoMsgWithLfd(5)
        self.servo_msg *= 2  # Increase steering sensitivity for parking
        
        print(f"[PARALLEL_PARKING] Waypoint: {waypoint}")
    
    def execute_parking_exit_mission(self, waypoint):
        """Execute parking exit"""
        self.current_mission = MissionType.PARALLEL_PARKING
        self.motor_msg = PARKING_SPEED
        self.setServoMsgWithLfd(8)
        self.mission_completed[MissionType.PARALLEL_PARKING] = True
        print(f"[PARKING_EXIT] Waypoint: {waypoint}")
    
    def execute_second_right_turn_mission(self, waypoint):
        """Execute second right turn (90°)"""
        self.current_mission = MissionType.TURN_RIGHT
        self.motor_msg = SLOW_SPEED
        self.setServoMsgWithLfd(8)
        self.drive_right_signal()
        print(f"[SECOND_RIGHT_TURN] Waypoint: {waypoint}")
    
    def execute_final_straight_mission(self, waypoint):
        """Execute final straight section"""
        self.current_mission = MissionType.STRAIGHT
        self.motor_msg = DEFAULT_SPEED
        self.setServoMsgWithLfd(DEFAULT_LFD)
        print(f"[FINAL_STRAIGHT] Waypoint: {waypoint}")
    
    def execute_final_left_turn_mission(self, waypoint):
        """Execute final left turn (90°)"""
        self.current_mission = MissionType.TURN_LEFT
        self.motor_msg = SLOW_SPEED
        self.setServoMsgWithLfd(8)
        self.drive_left_signal()
        print(f"[FINAL_LEFT_TURN] Waypoint: {waypoint}")
    
    def execute_finish_mission(self, waypoint):
        """Execute finish mission"""
        self.current_mission = MissionType.END
        
        # Gradual stop at finish line
        section_start, section_end = self.course_sections["finish"]
        if waypoint >= section_end - 10:  # Last 10 waypoints
            print("[FINISH] Approaching finish line - stopping")
            self.brake()
            self.parking()  # Set to parking mode
        else:
            self.motor_msg = SLOW_SPEED
            self.setServoMsgWithLfd(DEFAULT_LFD)
        
        print(f"[FINISH] Waypoint: {waypoint}")
    
    # Callback Functions
    def gpsCB(self, msg):
        """GPS callback function"""
        self.original_longitude = msg.longitude
        self.original_latitude = msg.latitude
        self.lat = msg.latitude
        self.lon = msg.longitude
        
        self.convertLL2UTM()
        
        # Transform broadcast
        self.br.sendTransform((self.x, self.y, 0.),
                             tf.transformations.quaternion_from_euler(0, 0, 0.),
                             rospy.Time.now(),
                             "base_link",
                             "map")
        
        # Update odometry
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0
    
    def convertLL2UTM(self):
        """Convert Latitude/Longitude to UTM coordinates"""
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x, self.y = xy_zone[0], xy_zone[1]
    
    def ImuCB(self, msg):
        """IMU callback function"""
        self.odom_msg.pose.pose.orientation.x = msg.orientation.x
        self.odom_msg.pose.pose.orientation.y = msg.orientation.y
        self.odom_msg.pose.pose.orientation.z = msg.orientation.z
        self.odom_msg.pose.pose.orientation.w = msg.orientation.w
        
        quaternion = (self.odom_msg.pose.pose.orientation.x,
                     self.odom_msg.pose.pose.orientation.y,
                     self.odom_msg.pose.pose.orientation.z,
                     self.odom_msg.pose.pose.orientation.w)
        _, _, self.yaw = euler_from_quaternion(quaternion)
        self.yaw = degrees(self.yaw)
    
    def trafficCB(self, msg):
        """Traffic light callback function"""
        self.green_count = msg.data[1]
        self.red_count = msg.data[0]
    
    def laneCB(self, msg):
        """Camera lane detection callback"""
        self.camera_servo_msg = msg.steering
    
    def dyObsCB(self, msg):
        """Dynamic obstacle callback"""
        self.dy_obs_info = msg.data
        if 0 < self.dy_obs_info[0] < 7.0 and abs(self.dy_obs_info[1]) < 3.0:
            self.dynamic_obstacle_detected = True
        else:
            self.dynamic_obstacle_detected = False
    
    # Control Functions
    def publishCtrlCmd(self):
        """Publish control command"""
        self.ctrl_cmd_msg.velocity = self.motor_msg
        self.ctrl_cmd_msg.steering = self.servo_msg
        self.ctrl_cmd_msg.brake = self.brake_msg
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
    
    def setServoMsgWithLfd(self, lfd):
        """Set servo message with look-ahead distance"""
        self.steering, _, _ = self.pure_pursuit.steering_angle(lfd)
        self.servo_msg = self.steering * self.steering_offset
    
    def brake(self):
        """Apply brake"""
        self.ctrl_cmd_msg.longlCmdType = 2
        self.motor_msg = 0.0
        self.servo_msg = 0.0
        self.brake_msg = 1.0
    
    # Service Request Functions
    def forward_mode(self):
        """Set vehicle to forward driving mode"""
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 0
        self.req.lamps.emergencySignal = 0
        response = self.req_service(self.req)
    
    def drive_left_signal(self):
        """Activate left turn signal"""
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 1
        response = self.req_service(self.req)
    
    def drive_right_signal(self):
        """Activate right turn signal"""
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 2
        response = self.req_service(self.req)
    
    def parking(self):
        """Set vehicle to parking mode"""
        self.req.option = 6
        self.req.gear = 1
        self.req.lamps.turnSignal = 0
        response = self.req_service(self.req)


if __name__ == '__main__':
    try:
        driving_test = DrivingTestTemplate()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simplified Pure Pursuit for L-Course Navigation
- GPS + IMU based autonomous driving
- Waypoint-based speed and steering control
- Optimized for L-shaped course with curve handling
"""

import sys
import os
# Add the current directory to Python path to find utils module
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Int64, Bool
from morai_msgs.msg import GPSMessage, CtrlCmd
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, sqrt, pow, atan2, pi, cos, sin
import tf
from pyproj import Proj

from utils import pathReader, purePursuit, findLocalPath

class SimplePurePursuit():
    def __init__(self):
        rospy.init_node('simple_pure_pursuit', anonymous=True)
        
        # Path name
        self.path_name = 'first_L_path'
        
        # Publishers
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/waypoint', Int64, queue_size=1)
        self.mission_pub = rospy.Publisher('/mission', Bool, queue_size=1)  # For lane detection
        
        # Subscribers (GPS + IMU + Camera)
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.ImuCB)
        rospy.Subscriber("/cam_steer", CtrlCmd, self.laneCB)  # Camera lane detection
        
        # Initialize variables
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.yaw = 0.0
        
        self.motor_msg = 0.0
        self.servo_msg = 0.0  # Initialize steering to 0 (straight)
        self.brake_msg = 1.0  # Initialize with brake on for safety
        
        self.ctrl_cmd_msg = CtrlCmd()
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id = '/base_link'
        
        # Initialize coordinate system
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.original_latitude = 0
        self.original_longitude = 0
        
        # Control parameters
        self.steering_offset = 1.0  # Enable steering for ㄱ course navigation
        self.default_velocity = 15.0
        self.default_lfd = 15
        
        # Camera lane detection
        self.camera_servo_msg = 0.0
        self.camera_motor_msg = 10.0
        self.use_camera_steering = True  # Enable by default for L-course
        self.max_steering_deviation = 0.3  # Maximum allowed steering deviation from GPS path
        self.lane_safety_factor = 0.8  # How much to trust camera vs GPS (0.8 = 80% camera, 20% GPS)
        
        # Classes
        path_reader = pathReader('path_maker')
        self.pure_pursuit = purePursuit()
        
        # Read path
        try:
            self.global_path = path_reader.read_txt(self.path_name + ".txt")
            rospy.loginfo(f"Path loaded successfully: {len(self.global_path.poses)} waypoints")
        except Exception as e:
            rospy.logerr(f"Failed to load path: {e}")
            return
            
        # TF broadcaster
        self.br = tf.TransformBroadcaster()
        
        # Wait for GPS signal
        rospy.loginfo("Waiting for GPS signal...")
        while not rospy.is_shutdown() and (self.latitude == 0.0 or self.longitude == 0.0):
            rospy.sleep(0.1)
        
        if not rospy.is_shutdown():
            rospy.loginfo("GPS signal received! Starting L-Course navigation...")
        
        # Main loop
        rate = rospy.Rate(20)  # 20 Hz
        rospy.loginfo("Simple Pure Pursuit started!")
        
        while not rospy.is_shutdown():
            self.main_loop()
            rate.sleep()
    
    def main_loop(self):
        if self.latitude == 0.0 or self.longitude == 0.0:
            # No GPS signal - keep vehicle stopped and straight
            self.motor_msg = 0.0
            self.servo_msg = 0.0
            self.brake_msg = 1.0
            self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
            return  # Wait for GPS data
            
        # Convert GPS to UTM
        try:
            xy_zone = self.proj_UTM(self.longitude, self.latitude)
            self.odom_msg.pose.pose.position.x = xy_zone[0]
            self.odom_msg.pose.pose.position.y = xy_zone[1]
            self.odom_msg.pose.pose.position.z = 0.0
            
            # Set orientation from IMU
            quat = quaternion_from_euler(0, 0, self.yaw * pi / 180)
            self.odom_msg.pose.pose.orientation.x = quat[0]
            self.odom_msg.pose.pose.orientation.y = quat[1] 
            self.odom_msg.pose.pose.orientation.z = quat[2]
            self.odom_msg.pose.pose.orientation.w = quat[3]
            
            # Find local path and current waypoint
            local_path, current_waypoint = findLocalPath(self.global_path, self.odom_msg)
            
            # Update pure pursuit with local path
            self.pure_pursuit.getPath(local_path)
            self.pure_pursuit.getEgoStatus(self.odom_msg, self.motor_msg, False)
            
            # Default control parameters
            self.motor_msg = self.default_velocity
            lfd = self.default_lfd
            
            # L-Course mission control based on waypoint (Updated with speed optimization)
            mission_phase = "Unknown"
            
            # Straight section 1 (waypoints 1-40): Fast approach
            if 1 <= current_waypoint <= 40:
                self.setMotorMsgWithVel(25.0)  # Increased from 15.0
                self.setServoMsgWithLfd(18)    # Increased LFD for stability
                lfd = 18
                mission_phase = "Straight-1"
                
            # Left turn section (waypoints 40-80): Use camera + GPS hybrid with safety
            elif 40 < current_waypoint <= 80:
                self.setMotorMsgWithVel(12.0)  # Reduced for safety during turn
                self.setServoMsgWithLfd(10)    
                lfd = 10
                mission_phase = "Left-Turn"
                
                # Use camera steering with safety limits
                if self.use_camera_steering:
                    gps_steering = self.servo_msg
                    
                    # Check if camera steering is reasonable (not too different from GPS)
                    if abs(self.camera_servo_msg - gps_steering) < self.max_steering_deviation:
                        # Safe to use camera steering
                        camera_weight = self.lane_safety_factor
                        self.servo_msg = camera_weight * self.camera_servo_msg + (1-camera_weight) * gps_steering
                        mission_phase = "Left-Turn-Safe-Hybrid"
                    else:
                        # Camera steering seems wrong, stick to GPS with slight camera influence
                        self.servo_msg = 0.8 * gps_steering + 0.2 * self.camera_servo_msg
                        mission_phase = "Left-Turn-GPS-Safe"
                        rospy.logwarn(f"Camera steering deviation too large: {abs(self.camera_servo_msg - gps_steering):.3f}")
                else:
                    mission_phase = "Left-Turn-GPS-Only"
                
            # Straight section 2 (waypoints 80-110): Fast cruise
            elif 80 < current_waypoint <= 110:
                self.setMotorMsgWithVel(20.0)  # Increased from 12.0
                self.setServoMsgWithLfd(15)    # Increased from 12
                lfd = 15
                mission_phase = "Straight-2"
                
            # Final section (waypoints 110-133): Moderate speed
            elif 110 < current_waypoint <= 133:
                self.setMotorMsgWithVel(15.0)  # New section
                self.setServoMsgWithLfd(12)
                lfd = 12
                mission_phase = "Final-Straight"
            
            # Final approach: Slow down
            if current_waypoint > 120:
                self.setMotorMsgWithVel(6.0)
                self.setServoMsgWithLfd(8)
                lfd = 8
                mission_phase = "Final-Approach"
            
            # End of path: Stop the vehicle
            if current_waypoint + 3 >= len(self.global_path.poses):
                self.motor_msg = 0.0
                self.servo_msg = 0.0
                self.brake_msg = 1.0
                rospy.logwarn("End of path reached - stopping vehicle")
            else:
                # Set brake
                self.brake_msg = 0.0
            
            # Publish control command
            self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
            
            # Publish global path, waypoint, and mission status
            self.global_path_pub.publish(self.global_path)
            self.waypoint_pub.publish(Int64(current_waypoint))
            
            # Enable lane detection for all sections where camera input is useful
            # Extended range to cover more of the course for better safety
            lane_detection_active = (20 < current_waypoint <= 100)  # Extended range for better coverage
            self.mission_pub.publish(Bool(lane_detection_active))
            
            # Publish TF
            self.br.sendTransform(
                (self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, 0),
                quat,
                rospy.Time.now(),
                "base_link",
                "map"
            )
            
            # Log status
            if current_waypoint % 5 == 0:  # Every 5 waypoints
                rospy.loginfo(f"[{mission_phase}] Waypoint: {current_waypoint}/{len(self.global_path.poses)}, "
                             f"Steering: {self.servo_msg:.3f}, Velocity: {self.motor_msg:.1f}, "
                             f"LFD: {lfd}, Yaw: {self.yaw:.1f}")
                
        except Exception as e:
            rospy.logerr(f"Error in main loop: {e}")
    
    def find_current_waypoint(self):
        if len(self.global_path.poses) == 0:
            return 0
            
        current_x = self.odom_msg.pose.pose.position.x
        current_y = self.odom_msg.pose.pose.position.y
        min_dist = float('inf')
        current_waypoint = 0
        
        for i, pose in enumerate(self.global_path.poses):
            dx = current_x - pose.pose.position.x
            dy = current_y - pose.pose.position.y
            dist = sqrt(dx*dx + dy*dy)
            
            if dist < min_dist:
                min_dist = dist
                current_waypoint = i
                
        return current_waypoint
    
    def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):
        self.ctrl_cmd_msg.velocity = motor_msg
        self.ctrl_cmd_msg.steering = servo_msg
        self.ctrl_cmd_msg.brake = brake_msg
        
        # Add acceleration for MORAI simulator
        if motor_msg > 0.0:
            self.ctrl_cmd_msg.accel = 1.0  # Forward acceleration
            self.ctrl_cmd_msg.acceleration = 1.0
            self.ctrl_cmd_msg.longlCmdType = 1  # Acceleration command type
        else:
            self.ctrl_cmd_msg.accel = 0.0
            self.ctrl_cmd_msg.acceleration = 0.0
            self.ctrl_cmd_msg.longlCmdType = 0
            
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
    
    def gpsCB(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        
        if self.original_latitude == 0:
            self.original_latitude = self.latitude
            self.original_longitude = self.longitude
            rospy.loginfo(f"Origin set: lat={self.latitude:.6f}, lon={self.longitude:.6f}")
    
    def ImuCB(self, msg):
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw_rad = euler_from_quaternion(quaternion)
        self.yaw = degrees(yaw_rad)
    
    def setMotorMsgWithVel(self, velocity):
        """Set motor velocity"""
        self.motor_msg = velocity
    
    def setServoMsgWithLfd(self, lfd):
        """Set steering with specific look-forward distance and safety limits"""
        try:
            steering, target_x, target_y = self.pure_pursuit.steering_angle(lfd)
            raw_steering = steering * self.steering_offset
            
            # Apply safety limits to prevent excessive steering
            max_steering = 0.3  # Reduced maximum steering angle limit for safety
            self.servo_msg = max(-max_steering, min(max_steering, raw_steering))
            
            # Additional safety check - if steering is too extreme, use 0
            if abs(self.servo_msg) > 0.25:
                rospy.logwarn(f"Large steering detected: {self.servo_msg:.3f}, limiting to safe range")
                self.servo_msg = 0.25 if self.servo_msg > 0 else -0.25
                
            # Log if steering was limited
            if abs(raw_steering) > max_steering:
                rospy.logwarn(f"Steering limited: {raw_steering:.3f} -> {self.servo_msg:.3f}")
                
        except Exception as e:
            rospy.logerr(f"Error in steering calculation: {e}")
            self.servo_msg = 0.0  # Safe default
    
    def laneCB(self, msg):
        """Camera lane detection callback"""
        self.camera_servo_msg = msg.steering
        self.camera_motor_msg = msg.velocity
        # rospy.logdebug(f"Camera steering: {self.camera_servo_msg:.3f}")

if __name__ == '__main__':
    try:
        simple_pure_pursuit = SimplePurePursuit()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simple Pure Pursuit stopped")

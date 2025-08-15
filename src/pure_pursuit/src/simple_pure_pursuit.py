#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simplified Pure Pursuit for GPS + IMU only testing
"""

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

from utils import pathReader, purePursuit

class SimplePurePursuit():
    def __init__(self):
        rospy.init_node('simple_pure_pursuit', anonymous=True)
        
        # Path name
        self.path_name = 'gps_offset_straight'
        
        # Publishers
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/waypoint', Int64, queue_size=1)
        
        # Subscribers (GPS + IMU only)
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.ImuCB)
        
        # Initialize variables
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.yaw = 0.0
        
        self.motor_msg = 0.0
        self.servo_msg = 0.0
        self.brake_msg = 0.0
        
        self.ctrl_cmd_msg = CtrlCmd()
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id = '/base_link'
        
        # Initialize coordinate system
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.original_latitude = 0
        self.original_longitude = 0
        
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
        
        # Main loop
        rate = rospy.Rate(20)  # 20 Hz
        rospy.loginfo("Simple Pure Pursuit started!")
        
        while not rospy.is_shutdown():
            self.main_loop()
            rate.sleep()
    
    def main_loop(self):
        if self.latitude == 0.0 or self.longitude == 0.0:
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
            
            # Update pure pursuit
            self.pure_pursuit.getPath(self.global_path)
            self.pure_pursuit.getEgoStatus(self.odom_msg, 5.0, False)  # 5 m/s default velocity
            
            # Find current waypoint
            current_waypoint = self.find_current_waypoint()
            
            # Compute steering
            lfd = 15  # look-forward distance
            steering, target_x, target_y = self.pure_pursuit.steering_angle(lfd)
            
            # Set control commands
            self.motor_msg = 10.0  # Default velocity
            self.servo_msg = steering * 0.015  # Steering offset
            self.brake_msg = 0.0
            
            # Publish control command
            self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
            
            # Publish global path and waypoint
            self.global_path_pub.publish(self.global_path)
            self.waypoint_pub.publish(Int64(current_waypoint))
            
            # Publish TF
            self.br.sendTransform(
                (self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, 0),
                quat,
                rospy.Time.now(),
                "base_link",
                "map"
            )
            
            # Log status
            if current_waypoint % 10 == 0:  # Every 10 waypoints
                rospy.loginfo(f"Waypoint: {current_waypoint}/{len(self.global_path.poses)}, "
                             f"Steering: {self.servo_msg:.3f}, Velocity: {self.motor_msg}")
                
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

if __name__ == '__main__':
    try:
        simple_pure_pursuit = SimplePurePursuit()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simple Pure Pursuit stopped")

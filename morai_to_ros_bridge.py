#!/usr/bin/env python3
"""
MORAI UDP to ROS Bridge
Receives MORAI lidar data via UDP and publishes as ROS PointCloud2
"""

import rospy
import socket
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from threading import Thread
import struct

class MoraiLidarBridge:
    def __init__(self):
        rospy.init_node('morai_lidar_bridge')
        
        # MORAI UDP settings
        self.host = '127.0.0.1'
        self.port = 2368  # MORAI sends to this port (different from velodyne)
        self.data_size = 1206
        self.channel = 32
        self.max_len = 150
        
        # ROS publisher
        self.pointcloud_pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=1)
        
        # Vertical angles for 32-channel lidar (HDL-32E)
        self.VerticalAngleDeg = np.array([
            [-30.67,-9.33,-29.33,-8.0,-28.0,-6.67,-26.67,-5.33,-25.33,-4,-24,-2.67,-22.67,-1.33,-21.33,
             0.0,-20.,1.33,-18.67,2.67,-17.33,4,-16,5.33,-14.67,6.67,-13.33,8,-12,9.33,-10.67,10.67]
        ])
        
        # Setup UDP socket
        self.setup_udp()
        
    def setup_udp(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(1.0)
            rospy.loginfo(f"MORAI Lidar Bridge listening on {self.host}:{self.port}")
            
            # Start receiving thread
            self.receive_thread = Thread(target=self.receive_data)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
        except Exception as e:
            rospy.logerr(f"Failed to setup UDP: {e}")
    
    def receive_data(self):
        while not rospy.is_shutdown():
            try:
                buffer_chunks = []
                
                # Receive data packets (MORAI protocol)
                for _ in range(self.max_len):  # 150 packets per scan
                    unit_block, addr = self.sock.recvfrom(self.data_size)
                    buffer_chunks.append(unit_block[:1200])
                
                # Process the complete buffer
                buffer = b''.join(buffer_chunks)
                self.process_lidar_data(buffer)
                
            except socket.timeout:
                rospy.logwarn_throttle(5, "No MORAI lidar data received")
            except Exception as e:
                rospy.logerr(f"Error receiving data: {e}")
    
    def process_lidar_data(self, buffer):
        try:
            # Parse MORAI data format
            buffer_np = np.frombuffer(buffer, dtype=np.uint8).reshape([-1, 100])
            
            # Extract azimuth, distance, and intensity
            azimuth = buffer_np[:,2] + 256*buffer_np[:,3]
            distance = (buffer_np[:,4::3].astype(np.float32) + 256*buffer_np[:,5::3].astype(np.float32))*2
            intensity = buffer_np[:,6::3].astype(np.float32)
            
            # Reshape data
            azimuth = azimuth.reshape([-1, 1]) / 100.0  # Convert to degrees
            distance = distance.reshape([-1, self.channel]) / 1000.0  # Convert to meters
            intensity = intensity.reshape([-1])
            
            # Convert to Cartesian coordinates
            x, y, z = self.spherical_to_cartesian(distance, azimuth)
            
            # Create and publish PointCloud2
            self.publish_pointcloud(x, y, z, intensity)
            
        except Exception as e:
            rospy.logerr(f"Error processing lidar data: {e}")
    
    def spherical_to_cartesian(self, r, azimuth):
        # Convert spherical coordinates to Cartesian
        x = r * np.cos(np.deg2rad(self.VerticalAngleDeg)) * np.sin(np.deg2rad(azimuth))
        y = r * np.cos(np.deg2rad(self.VerticalAngleDeg)) * np.cos(np.deg2rad(azimuth))
        z = r * np.sin(np.deg2rad(self.VerticalAngleDeg))
        
        return x.reshape([-1]), y.reshape([-1]), z.reshape([-1])
    
    def publish_pointcloud(self, x, y, z, intensity):
        # Create PointCloud2 message
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        
        # Create point cloud data
        points = []
        for i in range(len(x)):
            if not (np.isnan(x[i]) or np.isnan(y[i]) or np.isnan(z[i])):
                points.append([x[i], y[i], z[i], intensity[i] if i < len(intensity) else 0.0])
        
        # Define point fields
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)
        ]
        
        # Create PointCloud2
        cloud = pc2.create_cloud(header, fields, points)
        self.pointcloud_pub.publish(cloud)
        
        rospy.loginfo_throttle(1, f"Published pointcloud with {len(points)} points")

if __name__ == '__main__':
    try:
        bridge = MoraiLidarBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

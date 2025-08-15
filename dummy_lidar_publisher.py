#!/usr/bin/env python3
"""
Dummy Lidar Data Publisher for Testing
Publishes fake lidar data to test RViz visualization
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import math

class DummyLidarPublisher:
    def __init__(self):
        rospy.init_node('dummy_lidar_publisher')
        
        # Publisher
        self.pointcloud_pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=1)
        
        # Timer for publishing at 10 Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_dummy_data)
        
        rospy.loginfo("Dummy Lidar Publisher started - Publishing to /velodyne_points")
    
    def publish_dummy_data(self, event):
        # Create dummy point cloud data (circular pattern)
        points = []
        
        # Generate points in a circle pattern
        for angle in np.linspace(0, 2*math.pi, 360):  # 360 points in circle
            for r in range(1, 20, 2):  # Multiple rings
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = math.sin(angle * 4) * 2  # Some height variation
                intensity = 100.0
                
                points.append([x, y, z, intensity])
        
        # Add some random points for realistic look
        for _ in range(100):
            x = np.random.uniform(-10, 10)
            y = np.random.uniform(-10, 10) 
            z = np.random.uniform(-2, 2)
            intensity = np.random.uniform(0, 255)
            
            points.append([x, y, z, intensity])
        
        # Create PointCloud2 message
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        
        # Define point fields
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)
        ]
        
        # Create and publish PointCloud2
        cloud = pc2.create_cloud(header, fields, points)
        self.pointcloud_pub.publish(cloud)
        
        rospy.loginfo_throttle(2, f"Published dummy pointcloud with {len(points)} points")

if __name__ == '__main__':
    try:
        publisher = DummyLidarPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

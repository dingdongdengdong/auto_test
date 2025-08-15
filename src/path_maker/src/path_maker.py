#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS1 path recorder (raw latitude/longitude)

- Subscribes to `/gps` (`morai_msgs/GPSMessage`) to read WGS84 latitude/longitude/altitude
- Writes raw lat, lon, alt to a text file with a small angular spacing filter
- Output columns: latitude (deg), longitude (deg), altitude (m), mode (int, reserved)

Usage (ROS1):
- rosrun: `rosrun path_maker path_maker.py path FILE_BASENAME`
  (the file is saved to `<path_maker_package_path>/<folder>/<filename>.txt`)
"""

import os
import sys
import rospy
import rospkg
from morai_msgs.msg  import EgoVehicleStatus, GPSMessage
from math import pi, sqrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf

class PathMaker :
    """Create a path file by logging raw WGS84 lat/lon/alt from GPS."""
    def __init__(self):
        """Initialize ROS node, arguments, GPS subscriber, and output file."""
        rospy.init_node('path_maker', anonymous=True)
    
        self.cnt = 0

        # Command-line arguments: <folder_name> <file_basename>
        arg = rospy.myargv(argv = sys.argv)
        self.path_folder_name = arg[1]
        self.make_path_name = arg[2]
        
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0

        # Subscribe to GPS (MORAI or hardware), capturing WGS84 lat/lon/alt
        rospy.Subscriber("/gps",GPSMessage, self.gps_callback)

        self.prev_latitude = 0
        self.prev_longitude = 0

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('path_maker')
        # Output file path: <pkg>/<folder>/<filename>.txt
        full_path = pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        self.f = open(full_path, 'w')

        rate = rospy.Rate(30)  # Main loop at 30 Hz
        while not rospy.is_shutdown():
            self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):
        """Append a new lat/lon/alt point to file if moved sufficiently."""
        latitude = self.latitude
        longitude = self.longitude
        altitude = self.altitude
        
        # Spacing filter on angular distance (deg). 1e-5 deg ~ 1.1 m at mid-latitudes.
        distance = sqrt(pow(latitude - self.prev_latitude, 2) + pow(longitude - self.prev_longitude, 2))
        mode = 0
        if distance > 0.00001:
            data='{0}\t{1}\t{2}\t{3}\n'.format(latitude, longitude, altitude, mode)
            self.f.write(data)
            self.cnt += 1
            self.prev_latitude = latitude
            self.prev_longitude = longitude
            print(self.cnt, latitude, longitude)

    def gps_callback(self, msg): 
        """Capture the latest GPS fix (WGS84 lat/lon/alt) from `/gps`."""
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        
if __name__ == '__main__':
    try:
        path_maker_=PathMaker()
    except rospy.ROSInterruptException:
        pass


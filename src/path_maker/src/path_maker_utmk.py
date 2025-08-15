#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS1 path recorder (UTM/UTMK coordinates)

- Subscribes to `/gps` (`morai_msgs/GPSMessage`) to read WGS84 longitude/latitude/altitude
- Converts to UTM Zone 52 (WGS84) coordinates using pyproj and writes to a text file
- Output columns: x (meters), y (meters), z (meters, fixed 0), mode (int, reserved)

Usage (ROS1):
- roslaunch: use `path_maker/launch/path_maker.launch` and adjust the args
- rosrun: `rosrun path_maker path_maker_utmk.py path FILE_BASENAME`

The file is saved to `<path_maker_package_path>/<folder>/<filename>.txt`.
"""

import warnings
import os
import sys
import rospy
import rospkg
from pyproj import Proj, transform
from sensor_msgs.msg import NavSatFix
from morai_msgs.msg  import EgoVehicleStatus, GPSMessage
from math import pi, sqrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import tf


warnings.simplefilter(action='ignore', category=FutureWarning)

class PathMaker :
    """Create a path file by logging UTM coordinates derived from GPS."""
    def __init__(self):
        """Initialize ROS node, arguments, subscribers, and output file."""
        rospy.init_node('path_maker', anonymous=True)
    
        self.cnt = 0

        # Command-line arguments: <folder_name> <file_basename>
        arg = rospy.myargv(argv = sys.argv)
        self.path_folder_name = arg[1]
        self.make_path_name = arg[2]
        
        self.longitude = 0
        self.latitude = 0
        self.altitude = 0

        # Subscribe to GPS (from MORAI simulator or platform)
        rospy.Subscriber("/gps",GPSMessage, self.gps_callback)

        self.prev_longitude = 0
        self.prev_latitude = 0

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('path_maker')
        # Output file path: <pkg>/path/<folder>/<filename>.txt
        full_path = pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        self.f = open(full_path, 'w')

        rate = rospy.Rate(30)  # Main loop at 30 Hz
        while not rospy.is_shutdown():
            self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):
        """Convert incoming lon/lat to UTM (zone 52) and append to file with spacing filter."""
        utmk_coordinate = Point() 

        # UTM Zone 52 (Korea region). WGS84 ellipsoid.
        # NOTE: pyproj typically uses parameter name 'ellps' (not 'elips').
        self.proj_UTM = Proj(proj='utm', zone = 52, elips='WGS84', preserve_units=False)

        xy_zone = self.proj_UTM(self.longitude, self.latitude)
        self.x, self.y = xy_zone[0], xy_zone[1]

        utmk_coordinate.x = self.x
        utmk_coordinate.y = self.y
        utmk_coordinate.z = 0

        # Spacing filter: write only if moved by > 0.3 m since last recorded point
        distance = sqrt(pow(utmk_coordinate.x - self.prev_longitude, 2) + pow(utmk_coordinate.y - self.prev_latitude, 2))

        mode = 0
        if distance > 0.3:   # meters
            data='{0}\t{1}\t{2}\t{3}\n'.format(utmk_coordinate.x, utmk_coordinate.y, utmk_coordinate.z, mode)
            self.f.write(data)
            self.cnt += 1
            self.prev_longitude = utmk_coordinate.x
            self.prev_latitude = utmk_coordinate.y
        
            print(self.cnt, utmk_coordinate.x, utmk_coordinate.y)

    def gps_callback(self, msg): 
        """Capture the latest GPS fix (WGS84 lon/lat/alt) from `/gps`."""
        self.longitude = msg.longitude
        self.latitude = msg.latitude
        self.altitude = msg.altitude

        
if __name__ == '__main__':
    try:
        path_maker_=PathMaker()
    except rospy.ROSInterruptException:
        pass


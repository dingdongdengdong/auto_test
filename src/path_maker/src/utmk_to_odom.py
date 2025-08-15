"""
Helper script: broadcast a static TF and rewrite a path file.

This is not part of the runtime node set. It demonstrates how to:
1) Read an existing path text file (x, y, z, mode)
2) Broadcast a TF transform while iterating (for quick visualization)
3) Write the same content back to a new file

Note: The script uses hard-coded absolute paths. Adjust to your environment
before running, or refactor to use ROS package paths.
"""

import warnings
import sys,os
import rospy
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Bool, String, Int64MultiArray, Float64MultiArray, Int64, Float32MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, transform
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, GPSMessage, CtrlCmd, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv


from std_msgs.msg import Int64

import tf
import time
from math import *
rospy.init_node('pure_pursuit', anonymous=True)
br = tf.TransformBroadcaster()

# TODO: Replace with your own path. Example below assumes a workspace under /home/park
with open("/home/park/ISEV_2023/src/path_maker/path/final_path_first.txt", "r") as f_in:
    lines = f_in.readlines()

new_lines = []
for line in lines:
    x, y, z, m = line.strip().split()

    br.sendTransform((x, y, 0.),
        tf.transformations.quaternion_from_euler(0,0,0.),
        rospy.Time.now(),
        "base_link",
        "map")
    
    new_lines.append(f"{x}\t{y}\t{z}\t{m}\n")

# TODO: Replace with your preferred output file
with open("/home/park/ISEV_2023/src/path_maker/path/new_file.txt", "w") as f_out:
    f_out.writelines(new_lines)
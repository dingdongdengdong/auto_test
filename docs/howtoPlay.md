
### 0) Source your environment
```bash
source /opt/ros/noetic/setup.bash
source /home/jhj/catkin_ws/devel/setup.bash
```

### 1) If your sim doesn’t provide the Morai service
pure_pursuit waits for `/Service_MoraiEventCmd`. Run this tiny mock once in another terminal:
```bash
python3 - <<'PY'
import rospy
from morai_msgs.srv import MoraiEventCmdSrv, MoraiEventCmdSrvResponse
def cb(req): return MoraiEventCmdSrvResponse(result=True)
rospy.init_node('mock_morai_event_service')
s = rospy.Service('/Service_MoraiEventCmd', MoraiEventCmdSrv, cb)
rospy.spin()
PY
```

### 2) Start nodes with your sim topics (recommended: run individually with remaps)
- Replace placeholders with your topic names.
- Camera nodes expect `/image_jpeg/compressed`; LiDAR node expects `/velodyne_points`.

Perception:
```bash
rosrun traffic_sign traffic_sign.py /image_jpeg/compressed:=/your/camera/compressed
rosrun lane_detection line_detector.py /image_jpeg/compressed:=/your/camera/compressed /imu:=/your/imu /gps:=/your/gps
rosrun obstacle_detection dy_obstacle_detection /velodyne_points:=/your/lidar_points
```

Controller:
```bash
rosrun pure_pursuit faster_pure_pursuit.py /gps:=/your/gps /imu:=/your/imu
```

### 3) One-shot launch (if you prefer using the bundled launch)
This includes all perception nodes; remaps apply globally:
```bash
roslaunch pure_pursuit faster_pure_pursuit.launch image_jpeg/compressed:=/your/camera/compressed gps:=/your/gps imu:=/your/imu velodyne_points:=/your/lidar_points
```
Note: This launch includes `velodyne_pointcloud/32e_points.launch`. If your sim already publishes point clouds, the individual run (step 2) avoids driver conflicts.

### 4) Quick checks
- Verify topics:
```bash
rostopic list | grep -E 'gps|imu|image_jpeg|velodyne_points|ctrl_cmd|dy_obs_info|traffic_light'
```
- Sanity echoes:
```bash
rostopic echo -n1 /ctrl_cmd | head -n 20 | cat
rostopic echo -n1 /dy_obs_info | cat
```

- If lane/traffic nodes need GUI, ensure X/Wayland is set (WSL2 needs an X server).

- pure_pursuit loads paths from `path_maker/path/*.txt`. Ensure those files exist for your course.

- Tune obstacle params live (optional):
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

- If you want “normal” instead of “faster” behavior:
```bash
roslaunch pure_pursuit pure_pursuit.launch image_jpeg/compressed:=/your/camera/compressed gps:=/your/gps imu:=/your/imu velodyne_points:=/your/lidar_points
```

- To use only LiDAR obstacle detection:
```bash
roslaunch obstacle_detection dy_obstacle_detection.launch velodyne_points:=/your/lidar_points
```

- To stop safely:
```bash
rosnode kill -a
```

- If `setup.bash` not found, rebuild:
```bash
cd /home/jhj/catkin_ws && catkin_make
```

- If the controller hangs at startup, make sure the mock service is running and `/gps`, `/imu`, and camera topics are publishing.

- Built artifacts summary
  - `obstacle_detection/dy_obstacle_detection` executable is available.
  - Python nodes in `pure_pursuit`, `lane_detection`, `traffic_sign` run via `rosrun`.

- Key remaps you likely need
  - **camera**: `/image_jpeg/compressed:=/your/camera/compressed`
  - **gps**: `/gps:=/your/gps`
  - **imu**: `/imu:=/your/imu`
  - **lidar**: `/velodyne_points:=/your/lidar_points`

- Verified build and fixed earlier issues
  - Replaced ROS2 velodyne with ROS1 packages.
  - Fixed `obstacle_detection` include dir and catkin config.

Want me to generate a custom launch file tailored to your exact topics so you can start everything with a single command?
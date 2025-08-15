# Faster Pure Pursuit (ROS1)

This document describes how `pure_pursuit/launch/faster_pure_pursuit.launch` runs the fast-route controller and how to interface with its topics and missions.

## Overview
- Node: `pure_pursuit/src/faster_pure_pursuit.py`
- Launch: `pure_pursuit/launch/faster_pure_pursuit.launch`
- Path source: `path_maker/path/*.txt` (e.g., `first_faster.txt`, `second_faster.txt`, `parking_*.txt`)
- Coordinate system: WGS84 lat/lon converted to UTM zone 52 for planning

## Nodes and Includes
- `pure_pursuit` (from `faster_pure_pursuit.py`)
  - Publishes `CtrlCmd` to `/ctrl_cmd`
  - Publishes `Path` to `/global_path`
  - Publishes mission and waypoint markers
- Included perception launch files
  - `traffic_sign/launch/traffic_sign.launch`: Publishes `/traffic_light`
  - `lane_detection/launch/lane_detection_faster.launch`: Publishes `/cam_steer`
  - `lane_detection/launch/curve_detection_faster.launch`: Publishes `/curve_cmd`
  - `obstacle_detection/launch/dy_obstacle_detection.launch`: Publishes `/dy_obs_info`

Optional nodes (commented in the launch):
- `get_yaw_from_imu.py`: Publishes `/yaw` from IMU (not required here)
- `scoring.py`: Logs collisions and mission state to a file

## Topics
- Subscriptions
  - `/gps` (morai_msgs/GPSMessage): latitude/longitude (UTM-converted internally)
  - `/imu` (sensor_msgs/Imu): orientation → yaw
  - `/traffic_light` (std_msgs/Int64MultiArray): [red_count, green_count]
  - `/cam_steer` (morai_msgs/CtrlCmd): camera-based steering assist
  - `/curve_cmd` (morai_msgs/CtrlCmd): overrides for right-angle curve mission
  - `/dy_obs_info` (std_msgs/Float64MultiArray): dynamic obstacle [distance, lateral, ...]

- Publications
  - `/ctrl_cmd` (morai_msgs/CtrlCmd): velocity, steering, brake
  - `/global_path` (nav_msgs/Path): current global route for visualization
  - `/mission` (std_msgs/Bool): mission-stage flag used by other nodes
  - `/waypoint` (std_msgs/Int64): nearest waypoint index

## Services
- Client: `/Service_MoraiEventCmd` (MoraiEventCmdSrv)
  - Used to switch gear (D/R/P) and control turn/emergency signals

## Mission Flow (Fast Variant)
1. Load `first_faster.txt` and begin line driving
2. Start mission: initial steering attenuation and left signal
3. Uphill stop mission: brief stop at specified waypoint window
4. Right-angle (curve) mission: speed reduction and steering overrides
5. S-curve mission: camera steering in GPS shadow region; speed modulation by yaw
6. Parking (T mission): switch paths among `parking_1..4.txt`, including reverse mode
7. Transition to `second_faster.txt` and perform acceleration mission
8. Traffic light missions: conditional braking by red/green counts at windows
9. Final section: right signal, stop, and shift to Park

Refer to in-code docstrings in `faster_pure_pursuit.py` for exact waypoint windows and parameters.

## Running
```bash
roslaunch pure_pursuit faster_pure_pursuit.launch
```

Ensure the included perception packages are built and available, and that `path_maker` contains the required `*_faster.txt` and `parking_*.txt` files.

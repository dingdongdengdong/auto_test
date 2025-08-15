#!/bin/bash

echo "=== ROS Topic Test Script ==="
echo "Testing Pure Pursuit topics and dependencies"
echo ""

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /home/jhj/catkin_ws/devel/setup.bash

echo "1. Checking if roscore is running..."
if pgrep -x "rosmaster" > /dev/null; then
    echo "✅ roscore is running"
else
    echo "❌ roscore is NOT running. Start with: roscore"
    exit 1
fi

echo ""
echo "2. Listing all active topics..."
rostopic list | sort

echo ""
echo "3. Checking GPS topic..."
echo "GPS topic status:"
timeout 3s rostopic echo /gps -n 1 || echo "❌ No GPS data received"

echo ""
echo "4. Checking IMU topic..."
echo "IMU topic status:"
timeout 3s rostopic echo /imu -n 1 || echo "❌ No IMU data received"

echo ""
echo "5. Checking camera topic..."
echo "Camera topic status:"
timeout 3s rostopic echo /image_jpeg/compressed -n 1 || echo "❌ No camera data received"

echo ""
echo "6. Checking control command output..."
echo "Control command topic status:"
timeout 3s rostopic echo /ctrl_cmd -n 1 || echo "❌ No control commands being published"

echo ""
echo "7. Checking topic publication rates..."
echo "Control command rate:"
timeout 5s rostopic hz /ctrl_cmd || echo "❌ No /ctrl_cmd messages"

echo ""
echo "GPS rate:"
timeout 5s rostopic hz /gps || echo "❌ No /gps messages"

echo ""
echo "IMU rate:"
timeout 5s rostopic hz /imu || echo "❌ No /imu messages"

echo ""
echo "8. Checking running nodes..."
echo "Active ROS nodes:"
rosnode list | grep -E "(pure_pursuit|line_detector|gps|imu)" || echo "❌ No relevant nodes found"

echo ""
echo "9. Testing manual control command..."
echo "Publishing test control command..."
rostopic pub /ctrl_cmd morai_msgs/CtrlCmd "longlCmdType: 2
accel: 0.0
brake: 1.0
steering: 0.0
velocity: 0.0
acceleration: 0.0" -1

echo ""
echo "=== Test Complete ==="
echo "If you see ❌ marks, those systems need to be started"
echo ""
echo "To start missing components:"
echo "- GPS/IMU simulator: (check if MORAI is running)"
echo "- Pure pursuit: cd /home/jhj/catkin_ws/src/pure_pursuit/src && python3 simple_pure_pursuit.py"
echo "- Lane detection: roslaunch lane_detection lane_detection_faster.launch"


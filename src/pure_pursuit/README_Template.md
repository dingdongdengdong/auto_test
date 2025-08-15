# Autonomous Driving Test Template

This template implements a complete autonomous driving test course based on the "Green Path" scenario. It provides a structured, waypoint-based approach to handling complex driving maneuvers and missions.

## Overview

The template covers a comprehensive driving test course with the following sections:

1. **Start** → Initial positioning and system initialization
2. **Initial Straight** → Basic straight-line driving
3. **First Left Turn (90°)** → Standard left turn with turn signals
4. **Ramp Section** → Uphill driving with mandatory stop and downhill descent
5. **Second Left Turn (90°)** → Another standard left turn
6. **T-Course** → Consecutive 90° turns in narrow space
7. **Traffic Light Intersection** → Traffic light detection and response
8. **S-Curve** → Curved course navigation with GPS shadow handling
9. **First Right Turn (90°)** → Standard right turn
10. **Parallel Parking** → Complex parking maneuver
11. **Second Right Turn (90°)** → Another standard right turn
12. **Final Straight** → Final approach section
13. **Final Left Turn (90°)** → Last turn before finish
14. **Finish** → Course completion and vehicle stop

## File Structure

```
src/pure_pursuit/src/
├── driving_test_template.py    # Main template file
├── course_config.py           # Configuration parameters
├── utils.py                   # Utility functions (existing)
└── README_Template.md         # This documentation
```

## Quick Start

### 1. Configure Your Course

Edit `course_config.py` to match your specific path file:

```python
# Update waypoint ranges to match your path
COURSE_SECTIONS = {
    "start": (0, 50),              # Customize these ranges!
    "initial_straight": (51, 100),
    "first_left_turn": (101, 150),
    # ... add all sections
}
```

### 2. Create Your Path File

Ensure you have a path file named `green_path_test.txt` in your `path_maker` directory, or update the path name in the configuration:

```python
# In course_config.py
DEFAULT_PATH_NAME = 'your_path_name'  # Without .txt extension
```

### 3. Run the Template

```bash
# Source your ROS environment
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Run the driving test template
rosrun pure_pursuit driving_test_template.py
```

## Customization Guide

### Finding Waypoint Numbers

To customize the waypoint ranges for your specific path:

1. **Visualize your path** using rviz:
   ```bash
   roslaunch your_package path_visualizer.launch
   ```

2. **Identify section boundaries** by examining waypoint numbers at:
   - Turn entry and exit points
   - Mission-specific areas (ramp, T-course, etc.)
   - Traffic light positions
   - Parking areas

3. **Update `course_config.py`** with the correct waypoint ranges

### Speed and Steering Tuning

Adjust parameters in `course_config.py`:

```python
class SpeedConfig:
    DEFAULT_SPEED = 19.0    # Increase/decrease for overall speed
    SLOW_SPEED = 10.0      # Speed for complex maneuvers
    PARKING_SPEED = 5.0    # Very slow speed for precision

class SteeringConfig:
    DEFAULT_LFD = 18       # Look-ahead distance affects smoothness
    TURN_LFD = 8          # Smaller values = tighter turns
    STEERING_OFFSET = 0.015 # Base steering sensitivity
```

### Adding Custom Missions

To add a new mission section:

1. **Add section to configuration**:
   ```python
   # In course_config.py
   COURSE_SECTIONS = {
       # ... existing sections
       "your_custom_section": (waypoint_start, waypoint_end),
   }
   ```

2. **Add mission execution method**:
   ```python
   # In driving_test_template.py
   def execute_your_custom_section_mission(self, waypoint):
       """Execute your custom mission"""
       self.current_mission = "YOUR_MISSION"
       # Your custom logic here
       self.motor_msg = CUSTOM_SPEED
       self.setServoMsgWithLfd(CUSTOM_LFD)
       print(f"[YOUR_CUSTOM_SECTION] Waypoint: {waypoint}")
   ```

3. **Add to main execution logic**:
   ```python
   # In execute_mission_by_waypoint method
   elif current_section == "your_custom_section":
       self.execute_your_custom_section_mission(current_waypoint)
   ```

## Mission Details

### Ramp Mission
- **Purpose**: Test uphill driving capability and precise stopping
- **Behavior**: Increases power uphill, stops at 70% through section for 3 seconds
- **Configuration**: Adjust `RAMP_STOP_PERCENTAGE` and `RAMP_STOP_DURATION`

### T-Course Mission
- **Purpose**: Test tight maneuvering in narrow spaces
- **Behavior**: Uses very slow speed and high steering sensitivity
- **Configuration**: Adjust `PARKING_SPEED` and `TIGHT_TURN_LFD`

### Traffic Light Mission
- **Purpose**: Test traffic light detection and response
- **Behavior**: Stops on red light, proceeds on green
- **Configuration**: Adjust `TRAFFIC_LIGHT_THRESHOLD` for sensitivity

### S-Curve Mission
- **Purpose**: Test curved path navigation and GPS shadow handling
- **Behavior**: Uses camera steering when GPS is unavailable
- **Configuration**: Adjust GPS and yaw thresholds

### Parallel Parking Mission
- **Purpose**: Test complex parking maneuver
- **Behavior**: Slow speed with high steering sensitivity
- **Note**: This is a simplified implementation - real parking needs multiple phases

## ROS Topics

### Publishers
- `/global_path` - Global path visualization
- `/ctrl_cmd` - Vehicle control commands
- `/mission` - Current mission status
- `/waypoint` - Current waypoint number

### Subscribers
- `/gps` - GPS position data
- `/imu` - Vehicle orientation data
- `/traffic_light` - Traffic light detection
- `/cam_steer` - Camera-based steering commands
- `/dy_obs_info` - Dynamic obstacle information

### Services
- `/Service_MoraiEventCmd` - Vehicle mode control (gear, signals, etc.)

## Debugging and Monitoring

### Enable Logging
```python
# In course_config.py
class DebugConfig:
    ENABLE_SECTION_LOGGING = True   # Log section transitions
    ENABLE_MISSION_LOGGING = True   # Log mission status
```

### Monitor Topics
```bash
# Monitor current mission
rostopic echo /mission

# Monitor waypoint progress
rostopic echo /waypoint

# Monitor control commands
rostopic echo /ctrl_cmd
```

### Visualize in RViz
```bash
# Launch RViz with path visualization
rviz
# Add displays for /global_path and vehicle position
```

## Common Issues and Solutions

### 1. Vehicle Not Following Path Correctly
- **Check**: Waypoint ranges in `course_config.py`
- **Solution**: Verify section boundaries match your path file

### 2. Turns Too Sharp or Too Wide
- **Check**: `TURN_LFD` and `STEERING_OFFSET` values
- **Solution**: Decrease LFD for tighter turns, increase for wider turns

### 3. Speed Too Fast/Slow for Missions
- **Check**: Speed configuration in `SpeedConfig`
- **Solution**: Adjust speeds for specific sections

### 4. Traffic Light Not Responding
- **Check**: Traffic light topic data
- **Solution**: Verify traffic light detection and adjust threshold

### 5. GPS Shadow Area Issues
- **Check**: Camera steering topic `/cam_steer`
- **Solution**: Ensure camera-based steering is properly configured

## Advanced Features

### Dynamic Obstacle Avoidance
The template includes basic dynamic obstacle detection:
```python
# Obstacle detected - emergency stop
if self.dynamic_obstacle_detected:
    self.brake()
    self.emergency_mode()
```

### Multiple Path Support
Switch between different paths:
```python
# In course_config.py
ALTERNATIVE_PATHS = {
    'test_course': 'green_path_test',
    'practice_course': 'practice_path',
    'competition_course': 'competition_path'
}
```

### Mission State Tracking
Track completion of key missions:
```python
self.mission_completed = {
    MissionType.RAMP_UP: False,
    MissionType.T_COURSE: False,
    MissionType.TRAFFIC_LIGHT: False,
    # ...
}
```

## Testing and Validation

### 1. Configuration Validation
```bash
cd src/pure_pursuit/src/
python3 course_config.py
```

### 2. Dry Run Testing
Test individual sections by modifying waypoint ranges temporarily.

### 3. Gradual Integration
Start with simple sections (straight, turns) before testing complex missions.

## Contributing

When modifying this template:

1. Keep the waypoint-based structure
2. Add proper logging for new missions
3. Update configuration files
4. Test thoroughly before deploying
5. Document any new parameters or behaviors

## Support

For issues or questions:
1. Check the configuration matches your path file
2. Verify all ROS topics are publishing correctly
3. Test in simulation before real vehicle deployment
4. Monitor logs for debugging information

---

**Note**: This template is designed for ROS1 Noetic. Ensure all dependencies and message types are compatible with your ROS environment.

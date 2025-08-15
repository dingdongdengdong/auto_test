# Pure Pursuit Controller for Autonomous Vehicle Simulation

This project implements a Pure Pursuit controller for autonomous vehicle navigation using road data from `link_set.json`.

## What is Pure Pursuit?

Pure Pursuit is a path tracking algorithm that:
- Uses a "lookahead point" ahead of the vehicle on the desired path
- Calculates steering angle based on the geometry between vehicle and lookahead point
- Provides smooth path following for autonomous vehicles

## Files

- `pure_pursuit_controller.py` - Main controller implementation
- `test_pure_pursuit.py` - Test and visualization script
- `link_set.json` - Road network data (your existing file)
- `requirements.txt` - Python dependencies

## Installation

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

## Usage

### Basic Usage

```python
from pure_pursuit_controller import PurePursuitController

# Initialize controller
controller = PurePursuitController("link_set.json", lookahead_distance=5.0)

# Vehicle state (you need to provide this from your simulation)
vehicle_pos = [158.0, 189.0, 146.0]  # [x, y, z]
vehicle_heading = 0.0  # radians

# Get control commands
steering_angle, target_speed = controller.update_path(vehicle_pos, vehicle_heading)
```

### Running Tests

```bash
python test_pure_pursuit.py
```

This will:
- Test basic controller functionality
- Test pure pursuit steering calculations
- Test speed control
- Visualize the road network

## Key Features

1. **Road Data Integration**: Automatically loads and uses your `link_set.json` road data
2. **Pure Pursuit Algorithm**: Implements the core pure pursuit steering logic
3. **Speed Control**: Adjusts target speed based on road properties
4. **Path Following**: Automatically transitions between road links
5. **Visualization**: Plots road network for debugging

## Controller Parameters

- `lookahead_distance`: Distance ahead to look for target waypoint (default: 5.0m)
- `wheelbase`: Vehicle wheelbase for steering calculations (default: 2.7m)
- `max_steering_angle`: Maximum steering angle (default: 30°)
- `max_speed`: Maximum vehicle speed (default: 20.0 m/s)

## How It Works

1. **Load Road Data**: Reads `link_set.json` to get road network
2. **Find Current Position**: Locates vehicle on current road link
3. **Look Ahead**: Finds waypoint at `lookahead_distance` ahead
4. **Calculate Steering**: Uses pure pursuit geometry to determine steering angle
5. **Update Path**: Moves to next road link when current one is completed

## Integration with Your Simulation

To use this in your automobile simulation:

1. **Vehicle State**: Provide current vehicle position and heading
2. **Control Loop**: Call `update_path()` in your simulation loop
3. **Actuators**: Apply steering angle and target speed to vehicle model
4. **Sensors**: Update vehicle position based on simulation physics

## Example Control Loop

```python
# Simulation loop
while simulation_running:
    # Get current vehicle state from your simulation
    vehicle_pos = get_vehicle_position()
    vehicle_heading = get_vehicle_heading()
    
    # Get control commands
    steering_angle, target_speed = controller.update_path(vehicle_pos, vehicle_heading)
    
    # Apply controls to your vehicle model
    set_steering_angle(steering_angle)
    set_target_speed(target_speed)
    
    # Step simulation
    step_simulation()
```

## Road Data Structure

The controller expects `link_set.json` with this structure:
```json
[
  {
    "idx": "road_link_id",
    "from_node_idx": "start_node",
    "to_node_idx": "end_node",
    "points": [[x, y, z], [x, y, z], ...],
    "max_speed": "20",
    "link_length": 10.41
  }
]
```

## Troubleshooting

- **No waypoints found**: Check that `link_set.json` has valid `points` arrays
- **Import errors**: Ensure all dependencies are installed
- **Visualization issues**: Make sure matplotlib is properly installed

## Next Steps

1. Run the test script to verify everything works
2. Integrate with your vehicle simulation
3. Adjust controller parameters for your specific vehicle
4. Add additional features like lane changing or traffic rules

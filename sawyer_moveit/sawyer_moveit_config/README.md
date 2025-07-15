# Sawyer MoveIt Configuration for Bartender Robot

This repository contains the MoveIt configuration and scripts for controlling a Sawyer robot arm with a LEAP hand for bartending applications.

## Overview

The project includes:
- MoveIt configuration files for Sawyer robot arm
- Scripts for bottle grasping and manipulation
- LEAP hand control and sensor integration
- Temperature-based object detection

## Repository Structure

```
sawyer_moveit_config/
├── config/                 # MoveIt configuration files
├── launch/                 # ROS launch files
├── scripts/               # Python scripts for robot control
├── srdf/                  # Robot description files
├── urdf/                  # URDF files
└── leap_hand/            # LEAP hand model files
```

## Key Scripts

### `scripts/bartender_move.py`
Main script for bottle grasping with the Sawyer robot arm.

**Usage:**
```bash
python3 scripts/bartender_move.py --bottle_x 0.4 --bottle_y -0.3 --bottle_z -0.05 --hand_action close
```

**Arguments:**
- `--bottle_x`: X position of bottle (meters)
- `--bottle_y`: Y position of bottle (meters) 
- `--bottle_z`: Z position of bottle (meters)
- `--hand_action`: 'open' or 'close' the hand
- `--fast`: Enable fast mode (skip visualization)
- `--collision`: Enable/disable bottle collision object ('True'/'False')

**Exit Codes:**
- `0`: Grasp successful, object not cold
- `2`: Grasp successful, object is cold
- `1`: Grasp failed

### `/home/atrc234/ros_ws/src/ros_module/move_sim_hand.py`
Controls the LEAP hand and detects object temperature.

**Usage:**
```bash
python3 move_sim_hand.py waypoint_close --waypoint_delay 0.05
```

**Features:**
- Hand opening/closing with waypoints
- UDP sensor data processing
- Temperature-based cold object detection
- Pressure sensor integration

## Setup Instructions

### Prerequisites
- ROS Noetic
- MoveIt
- Sawyer robot packages
- LEAP hand hardware/software

### Installation
1. Clone this repository to your ROS workspace
2. Build the workspace:
   ```bash
   cd ~/ros_ws
   catkin_make
   source devel/setup.bash
   ```

### Launch Files
- `sawyer_moveit.launch`: Main Sawyer MoveIt configuration
- `sawyer_moveit_with_table.launch`: Sawyer with table environment
- `sawyer_moveit_with_pillar.launch`: Sawyer with pillar environment

## Features

### Motion Planning
- Orientation constraints for flexible grasping
- Multiple planner support (RRTConnect, PRMstar, etc.)
- Collision detection with bottles and environment
- Roll-iteration fallback for difficult poses

### Sensor Integration
- UDP-based pressure sensor reading
- Temperature monitoring for cold object detection
- Real-time grasp success detection

### Hand Control
- LEAP hand joint position control
- Thumb-priority waypoint generation
- Hardware limit enforcement

## Configuration

### MoveIt Parameters
- Planning time: 10-50 seconds (configurable)
- Planner selection: RRTConnect, PRMstar, etc.
- Orientation tolerances: Roll (any), Pitch/Yaw (±0.01 rad)

### Sensor Parameters
- UDP port: 12345
- Temperature rate threshold: -1.2 to -0.8 °C/sec
- Pressure threshold: >0 for grasp detection

## Troubleshooting

### Common Issues
1. **Planning fails**: Try different roll values or increase planning time
2. **No temperature reading**: Check UDP message format and sensor connection
3. **Grasp detection fails**: Verify pressure sensor data and thresholds

### Debug Commands
```bash
# Check robot state
rostopic echo /robot/limb/right/endpoint_state

# Monitor grasp status
rostopic echo /grasp_status

# Test hand control
python3 move_sim_hand.py close
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

[Add your license information here]

## Contact

[Add contact information here]

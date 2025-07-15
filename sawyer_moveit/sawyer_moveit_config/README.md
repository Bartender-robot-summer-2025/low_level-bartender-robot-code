# Sawyer Bottle Grasping with MoveIt!

This repository contains ROS/MoveIt! scripts for controlling a Sawyer robot to grasp bottles using computer vision.

## Features

- **Automatic Roll Retry**: Tries 18 different wrist roll values for planning success
- **Hand Geometry Logic**: Accounts for 4-finger right hand that grasps on left side
- **Camera-Robot Calibration**: Data collection script for coordinate mapping
- **Collision Avoidance**: Table, ceiling, and bottle collision objects
- **Visualization**: Robot base marker and RViz integration

## Files

- `scripts/moveit_sawyer_pose_goal.py`: Main grasping script
- `scripts/calibration_data_collection.py`: Camera calibration data collection
- `config/`: MoveIt! configuration files
- `launch/`: ROS launch files

## Usage

1. Launch MoveIt!:
   ```bash
   roslaunch sawyer_moveit_config sawyer_moveit.launch
   ```

2. Run grasping script:
   ```bash
   python3 scripts/moveit_sawyer_pose_goal.py
   ```

3. For calibration:
   ```bash
   python3 scripts/calibration_data_collection.py
   ```

## Requirements

- ROS Noetic
- MoveIt!
- Sawyer robot or simulation
- Python 3

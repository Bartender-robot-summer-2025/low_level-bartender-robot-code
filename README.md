# ROS Workspace - Bartender Robot Project

## Quick Start: Running the Sawyer Arm with MoveIt and LEAP Hand

Follow these steps to bring up the Sawyer arm, MoveIt, LEAP hand, and bartender robot scripts:

1. **(Optional) Reference Tutorial:**
   - [Intera MoveIt Tutorial (archived)](https://web.archive.org/web/20230128035522/https://sdk.rethinkrobotics.com/intera/MoveIt_Tutorial)

2. **Enable the Sawyer Robot:**
   ```bash
   rosrun intera_interface enable_robot.py -e
   ```

3. **Start the Joint Trajectory Action Server:**
   ```bash
   rosrun intera_interface joint_trajectory_action_server.py
   ```

4. **Launch MoveIt for Sawyer:**
   ```bash
   roslaunch sawyer_moveit_config sawyer_moveit.launch
   ```

5. **Start the LEAP Hand Node:**
   ```bash
   roslaunch example.launch
   ```

6. **Run the Bartender Robot Script:**
   ```bash
   python3 scripts/bartender_move.py --bottle_x 0.4 --bottle_y -0.3 --bottle_z -0.05 --hand_action close
   ```

---

This ROS workspace contains packages for the bartender robot project, including Sawyer robot control, LEAP hand integration, and sensor processing.

## Workspace Structure

```
ros_ws/src/
├── sawyer_moveit_config/     # Sawyer MoveIt configuration and control scripts
├── ros_module/              # LEAP hand control and sensor integration
└── [other packages...]      # Additional ROS packages
```

## Packages

### sawyer_moveit_config
- **Purpose**: MoveIt configuration and control for Sawyer robot arm
- **Key Features**: 
  - Bottle grasping with orientation constraints
  - Motion planning with multiple planners
  - Collision detection and avoidance
  - Temperature-based object detection
- **Main Scripts**: `bartender_move.py`, `moveit_sawyer_pose_goal.py`

### ros_module
- **Purpose**: LEAP hand control and sensor data processing
- **Key Features**:
  - Hand joint position control
  - UDP sensor data processing
  - Temperature monitoring for cold object detection
  - Pressure sensor integration
- **Main Scripts**: `move_sim_hand.py`

## Building the Workspace

```bash
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

## Launching the System

### Sawyer Robot
```bash
roslaunch sawyer_moveit_config sawyer_moveit.launch
```

### LEAP Hand Control
```bash
python3 ros_module/move_sim_hand.py waypoint_close
```

## Development

### Adding New Packages
1. Create package in `src/` directory
2. Add dependencies to `package.xml`
3. Build workspace: `catkin_make`
4. Source workspace: `source devel/setup.bash`

### Testing
- Test individual packages before integration
- Use simulation environment for development
- Verify sensor connections before deployment

## Dependencies

- ROS Noetic
- MoveIt
- Sawyer robot packages
- LEAP hand hardware/software
- Python 3
- Required Python packages (see individual package READMEs)

## Troubleshooting

### Common Issues
1. **Build errors**: Check package dependencies and ROS installation
2. **Launch failures**: Verify package paths and file permissions
3. **Sensor issues**: Check hardware connections and UDP ports

### Debug Commands
```bash
# Check ROS environment
echo $ROS_PACKAGE_PATH

# List installed packages
rospack list

# Check package dependencies
rospack depends-on <package_name>
```

## Contributing

1. Follow ROS package conventions
2. Add proper documentation
3. Test thoroughly before committing
4. Update this README when adding new packages

## License

[Add workspace license information]

## Contact

[Add contact information] 
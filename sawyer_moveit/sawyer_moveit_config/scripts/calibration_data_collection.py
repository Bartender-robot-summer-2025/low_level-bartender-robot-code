#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf_conversions
import json
import os
from datetime import datetime

def main():
    # Initialize moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('calibration_data_collection', anonymous=True)
    rospy.sleep(2)

    # Instantiate objects for robot and group
    robot = moveit_commander.RobotCommander()
    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    print("=== Camera-Robot Calibration Data Collection ===")
    print("This script will move the robot to predefined positions.")
    print("At each position, place a bottle and enter the YOLO detection coordinates.")
    print("Commands: 's' to skip current position, 'q' to quit")
    print("Press Enter to continue...")
    input()

    # Define calibration points (robot base frame coordinates)
    # Format: (x, y, z, description)
    calibration_points = [
        (-0.1, 0.7, -0.05, "Current bottle position"),
        (-0.2, 0.6, -0.05, "Left side, closer"),
        (-0.0, 0.8, -0.05, "Right side, further"),
        (-0.15, 0.65, -0.05, "Middle left"),
        (-0.05, 0.75, -0.05, "Middle right"),
        (-0.25, 0.55, -0.05, "Far left"),
        (0.05, 0.85, -0.05, "Far right"),
        (-0.1, 0.5, -0.05, "Closer to robot"),
        (-0.1, 0.9, -0.05, "Further from robot"),
        (-0.2, 0.8, -0.05, "Far left, far back"),
        (0.0, 0.6, -0.05, "Far right, close"),
        (-0.3, 0.7, -0.05, "Very far left"),
        (0.1, 0.7, -0.05, "Very far right"),
        (-0.1, 0.4, -0.05, "Very close to robot"),
        (-0.1, 1.0, -0.05, "Very far from robot"),
    ]

    # Store calibration data
    calibration_data = []
    
    # Create a safe home position (above the workspace)
    home_pose = geometry_msgs.msg.Pose()
    home_pose.position.x = 0.0
    home_pose.position.y = 0.0
    home_pose.position.z = 0.2  # Lower height, more reachable
    home_pose.orientation.x = 0.0
    home_pose.orientation.y = 0.0
    home_pose.orientation.z = 0.0
    home_pose.orientation.w = 1.0

    print(f"Moving to home position...")
    print("Press 's' to skip home position, or Enter to continue...")
    skip_home = input().lower().strip()
    
    if skip_home != 's':
        try:
            move_group.set_pose_target(home_pose)
            
            # Try planning with timeout
            plan = move_group.plan()
            if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
                print("Planning successful. Executing...")
                success = move_group.go(wait=True)
                if success:
                    move_group.stop()
                    move_group.clear_pose_targets()
                    print("Reached home position.")
                else:
                    print("Failed to execute plan. Trying alternative home position...")
                    # Try alternative home position
                    home_pose.position.x = 0.1
                    home_pose.position.y = 0.1
                    home_pose.position.z = 0.15
                    move_group.set_pose_target(home_pose)
                    plan = move_group.plan()
                    if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
                        success = move_group.go(wait=True)
                        if success:
                            move_group.stop()
                            move_group.clear_pose_targets()
                            print("Reached alternative home position.")
                        else:
                            print("Failed to reach any home position. Continuing from current position...")
                    else:
                        print("Failed to plan to alternative home position. Continuing from current position...")
            else:
                print("Failed to plan to home position. Trying alternative...")
                # Try alternative home position
                home_pose.position.x = 0.1
                home_pose.position.y = 0.1
                home_pose.position.z = 0.15
                move_group.set_pose_target(home_pose)
                plan = move_group.plan()
                if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
                    success = move_group.go(wait=True)
                    if success:
                        move_group.stop()
                        move_group.clear_pose_targets()
                        print("Reached alternative home position.")
                    else:
                        print("Failed to reach any home position. Continuing from current position...")
                else:
                    print("Failed to plan to any home position. Continuing from current position...")
        except Exception as e:
            print(f"Error during home movement: {e}")
            print("Continuing from current position...")
    else:
        print("Skipped home position. Starting from current position.")

    # Process each calibration point
    for i, (x, y, z, description) in enumerate(calibration_points):
        print(f"\n=== Point {i+1}/{len(calibration_points)}: {description} ===")
        print(f"Robot coordinates: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        print("Press 's' to skip this point, or Enter to continue...")
        
        skip_point = input().lower().strip()
        if skip_point == 's':
            print(f"Skipped point {i+1}: {description}")
            continue
        
        try:
            # Move robot to position above the target
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z + 0.15  # 15cm above the target position
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 1.0

            print(f"Moving robot to position above target...")
            move_group.set_pose_target(target_pose)
            plan = move_group.plan()
            
            if not plan or not hasattr(plan, 'joint_trajectory') or not plan.joint_trajectory.points:
                print(f"Failed to plan to position {i+1}. Skipping...")
                continue
                
            success = move_group.go(wait=True)
            if not success:
                print(f"Failed to execute plan for position {i+1}. Skipping...")
                continue
                
            move_group.stop()
            move_group.clear_pose_targets()
            print("Robot positioned. Place a bottle at the target location.")
            
        except Exception as e:
            print(f"Error moving to position {i+1}: {e}")
            print("Skipping this point...")
            continue
        
        # Get camera coordinates from user
        while True:
            try:
                print("\nEnter YOLO detection coordinates:")
                print("(or press 's' to skip this point)")
                u_input = input("u (pixel x): ").strip()
                if u_input.lower() == 's':
                    print(f"Skipped data collection for point {i+1}")
                    break
                    
                u = float(u_input)
                v = float(input("v (pixel y): "))
                bbox_width = float(input("Bounding box width (pixels): "))
                bbox_height = float(input("Bounding box height (pixels): "))
                
                # Validate inputs
                if u < 0 or v < 0 or bbox_width <= 0 or bbox_height <= 0:
                    print("Invalid coordinates. Please enter positive values.")
                    continue
                    
                # Store the calibration data
                data_point = {
                    "point_id": i + 1,
                    "description": description,
                    "robot_coordinates": {"x": x, "y": y, "z": z},
                    "camera_coordinates": {
                        "u": u,
                        "v": v,
                        "bbox_width": bbox_width,
                        "bbox_height": bbox_height
                    },
                    "timestamp": datetime.now().isoformat()
                }
                
                calibration_data.append(data_point)
                print(f"Data point {i+1} recorded successfully!")
                break
                
            except ValueError:
                print("Invalid input. Please enter numeric values or 's' to skip.")
                continue
            except Exception as e:
                print(f"Error processing input: {e}")
                print("Skipping this point...")
                break

        # Ask if user wants to continue
        if i < len(calibration_points) - 1:
            response = input("\nPress Enter to continue to next point, 's' to skip next, or 'q' to quit: ")
            if response.lower() == 'q':
                break
            elif response.lower() == 's':
                print("Skipping next point...")
                continue

    # Save calibration data
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"calibration_data_{timestamp}.json"
    
    # Create data directory if it doesn't exist
    data_dir = os.path.join(os.path.dirname(__file__), "calibration_data")
    os.makedirs(data_dir, exist_ok=True)
    
    filepath = os.path.join(data_dir, filename)
    
    with open(filepath, 'w') as f:
        json.dump({
            "calibration_data": calibration_data,
            "metadata": {
                "total_points": len(calibration_data),
                "collection_date": datetime.now().isoformat(),
                "robot_frame": "base",
                "camera_frame": "camera_optical_frame"
            }
        }, f, indent=2)
    
    print(f"\n=== Calibration Complete ===")
    print(f"Collected {len(calibration_data)} data points")
    print(f"Data saved to: {filepath}")
    print("\nNext steps:")
    print("1. Use this data to train a camera-to-robot coordinate mapping")
    print("2. Implement the mapping in your main script")
    print("3. Test with new bottle positions")

    # Return to home position
    print("\nReturning to home position...")
    move_group.set_pose_target(home_pose)
    plan = move_group.plan()
    if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        print("Returned to home position.")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 
#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import CollisionObject, OrientationConstraint, Constraints, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
import tf_conversions
import math
from geometry_msgs.msg import PoseStamped
from intera_core_msgs.msg import EndpointState
import argparse
import subprocess
import os
from std_msgs.msg import Bool

#move robot arm to bottle position
def get_current_eef_pose_from_topic():
    msg = rospy.wait_for_message('/robot/limb/right/endpoint_state', EndpointState, timeout=5)
    return msg.pose

def main():
    parser = argparse.ArgumentParser(description="MoveIt Sawyer pose goal with bottle position arguments.")
    parser.add_argument('--bottle_x', type=float, default=0.4, help='Bottle x position (meters)')
    parser.add_argument('--bottle_y', type=float, default=-0.3, help='Bottle y position (meters)')
    parser.add_argument('--hand_action', type=str, default='close', choices=['open', 'close'], help='Hand action after final approach: open or close')
    parser.add_argument('--bottle_z', type=float, default=-0.05, help='Bottle y position (meters)')
    parser.add_argument('--fast', action='store_true', help='Fast mode: skip visualization and reduce delays')
    parser.add_argument('--collision', type=str, default='True', help='Enable/disable bottle collision object (True/False)')

    args = parser.parse_args()

    # Define bottle and table positions at the start
    bottle_height = 0.22  # meters
    bottle_x = args.bottle_x  # from argument
    bottle_y = args.bottle_y  # from argument
    
    # Cap bottle_z to ensure it never spawns above the ceiling (z = 0.8)
    ceiling_height = 0.8  # meters - ceiling height from environment setup
    table_surface_z = -0.05  # meters - table surface height from environment setup
    max_bottle_z = ceiling_height - bottle_height  # Ensure bottle fits below ceiling
    min_bottle_z = table_surface_z  # Ensure bottle doesn't spawn below table
    
    # Cap the bottle z position between table surface and ceiling
    bottle_z = max(min_bottle_z, min(args.bottle_z, max_bottle_z))
    
    print(f"Bottle z position capped to {bottle_z:.3f} m (table at {table_surface_z:.3f} m, ceiling at {ceiling_height:.3f} m)")

    # Initialize moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_sawyer_pose_goal', anonymous=True)
    
    # Create publisher for grasp status
    grasp_publisher = rospy.Publisher('/grasp_status', Bool, queue_size=1)
    if not args.fast:
        rospy.sleep(0.5)  # Reduced from 2 seconds to 0.5 seconds
    else:
        rospy.sleep(0.1)  # Even faster in fast mode

    # Instantiate objects for robot, scene, and group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "right_arm"  # Use original group for now, will add hand collision checking manually
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set planning time and planner
    if args.fast:
        move_group.set_planning_time(1.0)  # Fast mode - shorter planning time
        # Use RRTConnect for speed in fast mode
        move_group.set_planner_id("RRTConnectkConfigFast")
    else:
        move_group.set_planning_time(2.0)  # Reduced from 3.0 to avoid timeouts
        # Use RRTConnect for reliability instead of RRT*
        move_group.set_planner_id("RRTConnectkConfigDefault")

    # Create a DisplayTrajectory publisher for RViz visualization
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

    # Print useful info
    planning_frame = move_group.get_planning_frame()
    print("Planning frame:", planning_frame)

    eef_link = move_group.get_end_effector_link()
    print("End effector link:", eef_link)

    group_names = robot.get_group_names()
    print("Available Planning Groups:", group_names)

    print("Robot state:")
    print(robot.get_current_state())

    # Get current end effector position from topic
    current_pose = get_current_eef_pose_from_topic()
    print(f"Current end effector position from topic:")
    print(f"  X: {current_pose.position.x:.3f} m")
    print(f"  Y: {current_pose.position.y:.3f} m") 
    print(f"  Z: {current_pose.position.z:.3f} m")

    ## --------------------
    ## Set End Effector Pose Goal with Waypoints
    ## --------------------

    # --- Define waypoints for controlled motion ---
    # Use the bottle coordinates defined at the top of the script
    # bottle_height, bottle_x, bottle_y, bottle_z are already defined above

    # Waypoint 2: Final approach position (very close to bottle)
    waypoint3 = geometry_msgs.msg.Pose()
    waypoint3.position.x = bottle_x - 0.1  # very close to bottle
    waypoint3.position.y = bottle_y
    waypoint3.position.z = bottle_z + (bottle_height / 2)# final approach height

    # Set orientation for all waypoints (side gripping, with 45 deg wrist rotation)
    base_roll = 0.7
    pitch = 1.5708  # 90 degrees in radians (π/2) - full side grip
    yaw = 0    # 45 degrees in radians
    
    # Define roll values to try if planning fails
    # Roll range is -π to +π radians (-180° to +180°)
    base_roll_values = [
        0.7,                    # 0.7 (original)
        0.0,                         # 0° (neutral)
        0.785,                       # 45° (π/4)
        1.2,                         # base_roll + 0.5
        1.57,                        # 90° (π/2)
        1.7,                         # base_roll + 1.0
        2.27,                        # base_roll + 1.57 (π/2)
        2.356,                       # 135° (3π/4)
        3.14,                        # 180° (π)
        # Extreme/negative roll values last
        -0.3,                        # base_roll - 1.0
        0.2,                         # base_roll - 0.5
        -0.785,                      # -45° (-π/4)
        -0.87,                       # base_roll - 1.57 (-π/2)
        -1.57,                       # -90° (-π/2)
        -2.356,                      # -135° (-3π/4)
        -2.44,                       # base_roll - 3.14 (-π)
        -3.14,                       # -180° (-π)
    ]
    
    # Track successful roll values to prioritize them
    successful_rolls = []
    
    def get_prioritized_roll_values(successful_rolls, base_roll_values):
        """Return roll values with successful ones prioritized first"""
        # Start with successful rolls (if any)
        prioritized = list(successful_rolls)
        
        # Add remaining base roll values (avoiding duplicates)
        for roll in base_roll_values:
            if roll not in prioritized:
                prioritized.append(roll)
        
        return prioritized
    
    # Simple approach direction - no need for roll-based offsets with custom end effector
    def get_approach_direction():
        """Get the approach direction for the custom end effector"""
        # With custom end effector, use a simple consistent approach
        return (0.0, 0.0)  # Approach from left side, slightly back

    # --- Add a soda bottle as a collision object BEFORE planning ---
    bottle_radius = 0.035  # meters
    bottle_pose = PoseStamped()
    bottle_pose.header.frame_id = move_group.get_planning_frame()
    bottle_pose.pose.position.x = bottle_x
    bottle_pose.pose.position.y = bottle_y
    # Use the bottle_z argument for the bottle base position
    bottle_pose.pose.position.z = bottle_z + (bottle_height / 2)  # center at half height above bottle_z
    bottle_pose.pose.orientation.x = 0.0
    bottle_pose.pose.orientation.y = 0.0
    bottle_pose.pose.orientation.z = 0.0
    bottle_pose.pose.orientation.w = 1.0
    
    # Conditionally add bottle as collision object
    collision_enabled = args.collision.lower() == 'true'
    if collision_enabled:
        scene.add_cylinder("bottle", bottle_pose, height=bottle_height, radius=bottle_radius)
        print("Added bottle as collision object")
        if not args.fast:
            rospy.sleep(0.1)  # Reduced from 0.5 to 0.1 seconds
        else:
            rospy.sleep(0.05)  # Even faster in fast mode
    else:
        print("Skipping bottle collision object (--collision=False)")
    
    # Function to remove bottle collision object if needed
    def remove_bottle_collision():
        """Remove the bottle collision object from the scene"""
        scene.remove_world_object("bottle")
        print("Removed bottle collision object")
    
    # Enable collision checking between hand links and bottle
    # This ensures MoveIt! considers hand-bottle collisions during planning
    if not args.fast:
        print("Enabling collision checking between hand and bottle...")
        
        # Add hand links as collision objects to ensure they're checked against the bottle
        hand_links = [
            "palm_lower", "mcp_joint", "pip", "dip", "fingertip",
            "mcp_joint_2", "pip_2", "dip_2", "fingertip_2",
            "mcp_joint_3", "pip_3", "dip_3", "fingertip_3",
            "pip_4", "thumb_pip", "thumb_dip", "thumb_fingertip"
        ]
        
        # Get current robot state to check if hand links exist
        robot_state = robot.get_current_state()
        # Get all link names from the robot model
        available_links = robot.get_link_names()
        print(f"Available robot links: {available_links}")
        
        # Check which hand links are available
        available_hand_links = [link for link in hand_links if link in available_links]
        print(f"Available hand links for collision checking: {available_hand_links}")
    else:
        print("Fast mode: Skipping detailed hand collision checking...")

    # Execute waypoints sequentially with roll retry logic
    waypoints = [waypoint3]
    waypoint_names = ["Final approach"]
    
    for i, (waypoint, name) in enumerate(zip(waypoints, waypoint_names)):
        print(f"Planning to waypoint {i+1}: {name}...")
        
        # Try different roll values
        planning_success = False
        for roll_idx, roll in enumerate(get_prioritized_roll_values(successful_rolls, base_roll_values)):
            print(f"  Trying roll = {roll:.2f} (attempt {roll_idx + 1}/{len(get_prioritized_roll_values(successful_rolls, base_roll_values))})")
            
            # Get simple approach direction for custom end effector
            approach_x, approach_y = get_approach_direction()
            
            # Set waypoint position with simple offset
            waypoint.position.x = bottle_x + approach_x
            waypoint.position.y = bottle_y + approach_y
            
            # Set orientation with current roll value
            q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
            norm = (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2) ** 0.5
            waypoint.orientation.x = q[0]/norm
            waypoint.orientation.y = q[1]/norm
            waypoint.orientation.z = q[2]/norm
            waypoint.orientation.w = q[3]/norm
            
            # Try planning with current planner
            move_group.set_pose_target(waypoint)
            plan_result = move_group.plan()

            if isinstance(plan_result, tuple):
                success, plan, planning_time, error_code = plan_result
                # Only accept exact solutions
                if error_code.val != MoveItErrorCodes.SUCCESS:
                    print(f"  Planning failed or approximate solution (error_code={error_code.val}), trying next...")
                    continue
                # Check for approximate solution in plan (if available)
                if hasattr(plan, 'is_approximate_solution') and plan.is_approximate_solution:
                    print("  Skipping approximate solution, trying next roll value...")
                    continue
            else:
                plan = plan_result
                # For older API, you may need to check plan validity another way
                if not plan or not hasattr(plan, 'joint_trajectory') or not plan.joint_trajectory.points:
                    print("  Planning failed, trying next...")
                    continue

            if success and plan is not None:
                print(f"  Planning successful with roll = {roll:.2f}")
                planning_success = True
                successful_rolls.append(roll)
                break
            else:
                print(f"  Planning failed with roll = {roll:.2f}, trying next...")
        

        
        if not planning_success:
            print(f"Planning to {name} failed with all roll values! Continuing to next waypoint if any...")
            continue  # Instead of return, continue to next waypoint

        ## --------------------
        ## Visualize the Plan in RViz
        ## --------------------
        if not args.fast:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            if plan and hasattr(plan, 'joint_trajectory'):
                display_trajectory.trajectory = []  # Initialize as empty list
                display_trajectory.trajectory.append(plan)
                print(f"Publishing trajectory to RViz for {name}...")
                display_trajectory_publisher.publish(display_trajectory)
                print(f"Trajectory published for {name}. Waiting 2 seconds for visualization...")
                rospy.sleep(0.2)  # Shorter pause for waypoints
            else:
                print(f"Plan for {name} does not have a joint_trajectory. Skipping visualization.")
        else:
            print(f"Skipping visualization for {name} in fast mode.")

        ## --------------------
        ## Execute the Plan
        ## --------------------
        if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
            print(f"Executing plan to {name}...")
            move_group.execute(plan, wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
            print(f"Completed waypoint {i+1}: {name}")
            # After waypoint 2 (Final approach), control the hand based on argument
            if i == 0: # Only waypoint is waypoint3
                env = os.environ.copy()
                env['ROS_MASTER_URI'] = 'http://localhost:11311'
                if args.hand_action == 'open':
                    subprocess.run([
                        'python3', '/home/atrc234/ros_ws/src/ros_module/move_sim_hand.py', 
                        'waypoint_open',  '--waypoint_delay', '0.05'
                    ], env=env)
                    print("Hand opened after final approach (via move_sim_hand.py subprocess call).")
                else:  # close
                    result = subprocess.run([
                        'python3', '/home/atrc234/ros_ws/src/ros_module/move_sim_hand.py',
                        'waypoint_close',  '--waypoint_delay', '0.05'
                    ], env=env, capture_output=True, text=True)
                    print("Hand closed after final approach (via move_sim_hand.py subprocess call).")
                    output = result.stdout.strip()
                    print(output)
                    grasp_success = False
                    cold_object = False
                    # Check for grasp and cold object markers in output
                    if "Hand grabbed something!" in output or "GRASP_SUCCESS" in output or output.endswith("yes"):
                        print("Hand grabbed something!")
                        print("GRASP_SUCCESS")  # Special marker for bartender_robo
                        grasp_success = True
                        grasp_publisher.publish(True)
                        if "Robot has grabbed something cold" in output:
                            cold_object = True
                            print("COLD_OBJECT_DETECTED")  # Special marker for bartender_robo
                        elif "Temperature change does not indicate a cold object." in output:
                            cold_object = False
                            print("NOT_COLD_OBJECT")  # Special marker for bartender_robo
                    else:
                        print("Hand did NOT grab anything.")
                        print("GRASP_FAILED")  # Special marker for bartender_robo
                        grasp_publisher.publish(False)
                    # Exit with appropriate code
                    if grasp_success:
                        move_group.clear_path_constraints()
                        if cold_object:
                            sys.exit(2)
                        else:
                            sys.exit(0)
                    else:
                        move_group.clear_path_constraints()
                        sys.exit(1)
        else:
            print(f"Plan for {name} is empty or invalid. Skipping execution.")
        # Small pause between waypoints
        if i == len(waypoints) - 1:  # Don't pause after the last waypoint
            if not args.fast:
                rospy.sleep(1)
            else:
                rospy.sleep(0.2)  # Much faster in fast mode

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()

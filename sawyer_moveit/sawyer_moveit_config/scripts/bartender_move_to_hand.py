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
    rospy.sleep(0.5)  # Reduced from 2 seconds to 0.5 seconds

    # Instantiate objects for robot, scene, and group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "right_arm"  # Use original group for now, will add hand collision checking manually
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set planning time and planner
    move_group.set_planning_time(20.0)  # Reduced from 3.0 to avoid timeouts
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

    # --- Define target pose for bottle approach ---
    # Use the bottle coordinates defined at the top of the script
    # bottle_height, bottle_x, bottle_y, bottle_z are already defined above

    # Target pose: Final approach position (very close to bottle)
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = bottle_x - 0.1  # very close to bottle
    target_pose.position.y = bottle_y
    target_pose.position.z = bottle_z + (bottle_height / 2)  # final approach height

    # Set orientation constraints for yaw and pitch, allowing roll to be flexible
    pitch = 1.5708  # 90 degrees in radians (π/2) - full side grip
    yaw = 0    # 0 degrees in radians
    
    def create_orientation_constraints():
        """Create orientation constraints that fix yaw and pitch but allow roll to be flexible"""
        constraints = Constraints()
        
        # Create orientation constraint for the end effector
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = move_group.get_planning_frame()
        orientation_constraint.link_name = move_group.get_end_effector_link()
        
        # Set the target orientation (roll will be flexible)
        q = tf_conversions.transformations.quaternion_from_euler(0.0, pitch, yaw)
        norm = (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2) ** 0.5
        orientation_constraint.orientation.x = q[0]/norm
        orientation_constraint.orientation.y = q[1]/norm
        orientation_constraint.orientation.z = q[2]/norm
        orientation_constraint.orientation.w = q[3]/norm
        
        # Set tolerances - tight for pitch and yaw, loose for roll
        orientation_constraint.absolute_x_axis_tolerance = 3.14  # Roll tolerance (very loose - allow full rotation)
        orientation_constraint.absolute_y_axis_tolerance = 0.5   # Pitch tolerance (tight)
        orientation_constraint.absolute_z_axis_tolerance = 0.5   # Yaw tolerance (tight)
        
        # Set weight
        orientation_constraint.weight = 0.7
        
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints
    
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
        rospy.sleep(0.1)  # Reduced from 0.5 to 0.1 seconds
    else:
        print("Skipping bottle collision object (--collision=False)")
    
    # Function to remove bottle collision object if needed
    def remove_bottle_collision():
        """Remove the bottle collision object from the scene"""
        scene.remove_world_object("bottle")
        print("Removed bottle collision object")
    
    # Enable collision checking between hand links and bottle
    # This ensures MoveIt! considers hand-bottle collisions during planning
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

    # Execute single target pose
    print("Planning to bottle approach position...")
    
    # Get simple approach direction for custom end effector
    approach_x, approach_y = get_approach_direction()
    
    # Set target position with simple offset
    target_pose.position.x = bottle_x + approach_x
    target_pose.position.y = bottle_y + approach_y
    
    # Set a default orientation (roll will be flexible due to constraints)
    q = tf_conversions.transformations.quaternion_from_euler(0.0, pitch, yaw)
    norm = (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2) ** 0.5
    target_pose.orientation.x = q[0]/norm
    target_pose.orientation.y = q[1]/norm
    target_pose.orientation.z = q[2]/norm
    target_pose.orientation.w = q[3]/norm
        
            # Set orientation constraints to fix yaw and pitch but allow roll to be flexible
    print("  Setting orientation constraints (yaw and pitch fixed, roll flexible)...")
    constraints = create_orientation_constraints()
    move_group.set_path_constraints(constraints)
    
    # Try planning with orientation constraints
    move_group.set_pose_target(target_pose)
    plan_result = move_group.plan()

    if isinstance(plan_result, tuple):
        success, plan, planning_time, error_code = plan_result
        # Only accept exact solutions
        if error_code.val != MoveItErrorCodes.SUCCESS:
            print(f"  Planning failed or approximate solution (error_code={error_code.val})")
            move_group.clear_path_constraints()
            print("Planning failed. Exiting.")
            sys.exit(1)
        # Check for approximate solution in plan (if available)
        if hasattr(plan, 'is_approximate_solution') and plan.is_approximate_solution:
            print("  Skipping approximate solution")
            move_group.clear_path_constraints()
            print("Planning failed. Exiting.")
            sys.exit(1)
    else:
        plan = plan_result
        # For older API, you may need to check plan validity another way
        if not plan or not hasattr(plan, 'joint_trajectory') or not plan.joint_trajectory.points:
            print("  Planning failed")
            move_group.clear_path_constraints()
            print("Planning failed. Exiting.")
            sys.exit(1)

    if success and plan is not None:
        print(f"  Planning successful with orientation constraints")
        
        ## --------------------
        ## Visualize the Plan in RViz
        ## --------------------
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        if plan and hasattr(plan, 'joint_trajectory'):
            display_trajectory.trajectory = []  # Initialize as empty list
            display_trajectory.trajectory.append(plan)
            print(f"Publishing trajectory to RViz...")
            display_trajectory_publisher.publish(display_trajectory)
            print(f"Trajectory published. Waiting 2 seconds for visualization...")
            rospy.sleep(0.2)  # Shorter pause for waypoints
        else:
            print(f"Plan does not have a joint_trajectory. Skipping visualization.")

        ## --------------------
        ## Execute the Plan
        ## --------------------
        if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
            print(f"Executing plan to bottle approach position...")
            move_group.execute(plan, wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
            print(f"Completed bottle approach")
            # Control the hand based on argument
            env = os.environ.copy()
            env['ROS_MASTER_URI'] = 'http://localhost:11311'
            if args.hand_action == 'open':
                subprocess.run([
                    'python3', '/home/atrc234/ros_ws/src/ros_module/move_sim_hand.py', 
                    'waypoint_open_grab',  '--waypoint_delay', '0.05'
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
            print(f"Plan is empty or invalid. Skipping execution.")
            sys.exit(1)
    else:
        print(f"  Planning failed with orientation constraints")
        move_group.clear_path_constraints()
        print("Planning failed. Exiting.")
        sys.exit(1)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()

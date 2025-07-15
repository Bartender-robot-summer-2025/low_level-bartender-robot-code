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

#move robot arm to bottle position
def get_current_eef_pose_from_topic():
    msg = rospy.wait_for_message('/robot/limb/right/endpoint_state', EndpointState, timeout=5)
    return msg.pose

def main():
    parser = argparse.ArgumentParser(description="MoveIt Sawyer pose goal with automatic orientation planning.")
    parser.add_argument('--bottle_x', type=float, default=0.4, help='Bottle x position (meters)')
    parser.add_argument('--bottle_y', type=float, default=-0.3, help='Bottle y position (meters)')
    parser.add_argument('--hand_action', type=str, default='close', choices=['open', 'close'], help='Hand action after final approach: open or close')
    parser.add_argument('--bottle_z', type=float, default=-0.05, help='Bottle z position (meters)')
    parser.add_argument('--fast', action='store_true', help='Fast mode: skip visualization and reduce delays')

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
    rospy.init_node('moveit_sawyer_auto_orientation', anonymous=True)
    if not args.fast:
        rospy.sleep(0.5)  # Reduced from 2 seconds to 0.5 seconds
    else:
        rospy.sleep(0.1)  # Even faster in fast mode

    # Instantiate objects for robot, scene, and group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "right_arm"  # Use original group for now, will add hand collision checking manually
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # IMPROVED PLANNING SETTINGS FOR SHORTER PATHS
    # Set longer planning time to allow for better path optimization
    if args.fast:
        move_group.set_planning_time(0.5)  # Fast mode - shorter planning time
    else:
        move_group.set_planning_time(2.0)  # Normal mode - longer planning time for better paths
    
    # Set explicit planner for JOINT MOVEMENT COST FUNCTION - prioritize cost-aware planners
    try:
        if args.fast:
            move_group.set_planner_id("PRMstarkConfigJointCost")
            print("Using PRM* Joint Cost planner for fast optimal joint paths")
        else:
            move_group.set_planner_id("RRTstarkConfigJointCost")
            print("Using RRT* Joint Cost planner for optimal minimal joint movement")
    except:
        try:
            move_group.set_planner_id("PRMstarkConfigDefault")
            print("Using PRM* planner for good joint optimization")
        except:
            try:
                move_group.set_planner_id("TRRTkConfigJointCost")
                print("Using TRRT Joint Cost planner for balanced joint optimization")
            except:
                try:
                    move_group.set_planner_id("RRTstarkConfigDefault")
                    print("Using RRT* planner as fallback")
                except:
                    print("Using default planner")

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
    ## Set End Effector Pose Goal with Automatic Orientation Planning
    ## --------------------

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
    scene.add_cylinder("bottle", bottle_pose, height=bottle_height, radius=bottle_radius)
    if not args.fast:
        rospy.sleep(0.1)  # Reduced from 0.5 to 0.1 seconds
    else:
        rospy.sleep(0.05)  # Even faster in fast mode
    
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

    ## --------------------
    ## AUTOMATIC ORIENTATION PLANNING - Let the planner find optimal roll
    ## --------------------
    
    print("Using automatic orientation planning - letting planner find optimal roll value...")
    
    # Define the target position (final approach to bottle)
    target_position = geometry_msgs.msg.Pose()
    target_position.position.x = bottle_x - 0.1  # Approach from left side (4-finger hand grasps on left)
    target_position.position.y = bottle_y
    target_position.position.z = bottle_z + (bottle_height / 2)  # At bottle center height
    
    # Set a base orientation (pitch = 90° for side grip, yaw = 0, roll will be optimized)
    pitch = 1.5708  # 90 degrees in radians (π/2) - side grip
    yaw = 0.0       # No yaw rotation
    base_roll = 0.7 # Starting roll value
    
    # Create orientation constraint to allow roll optimization
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = move_group.get_planning_frame()
    orientation_constraint.link_name = move_group.get_end_effector_link()
    
    # Set the base orientation
    q = tf_conversions.transformations.quaternion_from_euler(base_roll, pitch, yaw)
    orientation_constraint.orientation.x = q[0]
    orientation_constraint.orientation.y = q[1]
    orientation_constraint.orientation.z = q[2]
    orientation_constraint.orientation.w = q[3]
    
    # Allow significant roll variation (±π/2 = ±90°) for the planner to find optimal orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.5  # Allow roll variation
    orientation_constraint.absolute_y_axis_tolerance = 0.1  # Keep pitch relatively fixed
    orientation_constraint.absolute_z_axis_tolerance = 0.1  # Keep yaw relatively fixed
    orientation_constraint.weight = 1.0
    
    # Create constraints object
    constraints = Constraints()
    constraints.orientation_constraints.append(orientation_constraint)
    
    # Set the target pose with flexible orientation
    move_group.set_pose_target(target_position)
    move_group.set_path_constraints(constraints)
    
    print(f"Planning to bottle at ({bottle_x:.3f}, {bottle_y:.3f}, {bottle_z:.3f}) with flexible roll...")
    
    # Try planning with automatic orientation optimization
    plan_result = move_group.plan()
    
    if isinstance(plan_result, tuple):
        success, plan, planning_time, error_code = plan_result
        if error_code.val != MoveItErrorCodes.SUCCESS:
            print(f"Planning failed (error_code={error_code.val})")
            print("Trying with relaxed orientation constraints...")
            
            # Relax orientation constraints if planning fails
            orientation_constraint.absolute_x_axis_tolerance = 1.0  # Allow full roll variation
            orientation_constraint.absolute_y_axis_tolerance = 0.3  # Allow more pitch variation
            orientation_constraint.absolute_z_axis_tolerance = 0.3  # Allow more yaw variation
            
            constraints.orientation_constraints = [orientation_constraint]
            move_group.set_path_constraints(constraints)
            
            plan_result = move_group.plan()
            if isinstance(plan_result, tuple):
                success, plan, planning_time, error_code = plan_result
                if error_code.val != MoveItErrorCodes.SUCCESS:
                    print(f"Planning still failed with relaxed constraints (error_code={error_code.val})")
                    print("Trying without orientation constraints...")
                    
                    # Try without orientation constraints as last resort
                    move_group.clear_path_constraints()
                    plan_result = move_group.plan()
                    if isinstance(plan_result, tuple):
                        success, plan, planning_time, error_code = plan_result
                    else:
                        plan = plan_result
                        success = plan is not None and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points
                else:
                    plan = plan_result
            else:
                plan = plan_result
                success = plan is not None and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points
        else:
            plan = plan_result
    else:
        plan = plan_result
        success = plan is not None and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points

    if success and plan is not None:
        print("Planning successful with automatic orientation optimization!")
        
        ## --------------------
        ## Visualize the Plan in RViz
        ## --------------------
        if not args.fast:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            if plan and hasattr(plan, 'joint_trajectory'):
                display_trajectory.trajectory = []  # Initialize as empty list
                display_trajectory.trajectory.append(plan)
                print("Publishing trajectory to RViz...")
                display_trajectory_publisher.publish(display_trajectory)
                print("Trajectory published. Waiting 2 seconds for visualization...")
                rospy.sleep(0.2)  # Shorter pause
            else:
                print("Plan does not have a joint_trajectory. Skipping visualization.")
        else:
            print("Skipping visualization in fast mode.")

        ## --------------------
        ## Execute the Plan
        ## --------------------
        if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
            print("Executing plan...")
            move_group.execute(plan, wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
            move_group.clear_path_constraints()
            print("Completed movement to bottle")
            
            # Control the hand based on argument
            env = os.environ.copy()
            env['ROS_MASTER_URI'] = 'http://localhost:11311'
            if args.hand_action == 'open':
                subprocess.run([
                    'python3', '/home/atrc234/catkin_ws/src/ros_module/move_sim_hand.py', 
                    'waypoint_open', '--num_waypoints', '10', '--waypoint_delay', '0.05'
                ], env=env)
                print("Hand opened after reaching bottle (via move_sim_hand.py subprocess call).")
            else:  # close
                subprocess.run([
                    'python3', '/home/atrc234/catkin_ws/src/ros_module/move_sim_hand.py',
                    'waypoint_close', '--num_waypoints', '10', '--waypoint_delay', '0.05'
                ], env=env)
                print("Hand closed after reaching bottle (via move_sim_hand.py subprocess call).")
        else:
            print("Plan is empty or invalid. Skipping execution.")
    else:
        print("Planning failed with all orientation optimization attempts!")
        print("Consider adjusting bottle position or robot starting pose.")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main() 
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

def get_current_eef_pose_from_topic():
    msg = rospy.wait_for_message('/robot/limb/right/endpoint_state', EndpointState, timeout=5)
    return msg.pose

def main():
    # Define bottle and table positions at the start
    bottle_height = 0.15  # meters
    bottle_x = 0.4  # on the table at the front (was 0.0)
    bottle_y = -0.3  # centered on front table (was 0.8)
    bottle_z = -0.05  # lowered table surface height (was 0.0)

    # Initialize moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_sawyer_pose_goal', anonymous=True)
    rospy.sleep(2)  # Wait for MoveIt! and robot state to be available

    # Instantiate objects for robot, scene, and group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "right_arm"  # Check this matches your Sawyer MoveIt setup
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # IMPROVED PLANNING SETTINGS FOR SHORTER PATHS
    # Set longer planning time to allow for better path optimization
    move_group.set_planning_time(2.0)  # Increased from default to 2 seconds
    
    # Set explicit planner for shorter paths - try RRT* first, then RRT Connect with larger range
    available_planners = move_group.get_planner_id()
    print(f"Available planners: {available_planners}")
    
    # Try to set RRT* first (better for optimal paths), then RRT Connect with larger range
    try:
        move_group.set_planner_id("RRTstarkConfigDefault")
        print("Using RRT* planner for optimal paths")
    except:
        try:
            move_group.set_planner_id("RRTConnectkConfigFast")
            print("Using RRT Connect Fast planner for shorter paths")
        except:
            try:
                move_group.set_planner_id("RRTConnectkConfigDefault")
                print("Using RRT Connect Default planner")
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
    ## Add Table Obstacle (lowered height)
    ## --------------------
    print("Adding table obstacle at lowered height...")
    
    # Create a collision object for the table
    table = CollisionObject()
    table.header.frame_id = "base"
    table.id = "table_obstacle"
    
    # Create a box primitive for the table (positioned to the left of robot, not centered on base)
    table_primitive = SolidPrimitive()
    table_primitive.type = SolidPrimitive.BOX
    table_primitive.dimensions = [0.75, 0.59, 0.05]  # length, width, height (smaller, more realistic)
    
    # Set the table pose (positioned to the front of robot, rotated 90 degrees)
    table_pose = geometry_msgs.msg.Pose()
    table_pose.position.x = 0.6  # in front of the robot (was 0.0)
    table_pose.position.y = 0.0  # centered (was 0.8)
    table_pose.position.z = -0.075  # Half height so top surface is at -0.05 (lowered by 5cm)
    # Rotate table 90 degrees around z-axis
    table_pose.orientation.x = 0.0
    table_pose.orientation.y = 0.0
    table_pose.orientation.z = 0.707  # sin(90°/2) = sin(45°) = 0.707
    table_pose.orientation.w = 0.707  # cos(90°/2) = cos(45°) = 0.707
    
    # Add the primitive and pose to the collision object
    table.primitives.append(table_primitive)
    table.primitive_poses.append(table_pose)
    
    # Add the table to the scene
    scene.add_object(table)
    
    # Wait a moment for the scene to update
    rospy.sleep(1.0)
    print("Table obstacle added successfully!")

    ## --------------------
    ## Add Robot Base Marker
    ## --------------------
    print("Adding robot base marker...")
    
    # Create a collision object for the robot base marker
    base_marker = CollisionObject()
    base_marker.header.frame_id = "base"
    base_marker.id = "robot_base_marker"
    
    # Create a small sphere primitive for the base marker
    base_primitive = SolidPrimitive()
    base_primitive.type = SolidPrimitive.SPHERE
    base_primitive.dimensions = [0.05]  # 5cm diameter sphere
    
    # Set the base marker pose (at robot base origin)
    base_pose = geometry_msgs.msg.Pose()
    base_pose.position.x = 0.0  # robot base
    base_pose.position.y = 0.0  # robot base
    base_pose.position.z = 0.0  # robot base
    base_pose.orientation.x = 0.0
    base_pose.orientation.y = 0.0
    base_pose.orientation.z = 0.0
    base_pose.orientation.w = 1.0
    
    # Add the primitive and pose to the collision object
    base_marker.primitives.append(base_primitive)
    base_marker.primitive_poses.append(base_pose)
    
    # Add the base marker to the scene
    scene.add_object(base_marker)
    
    # Wait a moment for the scene to update
    rospy.sleep(1.0)
    print("Robot base marker added successfully!")

    ## --------------------
    ## Add Ceiling Obstacle (z = 1.0)
    ## --------------------
    print("Adding ceiling obstacle at z = 1.0...")
    
    # Create a collision object for the ceiling
    ceiling = CollisionObject()
    ceiling.header.frame_id = "base"
    ceiling.id = "ceiling_obstacle"
    
    # Create a box primitive for the ceiling (large enough to cover workspace)
    ceiling_primitive = SolidPrimitive()
    ceiling_primitive.type = SolidPrimitive.BOX
    ceiling_primitive.dimensions = [2.0, 2.0, 0.1]  # length, width, height
    
    # Set the ceiling pose (positioned at z = 0.5, so bottom surface is at z = 0.45)
    ceiling_pose = geometry_msgs.msg.Pose()
    ceiling_pose.position.x = 0.0
    ceiling_pose.position.y = 0.0
    ceiling_pose.position.z = 0.8  # Lower ceiling - half height so bottom surface is at 0.45
    ceiling_pose.orientation.w = 0.0  # No rotation
    
    # Add the primitive and pose to the collision object
    ceiling.primitives.append(ceiling_primitive)
    ceiling.primitive_poses.append(ceiling_pose)
    
    # Add the ceiling to the scene
    scene.add_object(ceiling)
    
    # Wait a moment for the scene to update
    rospy.sleep(1.0)
    print("Ceiling obstacle added successfully!")

    ## --------------------
    ## Set End Effector Pose Goal with Waypoints
    ## --------------------

    # --- Define waypoints for controlled motion ---
    # Use the bottle coordinates defined at the top of the script
    # bottle_height, bottle_x, bottle_y, bottle_z are already defined above

    # Waypoint 1: Move to a safe position above and to the side of the bottle
    waypoint1 = geometry_msgs.msg.Pose()
    waypoint1.position.x = bottle_x - 0.1  # further back from bottle
    waypoint1.position.y = bottle_y
    waypoint1.position.z = bottle_z + (bottle_height / 2) + 0.25  # higher approach (was 0.15)
    # Neutral orientation for first waypoint (don't care about orientation)
    waypoint1.orientation.x = 0.0
    waypoint1.orientation.y = 0.0
    waypoint1.orientation.z = 0.0
    waypoint1.orientation.w = 1.0

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
        base_roll,                    # 0.7 (original)
        base_roll + 0.5,             # 1.2
        base_roll - 0.5,             # 0.2
        base_roll + 1.0,             # 1.7
        base_roll - 1.0,             # -0.3
        base_roll + 1.57,            # 2.27 (π/2)
        base_roll - 1.57,            # -0.87 (-π/2)
        base_roll + 3.14,            # 3.84 (π)
        base_roll - 3.14,            # -2.44 (-π)
        0.0,                         # 0° (neutral)
        1.57,                        # 90° (π/2)
        -1.57,                       # -90° (-π/2)
        3.14,                        # 180° (π)
        -3.14,                       # -180° (-π)
        0.785,                       # 45° (π/4)
        -0.785,                      # -45° (-π/4)
        2.356,                       # 135° (3π/4)
        -2.356,                      # -135° (-3π/4)
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
    
    # Function to adjust bottle position based on roll to keep it on same side of palm
    def adjust_bottle_position_for_roll(roll, base_x, base_y):
        """Adjust bottle position so it's always on the same side of the palm regardless of roll"""
        # For all roll values, keep bottle at original position
        # The robot will always approach from the left side (smaller x value)
        return base_x, base_y
    
    def get_approach_direction_for_roll(roll):
        """Get the approach direction based on hand geometry - 4-finger right hand grasps on left side"""
        roll_normalized = roll % (2 * 3.14159)  # Normalize to 0-2π
        roll_degrees = roll_normalized * 180 / 3.14159
        
        # Define approach directions based on hand orientation
        # Right hand with 4 fingers can only grasp objects on the LEFT side of the hand
        # Coordinate system: X=forward, Y=left, Z=up
        
        if 0 <= roll_degrees < 45:  # 0° to 45°
            # Hand points forward, grasp on left side
            # Approach: slightly less than bottle_x, slightly more than bottle_y (more left)
            return (-0.1, 0.01)
                
        elif 45 <= roll_degrees < 90:  # 45° to 90°
            # Hand points forward-right, grasp on left side
            # Approach: slightly less than bottle_x, more than bottle_y (more left)
            return (-0.1, 0.11)
                
        elif 90 <= roll_degrees < 135:  # 90° to 135°
            # Hand points right, grasp on left side
            # Approach: same bottle_x, more than bottle_y (more left)
            return (0, 0.1)
                
        elif 135 <= roll_degrees < 180:  # 135° to 180°
            # Hand points back-right, grasp on left side
            # Approach: more than bottle_x, more than bottle_y (more left)
            return (0.05, 0.1)
                
        elif 180 <= roll_degrees < 225:  # 180° to 225°
            # Hand points backward, grasp on left side
            # Approach: more than bottle_x, slightly more than bottle_y
            return (0.05, 0.05)
                
        elif 225 <= roll_degrees < 270:  # 225° to 270°
            # Hand points back-left, grasp on left side
            # Approach: more than bottle_x, less than bottle_y (more right)
            return (0.05, -0.1)
                
        elif 270 <= roll_degrees < 315:  # 270° to 315° (-90° to -45°)
            # Hand points left, grasp on left side
            # Approach: same bottle_x, less than bottle_y (more right)
            return (0, -0.1)
                
        else:  # 315° to 360° (-45° to 0°)
            # Hand points forward-left, grasp on left side
            # Approach: slightly less than bottle_x, less than bottle_y (more right)
            return (-0.1, -0.1)

    # --- Add a soda bottle as a collision object BEFORE planning ---
    bottle_radius = 0.035  # meters
    bottle_pose = PoseStamped()
    bottle_pose.header.frame_id = move_group.get_planning_frame()
    bottle_pose.pose.position.x = bottle_x
    bottle_pose.pose.position.y = bottle_y
    # Table top surface is now at z = -0.05, so bottle base should be at -0.05
    bottle_pose.pose.position.z = -0.05 + (bottle_height / 2)  # center at half height above table
    bottle_pose.pose.orientation.x = 0.0
    bottle_pose.pose.orientation.y = 0.0
    bottle_pose.pose.orientation.z = 0.0
    bottle_pose.pose.orientation.w = 1.0
    scene.add_cylinder("bottle", bottle_pose, height=bottle_height, radius=bottle_radius)
    rospy.sleep(2)  # Wait for the scene to update

    # Execute waypoints sequentially with roll retry logic
    waypoints = [waypoint1, waypoint3]
    waypoint_names = ["Safe position", "Final approach"]
    
    for i, (waypoint, name) in enumerate(zip(waypoints, waypoint_names)):
        print(f"Planning to waypoint {i+1}: {name}...")
        
        # Try different roll values
        planning_success = False
        for roll_idx, roll in enumerate(get_prioritized_roll_values(successful_rolls, base_roll_values)):
            print(f"  Trying roll = {roll:.2f} (attempt {roll_idx + 1}/{len(get_prioritized_roll_values(successful_rolls, base_roll_values))})")
            
            # Adjust bottle position based on roll
            adjusted_bottle_x, adjusted_bottle_y = adjust_bottle_position_for_roll(roll, bottle_x, bottle_y)
            
            # Recalculate waypoint position based on adjusted bottle position
            if i == 0:  # Safe position
                waypoint.position.x = adjusted_bottle_x + get_approach_direction_for_roll(roll)[0]
                waypoint.position.y = adjusted_bottle_y + get_approach_direction_for_roll(roll)[1]
            else:  # Final approach
                waypoint.position.x = adjusted_bottle_x + get_approach_direction_for_roll(roll)[0]
                waypoint.position.y = adjusted_bottle_y + get_approach_direction_for_roll(roll)[1]
            
            # Set orientation with current roll value
            q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
            norm = (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2) ** 0.5
            waypoint.orientation.x = q[0]/norm
            waypoint.orientation.y = q[1]/norm
            waypoint.orientation.z = q[2]/norm
            waypoint.orientation.w = q[3]/norm
            
            # Try planning
            move_group.set_pose_target(waypoint)
            plan_result = move_group.plan()
            
            if isinstance(plan_result, tuple):
                success, plan, planning_time, error_code = plan_result
                # Only accept exact solutions
                if error_code.val != MoveItErrorCodes.SUCCESS:
                    print(f"  Planning failed or approximate solution (error_code={error_code.val}), trying next...")
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
            print(f"Planning to {name} failed with all roll values!")
            return

        ## --------------------
        ## Visualize the Plan in RViz
        ## --------------------
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        if plan and hasattr(plan, 'joint_trajectory'):
            display_trajectory.trajectory.append(plan)
            print(f"Publishing trajectory to RViz for {name}...")
            display_trajectory_publisher.publish(display_trajectory)
            print(f"Trajectory published for {name}. Waiting 3 seconds for visualization...")
            rospy.sleep(3)  # Shorter pause for waypoints
        else:
            print(f"Plan for {name} does not have a joint_trajectory. Skipping visualization.")

        ## --------------------
        ## Execute the Plan
        ## --------------------
        print(f"Executing plan to {name}...")
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        print(f"Completed waypoint {i+1}: {name}")
        
        # Small pause between waypoints
        if i < len(waypoints) - 1:  # Don't pause after the last waypoint
            rospy.sleep(1)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

def setup_environment():
    # Initialize moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('environment_setup', anonymous=True)
    rospy.sleep(2)  # Wait for MoveIt! and robot state to be available

    # Instantiate objects for robot, scene, and group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    print("Setting up environment (table, walls, ceiling, ruler)...")

    ## --------------------
    ## Add Table Obstacle (lowered height)
    ## --------------------
    print("Adding table obstacle at lowered height...")
    
    # Create a collision object for the table
    table = CollisionObject()
    table.header.frame_id = "base"
    table.id = "table_obstacle"
    table.primitives = []
    table.primitive_poses = []
    
    # Create a box primitive for the table
    table_primitive = SolidPrimitive()
    table_primitive.type = SolidPrimitive.BOX
    table_primitive.dimensions = [0.75, 0.59, 0.05]  # length, width, height
    
    # Set the table pose
    table_pose = geometry_msgs.msg.Pose()
    table_pose.position.x = 0.6
    table_pose.position.y = 0.0
    table_pose.position.z = -0.075
    table_pose.orientation.x = 0.0
    table_pose.orientation.y = 0.0
    table_pose.orientation.z = 0.707
    table_pose.orientation.w = 0.707
    
    table.primitives.append(table_primitive)
    table.primitive_poses.append(table_pose)
    scene.add_object(table)
    print("Table obstacle added successfully!")

    ## --------------------
    ## Add Ceiling Obstacle
    ## --------------------
    print("Adding ceiling obstacle...")
    
    ceiling = CollisionObject()
    ceiling.header.frame_id = "base"
    ceiling.id = "ceiling_obstacle"
    ceiling.primitives = []
    ceiling.primitive_poses = []
    
    ceiling_primitive = SolidPrimitive()
    ceiling_primitive.type = SolidPrimitive.BOX
    ceiling_primitive.dimensions = [2.0, 2.0, 0.1]
    
    ceiling_pose = geometry_msgs.msg.Pose()
    ceiling_pose.position.x = 0.0
    ceiling_pose.position.y = 0.0
    ceiling_pose.position.z = 0.8
    ceiling_pose.orientation.w = 0.0
    
    ceiling.primitives.append(ceiling_primitive)
    ceiling.primitive_poses.append(ceiling_pose)
    scene.add_object(ceiling)
    print("Ceiling obstacle added successfully!")

    ## --------------------
    ## Add Safety Walls
    ## --------------------
    print("Adding safety walls...")
    
    # Left wall
    left_wall = CollisionObject()
    left_wall.header.frame_id = "base"
    left_wall.id = "left_safety_wall"
    left_wall.primitives = []
    left_wall.primitive_poses = []
    
    left_wall_primitive = SolidPrimitive()
    left_wall_primitive.type = SolidPrimitive.BOX
    left_wall_primitive.dimensions = [1.6, 0.1, 0.9]
    
    left_wall_pose = geometry_msgs.msg.Pose()
    left_wall_pose.position.x = 0.0
    left_wall_pose.position.y = -0.6
    left_wall_pose.position.z = 0.4
    left_wall_pose.orientation.w = 1.0
    
    left_wall.primitives.append(left_wall_primitive)
    left_wall.primitive_poses.append(left_wall_pose)
    scene.add_object(left_wall)
    
    # Right wall
    right_wall = CollisionObject()
    right_wall.header.frame_id = "base"
    right_wall.id = "right_safety_wall"
    right_wall.primitives = []
    right_wall.primitive_poses = []
    
    right_wall_primitive = SolidPrimitive()
    right_wall_primitive.type = SolidPrimitive.BOX
    right_wall_primitive.dimensions = [1.6, 0.1, 0.9]
    
    right_wall_pose = geometry_msgs.msg.Pose()
    right_wall_pose.position.x = 0.0
    right_wall_pose.position.y = 0.6
    right_wall_pose.position.z = 0.4
    right_wall_pose.orientation.w = 1.0
    
    right_wall.primitives.append(right_wall_primitive)
    right_wall.primitive_poses.append(right_wall_pose)
    scene.add_object(right_wall)
    
    # Back wall
    back_wall = CollisionObject()
    back_wall.header.frame_id = "base"
    back_wall.id = "back_safety_wall"
    back_wall.primitives = []
    back_wall.primitive_poses = []
    
    back_wall_primitive = SolidPrimitive()
    back_wall_primitive.type = SolidPrimitive.BOX
    back_wall_primitive.dimensions = [0.1, 2.0, 0.9]
    
    back_wall_pose = geometry_msgs.msg.Pose()
    back_wall_pose.position.x = -0.4
    back_wall_pose.position.y = 0.0
    back_wall_pose.position.z = 0.4
    back_wall_pose.orientation.w = 1.0
    
    back_wall.primitives.append(back_wall_primitive)
    back_wall.primitive_poses.append(back_wall_pose)
    scene.add_object(back_wall)
    
    rospy.sleep(0.5)
    print("Safety walls added successfully!")

    ## --------------------
    ## Add Ruler Object
    ## --------------------
    print("Adding ruler object...")
    ruler = CollisionObject()
    ruler.header.frame_id = "base"
    ruler.id = "ruler_object"
    ruler.primitives = []
    ruler.primitive_poses = []

    ruler_primitive = SolidPrimitive()
    ruler_primitive.type = SolidPrimitive.BOX
    ruler_primitive.dimensions = [0.20, 0.02, 0.01]

    ruler_pose = geometry_msgs.msg.Pose()
    ruler_pose.position.x = table_pose.position.x - table_primitive.dimensions[0]/2 + ruler_primitive.dimensions[0]/2 -0.12
    ruler_pose.position.y = table_pose.position.y
    ruler_pose.position.z = table_pose.position.z + table_primitive.dimensions[2]/2 + ruler_primitive.dimensions[2]/2
    ruler_pose.orientation.w = 1.0

    ruler.primitives.append(ruler_primitive)
    ruler.primitive_poses.append(ruler_pose)
    scene.add_object(ruler)
    rospy.sleep(0.5)
    print("Ruler object added successfully!")

    print("Environment setup complete!")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    setup_environment() 
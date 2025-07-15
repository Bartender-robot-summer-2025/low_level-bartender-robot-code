#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import MoveItErrorCodes

def test_planner_selection():
    """Test different planners and compare their path lengths"""
    
    # Initialize moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test_planner_selection', anonymous=True)
    rospy.sleep(2)

    # Instantiate objects for robot, scene, and group
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("right_arm")

    # Set longer planning time for better optimization
    move_group.set_planning_time(3.0)

    # Define a simple test pose
    test_pose = geometry_msgs.msg.Pose()
    test_pose.position.x = 0.4
    test_pose.position.y = -0.3
    test_pose.position.z = 0.2
    test_pose.orientation.w = 1.0

    # List of planners to test
    planners_to_test = [
        "RRTstarkConfigDefault",      # RRT* - optimal but slower
        "RRTConnectkConfigFast",      # RRT Connect with larger range
        "RRTConnectkConfigDefault",   # RRT Connect default
        "RRTkConfigDefault",          # Basic RRT
        "ESTkConfigDefault",          # EST
    ]

    print("Testing different planners for path length optimization...")
    print("=" * 60)

    for planner_id in planners_to_test:
        try:
            print(f"\nTesting planner: {planner_id}")
            move_group.set_planner_id(planner_id)
            
            # Set the test pose as target
            move_group.set_pose_target(test_pose)
            
            # Plan
            plan_result = move_group.plan()
            
            if isinstance(plan_result, tuple):
                success, plan, planning_time, error_code = plan_result
                if error_code.val == MoveItErrorCodes.SUCCESS and plan and hasattr(plan, 'joint_trajectory'):
                    # Calculate path length
                    path_length = 0.0
                    trajectory = plan.joint_trajectory
                    
                    if len(trajectory.points) > 1:
                        for i in range(1, len(trajectory.points)):
                            prev_point = trajectory.points[i-1]
                            curr_point = trajectory.points[i]
                            
                            # Calculate Euclidean distance in joint space
                            joint_diff = 0.0
                            for j in range(len(prev_point.positions)):
                                joint_diff += (curr_point.positions[j] - prev_point.positions[j])**2
                            joint_diff = joint_diff**0.5
                            path_length += joint_diff
                    
                    print(f"  ✓ Success! Planning time: {planning_time:.2f}s")
                    print(f"  ✓ Path length: {path_length:.4f} (joint space)")
                    print(f"  ✓ Trajectory points: {len(trajectory.points)}")
                else:
                    print(f"  ✗ Failed with error code: {error_code.val}")
            else:
                plan = plan_result
                if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
                    print(f"  ✓ Success! (older API)")
                    print(f"  ✓ Trajectory points: {len(plan.joint_trajectory.points)}")
                else:
                    print(f"  ✗ Failed - no valid trajectory")
                    
        except Exception as e:
            print(f"  ✗ Exception: {e}")

    print("\n" + "=" * 60)
    print("Planner test completed!")
    print("RRT* should generally produce shorter paths but takes longer to plan.")
    print("RRT Connect with larger range should be faster and produce reasonable paths.")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    test_planner_selection() 
#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest


def main():
    # Initialize moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_sawyer_pose_goal_with_ik', anonymous=True)

    # Instantiate objects for robot, scene, and group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Define the target pose (same as moveit_sawyer_pose_goal.py)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.6385084314068623
    pose_goal.position.y = -0.09689558101951124
    pose_goal.position.z = 0.20095189628247253
    pose_goal.orientation.x = 0.6316418939967514
    pose_goal.orientation.y = 0.27267087736516693
    pose_goal.orientation.z = 0.6589547421237953
    pose_goal.orientation.w = 0.3040686735223963

    # Prepare IK request
    rospy.wait_for_service('compute_ik')
    ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = group_name
    ik_request.ik_request.pose_stamped.header.frame_id = move_group.get_planning_frame()
    ik_request.ik_request.pose_stamped.pose = pose_goal
    ik_request.ik_request.timeout = rospy.Duration(5)
    #ik_request.ik_request.attempts = 10
    ik_request.ik_request.avoid_collisions = True

    # Optionally, seed with current joint state
    current_joint_state = robot.get_current_state().joint_state
    ik_request.ik_request.robot_state.joint_state.name = current_joint_state.name
    ik_request.ik_request.robot_state.joint_state.position = current_joint_state.position

    # Call IK service
    try:
        ik_response = ik_service(ik_request)
        if ik_response.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            joint_names = ik_response.solution.joint_state.name
            joint_positions = ik_response.solution.joint_state.position
            print("IK Solution found:")
            for name, pos in zip(joint_names, joint_positions):
                print(f"  {name}: {pos}")
            # Set joint value target and plan (filter for right_arm joints only)
            right_arm_joint_names = move_group.get_active_joints()
            ik_joint_state = ik_response.solution.joint_state
            ik_joint_dict = dict(zip(ik_joint_state.name, ik_joint_state.position))
            right_arm_joint_values = [ik_joint_dict[name] for name in right_arm_joint_names if name in ik_joint_dict]
            move_group.set_joint_value_target(right_arm_joint_values)
            plan = move_group.plan()
            if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
                print("Executing plan...")
                move_group.go(wait=True)
                print("Motion executed.")
            else:
                print("Planning failed after IK solution.")
        else:
            print(f"IK failed with error code: {ik_response.error_code.val}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

    move_group.stop()
    move_group.clear_pose_targets()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main() 
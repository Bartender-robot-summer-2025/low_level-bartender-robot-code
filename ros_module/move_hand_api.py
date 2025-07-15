#!/usr/bin/env python3
"""
move_hand_api.py

This module provides functions to open and close the LEAP hand via ROS.
- Intended to be imported and called from another Python script.
- Provides open_hand(pub=None) and close_hand(pub=None) functions.
- Optionally, you can call main() to test interactively.

Usage:
    from move_hand_api import open_hand, close_hand, get_publisher
    pub = get_publisher()
    open_hand(pub)
    close_hand(pub)

If pub is not provided, each function will create its own publisher (less efficient for repeated calls).
"""
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time

def get_publisher():
    return rospy.Publisher("/leaphand_node/cmd_leap", JointState, queue_size=1)

def send_leap_hand_positions(sim_joint_positions, pub=None):
    if pub is None:
        pub = get_publisher()
        rospy.sleep(1)  # Wait for publisher to connect
    # Convert to LEAP hand (real hardware) convention by adding pi to each value
    leap_joint_positions = (np.array(sim_joint_positions) + np.pi)
    # Define the min and max for each joint in LEAP hand convention
    LEAPsim_min = np.array([-1.047, -0.314, -0.506, -0.366,
                            -1.047, -0.314, -0.506, -0.366,
                            -1.047, -0.314, -0.506, -0.366,
                            -0.349, -0.47, -1.20, -1.34])
    LEAPsim_max = np.array([1.047, 2.23, 1.885, 2.042,
                            1.047, 2.23, 1.885, 2.042,
                            1.047, 2.23, 1.885, 2.042,
                            2.094, 2.443, 1.90, 1.88])
    leap_min = LEAPsim_min + np.pi
    leap_max = LEAPsim_max + np.pi
    # Clip the joint positions to the allowed range
    leap_joint_positions = np.clip(leap_joint_positions, leap_min, leap_max).tolist()
    msg = JointState()
    msg.position = leap_joint_positions
    pub.publish(msg)
    print("Published LEAP hand joint positions to /leaphand_node/cmd_leap:", msg.position)

def open_hand(pub=None):
    open_positions = [0.0] * 16
    send_leap_hand_positions(open_positions, pub)

def close_hand(pub=None):
    # Use the moderately increased closed position from move_sim_hand.py
    base_closed = [0,0.487,1.19,0.778, 0,0.487,1.43,0.344, 0.0314,0.754,0.988,0.669]
    LEAPsim_max = [1.047, 2.23, 1.885, 2.042, 1.047, 2.23, 1.885, 2.042, 1.047, 2.23, 1.885, 2.042]
    closed_positions = [min(val + 0.3, mx) for val, mx in zip(base_closed, LEAPsim_max)] + [1.59, 0.262, 0.287, 1.07]
    send_leap_hand_positions(closed_positions, pub)

# Optional: allow running as a script for testing
if __name__ == "__main__":
    rospy.init_node("move_hand_api_test")
    pub = get_publisher()
    rospy.sleep(1)
    print("Press Enter to toggle the hand (open/close). Type 'q' + Enter to quit.")
    is_closed = False
    while not rospy.is_shutdown():
        user_input = input()
        if user_input.strip().lower() == 'q':
            print("Exiting.")
            break
        if is_closed:
            open_hand(pub)
            is_closed = False
            print("Hand opened.")
        else:
            close_hand(pub)
            is_closed = True
            print("Hand closed.") 
#!/usr/bin/env python3
"""
move_sim_hand.py

This script sends joint position commands to the LEAP hand via ROS.
- Toggles the hand between open (default) and closed positions each time you press Enter.
- Uses your custom closed position from the script, and all-zeros for open.
- Clips all joint commands to safe hardware limits before sending.
- Publishes to the /leaphand_node/cmd_leap topic as sensor_msgs/JointState.

Usage:
    1. Start ROS core and the leaphand_node (e.g., via example.launch).
    2. Run this script: python3 move_sim_hand.py
    3. Press Enter to toggle the hand (open/close). Type 'q' + Enter to quit.

Notes:
    - Requires the leap_hand ROS package and its services to be running.
    - Adjust 'closed_positions' as needed for your application.
"""
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time
import argparse
import socket
import re
import subprocess

def generate_waypoints(start, end, num_waypoints):
    start = np.array(start)
    end = np.array(end)
    return [((1 - alpha) * start + alpha * end).tolist() for alpha in np.linspace(0, 1, num_waypoints + 1)[1:]]

def generate_waypoints_with_thumb_priority(start, end, num_waypoints):
    """
    Generate waypoints with thumb priority:
    - When CLOSING: thumb joint 12 moves first, then joints 13-15
    - When OPENING: thumb joints 13-15 move first, then joint 12 moves last
    """
    start = np.array(start)
    end = np.array(end)
    waypoints = []
    
    # Detect if we're opening (moving toward zeros) or closing (moving away from zeros)
    # Check if the average of thumb joints is moving toward zero (opening) or away from zero (closing)
    start_thumb_avg = np.mean(start[12:16])
    end_thumb_avg = np.mean(end[12:16])
    is_opening = end_thumb_avg < start_thumb_avg  # Moving toward zero = opening
    
    # Generate waypoints for joints 0-11 (non-thumb joints)
    for alpha in np.linspace(0, 1, num_waypoints + 1)[1:]:
        waypoint = np.zeros(16)
        # Interpolate joints 0-11 normally
        waypoint[0:12] = (1 - alpha) * start[0:12] + alpha * end[0:12]
        
        if is_opening:
            # When OPENING: joints 13-15 move first, then joint 12 moves with more waypoints
            if alpha <= 0.5:  # First 50%: joints 13-15 move
                waypoint[12] = start[12]  # Joint 12 stays at start
                beta = alpha / 0.5  # Remap alpha from [0,0.5] to [0,1]
                waypoint[13:16] = (1 - beta) * start[13:16] + beta * end[13:16]
            else:  # Last 50%: joint 12 moves with more waypoints
                waypoint[13:16] = end[13:16]  # Joints 13-15 stay at target
                joint12_alpha = (alpha - 0.5) / 0.5  # Remap alpha from [0.5,1] to [0,1]
                waypoint[12] = (1 - joint12_alpha) * start[12] + joint12_alpha * end[12]
        else:
            # When CLOSING: joint 12 moves first, then joints 13-15
            if alpha <= 0.6:  # First 60%: joint 12 moves
                joint12_alpha = alpha / 0.6  # Remap alpha from [0,0.6] to [0,1]
                waypoint[12] = (1 - joint12_alpha) * start[12] + joint12_alpha * end[12]
                waypoint[13:16] = start[13:16]  # Joints 13-15 stay at start
            else:  # Last 40%: joints 13-15 move
                waypoint[12] = end[12]  # Joint 12 stays at target
                beta = (alpha - 0.6) / 0.4  # Remap alpha from [0.6,1] to [0,1]
                waypoint[13:16] = (1 - beta) * start[13:16] + beta * end[13:16]
        
        waypoints.append(waypoint.tolist())
    
    return waypoints

def send_leap_hand_positions(sim_joint_positions, pub):
    print("Sending LEAP hand joint positions to /leaphand_node/cmd_leap:", sim_joint_positions)
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

# Helper function to extract Sensor 2 temperature from UDP message
def extract_sensor2_temp(text):
    # Find the Sensor 2 section
    sensor2_match = re.search(r"Sensor 2:(.*?)(Sensor 1:|$)", text, re.DOTALL)
    if sensor2_match:
        sensor2_block = sensor2_match.group(1)
        #print("Sensor 2 block:", repr(sensor2_block))  # Debug print
        temp_match = re.search(r"Temp:\s*([\d.\-]+)", sensor2_block)
        if temp_match:
            return float(temp_match.group(1))
    return None

def check_grabbed_from_udp(port=12345, timeout=2.0, num_checks=4, check_temp_after_grab=False):
    """
    Check UDP sensor data multiple times. Return True if any check succeeds.
    If check_temp_after_grab is True, also check the rate of change of sensor 2's temperature over 1 second after grab.
    """
    grabbed = False
    for check_num in range(num_checks):
        print(f"UDP check {check_num + 1}/{num_checks}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)
        try:
            sock.bind(("127.0.0.1", port))
            data, _ = sock.recvfrom(4096)
            text = data.decode()
            # Parse the pressure values from the text
            pressures = [float(x) for x in re.findall(r"Pressure \d+: ([\d.]+)", text)]
            # There are 10 pressure values (5 per sensor)
            s1_pressures = pressures[:5]
            s2_pressures = pressures[5:10]
            # Count nonzero pressures for each sensor
            s1_nonzero = sum(1 for p in s1_pressures if p > 0)
            s2_nonzero = sum(1 for p in s2_pressures if p > 0)
            grabbed = s1_nonzero >= 1 or s2_nonzero >= 1  # Changed from AND to OR
            print(f"  Sensor 1 nonzero: {s1_nonzero}, Sensor 2 nonzero: {s2_nonzero}")
            if grabbed:
                print("  Hand grabbed something!")
                print("yes")
                if check_temp_after_grab:
                    # --- Temperature check for sensor 2 ---
                    # Take first reading
                    temp1 = None
                    sock.settimeout(1.0)  # instead of 0.5
                    for _ in range(10):   # instead of 5
                        try:
                            data, _ = sock.recvfrom(4096)
                            text = data.decode()
                            temp1 = extract_sensor2_temp(text)
                            if temp1 is not None:
                                print(f"Sensor 2 initial temperature: {temp1}")
                                break
                        except socket.timeout:
                            continue
                    if temp1 is None:
                        print("Could not get initial temperature reading for sensor 2.")
                        return True

                    # Wait 10 seconds
                    wait_time = 3.0

                    print(f"Waiting {wait_time} seconds before taking second temperature reading...")
                    time.sleep(wait_time)

                    # Flush any old UDP messages
                    sock.settimeout(0.1)
                    flush_start = time.time()
                    while time.time() - flush_start < 1.0:
                        try:
                            sock.recvfrom(4096)
                        except socket.timeout:
                            break

                    # Now get the first new message after the wait
                    temp2 = None
                    sock.settimeout(2.0)
                    for _ in range(10):
                        try:
                            data, _ = sock.recvfrom(4096)
                            text = data.decode()
                            temp2 = extract_sensor2_temp(text)
                            if temp2 is not None:
                                print(f"Sensor 2 temperature after {wait_time}s: {temp2}")
                                break
                        except socket.timeout:
                            continue
                    if temp2 is None:
                        print("Could not get second temperature reading for sensor 2.")
                        return True

                    rate = (temp2 - temp1) / wait_time
                    print(f"Rate of change: {rate:.2f} deg/sec")
                    if -20 < rate < 0:
                        print("Robot has grabbed something cold")
                    else:
                        print("Temperature change does not indicate a cold object.")
                return True
            else:
                print("  Hand did NOT grab anything.")
        except socket.timeout:
            print(f"  No sensor data received (timeout) on check {check_num + 1}.")
        finally:
            sock.close()
        # Small delay between checks (except on last check)
        if check_num < num_checks - 1:
            time.sleep(0.1)
    # If we get here, all checks failed
    print("All UDP checks failed - no grasp detected.")
    print("no")
    return False

def main():
    parser = argparse.ArgumentParser(description="Send open, close, or waypoint_close command to LEAP hand, or toggle interactively if no argument.")
    parser.add_argument('command', nargs='?', choices=['open', 'close', 'waypoint_close', 'waypoint_open'], help="Command: 'open', 'close', 'waypoint_close', or 'waypoint_open'. If omitted, toggles interactively.")
    parser.add_argument('--num_waypoints', type=int, default=15, help="Number of waypoints for waypoint_close/waypoint_open (default: 20)")
    parser.add_argument('--waypoint_delay', type=float, default=0.05, help="Delay (seconds) between waypoints (default: 0.05)")
    args = parser.parse_args()

    rospy.init_node("move_leap_hand")
    pub = rospy.Publisher("/leaphand_node/cmd_leap", JointState, queue_size=1)
    rospy.sleep(1)  # Wait for publisher to connect
    # Define open (default) and closed hand positions in LEAPsim convention
    open_positions = [0.0] * 16
    base_closed = [0,0.757,1.19,0.878, 0,0.587,1.43,0.544, 0,0.754,1.188,0.769]
    LEAPsim_max = [1.047, 2.23, 1.885, 2.042, 1.047, 2.23, 1.885, 2.042, 1.047, 2.23, 1.885, 2.042]
    closed_positions = [min(val, mx) for val, mx in zip(base_closed, LEAPsim_max)] + [1.59, 0.262, 0.287, 1.07]  # Thumb unchanged

    if args.command == 'open':
        send_leap_hand_positions(open_positions, pub)
        print("Hand opened.")
        return
    elif args.command == 'close':
        send_leap_hand_positions(closed_positions, pub)
        print("Hand closed.")
        time.sleep(0.5)  # Wait before checking sensors
        # Check for grab and temperature
        check_grabbed_from_udp(port=12345, check_temp_after_grab=True)
        return
    elif args.command == 'waypoint_close':
        waypoints = generate_waypoints_with_thumb_priority(open_positions, closed_positions, args.num_waypoints)
        print(f"Moving hand to closed position using {args.num_waypoints} waypoints...")
        for i, pos in enumerate(waypoints):
            send_leap_hand_positions(pos, pub)
            print(f"  Sent waypoint {i+1}/{len(waypoints)}")
            time.sleep(args.waypoint_delay)
        print("Hand closed with waypoints.")
        time.sleep(0.5)  # Wait before checking sensors
        # Check for grab and temperature
        check_grabbed_from_udp(port=12345, check_temp_after_grab=True)
        return
    elif args.command == 'waypoint_open':
        waypoints = generate_waypoints_with_thumb_priority(closed_positions, open_positions, args.num_waypoints)
        print(f"Moving hand to open position using {args.num_waypoints} waypoints...")
        for i, pos in enumerate(waypoints):
            send_leap_hand_positions(pos, pub)
            print(f"  Sent waypoint {i+1}/{len(waypoints)}")
            time.sleep(args.waypoint_delay)
        print("Hand opened with waypoints.")
        return

    # Interactive mode if no argument is given
    is_closed = False
    print("\nPress Enter to toggle the hand (open/close) using waypoints. Type 'q' + Enter to quit.")
    while not rospy.is_shutdown():
        user_input = input()
        if user_input.strip().lower() == 'q':
            print("Exiting.")
            break
        if is_closed:
            waypoints = generate_waypoints_with_thumb_priority(closed_positions, open_positions, args.num_waypoints)
            print(f"Moving hand to open position using {args.num_waypoints} waypoints...")
            for i, pos in enumerate(waypoints):
                send_leap_hand_positions(pos, pub)
                print(f"  Sent waypoint {i+1}/{len(waypoints)}")
                time.sleep(args.waypoint_delay)
            is_closed = False
            print("Hand opened.")
        else:
            waypoints = generate_waypoints_with_thumb_priority(open_positions, closed_positions, args.num_waypoints)
            print(f"Moving hand to closed position using {args.num_waypoints} waypoints...")
            for i, pos in enumerate(waypoints):
                send_leap_hand_positions(pos, pub)
                print(f"  Sent waypoint {i+1}/{len(waypoints)}")
                time.sleep(args.waypoint_delay)
            is_closed = True
            print("Hand closed.")
            time.sleep(0.5)  # Wait before checking sensors
            # Check for grab and temperature after closing in interactive mode
            check_grabbed_from_udp(port=12345, check_temp_after_grab=True)

if __name__ == "__main__":
    main() 
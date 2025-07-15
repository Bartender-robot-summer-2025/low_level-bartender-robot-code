#!/usr/bin/env python3
"""
ik_control.py

Load the Allegro-hand URDF, receive fingertip & vector data over UDP,
apply de-roll + scaling transforms, solve per-finger inverse kinematics each frame,
and display both raw and transformed fingertip positions in MeshCat.
"""
import os
import time
import threading
import socket
import json
import math
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from scipy.optimize import minimize
import meshcat.geometry as g
import meshcat.transformations as tf
import rospy
from sensor_msgs.msg import JointState

# === Paths (adjust if needed) ===
model_path = os.path.abspath("/home/atrc234/PinnoMeshHand/leap_hand/stl")
urdf_path  = os.path.join(model_path,
                          "/home/atrc234/PinnoMeshHand/leap_hand/robot.urdf")


# Build model
robot = RobotWrapper.BuildFromURDF(urdf_path, [model_path])
model, data = robot.model, robot.data
joint_names = model.names[1:]  # ✅ this gives joint names in order, skipping "universe"

# Joint limits (16 DOFs)
dof_lower = np.array([
    # index joints
    -0.314, -1.047, -0.506, -0.366,
    # thumb joints
    -0.349, -0.470, -1.200, -1.340,
    # middle joints
    -0.314, -1.047, -0.506, -0.366,
    # ring joints
    -0.314, -1.047, -0.506, -0.366,
])

dof_upper = np.array([
    # index joints
     2.230,  1.047,  1.885,  2.042,
    # thumb joints
     2.094,  2.443,  1.900,  1.880,
    # middle joints
     2.230,  1.047,  1.885,  2.042,
    # ring joints
     2.230,  1.047,  1.885,  2.042,
])

# Fingertip definitions: slices and frames
finger_defs = {
    "thumb":  {"slice": slice(4, 8),   "frame": "thumb_fingertip"},
    "index":  {"slice": slice(0, 4),   "frame": "fingertip"},
    "middle": {"slice": slice(8, 12),  "frame": "fingertip_2"},
    "ring":   {"slice": slice(12, 16), "frame": "fingertip_3"},
}
for name, f in finger_defs.items():
    f["lower"]    = dof_lower[f["slice"]]
    f["upper"]    = dof_upper[f["slice"]]
    f["frame_id"] = model.getFrameId(f["frame"])

# Visualization setup
viz = robot
viz.initViewer(loadModel=True)
viz.viewer.open()
# Shift entire hand so palm center is at origin

# Create MeshCat spheres for raw and transformed fingertips
for name in finger_defs.keys():
    viz.viewer[f"raw_{name}"].set_object(
        g.Sphere(0.005),
        g.MeshLambertMaterial(color=[0, 0, 1])
    )
    viz.viewer[f"target_{name}"].set_object(
        g.Sphere(0.005),
        g.MeshLambertMaterial(color=[1, 0, 0])
    )

class UDPReceiver(threading.Thread):
    """
    Threaded UDP receiver: listens for fingertip, vector, and wrist data.
    Stores parsed data in instance attributes.
    """
    def __init__(self, ip="0.0.0.0", port=12345):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.setblocking(False)
        self.targets = [np.zeros(3) for _ in range(4)]
        self.vector  = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.wrist   = {"x": 0.0, "y": 0.0, "z": 0.0}

    def run(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                payload = json.loads(data.decode('utf-8'))
                fp = payload.get('fingertips', [])
                ordered = []
                for name in finger_defs.keys():
                    match = next((d for d in fp if d.get('finger','').lower()==name), None)
                    if match:
                        ordered.append(np.array([match['x'], match['y'], match['z']]))
                    else:
                        ordered.append(np.zeros(3))
                self.targets = ordered
                self.vector  = payload.get('vector', self.vector)
                self.wrist   = {
                    'x': payload.get('wrist_x', self.wrist['x']),
                    'y': payload.get('wrist_y', self.wrist['y']),
                    'z': payload.get('wrist_z', self.wrist['z'])
                }
            except BlockingIOError:
                pass
            except json.JSONDecodeError:
                print("Invalid JSON received")
            time.sleep(0.001)


def solve_finger_ik(target, fdef):
    """
    Solve IK for a single finger to reach `target` (3D point).
    """
    sl, fid = fdef['slice'], fdef['frame_id']
    lower, upper = fdef['lower'], fdef['upper']
    def cost(q_l):
        q_all = np.zeros(model.nq)
        q_all[sl] = q_l
        pin.forwardKinematics(model, data, q_all)
        pin.updateFramePlacement(model, data, fid)
        pos = data.oMf[fid].translation
        return np.linalg.norm(pos - target)
    res = minimize(cost, x0=np.zeros(len(lower)), method='SLSQP',
                   bounds=list(zip(lower, upper)), options={'ftol':1e-6,'maxiter':80})
    return np.clip(res.x, lower, upper)


def send_leap_hand_positions(sim_joint_positions, pub):
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


def main():
    rospy.init_node("move_leap_hand")
    print("IK controller listening on UDP port 12345")
    receiver = UDPReceiver()
    receiver.start()
    pub = rospy.Publisher("/leaphand_node/cmd_leap", JointState, queue_size=1)
    rospy.sleep(1)  # Wait for publisher to connect
    try:
        while True:
            vx, vy = receiver.vector['x'], receiver.vector['y']
            theta = math.atan2(vy, vx)
            c, s = math.cos(theta), math.sin(theta)
            q_out = np.zeros(model.nq)
            for idx, (name, fdef) in enumerate(finger_defs.items()):
                rel_x, rel_y, rel_z = receiver.targets[idx]
                # Plot raw fingertip
                raw_x = rel_x / (.17/800)
                raw_y = rel_y / (.17/800)
                raw_z = rel_z / 50.0
                
                # De-roll + scale
                dx = rel_x - vx
                dy = (rel_y - vy)
                rot_x = dx*c + dy*s
                rot_y = -dx*s + dy*c
                x3 = rot_x * (.17/100)
                y3 = rot_y *(.17/100)
                z3 = rel_z / 50.0
                # if name=="thumb":
                #     y3=y3-0
                #     x3=x3-.2
                # else:
                #     y3=y3+.025
                #     x3=x3-.11
                # z3=z3+.20
            
                # else:
                """
                x -.20
                ring x -.10
                middle x -.1
                index -.1
                y points become more positive x points become more negative correctly, 
                y+.05
                """
                x3 = rot_x *(.17/160)
                y3 = rot_y *(.17/160)
                correction_angle_deg = 15  # adjust as needed
                theta_corr = math.radians(correction_angle_deg)
                cos_theta = math.cos(theta_corr)
                sin_theta = math.sin(theta_corr)

                x_rot = x3 * cos_theta - y3 * sin_theta
                y_rot = x3 * sin_theta + y3 * cos_theta
                x3, y3 = x_rot, y_rot
                z3=z3+.18
                z3=z3*1.5
                if name=="thumb":
                    x3=x3-.05
                    y3=y3-.025
                    x3=x3*1.25
                    y3=y3*1.4
                   # y3=y3-.04
                else:    
                    x3=x3-.05
                    y3=y3-.025
                    y3=y3*1.25
                    x3=x3*1.25
                print(name)
                print(x3)
                print(y3)
                print(z3)
                viz.viewer[f"raw_{name}"].set_transform(
                tf.translation_matrix([x3, y3, z3]))
                tgt = np.array([x3, y3, z3])
                # IK and display
                q_out[fdef['slice']] = solve_finger_ik(tgt, fdef)
                viz.viewer[f"target_{name}"].set_transform(
                    tf.translation_matrix(tgt.tolist())
                )
        
            viz.display(q_out)
            print("\n=== Live Joint Angles (radians) ===")
            for name, angle in zip(joint_names, q_out):
                print(f"{name:20s}: {angle:+.4f}")
            # Collect all joint angles into a list and print in one line for easy copy-paste
            joint_angle_list = [float(angle) for angle in q_out]
            print("Joint angles as list:", joint_angle_list)
            print("Space-separated:", " ".join(f"{a:.6f}" for a in joint_angle_list))

            # Remap from IK order to LEAP hand ROS node order
            remapped = [
                joint_angle_list[1],  joint_angle_list[0],  joint_angle_list[2],  joint_angle_list[3],   # Index
                joint_angle_list[9],  joint_angle_list[8],  joint_angle_list[10], joint_angle_list[11],  # Middle
                joint_angle_list[13], joint_angle_list[12], joint_angle_list[14], joint_angle_list[15],  # Ring
                joint_angle_list[4],  joint_angle_list[5],  joint_angle_list[6],  joint_angle_list[7],   # Thumb
            ]
            send_leap_hand_positions(remapped, pub)
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\nIK control stopped.")




if __name__ == "__main__":
    main()

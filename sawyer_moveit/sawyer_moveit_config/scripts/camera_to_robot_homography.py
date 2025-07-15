#!/usr/bin/env python3

import numpy as np
import cv2
import json
import argparse

# Load calibration data from JSON file
def load_calibration_data(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    image_points = []
    world_points = []
    for entry in data['calibration_data']:
        u = entry['camera_coordinates']['u']
        v = entry['camera_coordinates']['v']
        x = entry['robot_coordinates']['x']
        y = entry['robot_coordinates']['y']
        image_points.append([u, v])
        world_points.append([x, y])
    return np.array(image_points, dtype=np.float32), np.array(world_points, dtype=np.float32)

# Compute homography from calibration data
def compute_homography(image_points, world_points):
    H, status = cv2.findHomography(image_points, world_points)
    return H

# Map a new camera (u, v) to robot (x, y) using homography
def camera_to_robot(u, v, H):
    pt_img = np.array([[u, v, 1]], dtype=np.float32).T  # shape (3, 1)
    pt_world = np.dot(H, pt_img)
    pt_world /= pt_world[2]  # Normalize
    x, y = pt_world[0, 0], pt_world[1, 0]
    return x, y

# Command-line interface for testing
def main():
    parser = argparse.ArgumentParser(description="Map camera (u,v) to robot (x,y) using homography.")
    parser.add_argument('--calib', type=str, required=True, help='Path to calibration JSON file')
    parser.add_argument('--u', type=float, help='Camera u (pixel x)')
    parser.add_argument('--v', type=float, help='Camera v (pixel y)')
    args = parser.parse_args()

    image_points, world_points = load_calibration_data(args.calib)
    H = compute_homography(image_points, world_points)
    print("Homography matrix:")
    print(H)

    if args.u is not None and args.v is not None:
        x, y = camera_to_robot(args.u, args.v, H)
        print(f"Camera (u, v): ({args.u:.2f}, {args.v:.2f}) => Robot (x, y): ({x:.3f}, {y:.3f})")
    else:
        print("No (u, v) provided. Only computed homography.")

if __name__ == '__main__':
    main() 
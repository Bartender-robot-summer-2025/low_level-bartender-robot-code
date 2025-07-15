#!/usr/bin/env python3
import subprocess
import json
import time
from datetime import datetime

# Define bottle_x and bottle_y ranges
bottle_x_values = [0.4, 0.5, 0.6, 0.7, 0.8]  # 5 values
bottle_y_values = [-0.3, 0.0, 0.3]           # 3 values

# Generate grid of points (or you can randomize or use fewer points)
points = [(x, y) for x in bottle_x_values for y in bottle_y_values]

results = []

for idx, (bottle_x, bottle_y) in enumerate(points):
    print(f"\n=== Calibration Point {idx+1}/{len(points)} ===")
    print(f"Moving bottle to robot coordinates: x={bottle_x:.3f}, y={bottle_y:.3f}")
    # Call the MoveIt script
    ret = subprocess.run([
        "python3", "scripts/moveit_sawyer_pose_goal_with_args.py",
        "--bottle_x", str(bottle_x),
        "--bottle_y", str(bottle_y)
    ])
    if ret.returncode != 0:
        print(f"Warning: Script failed for (x={bottle_x}, y={bottle_y})")
    # Prompt for camera coordinates
    while True:
        try:
            camera_x = float(input("Enter camera_x (pixel or normalized): "))
            camera_y = float(input("Enter camera_y (pixel or normalized): "))
            break
        except ValueError:
            print("Invalid input. Please enter numeric values.")
    results.append({
        "robot_bottle_x": bottle_x,
        "robot_bottle_y": bottle_y,
        "camera_x": camera_x,
        "camera_y": camera_y
    })
    print(f"Saved: robot=({bottle_x:.3f}, {bottle_y:.3f}), camera=({camera_x}, {camera_y})")

# Save results to JSON
filename = f"bottle_camera_calibration_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
with open(filename, 'w') as f:
    json.dump(results, f, indent=2)
print(f"\nCalibration data saved to {filename}")

# Optionally, print a summary or fit
print("\nSummary of collected data:")
for entry in results:
    print(entry) 
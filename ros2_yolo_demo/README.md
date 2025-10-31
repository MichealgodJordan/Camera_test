# ROS2 YOLOv8 + Depth -> 3D Demo (with RViz, Tracking, Performance Monitor)

This package demonstrates an integrated ROS2 pipeline:
- Intel RealSense (realsense2_camera) publishes color, depth and camera_info.
- `yolo_tracker` performs YOLOv8 detection and a simple tracker (centroid-based fallback).
- `depth_to_3d_rviz` converts tracked 2D detections to 3D coordinates using depth and camera intrinsics,
  publishes `visualization_msgs/MarkerArray` for RViz visualization and `/detected_objects_3d`.
- `performance_monitor` logs FPS, CPU and memory to `/system_metrics` and `/tmp/yolo_metrics.csv`.

## What you get in this zip
- ROS2 package `yolov8_depth_demo` with three nodes and a launch file that starts realsense, nodes and RViz.
- RViz config (simple).
- README with run instructions.

## Requirements (install on your robot/development PC)
- ROS2 Humble or later
- realsense2_camera ROS2 package
- Python packages: ultralytics, opencv-python, numpy, cv_bridge, psutil
  Install via:
  ```bash
  pip install ultralytics opencv-python numpy psutil
  sudo apt install ros-${ROS_DISTRO}-cv-bridge
  ```
  Optional: filterpy for advanced tracker:
  ```bash
  pip install filterpy
  ```

## How to run
1. Build your workspace:
   ```bash
   cd ~/ros2_ws
   cp -r /path/to/yolov8_depth_demo .
   colcon build
   source install/setup.bash
   ```
2. Launch:
   ```bash
   ros2 launch yolov8_depth_demo vision_demo_with_rviz.launch.py
   ```
3. Open RViz (should be launched by the launch file). Add the `MarkerArray` display with topic `/detection_markers`
   to see spheres at detected 3D object positions. Also you can `ros2 topic echo /detected_objects_3d` to view JSON.

## Notes and Tips
- RealSense depth frames may be in millimeters (uint16) or meters (float32) depending on your driver settings.
  Adjust the code if needed (the demo tries to use the value as-is).
- If you prefer ByteTrack/DeepSORT, replace the SimpleCentroidTracker with your chosen tracker (dependencies required).
- The included tracker is intentionally simple for portability. For production, use a Kalman/Hungarian tracker.
- The performance monitor writes logs to `/tmp/yolo_metrics.csv`. Use the included `plot_metrics.py` to visualize.

## Included utility: plot_metrics.py
A simple script to read `/tmp/yolo_metrics.csv` and plot FPS, CPU and Memory over time using matplotlib.

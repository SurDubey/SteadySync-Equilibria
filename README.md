# SteadySync-Equilibria

## Overview

This project implements **visual servoing** for a **Kinova Gen3 7-DOF robotic arm** in **ROS 1 Noetic**. The robot uses **image-based visual servoing (IBVS)** to detect and track a rectangular object using its **end-effector camera**. The system extracts **corner points** of the rectangle and adjusts the robot's position dynamically to align with a predefined reference.

## Features

- **Real-time Rectangle Detection**: Uses **ORB feature detection and RANSAC filtering** for robust corner point detection.
- **ROS Integration**: Subscribes to **camera image topics**, processes frames in **OpenCV**, and publishes **detected features**.
- **Trajectory Control**: Uses **FollowJointTrajectoryAction** to move the Kinova Gen3 arm.
- **Visual Servoing Loop**: Computes **position corrections** based on detected rectangle corner deviations.
- **Gazebo Simulation Support**: Can run in both **real hardware** and **Ignition Gazebo**.

## System Architecture

```
+------------------------------------------------------+
|                      ROS Framework                   |
+------------------------------------------------------+
|   Image Capture    |   Feature Detection   | Control |
|  (Camera Topic)    | (ORB + RANSAC)       | (MoveIt!)|
+------------------------------------------------------+
|                   Kinova Gen3 Robot Arm             |
+------------------------------------------------------+
```

## Dependencies

Ensure you have the following installed:

- **ROS 1 Noetic**
- **MoveIt!**
- **OpenCV (cv\_bridge, image\_transport)**
- **Kinova ROS packages** (`kortex_driver`, `kortex_examples`)
- **Gazebo Fortress/Garden** (for simulation)

### Installation

```bash
# Clone your workspace and install dependencies
cd ~/Desktop/robot_ws/src
git clone https://github.com/your-repo/your-project.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Running the Project

### 1. **Launch Gazebo Simulation**

```bash
roslaunch kortex_examples gazebo3.launch
```

### 2. **Start the Visual Servoing Node**

```bash
rosrun kortex_examples visual_servoing.py
```

### 3. **View Processed Images**

```bash
rqt_image_view /edge_detected_image
```

### 4. **Move Robot to Initial Position**

```bash
rostopic pub /move_robot std_msgs/Empty {}
```

## How it Works

1. **Capturing Reference Image**: The arm moves to **position-1** and captures a reference image.
2. **Real-time Processing**: The camera continuously captures frames and detects the largest rectangle.
3. **Feature Extraction**: ORB detects corner points, filtered using **RANSAC** for robustness.
4. **Error Computation**: Compares current rectangle corners with the reference and computes an error vector.
5. **Robot Adjustment**: Adjusts the arm’s position iteratively to align with the reference.
6. **Convergence**: The process stops once the alignment error falls below a threshold.

## Expected Results

- The robot should **accurately align** with the rectangle even with slight perturbations.
- Robust corner detection should work even in **varying lighting conditions**.

## Future Improvements

- **Refine ORB & RANSAC parameters** for better robustness.
- Implement **deep learning-based corner detection** for improved accuracy.
- Extend to **6-DOF pose estimation** instead of just 2D alignment.

## Authors

- **Your Name** – *Surabhi Dwivedi, Deepraj Majumdar*
- **Institution/Organization** – *Indian Institute of Technology, Jodhpur*

---

⭐ **If you find this project useful, give it a star!** ⭐


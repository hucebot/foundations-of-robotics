# mobile_robotics

Lab material for the ***Foundations of Robotics*** practical session - **Mobile Robotics** module.

This repository provides a hands-on introduction to mobile robot simulation, sensing, and reactive navigation using a simulated **TurtleBot3 Burger** in **Gazebo Classic** with **ROS 2**. The exercises progressively build from basic simulator exploration and visualization to manual control and goal-driven navigation behaviors based on odometry, TF transforms, and 2D LiDAR data.

---

## Exercise overview

The goal of this lab is to familiarize students with core ROS 2 workflows for mobile robotics (topics, messages, TF, visualization) and to implement foundational navigation behaviors for a differential-drive robot:

- exploring and understanding the simulation interfaces (Gazebo + ROS 2 topics),
- visualizing robot state, TF frames, and sensor data in RViz2,
- implementing **keyboard teleoperation** by publishing velocity commands (`/cmd_vel`),
- implementing **point-to-point navigation** with a simple FSM using TF-based pose estimation and a go-to-goal controller,
- extending navigation with **reactive obstacle avoidance** via the **Bug 2** algorithm using 2D LiDAR (`/scan`) and M-line logic.

---

## Repository structure

The repository is organized as follows:

- **`exercise_navigation/`**  
  ROS 2 Python package containing the exercises and launch files used throughout the lab.
  - `keyboard_teleop.py`  
    Keyboard teleoperation node (uses `pynput`) publishing `geometry_msgs/msg/Twist` commands to `/cmd_vel`.
  - `point2point_nav.py`  
    Point-to-point navigation node built around a simple two-state FSM (IDLE / GO_TO_GOAL), using TF to estimate robot pose and publishing commands on `/cmd_vel`.
  - `bug_nav.py`  
    Bug 2 navigation node extending point-to-point navigation with wall-following based on `/scan` and M-line rejoin conditions.
  - `tb3_rviz2.launch.py`  
    Launch file that starts RViz2 with pre-configured displays for common TurtleBot3 topics (TF, RobotModel, LaserScan).

- **`run_docker.sh`**  
  Convenience script to start the Docker container and mount the `exercise_navigation` folder into the ROS 2 workspace inside the container.

---

## Getting the Docker image

A pre-built Docker image is available for this tutorial.

Pull the image with:

```bash
docker pull ghcr.io/fabio-amadio/turtlebot3-sim:2026
```

---

## Running the container

After the image has been downloaded, start the container and open a `terminator`  terminal inside it by running:

```bash
bash run_docker.sh
```

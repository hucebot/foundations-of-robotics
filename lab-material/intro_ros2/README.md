# intro_ros2

Lab material for the ***Foundations of Robotics*** practical session — **Intro to ROS 2** module.

This repository provides a first hands-on introduction to ROS 2 using **Python (`rclpy`)**. The tutorial walks through the core building blocks of ROS 2 applications—nodes, timers, publishers/subscribers, parameters, and launch files—by implementing a minimal pub/sub system and running it as a multi-node application.

---

## Lab overview

The goal of this lab is to become familiar with the ROS 2 programming model and basic developer workflow by implementing:

- a **publisher node** that periodically publishes `std_msgs/String` messages on a topic,
- a **subscriber node** that listens to the same topic and logs received messages,
- **ROS 2 parameters** to configure topic names and timing at runtime,
- a **launch file** to start and configure multiple nodes from a single command.

The exercises also highlight essential tooling for inspecting ROS 2 systems (e.g., topic introspection and graph visualization).

---

## Repository structure

The repository is organized around a single ROS 2 Python package:

- **`exercise_pub_sub/`**  
  ROS 2 Python package used for all exercises in this tutorial.
  - `exercise_pub_sub/pub_node.py`  
    Minimal publisher node (to be completed) publishing `std_msgs/String` on a configurable topic at a configurable period.
  - `exercise_pub_sub/sub_node.py`  
    Minimal subscriber node (to be completed) subscribing to the same topic and printing received messages.
  - `launch/pub_sub.launch.py`  
    Launch file (to be completed) that starts both nodes and passes launch arguments as node parameters.
  - Standard ROS 2 / Python package files (`package.xml`, `setup.py`, `setup.cfg`, `resource/`, `test/`, etc.).

- **`run_docker.sh`**  
  Convenience script to start the Docker container and mount `exercise_pub_sub` into the ROS 2 workspace inside the container.

---

## Getting the Docker image

A pre-built Docker image is available for this tutorial.

Pull the image with:

```bash
docker pull ghcr.io/fabio-amadio/intro-ros2:2026
```

---

## Running the container

After the image has been downloaded, start the container and open a `terminator` terminal inside it by running:

```bash
bash run_docker.sh
```

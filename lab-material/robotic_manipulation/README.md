# robot_manipulation

Lab material for the ***Foundations of Robotics*** practical session — **Robotic Manipulation** module.

This repository provides a self-contained simulation-based lab focused on Cartesian control and trajectory generation for a torque-controlled robotic manipulator. All experiments run inside a Docker container and use a simulated Franka Emika Panda robot in the MuJoCo physics engine, controlled via ROS 2.

---

## Exercise overview

The goal of this lab is to bridge theory and practice in robotic manipulation by having students implement and test:

- a **Cartesian impedance controller** in task space, including orientation control, null-space regulation, and reference filtering;
- a **linear Cartesian trajectory planner** that generates smooth position and orientation references using interpolation and SLERP.

Through these components, the lab illustrates key concepts such as task-space control, redundancy resolution, compliant behavior, and real-time reference tracking, all within a reproducible simulation environment.

---

## Repository structure

The repository is organized as follows:

- **`franka_example_controllers/`**  
  ROS 2 C++ package containing example controllers for the Franka Panda robot.
  - `ros2_control` plugin `ExerciseCartesianImpedanceController`  
    Skeleton implementation of a Cartesian impedance controller to be completed by the student.  
    It includes:
    - header files defining the controller interface, gains, and internal state,
    - source files implementing task-space control, null-space torques, and reference filtering.

- **`franka_simple_publishers/`**  
  ROS 2 Python package providing simple reference publishers.
  - `exercise_pose_publisher.py`  
    A linear Cartesian trajectory planner that publishes interpolated end-effector poses to the controller.

- **`run_docker.sh`**  
  Convenience script to start the Docker container and mount the repository into the ROS 2 workspace inside the container.

---

## Getting the Docker image

A pre-built Docker image is available for this tutorial.

Pull the image with:

```bash
docker pull ghcr.io/fabio-amadio/panda-ros2:2026
```

---

## Running the container

After the image has been downloaded, start the container and open a `terminator` terminal inside it by running:

```bash
bash run_docker.sh
```

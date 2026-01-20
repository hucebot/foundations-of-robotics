# lab-material

Lab material for the ***Foundations of Robotics*** practical sessions (*École des Mines de Nancy*, 2026).

This repository collects all **code, simulation environments, and supporting material** used in the practical sessions of the *Foundations of Robotics* course.  
It includes three hands-on laboratory modules covering ROS 2 fundamentals, mobile robotics, and robotic manipulation.

A **detailed, step-by-step guide to the practical sessions** is provided in [**Lab-notes.pdf**](Lab-notes.pdf), which should be considered the primary reference during the labs.

<table>
  <tr>
    <td width="50%">
      <video src="https://github.com/user-attachments/assets/084db2c7-10d7-412e-8d61-c911f514b09c" controls muted playsinline style="width:100%; height:auto;"></video>
    </td>
    <td width="50%">
      <video src="https://github.com/user-attachments/assets/eedb09ed-d2c6-4c21-8885-e0bd58a195fa" controls muted playsinline style="width:100%; height:auto;"></video>
    </td>
  </tr>
</table>

## Docker images

All labs run inside pre-configured Docker containers to ensure a consistent and reproducible environment.

Before starting any lab, pull the required images:

```bash
docker pull ghcr.io/fabio-amadio/intro-ros2:2026        # for the "Intro to ROS 2" module
docker pull ghcr.io/fabio-amadio/turtlebot3-sim:2026    # for the "Mobile Robotics" module
docker pull ghcr.io/fabio-amadio/panda-ros2:2026        # for the "Robotic Manipulation" module
```

Each lab directory contains a `run_docker.sh` script that launches the corresponding container and mounts the lab workspace automatically.

## Getting started

Work through the modules in the following order:

1. [`intro_ros2`](intro_ros2/README.md)
2. [`mobile_robotics`](mobile_robotics/README.md)
3. [`robotic_manipulation`](robotic_manipulation/README.md)

For each module:

- navigate to the corresponding module folder,
- use the provided `run_docker.sh` script to start the lab environment,
- consult the relevant section of [**Lab-notes.pdf**](Lab-notes.pdf) while working through the exercises.

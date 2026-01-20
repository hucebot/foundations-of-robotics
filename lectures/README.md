# lectures

Lecture slides and code examples for the ***Foundations of Robotics*** course (*École des Mines de Nancy*, 2026).

## Installation

To run the examples, it is recommended to build the provided `Dockerfile` (which has been tested in Ubuntu only).
To install Docker please check [this](https://docs.docker.com/engine/install/) website.

## Build the DockerFile

To build the `DockerFile`, run the following command in the terminal:

```docker build -t introduction_to_robotics .```

This command builds a Docker image named introduction_to_robotics.

## Run the Docker Image

To run the Docker image you can use the script `run_docker.sh`:

`./run_docker.sh`

This script runs the Docker image named introduction_to_robotics and mounts the lecture folders containing the example code into the `introduction_to_robotics` directory.

You can also run `terminator` in the first terminal window to open a `Terminator` session, which allows you to run multiple terminals in parallel.

# lectures

Lecture slides and code examples for the ***Foundations of Robotics*** course (*École des Mines de Nancy*, 2026).

## Getting the Docker image

A pre-built Docker image is available for this tutorial.

Pull the image with:

```bash
docker pull ghcr.io/fabio-amadio/robotics-lectures:2026
```

## Running the container

## Jupyter Notebooks

For Lectures 1, 2, 3, and 5, launch the Docker container and start Jupyter with:

```bash
bash run_jupyter.sh
```

This command starts Jupyter inside the container (with token/password disabled) and
mounts the lecture folders into `/home/introduction_to_robotics`. Keep the terminal open.

Open a browser on your host and go to:

```
http://localhost:8888
```

The Jupyter interface will open in your browser and list the lecture notebooks available in the container.

Lecture-specific instructions:

- [Lecture 1](lecture1/README.md)
- [Lecture 2](lecture2/README.md)
- [Lecture 3](lecture3/README.md)
- [Lecture 5](lecture5/README.md)

## Interactive session

For Lecture 4, launch the Docker container opening a terminal inside it with:

```bash
bash run_docker.sh
```

Lecture-specific instructions: [Lecture 4](lecture4/README.md)

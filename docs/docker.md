# Titan Simulation (Docker)

## Overview

The Titan Simulation Docker provides a ready-to-use environment for running Titan Robot simulation without installing ROS 2, Gazebo, or dependencies on the host system. It is intended for quick demonstrations, behavior validation, and first-time exploration of the Titan navigation stack.

---

## Running the Docker Image

Run the following commands on the host system:
```bash
xhost +local:root

sudo docker run -it \
--net=host \
--device=/dev/dri \
-e DISPLAY=$DISPLAY \
-e QT_X11_NO_MITSHM=1 \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
yashnair02/titan_docker:final
```

### What This Command Does

- Pulls the Docker image automatically if it does not already exist on the system
- Enables GUI forwarding so Ignition Gazebo and RViz2 can display on the host
- Uses host networking for seamless ROS 2 and Gazebo communication
- Provides GPU access for hardware-accelerated rendering

---

## Container Startup Behavior

When the container starts, a script automatically launches a **tmux session** with **four panes in a single terminal window**.

The layout is:

- Top-left pane: Simulation bringup
- Bottom-left pane: Navigation stack
- Top-right pane: Idle (user commands)
- Bottom-right pane: Idle (user commands)

---

## Automatic Launch Sequence

Inside the container:

- The Titan simulation bringup is launched automatically
- Ignition Gazebo starts with the warehouse world
- The robot is spawned in simulation
- Nav2 is launched with simulation time enabled

At this point, the user only needs to:

- Open RViz2
- Set an initial pose if required
- Send a **2D Goal Pose**

The robot will immediately begin autonomous navigation.  
This setup is intended to quickly demonstrate robot behavior without manual setup.

---

## Custom Usage

If users want to experiment further, they can:

- Stop running nodes in any tmux pane
- Launch SLAM for mapping
- Load custom maps
- Tune navigation parameters

This Docker environment supports the full simulation workflow described in the [Simulation documentation](https://mrs111-os.github.io/titan_docs/simulation/).

---

## Navigating tmux Panes

To move between tmux panes:

- Press Ctrl + b, then use arrow keys to switch panes

Additional useful commands:

- Ctrl + b, then d → Detach from tmux
- tmux attach → Reattach to the session
- Ctrl + d → Close a pane

---

## Further Changes

To make further changes in the docker image:
1. Run the image and make changes in the container.
2. List the cuurent containers
```bash
sudo docker ps
```
3. Note the container ID
4. Then ```docker commit container_id username/repository_name:tag_name``` . This creates a new image called username/repository_name:tag_name
5. If you want to push the image to your personal docker hub, ```docker push username/repository_name:tag_name``` 

## Use Case Summary

This Docker setup is ideal for:

- Quick demonstrations
- Behavior visualization
- Testing navigation without hardware
- Sharing a reproducible simulation environment

No local ROS 2 or Gazebo installation is required.

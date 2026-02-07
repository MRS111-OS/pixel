# Setup & Installation 🤖

## Overview

This section explains how to build, launch, and verify the complete software stack of **Pixel Robot**.  
Robot bringup refers to starting all essential components required for the robot to operate correctly, including sensors, motor control, state publishing, and visualization.

---

## Prerequisites

Before proceeding, ensure that:

- Ubuntu 22.04 is installed
- ROS 2 Humble is installed and sourced
- The robot hardware is fully assembled
- Battery is charged and power system is functional
- ESP32 firmware is already flashed
- All sensors are properly connected

---

## Update system packages and install essential tools

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl wget htop net-tools openssh-server rsync tmux python3 python3-pip python3-rosdep gazebo git python3-colcon-common-extensions terminator
sudo apt install ros-humble-rviz2 ros-humble-slam-toolbox ros-humble-turtlebot3-gazebo ros-humble-joint-state-publisher-gui ros-humble-gazebo-ros-pkgs
```

## One time setup

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/titan_ws/src/titan_simulation/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc
echo 'source ~/titan_ws/install/setup.bash' >> ~/.bashrc
```

## Workspace Setup

Create and initialize a ROS 2 workspace:

```bash
mkdir -p ~/titan_ws
cd ~/titan_ws
```

Clone the Pixel Robot repository:
```bash
git clone https://github.com/MRS111-OS/titan_robot.git
mv titan_robot/ src
```

Install dependencies
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
```

Build the workspace:
```bash
cd ~/titan_ws
colcon build --symlink-install --parallel-workers 4
```

Source the workspace:
```bash
source install/setup.bash
```


> It is recommended to add the source command to your `.bashrc` for convenience.

---

## Bringup Launch Description

Pixel Robot uses a single bringup launch file that initializes all core components.

The bringup launch typically starts:

- Robot description (URDF)
- State publisher
- LIDAR driver
- ESP32 serial communication node
- Static and dynamic TFs
---

## Launching the Robot

Power on the robot and run the bringup launch file:

```bash
ros2 launch titan_bringup titan_bringup.launch.py
```


Once launched, the following should occur:

- LIDAR begins publishing scan data
- ESP32 establishes serial communication
- Odometry is published to `/odom`
- TF tree is available
- RViz2 opens with a preconfigured view

---

## Verifying Bringup

After launching, verify that all core components are running correctly.

### Check Active Topics

```bash
ros2 topic list
```


You should see topics such as:

- `/scan`
- `/odom`
- `/cmd_vel`
- `/tf`
- `/tf_static`

---

### Verify Odometry
```bash
ros2 topic echo /odom
```

- Values should update when the robot moves
- Position and orientation should change smoothly

---

### Verify LIDAR Data
```bash
ros2 topic echo /scan
```

Or visualize directly in RViz2.

---

### Verify TF Tree

```bash
ros2 run tf2_tools view_frames
```


Confirm that frames such as `odom`, `base_link`, and `laser` are present.

---

## Common Bringup Issues

| Issue | Possible Cause |
|-----|---------------|
| No odometry | ESP32 not connected or firmware not running |
| No LIDAR data | USB connection or driver issue |
| Robot not moving | Motor power or serial issue |
| RViz not showing data | Incorrect fixed frame or TF missing |

---

## Keyboard Teleoperation

Keyboard teleoperation is useful for testing hardware, mapping environments, and manual control.

### Run Keyboard Teleop

Open a new terminal and source the workspace:

```bash
source ~/titan_ws/install/setup.bash
```


Run the teleop node:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

### Teleop Controls

```bash
i Forward
k Stop
j Rotate Left
l Rotate Right
, Backward

q/z Increase/Decrease Linear Speed
w/x Increase/Decrease Angular Speed
```


Velocity commands are published to the `/cmd_vel` topic and sent to the ESP32.

---

## Safety Notes

- Always keep the robot in an open area during teleoperation
- Start with low velocity limits
- Be ready to stop the robot immediately
- Do not lift the robot while motors are powered


Your robot is now up and running 🚀

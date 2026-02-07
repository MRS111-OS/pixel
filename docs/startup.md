## On-Boot System Monitor (OLED Display)

When the robot powers on and the bringup sequence starts, the **OLED screen automatically turns ON**.

The OLED acts as a real-time system monitor and displays essential runtime information:

- **WiFi SSID** – the network the robot is currently connected to  
- **IP Address** – the robot’s local IP (useful for SSH, ROS networking, debugging)
- **Memory Usage** – used vs total system RAM  
- **CPU Load** – current CPU load average  

This allows you to quickly verify that:
- The robot is connected to WiFi
- Networking is working correctly
- The system is not overloaded

![OLED system info](images/oled.png)

> The OLED starts automatically at boot and requires no manual command.

---

## Default TMUX Session (PIXEL)

Once the robot starts, a **tmux session named `PIXEL` is already running automatically**.

This session organizes all critical robot processes into structured panes so you don’t have to manually launch everything.

**Attach to this tmux session by entering the following command :** 
```bash 
tmux attach-session -t PIXEL
```

To know more about Tmux and the service that is launching it as robot powers on, please refer to the additional information page of this documentation. 

---

## TMUX Session Layout

The `PIXEL` session is divided into multiple panes with the following responsibilities:

---

**1st Pane (Red)**

``` bash 
ros2 launch titan_bringup titan_bringup.launch.py
```

This launches:
- Robot description (URDF)
- State publisher
- ESP32 serial communication
- Sensors (LIDAR, encoders, etc.)
- Core TF tree

**2nd Pane (yellow)**

```bash 
ros2 launch nav2_bringup bringup_launch.py map:=/home/titan/titan_ws/src/titan_bringup/map/map2.yaml
```

This runs **Nav2 in localization mode** using a prebuilt map.

**3rd Pane (green)**

This pane is reserved for **SLAM Toolbox**.

If you want to **generate a new map**:

1. Stop the Nav2 bringup (localization)
2. Run SLAM Toolbox:

ros2 launch slam_toolbox online_async_launch.py

3. Drive the robot and build a new map

**4th Pane (orange)**

```bash 
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Used to manually move the robot with the keyboard.

**5th Pane (blue)**

```bash 
ros2 run oled_pkg oled_ip_pi4.py
```

This node updates the OLED with WiFi, IP, memory, and CPU stats.

**6th Pane (purple)**

- Empty pane reserved for:
  - Debugging commands
  - ROS topic inspection
  - Launching extra nodes

---

![TMUX PIXEL session](images/tmux.png)

---

## Useful TMUX Commands

### Attach to the PIXEL session

```bash 
tmux attach-session -t PIXEL
```

### Detach from tmux (leave session running)

Ctrl + b then d

### Switch between panes

Ctrl + b then Arrow Keys

### Switch between windows

Ctrl + b then n (next window)
Ctrl + b then p (previous window)

### Enable mouse support (recommended)

```bash 
tmux set -g mouse on
```

---

With the OLED system monitor and the preconfigured `PIXEL` tmux session, the robot is **fully observable and controllable immediately after boot**  
No manual setup is required—just power on and start working.

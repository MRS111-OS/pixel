# Introduction to tmux

## What is tmux?

**tmux** (terminal multiplexer) is a command-line tool that allows you to run multiple terminal sessions inside a single terminal window. It enables you to create, manage, and switch between multiple shells, all while keeping them running in the background.

With tmux, your terminal sessions stay alive even if:
- Your SSH connection drops
- You close the terminal accidentally
- Your system goes to sleep

This makes tmux especially powerful for developers, system administrators, and anyone working on remote machines.

---

## Why Developers Use tmux

Developers rely on tmux because it significantly improves productivity and workflow.

### 1. Persistent Sessions
You can detach from a tmux session and reattach later without losing running programs (editors, servers, scripts).

### 2. Multiple Panes and Windows
tmux lets you split a terminal into multiple panes and windows, allowing you to:
- Run code in one pane
- Monitor logs in another
- Edit files in a third

All at the same time.

### 3. Ideal for Remote Development
tmux is widely used on servers via SSH. Even if your network disconnects, your work continues safely on the server.

### 4. Lightweight and Fast
tmux runs entirely in the terminal and consumes very little system resources compared to GUI-based tools.

### 5. Keyboard-Driven Workflow
Once learned, tmux allows extremely fast navigation and control using keyboard shortcuts.

---

## Basic tmux Concepts

Before commands, it helps to understand tmux terminology:

- **Session** → A collection of windows (entire workspace)
- **Window** → Like a tab in a browser
- **Pane** → A split section inside a window

---

## Starting and Managing tmux

### Start tmux
```bash
tmux
```

### Start a named session
```bash
tmux new -s mysession
```

### List running sessions
```bash
tmux ls
```

### Attach to an existing session
```bash
tmux attach -t mysession
```

### Detach from a session (without closing it)
Press:
```
Ctrl + b, then d
```

---

## Basic tmux Commands

> **Note:** Most tmux shortcuts start with the **prefix key**  
> Default prefix: `Ctrl + b`

### Session Commands
| Action | Shortcut |
|------|---------|
| Detach session | `Ctrl + b d` |
| Rename session | `Ctrl + b $` |
| Kill session | `tmux kill-session` |

---

### Window Commands
| Action | Shortcut |
|------|---------|
| Create new window | `Ctrl + b c` |
| Next window | `Ctrl + b n` |
| Previous window | `Ctrl + b p` |
| List windows | `Ctrl + b w` |
| Rename window | `Ctrl + b ,` |
| Close window | `Ctrl + b &` |

---

### Pane Commands
| Action | Shortcut |
|------|---------|
| Split vertically | `Ctrl + b %` |
| Split horizontally | `Ctrl + b "` |
| Move between panes | `Ctrl + b` + arrow key |
| Resize pane | `Ctrl + b` + hold arrow key |
| Close pane | `Ctrl + b x` |

---

## Example Developer Workflow

A common tmux setup:
- **Window 1:** Code editor (vim / nano)
- **Window 2:** Build or run commands
- **Window 3:** Logs or debugging output

All managed inside one terminal session.

---

## Service that automates Tmux once Robot is Powered On

In many robotics systems, tmux is used to **automatically start and manage robot software at boot time**. This ensures that core processes come up reliably without manual intervention after powering on the robot.

The setup used in this project relies on **systemd services + tmux sessions**.

### High-Level Idea

1. The robot boots up (Linux starts)
2. A `systemd` service is triggered automatically
3. The service launches a shell script
4. The shell script starts a tmux session
5. Inside tmux, robot bringup commands are executed

This allows developers to later SSH into the robot and **attach to the same tmux session** to observe logs or debug issues.

---

### systemd Service Role

A systemd service is responsible for starting tmux at boot.

Typical responsibilities:
- Run after networking is ready
- Execute as a specific user (not root)
- Restart automatically if it fails

Conceptually, the service runs a command similar to:

```bash
tmux new-session -d -s robot
```

This creates a **detached tmux session** named `robot` during boot.

---

### robot_bringup.sh Script Role

The bringup script is executed *inside* the tmux session. Its job is to:

- Source ROS 2 environment
- Source workspace overlays
- Launch core robot nodes
- Keep processes running in a controlled environment

A simplified flow looks like:

```bash
#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/pixel_ws/install/setup.bash

ros2 launch pixel_bringup robot.launch.py
```

This script becomes the **entry point** for robot startup.

---

### Why tmux is Used Here

Using tmux for robot bringup provides several advantages:

- **Persistence**: Processes keep running even if SSH disconnects
- **Debugging**: Developers can attach later using:
  ```bash
  tmux attach -t robot
  ```
- **Visibility**: Logs are immediately visible without `journalctl`
- **Manual recovery**: Panes or windows can be restarted independently

---

### Typical Debug Workflow

1. Power on the robot
2. systemd starts the tmux session automatically
3. Robot nodes launch inside tmux
4. Developer SSHs into the robot
5. Attach to the running session:
   ```bash
   tmux attach -t robot
   ```
6. Inspect logs, restart nodes, or split panes as needed


---


# Networking & Remote Access

## Overview

Networking and remote access allow you to control, monitor, and debug Titan Robot without directly connecting a keyboard and monitor.  
This section explains how to connect the robot to a network, access it remotely, and configure ROS 2 communication across machines.

Titan Robot supports:
- WiFi and Ethernet networking
- SSH-based remote access
- Remote RViz visualization
- Multi-machine ROS 2 setups

---

## Network Requirements

- Robot and control PC must be on the same network
- Stable WiFi connection recommended for visualization and debugging

---

## Connecting the Robot to WiFi

### Using Network Manager (Recommended)

List available WiFi networks:
```bash
nmcli device wifi list
```

Connect to a WiFi network:
```bash
nmcli device wifi connect "YOUR_SSID" password "YOUR_PASSWORD"
```
Verify connection:
```bash
ip addr
```

![ip](images/ip.png)

Look for a valid IP address assigned to the wireless interface.

---

## Finding the Robot’s IP Address

On the robot, run:
```bash
hostname -I
```
This will display the robot’s IP address.

---

### Finding the Robot from Another Device

If you do not know the IP address, scan the network:
```bash
sudo apt install nmap  
sudo nmap -sn 192.168.1.0/24
```
Look for the device corresponding to the robot.

---

## SSH Remote Access

SSH allows you to access the robot’s terminal remotely.

### Enable SSH (if not already enabled)
```bash
sudo systemctl enable ssh  
sudo systemctl start ssh
```
---

### Connecting via SSH

From your laptop or desktop:
```bash
ssh <username>@<robot_ip_address>
```
Example:
```bash
ssh titan@192.168.1.42
```
Once connected, you can run ROS 2 commands as if you were physically on the robot.

---

## Remote ROS 2 Development

### Sourcing ROS 2

After logging in via SSH, ensure ROS 2 is sourced:
```bash
source /opt/ros/humble/setup.bash  
source ~/titan_ws/install/setup.bash
```
---

## Remote RViz2 Usage

RViz2 can be run either:
- On the robot
- On a remote PC connected over the network

Running RViz2 remotely is recommended to reduce CPU load on the robot.

---

### Running RViz2 on a Remote PC

Ensure:
- ROS 2 Humble is installed on the remote PC
- The same workspace (or compatible message definitions) is available

Set the same ROS 2 domain ID on both machines.

---

## ROS 2 Networking Configuration

ROS 2 uses DDS for communication and works automatically when machines are on the same network.

### ROS_DOMAIN_ID

Set the same domain ID on all machines:
```bash
export ROS_DOMAIN_ID=0
```
Add this to `.bashrc` on both robot and remote PC to persist.

---

### Verifying ROS 2 Communication

On the remote PC:
```bash
ros2 node list  
ros2 topic list
```
If nodes and topics from the robot appear, communication is working correctly.

---

## Multi-Machine ROS 2 Setup

Common setup:
- Robot runs sensors, control, and navigation
- Remote PC runs RViz2 and debugging tools

Ensure:
- Same ROS_DOMAIN_ID
- Same ROS 2 distribution
- Firewall does not block DDS traffic

---

## Firewall Considerations

If communication issues occur, check firewall settings:
```bash
sudo ufw status
```
If enabled, allow traffic or temporarily disable for testing:
```bash
sudo ufw disable
```
---

## Ethernet Connection (Optional)

For development and testing:
- Connect robot directly to a PC using Ethernet
- Assign IP addresses automatically or manually
- Provides lower latency and higher reliability than WiFi

---

## Security Best Practices

- Change default passwords
- Disable unused services
- Use SSH keys instead of passwords
- Avoid running robot on public networks

---

## Common Networking Issues

| Issue | Possible Cause | Solution |
|-----|---------------|---------|
| Cannot SSH | Wrong IP or SSH disabled | Verify IP and SSH service |
| ROS topics not visible | Different ROS_DOMAIN_ID | Set same domain ID |
| High latency | Weak WiFi signal | Move closer or use Ethernet |
| RViz lag | Running on robot | Run RViz on remote PC |

You can now fully operate and debug Titan Robot remotely 🚀

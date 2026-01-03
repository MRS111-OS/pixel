# Troubleshooting & FAQ

## Overview

This section provides guidance for diagnosing and resolving common issues encountered while using Titan Robot.  
It is intended as a quick reference during development, testing, and deployment.

The **Troubleshooting** section focuses on symptom-based diagnosis, while the **FAQ** addresses common conceptual and usage questions.

---

## Troubleshooting

### Robot Bringup Issues

| Symptom | Possible Cause | Recommended Action |
|-------|---------------|-------------------|
| Bringup launch fails | Workspace not built | Run colcon build and source the workspace |
| Nodes not starting | Missing dependencies | Install required ROS 2 packages |
| RViz2 does not open | Display issue or misconfiguration | Launch RViz manually and check logs |
| Robot description not visible | URDF not loaded | Verify robot_state_publisher is running |

---

### Motion & Motor Issues

| Symptom | Possible Cause | Recommended Action |
|-------|---------------|-------------------|
| Robot does not move | No velocity commands | Check /cmd_vel topic |
| Wheels move unevenly | PID not tuned | Tune motor PID parameters |
| Robot jerks or oscillates | Aggressive control gains | Reduce PID gains |
| Motors unresponsive | Power or wiring issue | Verify motor power and connections |

---

### Odometry & Localization Issues

| Symptom | Possible Cause | Recommended Action |
|-------|---------------|-------------------|
| Odometry not updating | Serial communication failure | Check ESP32 connection |
| Robot pose jumps | Incorrect initial pose | Reset pose in RViz |
| Drift at standstill | Encoder noise | Improve encoder filtering |
| Robot rotates in place | Incorrect wheel parameters | Verify wheel separation and radius |

---

### LIDAR & Sensor Issues

| Symptom | Possible Cause | Recommended Action |
|-------|---------------|-------------------|
| No LIDAR data | USB connection issue | Reconnect LIDAR and restart driver |
| Laser scan rotated | Incorrect TF | Verify LIDAR frame alignment |
| Incomplete scan | Sensor obstruction | Remove physical obstructions |
| No camera feed | Camera not detected | Check camera driver and permissions |

---

### Navigation Issues (Nav2)

| Symptom | Possible Cause | Recommended Action |
|-------|---------------|-------------------|
| No path generated | Localization not converged | Set initial pose again |
| Robot stuck | Obstacle inflation too large | Reduce inflation radius |
| Collisions | Incorrect footprint | Update robot footprint |
| Oscillations | Controller tuning issue | Tune Nav2 controller parameters |

---

### Networking & Remote Access Issues

| Symptom | Possible Cause | Recommended Action |
|-------|---------------|-------------------|
| Cannot SSH | SSH service disabled | Enable and start SSH |
| ROS topics not visible | Different ROS_DOMAIN_ID | Set same domain ID |
| RViz lag | Running RViz on robot | Run RViz remotely |
| High latency | Weak WiFi signal | Use Ethernet or improve WiFi |

---

## General Debugging Checklist

| Step | Action |
|----|-------|
| 1 | Check power and battery level |
| 2 | Verify robot bringup launched correctly |
| 3 | Confirm /scan and /odom are publishing |
| 4 | Check TF tree consistency |
| 5 | Verify Nav2 nodes are active |
| 6 | Test teleoperation |

---

## Frequently Asked Questions (FAQ)

### Q: The robot is powered on but not responding. What should I check first?
A: Verify battery voltage, motor power, and ensure the ESP32 firmware is running and connected.

---

### Q: Why does the robot drift even when standing still?
A: This is usually caused by encoder noise or incorrect PID tuning. Check encoder filtering and reduce control gains.

---

### Q: Do I need to remap the environment every time?
A: No. As long as the environment and sensor mounting remain unchanged, a saved map can be reused.

---

### Q: Can I run RViz2 on my laptop instead of the robot?
A: Yes. Running RViz2 remotely is recommended to reduce CPU load on the robot.

---

### Q: Why does Nav2 fail to generate a path?
A: Common causes include poor localization, incorrect costmap configuration, or an invalid initial pose.

---

### Q: Is it safe to run navigation without teleoperation testing?
A: No. Always verify basic teleoperation and sensor data before enabling autonomous navigation.

---

### Q: Can I add new sensors to the robot?
A: Yes. Titan Robot is designed to be modular and supports additional sensors via USB or GPIO.

---

### Q: How do I stop the robot immediately in case of an issue?
A: Use keyboard teleop to stop motion, power off the motors, or disconnect power if necessary.

---

## When to Seek Help

If an issue persists after troubleshooting:

- Check logs using rqt_console
- Verify configuration files
- Review recent changes
- Consult project documentation or community channels


This concludes the core documentation for Titan Robot 🚀

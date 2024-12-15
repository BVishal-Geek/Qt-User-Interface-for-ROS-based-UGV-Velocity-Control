# Robot Steering Control Application

## Overview
This project provides a Python application to control a ROS 1 robot acting as a master. The application offers an intuitive interface to monitor the robot's status, control its movements, and interact with its sensors. Designed for real-time robotic control, the application integrates essential features for enhanced usability and operational monitoring.

---

## Features
- **Battery Monitoring**: View the robot's current battery status and percentage.
- **Velocity Control**: Set and adjust the robot's velocity for precise movement.
- **Live Camera Feed**: Watch a real-time video stream from the robot's camera.
- **ROS Integration**: Seamlessly connects to a ROS 1 robot acting as the master.

---

## Requirements

### **Python Version**
- Python 3.8 or higher is recommended.

### **Dependencies**
Ensure you have the required Python libraries installed. Use the following command to install dependencies:
```bash
numpy
opencv-python
matplotlib
PyQt5
python-qt-binding

```
### Additional Setup for ROS
```bash
sudo apt install ros-<distro>-rospy ros-<distro>-geometry-msgs ros-<distro>-sensor-msgs ros-<distro>-std-msgs
```

### How to run
```bash
roscore
```
```bash
python3 robot_steering.py
```




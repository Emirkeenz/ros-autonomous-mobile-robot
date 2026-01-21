# UGV02 Obstacle Avoidance & Mapping (ROS)

Autonomous obstacle avoidance and SLAM mapping system for an unmanned ground vehicle (UGV) based on **ROS**, **RPLidar**, **Hector SLAM**, and **HTTP-based motor control via ESP**.

The project was developed as a **team-based robotics project**, with emphasis on modular architecture, stability, and clean separation between mapping and motion control.

---

## Robot Overview

The robot is an unmanned ground vehicle (UGV) equipped with:
- RPLidar A1 for environment perception
- ESP-based motor controller (HTTP interface)
- NVIDIA Jetson as ROS master
- Differential drive locomotion

The system supports three operating modes:
- STOP – motors stopped
- MAP – SLAM mapping without motion commands
- AUTO – autonomous obstacle avoidance

---

## System Architecture
RPLidar → /scan → Obstacle Avoidance Node  
├─ Decision logic  
├─ GO / TURN state machine  
└─ HTTP commands → ESP → Motors 

---

## 📁 Project Structure
robot_nav/
├── scripts/
│   ├── avoidance.py        # Obstacle avoidance logic
│   └── map_autosaver.py    # Automatic map saving
│
├── launch/
│   └── bringup.launch      # Full system startup
│
├── docs/
│   └── *.md                # Team documentation
│
├── README.md
└── requirements.txt

---

## Parameters (bringup.launch)

Main tuning parameters:

- safe_front – front obstacle distance (m)
- safe_side – side obstacle distance (m)
- fwd_speed – forward speed
- turn_speed – turning speed
- turn_time – fixed turn duration
- min_send_dt – minimum delay between HTTP commands
- cmd_hz – control loop frequency

All behavior tuning is done via launch parameters without modifying the code.

---

## Team Workflow

The project follows a GitHub-based team workflow:

- Each team member works in a separate feature branch
- Contributions are submitted via Pull Requests
- The main branch is merged by the project lead
- All changes are traceable via commit history

---

## Technologies Used

- ROS (Robot Operating System)
- Python 3
- RPLidar ROS driver
- Hector SLAM
- HTTP-based motor control
- Git & GitHub

---

## Dependencies & Installation

### Python (standard library)
The following modules are part of Python and do **not** require installation:
- math
- time

### Python packages (pip)
Required Python packages:
- requests

Install with:
```bash
pip3 install requests
```

ROS packages

Required ROS packages:

rospy

sensor_msgs

std_msgs

rplidar_ros

hector_mapping

hector_trajectory_server

tf

Install with:
```bash
sudo apt install ros-$ROS_DISTRO-rospy \
                 ros-$ROS_DISTRO-sensor-msgs \
                 ros-$ROS_DISTRO-std-msgs \
                 ros-$ROS_DISTRO-rplidar-ros \
                 ros-$ROS_DISTRO-hector-slam \
                 ros-$ROS_DISTRO-tf
```

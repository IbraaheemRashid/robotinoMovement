# Robotino Hardware Abstraction Layer (HAL) and Navigation Stack

This repository contains a ROS 2-based Hardware Abstraction Layer (HAL) for the Festo Robotino 3 platform, along with a complete navigation stack implementation using Nav2. The project aims to provide standardized access to the robot's sensors and actuators, enabling seamless integration with ROS 2's navigation capabilities.

## Table of Contents
- [Description](#description)
- [System Requirements](#system-requirements)
- [Installation](#installation)
  - [Robotino Setup](#robotino-setup)
  - [Development Machine Setup](#development-machine-setup)
- [Project Structure](#project-structure)
- [Hardware Abstraction Layer](#hardware-abstraction-layer)
  - [Battery Monitor](#battery-monitor)
  - [Motor Control](#motor-control)
  - [Sensor Monitor](#sensor-monitor)
  - [Odometry Monitor](#odometry-monitor)
  - [Camera Monitor](#camera-monitor)
  - [LiDAR Integration](#lidar-integration)
- [Navigation Stack](#navigation-stack)
  - [SLAM and Mapping](#slam-and-mapping)
  - [Nav2 Configuration](#nav2-configuration)
  - [Path Planning and Control](#path-planning-and-control)
- [Usage](#usage)
  - [Basic Control](#basic-control)
  - [Navigation](#navigation)
  - [Custom Command Examples](#custom-command-examples)
- [Automated Startup](#automated-startup)
- [Troubleshooting](#troubleshooting)

## Description

This project provides a comprehensive ROS 2 integration for the Festo Robotino 3 platform, focusing on creating:

1. A Hardware Abstraction Layer (HAL) that standardizes access to the robot's sensors and actuators
2. A complete navigation stack using Nav2 for autonomous navigation

The HAL interfaces with the Robotino API to provide ROS 2 topics for battery status, motor control, sensor data, odometry, and camera feed. The navigation stack utilizes SLAM for mapping and Nav2 for autonomous navigation.

## System Requirements

### Robotino Hardware
- Festo Robotino 3
- RPLiDAR A1M8 (or compatible LiDAR)
- Functioning batteries (robot needs to move untethered)

### Software
- Ubuntu 22.04 (on both Robotino and development machine)
- ROS 2 Humble Hawksbill
- Python 3.10+
- Robotino API2 (for interfacing with the robot)

## Installation

### Robotino Setup

1. Restore the Robotino operating system using the USB restore process:
   - If using the USB stick that shipped with Robotino, skip to step 6
   - Otherwise:
     1. Download the Robotino custom image
     2. Format a USB stick (quick format is sufficient)
     3. Use UNetbootin to install the custom image to the USB stick
     4. Download a Robotino3 image and its MD5 file (right-click and "save link" in Firefox)
     5. Check the integrity with `md5 -Check:imagefilename.fsa.md5`
   - Copy the image files (.fsa, .f01, etc.) to the `/images/` directory on your USB stick (if missing, copy to boot folder)
   - Connect monitor and keyboard to the Robotino
   - Insert the USB stick into a Robotino USB port
   - Ensure Robotino is connected to power supply
   - Power on Robotino and repeatedly press **F7** (for Robotino Professional with Core i5) or **F11** (for Robotino Basic with Atom) during BIOS startup
   - Select your USB stick from the list (not the UEFI entry)
   - Select Microcore from the list to start the automatic updater
   - After booting into tinycore, type `restore.sh` and press Enter to start the restore process

2. Install ROS 2 Foxy on the Robotino:
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-foxy-desktop
   ```

3. Install the Robotino API:
   ```bash
   # Download and install the Robotino API packages
   sudo apt install ./rec-rpc_3.0.2_amd64.deb
   sudo apt install ./robotino-dev_1.1.2_amd64.deb
   sudo apt install ./robotino-api2_1.1.2_amd64.deb
   sudo apt install ./robotino-daemons_1.1.2_amd64.deb
   ```

4. Install LiDAR dependencies:
   ```bash
   sudo apt install ros-humble-rplidar-ros
   ```

### Development Machine Setup

1. Install ROS 2 Humble (same as above)

2. Clone this repository:
   ```bash
   mkdir -p ~/robotino_ws/src
   cd ~/robotino_ws/src
   git clone https://github.com/yourusername/robotino_hal.git
   ```

3. Install dependencies:
   ```bash
   cd ~/robotino_ws
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the workspace:
   ```bash
   cd ~/robotino_ws
   colcon build
   source install/setup.bash
   ```
   
## Project Structure

The project is organized in the following structure:

```
robotino_hal/
├── config/                 # Configuration files
│   ├── robot_params.yaml   # Robot parameters
│   ├── nav2_params.yaml    # Navigation parameters
│   └── rviz_config.rviz    # RViz configuration
├── description/            # Robot description files
│   └── robotino.urdf       # URDF model
├── launch/                 # Launch files
│   ├── display.launch.py   # Launch for URDF visualization
│   ├── navigation.launch.py # Launch for navigation
│   └── total_launch.py     # Main launch file
├── maps/                   # Map files
├── nodes/                  # ROS 2 nodes
│   ├── battery_monitor.py  # Battery monitoring node
│   ├── camera_monitor.py   # Camera interface node
│   ├── motor_control.py    # Motor control node
│   ├── navigation.py       # Simple navigation node
│   ├── odometry_monitor.py # Odometry node
│   └── sensor_monitor.py   # IR sensor node
└── tests/                  # Unit tests
```

## Hardware Abstraction Layer

### Battery Monitor

The Battery Monitor node (`battery_monitor.py`) retrieves battery status from the Robotino API and publishes it as a standard ROS 2 `BatteryState` message. It provides information about:

- Current voltage
- Estimated charge percentage
- Power supply status and health

```python
# Example of accessing battery data
ros2 topic echo /robotino/battery_state
```

### Motor Control

The Motor Control node (`motor_control.py`) enables control of the Robotino's omnidirectional drive system using standard ROS 2 `Twist` messages. Features include:

- Safety limits for velocity and acceleration
- Emergency stop functionality
- Smooth velocity transitions

```python
# Example of sending velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Sensor Monitor

The Sensor Monitor node (`sensor_monitor.py`) publishes data from the Robotino's infrared distance sensors as a `Float32MultiArray` message. The values are normalized to meters for easier integration with navigation packages.

```python
# Example of accessing sensor data
ros2 topic echo /robotino/distancesensorarray
```

### Odometry Monitor

The Odometry Monitor node (`odometry_monitor.py`) processes the robot's wheel encoder data to generate odometry information. It publishes:

- Odometry messages (`nav_msgs/Odometry`)
- TF transforms between the `odom` and `base_link` frames

```python
# Example of accessing odometry data
ros2 topic echo /robotino/odom
```

### Camera Monitor

The Camera Monitor node (`camera_monitor.py`) interfaces with the Robotino's camera and publishes standard ROS 2 `Image` and `CameraInfo` messages.

```python
# Example of viewing camera feed
ros2 run rqt_image_view rqt_image_view
```

### LiDAR Integration

The RPLiDAR A1M8 is integrated using the `rplidar_ros` package, which publishes LaserScan messages for use in navigation and obstacle avoidance.

```python
# Example of accessing LiDAR data
ros2 topic echo /scan
```

## Navigation Stack

### SLAM and Mapping

The project uses `slam_toolbox` for Simultaneous Localization and Mapping (SLAM). The online_async mode allows for real-time mapping and localization.

```bash
# To create a new map
ros2 launch slam_toolbox online_async_launch.py

# To save a map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'my_map'"
```

### Nav2 Configuration

The Nav2 configuration is stored in `config/nav2_params.yaml` and includes settings for:

- Global and local costmaps
- Path planners
- Controllers
- Recovery behaviors

### Path Planning and Control

The navigation stack uses:

- Global planner: Theta* for smooth, any-angle paths
- Local planner: Model Predictive Path Integral (MPPI) controller for dynamic obstacle avoidance

## Usage

### Basic Control

Launch the basic HAL components:

```bash
ros2 launch robotino_hal total_launch.py
```

### Navigation

#### Using Maps

To create a new map using SLAM:

```bash
# Launch SLAM toolbox in online mode
ros2 launch slam_toolbox online_async_launch.py

# Drive the robot around to map the environment
# You can use teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save the created map when finished:

```bash
# Save the map to a file
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_lab_map'}}"
```

To navigate using a pre-existing map:

```bash
# Launch Nav2 with your map file
ros2 launch nav2_bringup bringup.launch.py map:=/path/to/your/map.yaml

# Alternative with the robotino_hal package
ros2 launch robotino_hal navigation.launch.py map:=/path/to/your/map.yaml
```

The map parameter can be an absolute path or a relative path to a map file stored in the package:

```bash
# Using a map from the robotino_hal package
ros2 launch nav2_bringup bringup.launch.py map:=$(ros2 pkg prefix robotino_hal)/share/robotino_hal/maps/lab_map.yaml
```

Visualize in RViz and set navigation goals:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix robotino_hal)/share/robotino_hal/config/rviz_config.rviz
```

In RViz:
1. Use "2D Pose Estimate" to set the initial robot position
2. Use "Navigation2 Goal" to set navigation goals for the robot

### Custom Command Examples

Emergency stop:
```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: true"
```

Resume operation:
```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: false"
```

Send a navigation goal:
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## Troubleshooting

Common issues and solutions:

- **Robot not responding to commands**: Check if emergency stop is enabled or if battery level is critically low.
- **LiDAR not working**: Verify USB connection and permissions (`sudo chmod 666 /dev/ttyUSB0`).
- **Navigation errors**: Ensure the TF tree is correctly configured with `ros2 run tf2_tools view_frames`.
- **Localization failures**: Reset the localization using `ros2 service call /reinitialize_global_localization std_srvs/srv/Empty`.

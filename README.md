# 🤖 Aokkai Everyday – Team Overview

Welcome to the official repository of **Aokkai Everyday**, a student-led robotics team from **Khon Kaen University, Thailand**, participating in the Thailand Open ROS 2025 competition under the @Home category.

---
  ```bash
  source ~/.bashrc
```


## Code for Bringup

SSH (Remote pc)
```bash
  ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

Bring up (turtlebot3)
```bash
  ros2 launch turtlebot3_bringup robot.launch.py
```
Show topic
```bash
  ros2 topic list
```

Show LiDAR data
```bash
  ros2 launch turtlebot3_bringup rviz2.launch.py 
```

Teleop (Remote PC)
```bash
  ros2 run turtlebot3_teleop teleop_keyboard
```

## Code for SLAM
SSH (Remote pc)
```bash
  ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

Bring up (Turtlebot3)
```bash
  ros2 launch turtlebot3_bringup robot.launch.py
```

Cartographer (Remote PC)
```bash
  ros2 launch turtlebot3_cartographer cartographer.launch.py
```

Save Map (Remote PC)
```bash
  ros2 run nav2_map_server map_saver_cli -f ~/map
```


## Code for Gazebo
install
  ```bash
source ~/.bashrc
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws && colcon build --symlink-install
```
Terminal 1
```bash
source ~/.bashrc
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Terminal 2
```bash
source ~/.bashrc
ros2 run turtlebot3_teleop teleop_keyboard
```
Terminal 3
```bash
source ~/.bashrc
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
Terminal 4
```bash
source ~/.bashrc
ros2 run nav2_map_server map_saver_cli -f ~/map
   ```

## Code for Nav

Terminal 1
```bash
source ~/.bashrc
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Terminal 2
```bash
source ~/.bashrc
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
   ```


## 🎯 Team Mission

Aokkai Everyday is dedicated to advancing home service robotics by integrating **ROS 2**, **Artificial Intelligence**, and **Embedded Systems** to solve real-world automation challenges.  
Our goals include:

- Designing autonomous robots capable of navigating dynamic indoor environments
- Implementing real-time object detection and voice interaction
- Gaining hands-on experience in full-stack robot development
- Contributing open-source robotics solutions for learning and innovation

We believe in learning through doing — and competitions like ROS @Home provide the perfect playground to apply our knowledge, collaborate, and push the boundaries of what service robots can do.

---

## 👥 Team Members

| Name                        | Role                                      |
|-----------------------------|-------------------------------------------|
| **Theeraphat Tonglertwong** | Sleeping Engineer                         |
| **Phubadine Mehom**         | Sleeping Engineer                         |
| **Phubet Jitpilai**         | Sleeping Engineer                         |
| **Punnawit Seangsuriyakat** |  Sleeping Engineer                        |
| **Sirawit Polkhumkaew**     | Sleeping Engineer                         |

---

## 🏫 Affiliation

**Faculty of Engineering**  
Khon Kaen University, Thailand  
Departments:  
- Computer Engineering  
- Automation, Robotics & AI Engineering  
- Industrial Engineering

---

## 📄 Team Description Paper (TDP)

For more technical details and development approach, please refer to our [Team Description Paper (TDP)](./TDP_AokkaiEveryDay_KKU.pdf).

---

Thank you for visiting our repository! 🚀




# Roadmap🚀 ROS2 LIDAR Robot Development

This is a development plan and guide for integrating **LIDAR** with a robot using **ROS2**. It covers the basic integration with **TF**, **odometry**, **SLAM**, and other sensors. The goal is to create a fully functional robot capable of autonomous navigation, mapping, and localization.

## Table of Contents

1. [Introduction](#introduction)
2. [System Requirements](#system-requirements)
3. [LIDAR Integration with ROS2](#lidar-integration-with-ros2)
4. [TF Integration](#tf-integration)
5. [Odometry and Localization](#odometry-and-localization)
6. [SLAM Integration](#slam-integration)
7. [Other Sensors and Data Fusion](#other-sensors-and-data-fusion)
8. [To-Do List](#to-do-list)

---

### 1. Introduction

This ROS2 package is designed to integrate **LIDAR** sensors with a robot, enabling autonomous navigation, object detection, and mapping. The package will integrate multiple technologies such as **TF** for transformation management, **Odometry** for tracking robot movement, and **SLAM** for environment mapping.

---

### 2. System Requirements

- **ROS2 Installation**: Make sure ROS2 (Foxy or later) is installed.
- **LIDAR Sensor**: Compatible with the sensor (e.g., RPLIDAR, Hokuyo, Velodyne).
- **Dependencies**:
  - `rplidar_ros` (for RPLIDAR sensor)
  - `robot_localization`
  - `slam_toolbox`
  - `nav2_bringup`
  - `tf2` for transformation

---

### 3. LIDAR Integration with ROS2

To integrate LIDAR into ROS2, install the necessary package that supports the sensor.

#### Example for RPLIDAR:
1. **Install the RPLIDAR ROS package**:
   ```bash
   git clone https://github.com/robopeak/rplidar_ros.git
   cd rplidar_ros
   colcon build
   ```

2. **Configure and Launch the RPLIDAR Node**:
   ```xml
   <launch>
     <node pkg="rplidar_ros" exec="rplidarNode" name="rplidar" output="screen">
       <param name="frame_id" value="laser_frame"/>
       <param name="scan_mode" value="Sensory"/>
     </node>
   </launch>
   ```

3. The sensor data will now be available via `/scan` topic, and the **LIDAR frame** will be referenced in transformations.

---

### 4. TF Integration

**TF** is crucial for managing the transformations between different coordinate frames like `base_link`, `laser_frame`, `odom`, and `map`.

1. **Setting up TF Publisher for Static Transforms**:
   ```bash
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map laser_frame
   ```
   - This ensures that LIDAR's coordinate frame (`laser_frame`) is associated with the global frame (`map`).

2. **TF Transformations**:
   - All sensor frames and robot frames (e.g., `base_link`, `odom`, `laser_frame`) need to be properly linked to ensure accurate localization.

---

### 5. Odometry and Localization

**Odometry** tracks the movement of the robot based on encoders or IMU data, while **localization** corrects the robot's position using sensor data (e.g., LIDAR).

1. **Odometry Setup**:
   - The robot’s position can be tracked using wheel encoders or an IMU sensor. The odometry data will be published to the `/odom` topic.

2. **Localization Setup using AMCL**:
   - Use **AMCL (Adaptive Monte Carlo Localization)** to improve the robot's position accuracy.
   ```xml
   <launch>
     <node pkg="nav2_bringup" exec="amcl" name="amcl" output="screen">
       <param name="base_frame_id" value="base_link"/>
       <param name="odom_frame_id" value="odom"/>
       <param name="laser_frame_id" value="laser_frame"/>
     </node>
   </launch>
   ```

3. **EKF Localization**:
   - For advanced filtering and state estimation, integrate **robot_localization** package to fuse data from IMU, odometry, and LIDAR.
   ```xml
   <launch>
     <node pkg="robot_localization" exec="ekf_localization_node" name="ekf_localization" output="screen">
       <param name="odom_frame" value="odom"/>
       <param name="base_frame" value="base_link"/>
       <param name="imu0" value="/imu/data"/>
       <param name="imu0_config" value="[true, true, true, false, false, false, false, false, false]"/>
     </node>
   </launch>
   ```

---

### 6. SLAM Integration

**SLAM (Simultaneous Localization and Mapping)** allows the robot to build a map of the environment while localizing itself within that map.

1. **SLAM Tool Setup**:
   - Install **slam_toolbox** for ROS2.
   ```bash
   sudo apt install ros-foxy-slam-toolbox
   ```

2. **Launch SLAM**:
   ```xml
   <launch>
     <node pkg="slam_toolbox" exec="sync_slam_toolbox_node" name="slam_toolbox" output="screen">
       <param name="use_sim_time" value="true"/>
       <param name="base_frame" value="base_link"/>
       <param name="odom_frame" value="odom"/>
     </node>
   </launch>
   ```
   - This will launch SLAM and start creating a map from the LIDAR data.

---

### 7. Other Sensors and Data Fusion

Integrating other sensors, such as IMU, cameras, or additional LIDAR units, can improve the robot’s performance.

1. **IMU Integration**:
   - Use **IMU** for better motion tracking and filtering, integrating it with **robot_localization**.

2. **Camera Integration**:
   - Use **OpenCV** or **ROS2 Camera packages** for object detection, obstacle avoidance, or human recognition.

3. **Data Fusion**:
   - Combine LIDAR, IMU, odometry, and camera data using **sensor fusion algorithms** like **Kalman Filter** or **Extended Kalman Filter (EKF)**.

---

### 8. To-Do List

#### Tasks Completed:
- [ ] **LIDAR sensor setup and integration** with ROS2 (RPLIDAR example)
- [ ] **TF integration** to link the LIDAR frame with global map/frame
- [ ] **Odometry setup** for basic movement tracking
- [ ] **AMCL localization** setup for improving robot position accuracy
- [ ] **SLAM** integration for map creation and navigation

#### Tasks In Progress:
- [ ] **IMU sensor integration** with **robot_localization** for better position tracking
- [ ] **Camera integration** for object detection (YOLOv5 or OpenCV)
- [ ] **Path planning** setup with ROS2 Navigation Stack (Nav2)
- [ ] **Obstacle avoidance** algorithm based on LIDAR and IMU data

#### Upcoming Tasks:
- [ ] Test robot in real-world environments and adjust sensor calibration
- [ ] Optimize SLAM for faster mapping in dynamic environments
- [ ] Implement multi-sensor fusion to improve localization accuracy
- [ ] Finalize robot behavior for task execution (e.g., human interaction, object delivery)

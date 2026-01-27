# Differential Drive Robot
This project documents my ongoing effort to learn and understand mobile robot simulation using ROS2. It revolves around simulating a simple, custom differential drive robot that primarily exists in a virtual environment using GazeboSim & RViz.\
The repository is intended as a beginner-friendly reference, capturing both working configurations and the lessons learned while setting up robot descriptions, simulation workflows and sensor integration in ROS2. 

>TL;DR\
This project focuses on simulating a simple differential drive robot using ROS2, GazeboSim & RViz. It is beginner-oriented, serving as a living document of my learning and thought process, as well as discoveries while working with ROS2 simulation tools.
## System Requirements
- Ubuntu: Ubuntu Noble 24.04.3 LTS
- ROS2: Kilted
- Gazebo: Ionic
  
## Overview
### Control Plugins: 
User may toggle between gazebo_control or ros2_control via `use_ros2_control` found in `launch_sim.launch.py`
- Gazebo DiffDrive plugin (gazebo_control) -----> Set `use_ros2_control` to `false`
- ros2_control               -----> Set `use_ros2_control` to `true`

### Simulated Sensors:
- LiDAR (Light Detection and Ranging)
- Camera
- Depth Camera (w/ PointCloud visualization)

## Credits
This project is inspired by the work of joshnewans and his Articulated Robotics blog and tutorial. The code in this repository is written and maintained for ROS 2 Kilted and Gazebo Ionic.

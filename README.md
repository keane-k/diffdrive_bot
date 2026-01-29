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
#### Using `teleop_twist_keyboard`
<img width="477" height="342" alt="image" src="https://github.com/user-attachments/assets/1813da91-7bab-401a-a580-53b2af2fdf0a" />

### Simulated Sensors:
#### LiDAR (Light Detection and Ranging)
<img width="1399" height="690" alt="image" src="https://github.com/user-attachments/assets/080104c9-e61d-4b53-a6cf-077da3c58a4e" />

#### Camera
<img width="859" height="514" alt="image" src="https://github.com/user-attachments/assets/672f31b8-0e26-4d48-adec-3c3f16e8cd01" />

#### Depth Camera (w/ PointCloud visualization)
<img width="922" height="828" alt="image" src="https://github.com/user-attachments/assets/02bc0777-9c9e-438e-990d-bff4ef293546" />
<img width="869" height="839" alt="image" src="https://github.com/user-attachments/assets/a287dc9e-2136-4949-821e-0cbcd61de498" />
<img width="1260" height="641" alt="image" src="https://github.com/user-attachments/assets/f671c500-7163-4f26-8371-b2ed9ffccc92" />

## Future Works
- Teleop (Using gamepad controllers/joysticks)
- Adv. Teleop (Using phone/tablet)
  > Might not be implemented if a real robot is required
- Object Tracking using OpenCV
- SLAM using slam_toolbox
- Autonomous Navigation with Nav2
## Credits
This project is inspired by the work of joshnewans and his Articulated Robotics blog and tutorial. The code in this repository is written and maintained for ROS 2 Kilted and Gazebo Ionic.

# mini_pi_description

> **Mini_Pi Robot – ROS 2 description & simulation package**
<p align="center">
  <img width="430" height="384" alt="mini3" src="https://github.com/user-attachments/assets/0c2ba581-3e7f-4b3a-86cf-d5c140c1518d" />
</p>


<ins>Description:</ins> 
Mini_Pi is a small differential drive robot with ultrasonic sensors, modeled in URDF/Xacro and simulated in Gazebo for navigation and SLAM experiments.
This is a [GIAR](https://giar.ai/) research project

---

## 1. Project Overview

`mini_pi_description` provides the URDF/Xacro description and Gazebo integration for the **Mini_Pi** robot:

- A small differential-drive robot intended for **education, research, and prototyping**.
- Designed as a **ROS 2-first** robot: easy to use with `rviz2`, `Gazebo`, and standard ROS 2 tools.
- Focused on:
  * Basic navigation experiments (odometry, wheel kinematics).
  - SLAM and mapping (with range sensors and simulated worlds).
  - Testing control and perception algorithms in simulation before moving to real hardware.

The package focuses on a clean, modular Xacro structure so it can be reused or extended in other projects.

---
## 2. Features

- ✅ **Differential-drive mobile base**
  - Left/right wheel joints with configurable radius and separation.
- ✅ **Ultrasonic range sensors**
  - Front-mounted ultrasonic modules (and additional ones if enabled via parameters).
- ✅ **URDF/Xacro modular design**
  - Reusable macros for chassis, wheels, sensors, etc.
  - Easy to tweak dimensions and inertial parameters.
- ✅ **Gazebo simulation-ready**
  - Includes Gazebo plugins for differential drive and sensor simulation.
  - Compatible with `use_sim_time` and standard ROS 2 workflows.
- ✅ **RViz visualization**
  - Robot model can be visualized via `robot_state_publisher` + `joint_state_publisher`.
- ✅ **Prepared for navigation experiments**
  - Standard frames (`base_link`, `odom`, etc.).
  - Structured to be integrated later with Nav2 and SLAM packages.



---
## 3. Requirements

- **ROS 2** (Jazzy)
    
- **Gazebo** (Harmonic)
    
- `xacro`
    
- `robot_state_publisher`
    
- `joint_state_publisher` or `joint_state_publisher_gui`
    
- `gazebo_ros` (if using Gazebo Classic integrations)

---
## 4. Build & Installation

Clone into your ROS 2 workspace and build:

```bash
# 1. Go to your workspace
cd ~/ros2_ws/src

# 2. Clone the repo
git clone https://github.com/<your-user>/mini_pi_description.git

# 3. Build
cd ~/ros2_ws
colcon build --packages-select mini_pi_description

# 4. Source the workspace
source install/setup.bash
```

## 5. Robot Description Details

High-level structure of the robot model:

- **Base / Chassis**
    
    - Main `base_link` defined as a box or mesh.
        
    - Inertial, collision, and visual elements defined in Xacro for reuse and clarity.
        
- **Wheels**
    
    - Two driven wheels:
        
        - `left_wheel_link` / `right_wheel_link`
            
        - `left_wheel_joint` / `right_wheel_joint`
            
    - Parameters:
        
        - `wheel_radius`
            
        - `wheel_separation`
            
- **Caster / Support (optional)**
    
    - Passive caster wheel.
        
- **Ultrasonic Sensors**
    
    - Typically attached to a frame like `ultrasonic_front_link`.
        
    - Positioned with configurable offsets from `base_link`.
        
- **Coordinate Frames**
    
    - `base_link` (main body)
        
    - `base_footprint` (optional, planar projected frame)
        
    - Sensor frames (e.g., `ultrasonic_front_link`)
        
    - `odom` frame for wheel odometry (usually provided by a controller/odometry node, not by this package alone).
        

The description is written in **Xacro** to allow parameterization and reuse.


## 6. Topics, Frames & Controllers

- `/tf` / `/tf_static`  
    Standard transform topics for the robot’s TF tree.
    
- `/joint_states`  
    Joint states (wheel joints, sensor joints if any).
    
- `/diff_drive_controller/cmd_vel`  
    Geometry_msgs `Twist` commands for the differential drive controller must be `stamped:=true`. 
    
- `/odom`  
    Odometry (if provided by a controller or an external node).

## 7. Common Issues & Troubleshooting
> [!WARNING]
> Do not name the robot `<name_of_robot>_robot_description` because `robot_description` messes up the argument in `ros2_control` and it will fail—it's a Jazzy bug. To fix it, do not use that name. You can use `mini_pi_description` or `model` or whatever you like. 

## 8. License
This project is licensed under the **Apache License 2.0**. 
See the [LICENSE](./LICENSE) file for the full text.

---
## 9. Photos 
<img width="2955" height="1315" alt="2025-11-18T13:28:39 741342616" src="https://github.com/user-attachments/assets/5ba25410-6dbc-4b7b-8df2-6381c32fc00d" /> 
<img width="2955" height="1315" alt="2025-11-18T13:29:29 984514895" src="https://github.com/user-attachments/assets/277934a3-a26d-4973-b3fb-5ad09dcd30dc" />
<img width="2955" height="1315" alt="2025-11-18T13:25:42 179188737" src="https://github.com/user-attachments/assets/13a7875a-51b8-4e55-b062-fcbde5398a6d" />

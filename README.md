# Testbot updated V0

Testbot is being developed as a ROS2-based (Humble Distribution) robotic system integrating navigation, control, and sensor fusion. It utilizes Gazebo Fortress with Husarion World model for simulation and ros2_control for managing actuator control. The robot is designed for autonomous navigation, leveraging LIDAR, IMU, and odometry sensors for localization and path planning.

## Key Components and Features

### 1. Simulation & Modeling

  #### Implemented robot URDF and SDF models.

 #### Integrated Gazebo Fortress for realistic physics-based simulation.
 #### added [Husarion](https://husarion.com/tutorials/howtostart/rosbotxl-quick-start/) world for a complex world to simulate the Navigation/mapping

#### Configured LIDAR and other sensors for environment perception(to be added).

### 2. Control System

#### Configured diff_drive_controller for wheel-based movement.

#### Integrated joint_state_publisher for robot state representation.

#### Used ros2_control for hardware abstraction and control management.

### 3. Navigation & Localization

#### Utilized ros_gz_bridge for ROS-Gazebo communication.

#### Integrated joint_state_broadcaster for sensor data streaming.

#### Works on fusing odometry, LIDAR data for environmental perception and mapping and IMU data for stable motion tracking.

> below is a picture of the Testbot in different(TurtleBot Arena and Husarion) world and the TF tree
![Screenshot from 2025-02-28 14-45-56](https://github.com/user-attachments/assets/fa275afd-6fe1-4251-8eb2-502beef356f6)
![Screenshot from 2025-02-28 14-24-16](https://github.com/user-attachments/assets/26ac0b15-961a-4dd8-9d2d-2f8a9f808fb3)![Screenshot from 2025-02-28 14-26-11](https://github.com/user-attachments/assets/4479aa42-eeda-4079-b791-50568485f316)


## Challenges Faced & Solutions Implemented(troubleshoot)

### LIDAR Visualization Error

Issue: 'Error entity lidar_link doesn't exist and cannot be used to set lidar visual pose' which results in LIDAR showing LIDAR ray in a position not from the URDF translated position
![Screenshot from 2025-02-28 14-29-13](https://github.com/user-attachments/assets/b3c83e36-0250-4c98-bad1-50bb0546a82d)

> Cause/reason
##### Lidar Link Joint is not well declared or positioned in the URDF which results in a bad Transform
##### '<ignition_frame_id>' issue as joint gets removed when converting the URDF to SDFormat due to what's called 'Fixed Joint Lumping'
> Solutions: 
##### Ensured lidar_link is correctly defined in URDF and checked TF tree consistency.
#####  include the Fixed_joint_Lumping Tag block 
    <gazebo reference='laser_joint'>       
    <preserveFixedJoint>true</preserveFixedJoint>
    </gazebo>

#### ROS2 Package Dependency Issues

> Issue: Missing ament_cmake, ros_ign_bridge, and ros_gz_bridge during compilation.

>Solution: Verified and installed missing dependencies, updated CMAKE_PREFIX_PATH.

#### Docker & NVIDIA GPU Issues

>Issue: nvidia-container-cli: initialization error: nvml error: driver not loaded

>Solution: Reinstalled NVIDIA drivers and ensured proper configuration of the NVIDIA Container Toolkit.

#### TF Frame Misalignment & World Rotation

> Issue: World in RViz rotated unexpectedly when the bot moved, which cause generation distorted map while performing SLAM
![WhatsApp Image 2025-01-31 at 4 05 24 PM](https://github.com/user-attachments/assets/09e742e6-99db-4a75-93c8-2361319d9a77)


>Solutions:
> a. Corrected odom frame reference in TF tree, verified IMU data, and adjusted navigation parameters.

> b. Reducing bot rotation speed in the Teleoperate function allows the bot to reset the scan position after being rotated.
> checkout the created Map after issue resolved
![Screenshot from 2025-02-28 16-47-36](https://github.com/user-attachments/assets/4e8abd2e-03ae-41ec-b404-2a82de2e6a18)


#### Control Activation Errors

Issue: ros2 control load_controller --set-state active failing due to missing controllers.

Solution: Ensured ros2_control components were properly loaded and configuration files were correctly structured.

## 🐞 Known Issues and Troubleshooting (ROS 2 Nav2 - Humble)

Below is a list of issues encountered during the setup and launch of Nav2, along with debugging steps and solutions.

---

### ❌ 1. RViz Not Launching

**Symptoms:**
- `rviz2` does not appear when launching.
- No GUI, or RViz silently crashes.

**Possible Causes:**
- Incorrect or missing `.rviz` config file.
- RViz node misconfigured in launch file.
- Missing display environment if running remotely (e.g., via SSH).

**Solutions:**
- Ensure the RViz config file exists: `testbot/rviz/amcl.rviz`
- Replace RViz node parameters:
  ```python
  parameters=[{'use_sim_time': use_sim_time}]
  
### ❌ ROS 2 Navigation Launch Error: `yaml_filename` Not Initialized

- [ERROR] [map_server]: Original error: parameter 'yaml_filename' is not initialized
- [FATAL] [map_server]: Lifecycle node map_server does not have error state implemented

#### ✅ Fix

Ensure your **launch file** (e.g., `nav.launch.py`) includes the `yaml_filename` parameter for the `map_server` node.

#### 🔧 Python Launch File Example

- Replace RViz node parameters:
  ```python
  from launch_ros.actions import Node

  map_yaml_file = '/path/to/your/map.yaml'

  map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[{'yaml_filename': map_yaml_file}]
  )
- Insert the Map full path in .yaml file
  ```yaml
  map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: "/path/to/your/map.yaml"
### ❌ Failed to create global planner.:

* Error Summary
  ```vbnet
  Failed to create global planner.
  Exception: According to the loaded plugin descriptions the class nav2_navfn_planner::NavfnPlanner with base class type nav2_core::GlobalPlanner does not exist.
  Declared types are: 
   - nav2_navfn_planner/NavfnPlanner 
   - nav2_smac_planner/SmacPlanner2D 
   - nav2_smac_planner/SmacPlannerHybrid 
   - nav2_smac_planner/SmacPlannerLattice 
   -  nav2_theta_star_planner/ThetaStarPlanner
 *  ✅ Fix
  -The correct plugin class string must match exactly what the system has registered(change in your navigation.yaml):
    ```yaml
    planner_server:
      ros__parameters:
      planner_plugins: ["GridBased"]
      GridBased:
        plugin: "nav2_navfn_planner/NavfnPlanner"
    
  -This should also be changed for the following plugins:
    
        plugin: "nav2_behaviors/Spin"
        plugin: "nav2_behaviors/Backup"
        plugin: "nav2_behaviors/Wait"
   
    
  
Next Steps

Improve localization stability using SLAM techniques.

Optimize motion control parameters for better trajectory accuracy.

Implement obstacle avoidance using LIDAR data.

Transition to a physical test environment after stable simulation validation.

This document serves as a consolidated summary of Testbot's design progress (to be revised), challenges faced, and solutions implemented.



# Custom DWA Local Planner for ROS2 Humble

A from-scratch implementation of the Dynamic Window Approach (DWA) local planner for a TurtleBot3 in ROS2 Humble.

**Author:** Bhavya Dalal

## Quick Start Guide

### 1. Prerequisites

* ROS2 Humble, Gazebo, and TurtleBot3 packages installed.

* Ensure your Gazebo environment is correctly sourced. Add the following line to your `~/.bashrc` file if you haven't already:

  ```
  source /usr/share/gazebo/setup.sh
  
  ```

### 2. Build the Package

Navigate to your ROS2 workspace root (e.g., `~/ros2_ws`) and run the build command:

```
colcon build --packages-select dwa_planner_bd

```

### 3. Launch the Simulation

In a new terminal, source your workspace and run the launch file:

```
# Make sure to source your workspace first
source install/setup.bash

# Launch the simulation, robot, and planner
ros2 launch dwa_planner_bd dwa_planner_bd_launch.py

```

### 4. Run the Planner

* Once Gazebo and RViz have loaded, use the **"Nav2 Goal"** tool in the RViz toolbar to set a destination for the robot.

* The robot will begin navigating towards the goal while avoiding obstacles.

### Command Repository

* **Create Package:** `ros2 pkg create --build-type ament_python dwa_planner_bd --node-name dwa_planner`

* **Build Package:** `colcon build --packages-select dwa_planner_bd`

* **Run Node Directly:** `ros2 run dwa_planner_bd dwa_planner`

* **Launch System:** `ros2 launch dwa_planner_bd dwa_planner_bd_launch.py`

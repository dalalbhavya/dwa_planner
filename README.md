# Custom DWA Local Planner for ROS2 Humble

A from-scratch implementation of the Dynamic Window Approach (DWA) local planner for a TurtleBot3 in ROS2 Humble.

**Author:** Bhavya Dalal

## Quick Start Guide

### 1. Prerequisites

* ROS2 Humble, Gazebo, and a sourced ROS 2 environment.

* Ensure your Gazebo environment is correctly sourced. Add the following line to your `~/.bashrc` file if you haven't already:

  ```
  source /usr/share/gazebo/setup.sh
  
  ```

### 2. Install Dependencies

Before building, you need to install the required ROS and Python dependencies.

**ROS Packages:**
From the root of your workspace (e.g., `~/ros2_ws`), run `rosdep` to install all the ROS packages listed in `package.xml`:

```
sudo apt-get update
rosdep install -i --from-path src -y --rosdistro humble

```

**Python Libraries:**
Install the required Python libraries using `pip` and the `requirements.txt` file:

```
pip install -r src/dwa_planner_bd/requirements.txt

```

### 3. Build the Package

Navigate to your ROS2 workspace root and run the build command:

```
colcon build --packages-select dwa_planner_bd

```

### 4. Launch the Simulation

In a new terminal, source your workspace and run the launch file:

```
# Make sure to source your workspace first
source install/setup.bash

# Launch the simulation, robot, and planner
ros2 launch dwa_planner_bd dwa_planner_bd_launch.py

```

### 5. Run the Planner

* Once Gazebo and RViz have loaded, use the **"Nav2 Goal"** tool in the RViz toolbar to set a destination for the robot.

* The robot will begin navigating towards the goal while avoiding obstacles.

### Command Repository

* **Create Package:** `ros2 pkg create --build-type ament_python dwa_planner_bd --node-name dwa_planner`

* **Build Package:** `colcon build --packages-select dwa_planner_bd`

* **Run Node Directly:** `ros2 run dwa_planner_bd dwa_planner`

* **Launch System:** `ros2 launch dwa_planner_bd dwa_planner_bd_launch.py`

# first-mega-project

## 📝 Overview
This repository features a comprehensive, ROS2-based autonomous navigation stack designed for a differential drive mobile robot. The project bridges intelligent path-planning algorithms with standard ROS2 navigation frameworks, showcasing full-stack autonomous mobile robot (AMR) development from physical simulation to high-level decision-making.

By leveraging the **ROS2 Nav2** ecosystem, this project implements custom global path planners, enabling the robot to map environments, avoid obstacles, and efficiently navigate dynamic spaces.

## 🛠️ Key Features
* **Custom Nav2 Global Planners:** Custom C++ and Python implementations of **A\*** and **Dijkstra** pathfinding algorithms, integrated directly as plugins within the Nav2 stack (`bumperbot_planning`).
* **High-Fidelity Simulation:** Integrated with **Ignition Gazebo** for realistic physics, sensor feedback, and environment mapping.
* **Real-time Visualization:** Configured **RViz2** dashboards to visualize sensor data, costmaps, transforms (TF), and planned global path trajectories.
* **Modular ROS2 Architecture:** Structured with isolated packages for robot descriptions (`single_robo_discription`), hardware/simulation bringup (`single_robo_bringup`), and custom navigation logic.

---

## 📊 Project Visuals

### RViz Simulation
<img width="1920" height="1080" alt="image" src="https://github.com/user-attachments/assets/165ad539-16f8-44d8-84f3-39381aae1382" />

### Ignition Gazebo Environment
<img width="1920" height="1080" alt="image" src="https://github.com/user-attachments/assets/d7e0ed64-ab0f-4e9e-b1e4-c24369dc2abe" />

### Nav2 Architecture
<img width="2398" height="1792" alt="Gemini_Generated_Image_4o43nr4o43nr4o43" src="https://github.com/user-attachments/assets/2cced965-f54b-4f8f-a6e1-3449ec7b9195" />

---

## 🚀 Getting Started

Follow these steps to clone, build, and launch the project in your local ROS2 environment:

### 1. Clone the Repository
```bash
git clone https://github.com/Asd1agd/first-mega-project.git
cd first-mega-project

```

### 2. Build the Workspace

Make sure you have your ROS2 distribution (e.g., Humble/Iron/Jazzy) properly sourced, then build the packages:

```bash
colcon build

```

### 3. Source and Launch

Source the local workspace overlay and launch the complete robot system:

```bash
source install/setup.bash
ros2 launch single_robo_bringup single_robo_bringup.launch.py

```

---

## 🎛️ Runtime Command Cheat Sheet

Use these `ros2 param` and `ros2 service` commands to dynamically interact with and tune your navigation stack at runtime.

### 🧭 Global Planner Plugins

```bash
# Set standard Navfn Planner
ros2 param set /planner_server GridBased.plugin "nav2_navfn_planner::NavfnPlanner"

# Set custom Dijkstra Planner
ros2 param set /planner_server GridBased_DJ.plugin "bumperbot_planning::DijkstraPlanner"

# Set custom A* Planner
ros2 param set /planner_server GridBasedFast.plugin "bumperbot_planning::AStarPlanner"

```

### 🏎️ Local Controller Plugins

```bash
# Switch to classic Pure Pursuit
ros2 param set /controller_server FollowPath.plugin "nav2_pure_pursuit_controller::PurePursuitController"

# Switch to Model Predictive Path Integral (MPPI)
ros2 param set /controller_server FollowPath.plugin "nav2_mppi_controller::MPPIController"

# ALTERNATIVE: Turn Regulated Pure Pursuit into Vanilla Pure Pursuit (Highly Recommended for dynamic switching)
ros2 param set /controller_server FollowPath.use_regulated_linear_velocity_scaling False
ros2 param set /controller_server FollowPath.use_cost_regulated_linear_velocity_scaling False
ros2 param set /controller_server FollowPath.use_collision_detection False

```

### 🕵️‍♂️ Kidnapped Robot Recovery (AMCL)

```bash
# Enable AMCL automatic random particle generation when lost
ros2 param set /amcl recovery_alpha_slow 0.001
ros2 param set /amcl recovery_alpha_fast 0.1

# INSTANT RESCUE: Explode particles globally across the map immediately
ros2 service call /amcl/reinitialize_global_localization std_srvs/srv/Empty {}

```

### 🪟 Squeezing Through Narrow Paths (Inflation & Clearing)

```bash
# Shrink Global Costmap buffers
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.55
ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 3.0

# Shrink Local Costmap buffers
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.55
ros2 param set /local_costmap/local_costmap inflation_layer.cost_scaling_factor 3.0

# Force-refresh costmaps to instantly apply the new thin boundaries
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap {}
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}

```

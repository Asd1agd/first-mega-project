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
git clone [https://github.com/Asd1agd/first-mega-project.git](https://github.com/Asd1agd/first-mega-project.git)
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

```


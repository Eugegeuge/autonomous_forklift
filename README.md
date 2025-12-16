# Autonomous Forklift (ROS 2 Jazzy)

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![Simulator](https://img.shields.io/badge/Simulator-Mvsim-green)
![Status](https://img.shields.io/badge/Status-Development-orange)

This project implements an autonomous forklift system for warehouse logistics using **ROS 2 Jazzy**. It currently uses **Mvsim** for lightweight simulation and **Nav2** (planned) for graph-based navigation.

## 🏗 Architecture

The system is designed to operate in a structured warehouse environment.

*   **Simulation**: Mvsim (2.5D) with a 3D warehouse environment.
*   **Robot**: Turtlebot3 (Placeholder for navigation) / Custom Forklift Model (Planned for Gazebo).
*   **Navigation**: Nav2 Route Server (Graph-based navigation).
*   **Perception**: ArUco markers for precise pallet approach.

## 🚀 Installation

### Prerequisites
*   Ubuntu 24.04 (Noble Numbat)
*   ROS 2 Jazzy Jalisco
*   Mvsim (`sudo apt install ros-jazzy-mvsim`)

### Build
```bash
cd ~/Documents/autonomous_forklift
colcon build --symlink-install
source install/setup.bash
```

## 🎮 Usage

### Launch Simulation
To start the simulation with the warehouse map and the robot:

```bash
ros2 launch autonomous_forklift simulation.launch.py
```

This will open:
1.  **Mvsim**: The physics simulator showing the 3D warehouse.
2.  **Rviz2**: For visualization of sensors and robot state.

### Manual Control
You can control the robot using the keyboard (in a separate terminal):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/forklift1/cmd_vel
```

## 🗺 Roadmap

For a detailed breakdown of the project phases and future iterations, please refer to the [Project Roadmap](project_roadmap.html) included in this repository.

### Current Status
- [x] Basic Simulation Setup (Mvsim + Warehouse Map).
- [x] Robot Integration (Turtlebot3 with Pallet Jack visual).
- [ ] Nav2 Route Server Configuration.
- [ ] ArUco Approach Logic.
- [ ] Manipulation (Load/Unload).

## 🤝 Contributing
1.  Fork the repository.
2.  Create your feature branch (`git checkout -b feature/AmazingFeature`).
3.  Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4.  Push to the branch (`git push origin feature/AmazingFeature`).
5.  Open a Pull Request.

## 👥 Project Team
*   Hugo Sevilla Martínez
*   Hugo López Pastor
*   Juan Diego Serrato Tovar
*   Pablo Molina Pérez

## 📄 License
Universidad de Alicante.

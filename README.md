# Romi Robot Demos

Comprehensive collection of robot simulation, control, and visualization tools for the Pololu Romi robot platform. Includes Arduino firmware, C++ Qt applications, and Python simulations for forward/inverse kinematics, motor control, and waypoint navigation.

## Overview

This repository provides a complete software stack for the Pololu Romi 32U4 robot, covering multiple implementation approaches and use cases:

- **Arduino Firmware**: Low-level robot control with PID-based motion control, supporting both serial and I2C communication protocols
- **C++ Qt Applications**: Desktop applications with OpenGL visualization for robot control, simulation, and monitoring
- **Python Simulations**: Interactive simulations using PySide6 and OpenGL for algorithm development and testing
- **PyBullet Simulations**: Physics-based simulations using PyBullet for realistic robot motion, camera visualization, and stadium environments
- **ROS2/Gazebo Integration**: ROS2 plugins and Gazebo simulation for integration with the ROS ecosystem
- **Docker Environment**: Containerized ROS2 and Gazebo setup for reproducible development environments

All projects share common robot parameters and control algorithms, allowing seamless transition between simulation and hardware deployment.

## Project Structure

### 📁 `arduino/`
Arduino firmware for the Romi 32U4 robot. Includes implementations for waypoint navigation, velocity control, and motor characterization.

### 📁 `qt_cpp/`
C++ Qt applications with OpenGL visualization for robot control and simulation. Built with Qt5 (Qt6 compatible with minor changes). Contains desktop applications, simulation servers, viewer tools, and supporting libraries.

### 📁 `python_simulations/`
Python-based robot simulations using PySide6 (Qt6 bindings) and OpenGL for interactive development and testing. Includes forward/inverse kinematics simulations and shared robot models.

### 📁 `pybullet_simulation/`
PyBullet-based physics simulation for realistic robot motion and camera visualization. Includes inverse kinematics simulations with optional camera support.

### 📁 `romi_gazebo/`
ROS2 and Gazebo integration for the Romi robot, including encoder-based odometry plugins, world files, and launch configurations.

### 📁 `ros2_docker/`
Docker containerization for ROS2 Humble and Gazebo Fortress development environment. Provides reproducible containerized setup for ROS2 development.

---

## Quick Start

### Arduino Firmware
```bash
# Open sketches in Arduino IDE
cd arduino/inverse_kinematics
# Upload inverse_kinematics.ino to Romi robot
```

### C++ Applications
```bash
# Build all Qt projects (Qt5 by default, Qt6 supported with minor changes)
cd qt_cpp
mkdir build && cd build
cmake ..
cmake --build . --config Release

# Run applications
./inverse_kinematics_sim_app/Release/inverse_kinematics_sim_app
./inverse_kinematics_viewer/Release/inverse_kinematics_viewer
```

### Python Simulations
```bash
# Install dependencies
pip install PySide6 PyOpenGL numpy

# Run forward kinematics simulation
cd python_simulations/forward_kinematics_sim
python forward_kinematics_sim.py

# Run inverse kinematics simulation
cd python_simulations/inverse_kinematics_sim
python inverse_kinematics_sim.py
```

### PyBullet Simulations
```bash
# Install dependencies
pip install pybullet numpy pybullet_data

# Run basic inverse kinematics simulation
cd pybullet_simulation
python inverse_kinematics.py

# Run camera-enabled simulation
python inverse_kinematics_cam_viz.py
```

---

## Dependencies

### Arduino
- Pololu Romi 32U4 Control Board
- Arduino IDE or PlatformIO

### C++
- Qt5 (Core, Widgets, OpenGL, Gamepad, Network, SerialPort)
  - **Note:** Qt6 is also supported with minor code and CMakeLists.txt changes
- CMake 3.8+
- Eigen3 3.3+
- C++17 compiler

### Python
- Python 3.8+
- PySide6 (official Qt for Python framework - Qt6 bindings)
- PyOpenGL
- NumPy

### PyBullet
- Python 3.8+
- pybullet (physics simulation engine)
- pybullet_data (required for default assets)
- NumPy

---

## Features

### Simulations
- ✅ Forward kinematics with odometry drift visualization
- ✅ Inverse kinematics with waypoint navigation
- ✅ Realistic motor dynamics (first-order model)
- ✅ Encoder noise simulation (Gaussian noise)
- ✅ PID velocity control with anti-windup

### Controls
- ✅ Virtual joystick interfaces (multi-touch enabled)
- ✅ Gamepad support (physical controllers)
- ✅ Waypoint click-to-add functionality
- ✅ Real-time trajectory visualization

### Communication
- ✅ Serial communication (Arduino ↔ PC) - ASCII command protocol
- ✅ Network communication (TCP/IP between inverse_kinematics_app/sim_app and inverse_kinematics_viewer)
- ✅ Human-readable text commands for easy debugging and manual control

### OpenGL-Based UI
- ✅ **Resolution Independent**: All C++ Qt visualizations (forward/inverse kinematics) use OpenGL for scalable rendering
- ✅ **Current Implementation**: 2D geometry rendering (robot trajectories, waypoints, grid)
- 🚧 **Future Enhancements**: 
  - Support for 3D model loading (STL/OBJ file formats)
  - 3D world environments for more realistic visualization
  - Enhanced spatial representation with depth perception

**Benefits of OpenGL Approach:**
- Smooth rendering at any resolution or zoom level
- Hardware-accelerated graphics performance
- Easy integration of future 3D features
- Cross-platform graphics consistency

---

## Robot Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Wheel Diameter | 0.07 | m |
| Wheelbase | 0.14 | m |
| Encoder Resolution | 1440 | ticks/rotation |
| Control Frequency | 40 | Hz |
| Max Velocity | ±50 | ticks/sample |

---

## Contributing

This is a demonstration project. Feel free to use as reference for your own robot control implementations.


# C++ Qt Applications

C++ Qt applications with OpenGL visualization for robot control and simulation. Built with Qt5 (Qt6 compatible with minor changes).

## Overview

This directory contains desktop applications, simulation servers, and supporting libraries for the Pololu Romi robot. All applications use OpenGL for hardware-accelerated rendering, providing smooth visualization at any resolution.

## Project Structure

### Applications

- **`forward_kinematics_sim/`**: Forward kinematics simulation with gamepad/virtual joystick control. Visualizes odometry drift compared to ground truth trajectory.
- **`inverse_kinematics_viewer/`**: GUI application for monitoring robot navigation. Connects to robot/simulator to visualize trajectory and waypoints in real-time.
- **`inverse_kinematics_sim_app/`**: Headless inverse kinematics simulation server. Network-based waypoint receiver that simulates robot motion for testing navigation algorithms.
- **`inverse_kinematics_app/`**: Server application for physical robot control via serial communication. Receives waypoints over network and executes inverse kinematics navigation on actual hardware.
- **`inverse_kinematics_app_i2c/`**: Server application for physical robot control via I2C (Raspberry Pi). Bridges I2C communication with network clients.

### Libraries

- **`helper_opengl/`**: OpenGL utility library for Qt applications. Manages buffers, shaders, textures, and provides common rendering functionality.
- **`robot_sim_models/`**: C++ library with PID controller and wheel dynamics models. Provides realistic motor simulation with first-order dynamics and encoder noise.
- **`virtual_controller/`**: Qt widget library providing virtual joystick controls. Reusable multi-touch enabled joystick components for robot control interfaces.

### Tools

- **`wheel_models/`**: Python scripts for motor characterization and modeling. Includes data visualization, parameter identification, and simulation validation tools.

## Building

### Prerequisites

- **Qt5** (Core, Widgets, OpenGL, Gamepad, Network, SerialPort)
  - **Note:** Qt6 is also supported with minor code and CMakeLists.txt changes
- **CMake** 3.8+
- **Eigen3** 3.3+
- **C++17** compiler

### Build Steps

```bash
# Navigate to qt_cpp directory
cd qt_cpp

# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build all projects
cmake --build . --config Release

# Or on Linux/Mac
make -j$(nproc)
```

### Build Output

Executables will be located in:
- `build/forward_kinematics_sim/forward_kinematics_sim`
- `build/inverse_kinematics_viewer/inverse_kinematics_viewer`
- `build/inverse_kinematics_sim_app/inverse_kinematics_sim_app`
- `build/inverse_kinematics_app/inverse_kinematics_app`
- `build/inverse_kinematics_app_i2c/inverse_kinematics_app_i2c`

Libraries will be built as static libraries:
- `build/helper_opengl/libhelper_opengl.a`
- `build/robot_sim_models/librobot_sim_models.a`
- `build/virtual_controller/libvirtual_controller.a`

## Running Applications

### Forward Kinematics Simulation

```bash
cd build/forward_kinematics_sim
./forward_kinematics_sim
```

Control the robot using:
- Virtual joystick (on-screen)
- Gamepad controller
- Keyboard (if implemented)

### Inverse Kinematics Viewer

```bash
cd build/inverse_kinematics_viewer
./inverse_kinematics_viewer
```

Connect to:
- `inverse_kinematics_sim_app` (simulation server)
- `inverse_kinematics_app` (physical robot via serial)
- `inverse_kinematics_app_i2c` (physical robot via I2C)

### Simulation Server

```bash
cd build/inverse_kinematics_sim_app
./inverse_kinematics_sim_app
```

Starts a network server (default port: 60000) that accepts waypoint commands and simulates robot motion.

### Physical Robot Server (Serial)

```bash
cd build/inverse_kinematics_app
./inverse_kinematics_app
```

Requires:
- Arduino robot running `inverse_kinematics` sketch
- Serial port connection (USB)

### Physical Robot Server (I2C)

```bash
cd build/inverse_kinematics_app_i2c
./inverse_kinematics_app_i2c
```

Requires:
- Raspberry Pi with I2C enabled
- Arduino robot running `inverse_kinematics_i2c` sketch
- I2C connection (SDA, SCL, GND)

See `inverse_kinematics_app_i2c/README.md` for detailed setup instructions.

## Architecture

### OpenGL Rendering

All visualization applications use OpenGL for rendering:
- **Resolution Independent**: Smooth rendering at any zoom level
- **Hardware Accelerated**: Leverages GPU for performance
- **2D Geometry**: Robot trajectories, waypoints, grid overlays
- **Future Ready**: Easy to extend with 3D features

### Communication Protocols

- **Network (TCP/IP)**: Used between viewer and server applications
  - ASCII-based protocol for waypoint commands and status updates
  - Default port: 60000
- **Serial (USB)**: Used by `inverse_kinematics_app` for Arduino communication
  - ASCII command protocol
  - Baud rate: 115200
- **I2C**: Used by `inverse_kinematics_app_i2c` for Raspberry Pi communication
  - Binary protocol (PololuRPiSlave)
  - I2C address: 20 (0x14)

### Threading Model

Applications use Qt's threading for:
- **Network I/O**: Separate threads for TCP/IP communication
- **Serial/I2C I/O**: Background threads for robot communication
- **UI Updates**: Main thread for GUI responsiveness

## Dependencies

### Qt Modules

- **Qt5::Core**: Core functionality
- **Qt5::Widgets**: GUI components
- **Qt5::OpenGL**: OpenGL integration
- **Qt5::Gamepad**: Gamepad controller support
- **Qt5::Network**: TCP/IP networking
- **Qt5::SerialPort**: Serial communication (for serial-based apps)

### External Libraries

- **Eigen3**: Matrix and vector operations
- **OpenGL**: Graphics rendering (system library)

## Development

### Adding a New Application

1. Create a new subdirectory in `qt_cpp/`
2. Add `CMakeLists.txt` with Qt5 dependencies
3. Add subdirectory to root `CMakeLists.txt`
4. Link against required libraries (`helper_opengl`, `robot_sim_models`, etc.)

### Qt6 Migration

To use Qt6 instead of Qt5:
1. Update `find_package(Qt6 ...)` in CMakeLists.txt files
2. Change `Qt5::*` to `Qt6::*` in target_link_libraries
3. Update any deprecated Qt5 APIs

## Related Projects

- **`arduino/`**: Arduino firmware that these applications communicate with
- **`python_simulations/`**: Python equivalents using PySide6
- **`pybullet_simulation/`**: PyBullet-based physics simulation


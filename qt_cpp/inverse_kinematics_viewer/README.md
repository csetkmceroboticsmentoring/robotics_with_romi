# Inverse Kinematics Viewer

GUI application for monitoring inverse kinematics robot navigation. Connects to robot servers (simulation or physical hardware) to visualize trajectory and waypoints in real-time.

## Overview

The Inverse Kinematics Viewer is a desktop application that provides real-time visualization of robot navigation. It connects to server applications over TCP/IP network (port 60000) and displays:

**Compatible Servers:**
- `inverse_kinematics_sim_app` - Simulation server
- `inverse_kinematics_app` - Physical robot via USB serial
- `inverse_kinematics_app_i2c` - Physical robot via Raspberry Pi I2C

**Visualization Features:**

- **Robot Position**: Current robot pose (x, y, heading)
- **Trajectory**: Real-time path visualization as the robot moves
- **Waypoints**: Target waypoints and navigation path
- **Interactive Control**: Click-to-add waypoints directly on the visualization

## Features

- ✅ Real-time trajectory visualization using OpenGL
- ✅ Interactive waypoint addition (click on view to add waypoints)
- ✅ Network-based communication (TCP/IP)
- ✅ Connects to simulation or physical robot servers
- ✅ Smooth rendering at any zoom level
- ✅ Robot texture visualization
- ✅ Background grid and coordinate system

## Building

### Prerequisites

- **Qt5** (Widgets, Network, SerialPort)
- **CMake** 3.8+
- **Eigen3** 3.3+
- **C++14** compiler
- **helper_opengl** library (built from `../helper_opengl/`)

### Build Steps

```bash
# From qt_cpp directory
cd qt_cpp
mkdir build && cd build
cmake ..
cmake --build . --config Release

# Executable will be at:
# build/inverse_kinematics_viewer/inverse_kinematics_viewer
```

## Usage

### Starting the Application

```bash
cd build/inverse_kinematics_viewer
./inverse_kinematics_viewer
```

### Connecting to a Server

1. **Enter Server IP Address**: Type the IP address or hostname of the server
   - For local simulation: `localhost` or `127.0.0.1`
   - For Raspberry Pi: Enter the Pi's IP address (e.g., `192.168.1.100`)

2. **Click Connect**: The application will attempt to connect to the server on port 60000

3. **Status**: Connection status is displayed in the UI

### Adding Waypoints

1. **Enable Waypoint Mode**: Ensure waypoint mode is enabled (default: enabled)
2. **Click on View**: Click anywhere on the visualization to add a waypoint
3. **Waypoints Sent**: Waypoints are automatically sent to the connected server

### View Controls

- **Zoom**: Use mouse wheel or keyboard to zoom in/out
- **Pan**: Click and drag to pan the view
- **Clear**: Clear the trajectory visualization

## Communication Protocol

The viewer communicates with server applications using a simple ASCII-based protocol over TCP/IP (port 60000).

### Waypoint Command (Viewer → Server)

```
w: <x>, <y>
```

- **x, y**: Coordinates in meters (floating point)
- **Example**: `w: 0.5, 0.3`

### Status Response (Server → Viewer)

```
s: <x>, <y>, <yaw>
```

- **x, y**: Position in meters
- **yaw**: Heading angle in radians
- **Example**: `s: 0.45, 0.28, 1.57`

## Compatible Servers

The viewer can connect to three types of server applications:

### 1. `inverse_kinematics_sim_app` (Simulation)
- **Purpose**: Simulation server for testing navigation algorithms
- **Location**: `../inverse_kinematics_sim_app/`
- **Runs on**: Any PC
- **Use case**: Algorithm development and testing without physical hardware

### 2. `inverse_kinematics_app` (USB Serial)
- **Purpose**: Physical robot server using USB serial communication
- **Location**: `../inverse_kinematics_app/`
- **Requires**: 
  - Arduino robot with `inverse_kinematics` sketch
  - USB cable connection
- **Communication**: USB Serial (115200 baud)
- **Use case**: Direct connection to physical robot via USB

### 3. `inverse_kinematics_app_i2c` (Raspberry Pi)
- **Purpose**: Physical robot server using I2C communication
- **Location**: `../inverse_kinematics_app_i2c/`
- **Requires**: 
  - Raspberry Pi (runs the server application)
  - Arduino robot with `inverse_kinematics_i2c` sketch
  - I2C connection (SDA, SCL, GND)
- **Communication**: I2C bus (address 0x14)
- **Use case**: Remote robot control via Raspberry Pi, no USB cable required

## Architecture

### Components

1. **MainDlg** (`main_dlg.h/cpp`)
   - Main dialog window
   - Manages UI and user interactions
   - Coordinates between communication thread and visualization

2. **PathViewWidget** (`path_view_widget.h/cpp`)
   - OpenGL-based visualization widget
   - Renders robot, trajectory, waypoints, and background
   - Handles mouse/keyboard input for waypoint addition

3. **CommsThread** (`comms_thread.h/cpp`)
   - Background thread for network communication
   - Handles TCP/IP socket connection
   - Sends waypoints and receives status updates

### Threading Model

- **Main Thread**: UI updates and user interactions
- **CommsThread**: Network I/O in background thread
- **Thread Safety**: Mutex-protected queues for data exchange

### OpenGL Rendering

- Uses `helper_opengl` library for buffer and shader management
- Hardware-accelerated rendering
- Resolution-independent visualization
- Robot texture loaded from resources

## File Structure

```
inverse_kinematics_viewer/
├── README.md              # This file
├── CMakeLists.txt         # Build configuration
├── main.cpp               # Application entry point
├── main_dlg.h/cpp         # Main dialog window
├── main_dlg.ui            # Qt Designer UI file
├── path_view_widget.h/cpp # OpenGL visualization widget
├── comms_thread.h/cpp     # Network communication thread
├── resources.qrc          # Qt resource file
└── romi_small.png         # Robot texture image
```

## Dependencies

### Qt Modules

- **Qt5::Widgets**: GUI components and dialogs
- **Qt5::Network**: TCP/IP networking
- **Qt5::SerialPort**: Serial port support (for future use)

### External Libraries

- **Eigen3**: Matrix and vector operations
- **helper_opengl**: OpenGL utility library (from `../helper_opengl/`)

## Troubleshooting

### Cannot Connect to Server

- Verify server is running: Check if the server application is started
- Check network connectivity: `ping <server_ip>`
- Verify port: Default port is 60000
- Check firewall: Ensure port 60000 is not blocked

### Waypoints Not Being Sent

- Verify connection status: Check if "Connected" is displayed
- Check waypoint mode: Ensure waypoint mode is enabled
- Verify server is receiving: Check server console output

### Visualization Not Updating

- Check connection: Ensure server is sending status updates
- Verify data format: Server should send `s: x, y, yaw` format
- Check console: Look for error messages in application output

## Related Projects

- **`../inverse_kinematics_sim_app/`**: Simulation server for testing
- **`../inverse_kinematics_app/`**: Physical robot server (serial)
- **`../inverse_kinematics_app_i2c/`**: Physical robot server (I2C)
- **`../helper_opengl/`**: OpenGL utility library
- **`arduino/inverse_kinematics/`**: Arduino firmware (serial version)
- **`arduino/inverse_kinematics_i2c/`**: Arduino firmware (I2C version)


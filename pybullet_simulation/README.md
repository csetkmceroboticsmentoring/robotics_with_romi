# PyBullet Simulation for Romi Robot

PyBullet-based physics simulation of the Pololu Romi robot performing waypoint navigation using inverse kinematics.

## Overview

This simulation demonstrates autonomous waypoint following using:
- **Inverse Kinematics**: Computes wheel velocities to reach target waypoints
- **PID Control**: Separate PID controllers for heading and distance control
- **Motor Dynamics**: Realistic wheel encoder models with motor dynamics and PID velocity control
- **Physics Simulation**: PyBullet physics engine for accurate robot motion

## Features

- ✅ Realistic motor dynamics with encoder feedback
- ✅ PID-based heading and distance control
- ✅ Continuous waypoint looping
- ✅ Real-time trajectory visualization
- ✅ Waypoint markers and path visualization
- ✅ Physics-based robot motion
- ✅ **Raspberry Pi Camera Module 2** simulation (in `inverse_kinematics_cam_viz.py`)
- ✅ Programmatic camera image capture from robot's perspective
- ✅ Stadium environment for realistic ground simulation

## Requirements

### Python Environment

**Recommended:** Use a virtual environment to avoid package conflicts.

#### Option 1: Using `venv` (Python 3.3+)

```bash
# Create virtual environment
python3 -m venv pybullet_env

# Activate virtual environment
# On Linux/Mac:
source pybullet_env/bin/activate
# On Windows:
# pybullet_env\Scripts\activate

# Install packages
pip install pybullet numpy pybullet_data
```

#### Option 2: Using `conda`

```bash
# Create conda environment
conda create -n pybullet_env python=3.8

# Activate environment
conda activate pybullet_env

# Install packages
pip install pybullet numpy pybullet_data
```

### Python Packages

- `pybullet` - Physics simulation engine
- `numpy` - Numerical computations
- `pybullet_data` - Required for default assets like ground plane (`plane.urdf`)

**Note:** Always activate your virtual environment before running the simulation.

### Dependencies

The simulation depends on the `python_simulations` package in the workspace root:
- `python_simulations.robot_models` - PID controllers, wheel encoders, and robot parameters

## Installation

1. **Create and activate a virtual environment** (recommended to avoid package conflicts):
   ```bash
   # Using venv
   python3 -m venv pybullet_env
   source pybullet_env/bin/activate  # On Windows: pybullet_env\Scripts\activate
   
   # Or using conda
   conda create -n pybullet_env python=3.8
   conda activate pybullet_env
   ```

2. **Install Python dependencies:**
   ```bash
   pip install pybullet numpy pybullet_data
   ```

3. **Ensure workspace structure:**
   ```
   romi_robot_demos/
   ├── pybullet_simulation/
   │   ├── inverse_kinematics.py          # Basic simulation (no camera)
   │   ├── inverse_kinematics_cam_viz.py  # Simulation with camera visualization
   │   ├── motion_controller.py
   │   └── romi_meshes.urdf
   └── python_simulations/
       └── robot_models/
   ```

## Usage

### Running the Simulation

There are two simulation scripts available:

**1. Basic Simulation (no camera):**
```bash
# From pybullet_simulation directory
python inverse_kinematics.py

# Or from workspace root
python -m pybullet_simulation.inverse_kinematics
```

**2. Camera Visualization Simulation:**
```bash
# From pybullet_simulation directory
python inverse_kinematics_cam_viz.py

# Or from workspace root
python -m pybullet_simulation.inverse_kinematics_cam_viz
```

The camera visualization version includes:
- Camera image capture from robot's perspective
- Stadium ground environment (instead of simple plane)
- RGB buffer preview window (can be disabled)

### Controls

- **ESC Key**: Exit simulation

### Default Waypoints

The simulation follows a square path with these waypoints (in meters):
- `(0.5, 0.0)` - Forward
- `(0.0, 0.5)` - Left
- `(-0.5, 0.0)` - Backward
- `(0.0, -0.5)` - Right
- `(0.5, 0.0)` - Forward again
- `(0.0, 0.0)` - Return to start

The robot continuously loops through these waypoints.

## Customization

### Changing Waypoints

Edit the `waypoints` list in either simulation script (around line 360 in `inverse_kinematics.py` or line 496 in `inverse_kinematics_cam_viz.py`):

```python
waypoints = [
    (0.5, 0.0),    # (x, y) in meters
    (0.0, 0.5),
    # Add more waypoints...
]
```

Waypoints are defined in PyBullet's coordinate system (X forward, Y left, Z up).

### Adjusting Control Parameters

Edit `motion_controller.py` to modify PID gains:

```python
# In MotionController.__init__()
self.heading_pid = PidControl(kp=0.125, ki=0.0, kd=0.01)
self.distance_pid = PidControl(kp=1.5, ki=0.0, kd=0.01)
```

### Changing Robot Model

Replace `romi_meshes.urdf` with your own URDF file. The script automatically loads it from the same directory.

### Camera Configuration

The camera functionality is available in `inverse_kinematics_cam_viz.py`. Camera settings can be adjusted in that file:

```python
# Camera settings (around line 177)
self.camera_width = 640          # Image width in pixels
self.camera_height = 480         # Image height in pixels (4:3 aspect ratio)
self.camera_fov = 62.2          # Horizontal field of view in degrees
self.camera_update_interval = 0.1  # Capture rate (0.1s = 10 Hz)
```

**Camera Specifications:**
- **Model**: Raspberry Pi Camera Module 2
- **Resolution**: 640x480 (configurable, default optimized for performance)
- **Horizontal FOV**: 62.2° (matches real camera module)
- **Aspect Ratio**: 4:3
- **Update Rate**: 10 Hz (configurable)
- **Near Plane**: 1.25 cm (prevents self-occlusion)

**Accessing Camera Images:**

Camera images are captured programmatically and stored in `self.last_camera_image` as a numpy array (height, width, 3) with RGB values. The camera automatically captures images at the configured update rate during simulation.

To modify the capture rate, change `camera_update_interval`:
- `0.1` seconds = 10 Hz
- `0.2` seconds = 5 Hz
- `0.05` seconds = 20 Hz

**Note:** The basic `inverse_kinematics.py` script does not include camera functionality. Use `inverse_kinematics_cam_viz.py` for camera features.

## File Structure

```
pybullet_simulation/
├── README.md                      # This file
├── __init__.py                    # Package initialization
├── inverse_kinematics.py          # Basic simulation (no camera)
├── inverse_kinematics_cam_viz.py # Simulation with camera visualization
├── motion_controller.py            # Waypoint following controller
├── romi_meshes.urdf               # Robot URDF model (includes Raspberry Pi Camera Module 2)
└── meshes/
    └── visual/                     # 3D mesh files
        ├── chasis.stl
        ├── control_board.stl
        └── wheel.stl
```

## Architecture

### Components

1. **`RomiSimulator`** (`inverse_kinematics.py` and `inverse_kinematics_cam_viz.py`)
   - PyBullet physics simulation setup
   - Robot loading and visualization
   - Main simulation loop
   - Trajectory visualization
   - Camera image capture (only in `inverse_kinematics_cam_viz.py`)
   - Stadium environment loading (only in `inverse_kinematics_cam_viz.py`)

2. **`MotionController`** (`motion_controller.py`)
   - Waypoint queue management
   - PID-based heading and distance control
   - Wheel velocity computation
   - Motor dynamics integration

3. **Robot Models** (`python_simulations.robot_models`)
   - `PidControl`: PID controller implementation
   - `WheelWithEncoder`: Motor dynamics with encoder feedback
   - Robot parameters (wheel diameter, encoder ticks, etc.)

### Control Flow

1. **Initialization**: Load robot, set up physics, initialize controller
2. **Waypoint Loop**: 
   - Calculate distance and heading to current waypoint
   - Use PID controllers to compute wheel velocities
   - Apply velocities through motor dynamics models
   - Update robot pose from physics simulation
3. **Visualization**: Draw trajectory, waypoints, and robot path
4. **Waypoint Reached**: Move to next waypoint (loops continuously)

## Coordinate Systems

The simulation handles coordinate system conversions between:
- **PyBullet**: X forward, Y left, Z up
- **Controller**: X forward, Y right, Z up

Conversions are handled automatically in the simulation loop.

## Performance

- **Physics Timestep**: 200 Hz (5ms)
- **Control Loop**: 40 Hz (25ms)
- **Trajectory Update**: Periodic (configurable)
- **Camera Capture**: 10 Hz (100ms interval, configurable, only in `inverse_kinematics_cam_viz.py`)

The simulation runs in real-time by default. Adjust `p.setTimeStep()` and control loop frequency as needed.

**Note**: Camera resolution is set to 640x480 by default for better rendering performance. Higher resolutions (e.g., 1280x960) may slow down the simulation.

**Environment:**
- `inverse_kinematics.py`: Uses simple `plane.urdf` ground
- `inverse_kinematics_cam_viz.py`: Uses `stadium_no_collision.sdf` for more realistic ground simulation


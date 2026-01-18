# Inverse Kinematics - Arduino Sketch (I2C Version)

Autonomous waypoint navigation for the Pololu Romi 32U4 robot using inverse kinematics with PID control, communicating over **I2C** with a Raspberry Pi.

![Inverse Kinematics Viewer](../../inv_kin_viewer.png)
*Inverse Kinematics Viewer - GUI application for monitoring robot navigation and sending waypoints*

## Overview

This Arduino sketch enables the Romi robot to:
- Receive waypoints over **I2C communication** (designed for Raspberry Pi)
- Navigate autonomously to each waypoint in sequence
- Use PID control for both heading and distance
- Maintain a queue of up to 4 waypoints
- Report position and status back to the host via I2C

## Hardware Requirements

- **Pololu Romi 32U4 Control Board**
- **Romi Chassis** with motors and encoders
- **Wheels**: 70mm diameter (default Romi wheels)
- **Raspberry Pi** (as I2C master) - connected via I2C bus
- **I2C Connection**: SDA and SCL pins connected between Romi and Raspberry Pi
- **USB Cable** for programming (I2C is used for runtime communication)

## Features

### Motion Control
- **Dual PID Controllers**: Separate control for heading and distance to waypoint
- **Velocity Control**: Individual wheel velocity control with PID feedback
- **Point-and-Navigate**: Robot continuously adjusts heading toward waypoint and moves forward (curved convergence path)

### Waypoint Management
- **Queue System**: Circular queue stores up to 4 waypoints
- **Sequential Navigation**: Robot processes waypoints in order
- **Automatic Progression**: Moves to next waypoint when current one is reached

### Communication
- **I2C Protocol**: Binary communication using PololuRPiSlave library
- **I2C Address**: Default address 20 (configurable in `slave.init(20)`)
- **Shared Memory Buffer**: Uses structured data format for efficient communication
- **Command ID Tracking**: Each command includes a `cmd_id` to track command/response pairs and prevent duplicate processing
- **Command Types**:
  - `w`: Add waypoint (x, y coordinates in millimeters, transmitted as int32_t × 1000)
  - `s`: Get robot status (position, heading, queue state, timestamp)
  - `r`: Reset robot state to origin
- **Binary Format**: All values transmitted as integers (multiplied by 1000 to avoid floating point)
- **Designed for Raspberry Pi**: Optimized for I2C communication with Raspberry Pi as master

## Control Algorithm

### Inverse Kinematics Approach

1. **Calculate Error**:
   - Distance error: Euclidean distance to waypoint
   - Heading error: Angular difference between current heading and desired heading

2. **PID Control**:
   - Distance PID generates forward velocity correction
   - Heading PID generates rotational velocity correction

3. **Differential Drive**:
   ```
   right_wheel = dist_correction * cos(heading_error) + heading_correction
   left_wheel  = dist_correction * cos(heading_error) - heading_correction
   ```

4. **Velocity Control**:
   - Each wheel has its own PID velocity controller
   - Maintains commanded wheel velocities using encoder feedback

### Default PID Gains

**Distance PID** (Heading to waypoint distance):
- Kp = 0.125
- Ki = 0.0
- Kd = 0.01

**Heading PID** (Angular orientation):
- Kp = 0.0625
- Ki = 0.0
- Kd = 0.01

**Velocity PID** (Per-wheel motor control):
- Kp = 0.1613
- Ki = 12.9032
- Kd = 0.0

**Note**: This I2C version does not support runtime PID parameter updates via the `k` command. PID gains are fixed at compile time.

## Robot Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Wheel Diameter | 0.07 | m |
| Wheelbase | 0.14 | m |
| Encoder Resolution | 1440 | ticks/rotation |
| Control Frequency | 40 | Hz (25ms period) |
| Position Report Rate | 10 | Hz |
| Waypoint Threshold | 0.02 | m (2 cm) |

## File Structure

```
inverse_kinematics_i2c/
├── inverse_kinematics_i2c.ino  # Main sketch (setup/loop)
├── application.h/cpp            # Application logic and I2C communication
├── motor_with_encoder.h/cpp     # Motor control with encoder feedback
├── pid_control.h/cpp            # PID controller implementation
├── circular_queue.h              # Waypoint queue data structure
└── cmd_response.h                # I2C command/response data structures
```

## Installation

1. **Install Arduino IDE** (1.8.x or 2.x)
2. **Install Pololu Romi 32U4 Board Support**:
   - In Arduino IDE: File → Preferences
   - Add to "Additional Boards Manager URLs":
     ```
     https://files.pololu.com/arduino/package_pololu_index.json
     ```
   - Tools → Board → Boards Manager → Search "Pololu" → Install

3. **Select Board**:
   - Tools → Board → Pololu A-Star 32U4 → Pololu Romi 32U4

4. **Upload Sketch**:
   - Open `inverse_kinematics_i2c.ino`
   - Click Upload button (or Ctrl+U)
   - **Note**: After uploading, disconnect USB if using I2C for runtime communication

## Usage

### Basic Operation

1. **Upload sketch** to Romi robot
2. **Connect I2C** between Romi and Raspberry Pi:
   - Connect SDA (data) and SCL (clock) pins
   - Ensure proper pull-up resistors (usually on Raspberry Pi)
   - Connect ground between both devices
3. **Configure I2C on Raspberry Pi**:
   - Enable I2C interface: `sudo raspi-config` → Interface Options → I2C → Enable
   - Verify I2C device: `ls /dev/i2c-*` (should show `/dev/i2c-1` on most Pi models)
4. **Launch I2C Server Application** on Raspberry Pi:
   ```bash
   ./inverse_kinematics_app_i2c
   ```
   - The app communicates with Arduino over I2C
   - Bridges I2C communication with network for the viewer

5. **Launch Viewer Application** (can be on same or different computer):
   ```bash
   ./inverse_kinematics_viewer
   ```
   - Set IP address (Raspberry Pi running inverse_kinematics_app_i2c)
   - Click **Connect** button
   - Click on the view to add waypoints
   - Robot receives commands via I2C and navigates to each waypoint

### I2C Connection Setup

**Physical Connections:**
- **SDA**: Connect Romi's SDA pin to Raspberry Pi's SDA (GPIO 2, pin 3)
- **SCL**: Connect Romi's SCL pin to Raspberry Pi's SCL (GPIO 3, pin 5)
- **GND**: Connect ground between Romi and Raspberry Pi
- **VCC**: Ensure both devices share common power ground (Romi powered separately)

**I2C Address:**
- Default I2C slave address: **20** (set in `slave.init(20)` in `application.cpp`)
- To change address, modify line 94 in `application.cpp` and re-upload

**Verifying I2C Connection:**
```bash
# On Raspberry Pi, scan for I2C devices
sudo i2cdetect -y 1

# Should show device at address 0x14 (20 in decimal)
```

### Network Setup
- Viewer connects to server over TCP/IP
- Server forwards commands to Arduino via I2C
- Allows remote monitoring and control from any network-connected device

### I2C Communication Protocol

**Protocol**: PololuRPiSlave library (index-based I2C communication)  
**I2C Address**: 20 (0x14 in hexadecimal)  
**Format**: Binary structured data (shared memory buffer)

#### Data Structures

All data is transmitted as integers (multiplied by 1000) to avoid floating-point issues on AVR:

**Waypoint Command** (`cmd='w'`):
```cpp
struct Waypoint {
  int32_t x;  // X coordinate in millimeters (×1000)
  int32_t y;  // Y coordinate in millimeters (×1000)
};
```

**Status Response** (`cmd='s'`):
```cpp
struct Status {
  int32_t x;              // Position X in millimeters (×1000)
  int32_t y;              // Position Y in millimeters (×1000)
  int32_t yaw;            // Heading in milliradians (×1000)
  uint8_t is_queue_full;  // 1 if queue full, 0 otherwise
  uint32_t time_stamp;    // Timestamp in milliseconds
};
```

**Command/Response Tracking:**
```cpp
struct CommandData {
  char cmd;               // Command character ('s', 'w', 'r')
  uint8_t cmd_id;         // Command ID for tracking (incremented by host)
  union CommandUnion {
    Waypoint way_point;
    // ... other command data
  } data;
};

struct ResponseData {
  char cmd;               // Echo of command character
  uint8_t cmd_id;         // Echo of command ID (matches request)
  union ResponseUnion {
    Status status;
    Waypoint way_point;
    // ... other response data
  } data;
};
```

#### Commands (Host → Robot via I2C)

**Command 'w'**: Add waypoint
- **Format**: Binary waypoint structure with `cmd_id` for tracking
- **x, y**: Coordinates in millimeters (transmitted as int32_t × 1000)
- **Command ID**: Used to track command/response pairs and avoid processing duplicate commands
- **Example**: To send waypoint at (500mm, 300mm):
  - Host writes: `cmd='w'`, `cmd_id=<unique_id>`, `data.way_point.x=500000`, `data.way_point.y=300000`
- **Response**: Echo of waypoint with matching `cmd_id`

**Command 's'**: Get robot status
- **Format**: Command byte with `cmd_id` for tracking
- **Response**: Status structure with position, heading, queue state, and timestamp
- **Command ID**: Used to track command/response pairs and avoid processing duplicate commands
- **Example Response**: 
  - `x=450000` (450mm), `y=280000` (280mm), `yaw=1570000` (1.57 rad), `is_queue_full=0`, `time_stamp=12500`

**Command 'r'**: Reset robot state
- **Format**: Command byte with `cmd_id` for tracking
- **Action**: Resets position to (0, 0), heading to 0, clears waypoint queue, clears I2C buffer
- **Motors**: Stop immediately
- **Command ID**: Used to track command/response pairs

#### Safety Watchdog

The sketch includes a safety watchdog that stops the robot if no commands are received for 1 second (40 control loops). This prevents runaway behavior when I2C connection is lost.

**To disable watchdog (for testing only):**
- Comment out the watchdog check in `application.cpp` (around line 126):
  ```cpp
  // if (loop_count_last_hb > loop_halt_count) {
  //   left_motor.stop();
  //   right_motor.stop();
  //   return;
  // }
  ```
- Re-upload the sketch
- **Remember to re-enable it for normal use!**

**Note**: The watchdog is reset whenever any command is received (`loop_count_last_hb = 0`).

## Tuning PID Parameters

### Pre-tuned Parameters

The default PID gains in this sketch have been tuned using:
1. **Mathematical Model**: Motor dynamics characterized using difference equations (first-order model)
2. **Python Simulation**: Velocity control gains optimized using the `wheel_models/` Python scripts
3. **C++ Simulation**: Distance and heading PIDs validated using C++ Qt-based robot simulations (`inverse_kinematics_sim_app`)

This approach ensures good performance without manual trial-and-error tuning on the physical robot.

### Velocity PID (Motor Control)
Tuned using motor characterization data (`pwm_to_enc_ticks` Arduino sketch) and system identification:
- **Kp = 0.1613**: Provides responsive velocity tracking
- **Ki = 12.9032**: Eliminates steady-state error
- **Kd = 0.0**: Not needed for this motor model

### Distance PID
Tuned via simulation for smooth waypoint approach:
- **Kp = 0.125**: Conservative approach speed, minimal overshoot
- **Ki = 0.0**: Kept at 0 to avoid integral windup
- **Kd = 0.01**: Small damping for stability

### Heading PID
Tuned via simulation for stable orientation control:
- **Kp = 0.0625**: Smooth turning response (lower than serial version for stability)
- **Ki = 0.0**: Not needed for heading control
- **Kd = 0.01**: Light damping to prevent oscillations

### Manual Tuning Tips (if needed)

If you modify robot parameters (different wheels, weight, etc.), you may need to retune:

1. Start with Kp only, set Ki and Kd to 0
2. Increase Kp until system oscillates
3. Add small Kd to reduce oscillations
4. Test with various waypoint configurations

**Better approach**: Use the simulation tools in `wheel_models/` (for velocity PID) and `inverse_kinematics_sim_app` (for distance/heading PIDs) to tune gains before deploying to hardware.

## Dependencies

Required libraries (included with Pololu Romi 32U4 board support):
- **Romi32U4**: Motor and encoder functions
- **Arduino.h**: Standard Arduino functions
- **PololuRPiSlave**: I2C communication library (included with Romi board support)

**PololuRPiSlave Library:**
- Provides index-based I2C protocol for efficient communication
- Uses shared memory buffer structure for command/response data
- Handles I2C slave mode on Arduino (Raspberry Pi acts as master)
- Library documentation: [PololuRPiSlave Arduino Library](https://github.com/pololu/pololu-rpi-slave-arduino-library)

## Performance

- **Control Loop**: 40 Hz (25ms period)
- **Waypoint Threshold**: 2 cm (adjustable)
- **Typical Navigation**: ~0.3 m/s linear speed
- **Turn Radius**: Differential drive (can turn in place)

## Related Projects

This sketch pairs with:

### C++ Qt Applications

**inverse_kinematics_viewer** - GUI for monitoring and sending waypoints:

![Inverse Kinematics Viewer](../../inv_kin_viewer.png)

**inverse_kinematics_app_i2c** - Server application that communicates with this Arduino sketch over I2C and bridges communication with the viewer over network

**forward_kinematics_sim** - Forward kinematics simulation for testing and visualization:

![Forward Kinematics Simulation](../../for_kin_sim.png)

### Comparison with Serial Version

This I2C version differs from the serial version (`inverse_kinematics/`) in:
- **Communication**: I2C instead of serial/USB
- **Protocol**: Binary structured data instead of ASCII text
- **Host**: Designed specifically for Raspberry Pi as I2C master
- **PID Updates**: Does not support runtime PID parameter updates (fixed at compile time)
- **Advantages**: 
  - More reliable for embedded systems
  - Lower latency
  - No USB cable required for runtime communication
  - Better for permanent installations

## Contributing

This is a demonstration project. Feel free to modify PID gains and control parameters for your specific robot configuration.

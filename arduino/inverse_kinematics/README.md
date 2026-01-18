# Inverse Kinematics - Arduino Sketch

Autonomous waypoint navigation for the Pololu Romi 32U4 robot using inverse kinematics with PID control.

![Inverse Kinematics Viewer](../../inv_kin_viewer.png)
*Inverse Kinematics Viewer - GUI application for monitoring robot navigation and sending waypoints*

## Overview

This Arduino sketch enables the Romi robot to:
- Receive waypoints over serial communication
- Navigate autonomously to each waypoint in sequence
- Use PID control for both heading and distance
- Maintain a queue of up to 4 waypoints
- Report position and status back to the host

## Hardware Requirements

- **Pololu Romi 32U4 Control Board**
- **Romi Chassis** with motors and encoders
- **Wheels**: 70mm diameter (default Romi wheels)
- **USB Cable** for programming and serial communication

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
- **Serial Protocol**: ASCII-based commands and responses for easy debugging
- **Command Types**:
  - `w`: Add waypoint (x, y coordinates in millimeters)
  - `s`: Get robot status (position, heading, queue state)
  - `k`: Update PID parameters (all gains × 1000)
  - `r`: Reset robot state to origin
- **Interactive**: Commands can be sent from Arduino Serial Monitor or programmatically

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
- Kp = 1.5
- Ki = 0.0
- Kd = 0.01

**Heading PID** (Angular orientation):
- Kp = 0.125
- Ki = 0.0
- Kd = 0.01

**Velocity PID** (Per-wheel motor control):
- Kp = 0.1613
- Ki = 12.9032
- Kd = 0.0

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
inverse_kinematics/
├── inverse_kinematics.ino    # Main sketch (setup/loop)
├── application.h/cpp          # Application logic and communication
├── motor_with_encoder.h/cpp   # Motor control with encoder feedback
├── pid_control.h/cpp          # PID controller implementation
└── circular_queue.h           # Waypoint queue data structure
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
   - Open `inverse_kinematics.ino`
   - Click Upload button (or Ctrl+U)

## Usage

### Basic Operation

1. **Upload sketch** to Romi robot
2. **Connect** via USB (serial port appears as `/dev/ttyACM0` on Linux, `COM*` on Windows)
3. **Send waypoints** using the binary protocol
4. **Monitor** position updates and status messages

### Example Workflow

**Using Arduino Serial Monitor:**
1. Open Serial Monitor (Tools → Serial Monitor)
2. Set baud rate to 115200
3. Send commands:
   ```
   w 500, 0
   w 500, 500
   s
   ```
4. Robot navigates to waypoints, responds with status
5. **Note**: Send `s` command periodically (every second) to keep safety watchdog happy, or disable the watchdog for manual testing (see Arduino Serial Monitor Usage section)

**Using C++ Applications:**

1. **Connect Robot to Raspberry Pi/PC**:
   - Connect Romi robot to Raspberry Pi or PC using USB cable
   - Robot will appear as serial device (e.g., `/dev/ttyACM0` on Linux, `COM3` on Windows)

2. **Launch Server Application**:
   ```bash
   # On Raspberry Pi or PC connected to robot
   ./inverse_kinematics_app
   ```
   - **Important**: Set the correct serial port name in the application or via command line
   - The app bridges serial communication with network for the viewer

3. **Launch Viewer Application** (can be on same or different computer):
   ```bash
   ./inverse_kinematics_viewer
   ```
   - Set IP address (Raspberry Pi/PC running inverse_kinematics_app)
   - Click **Connect** button
   - Click on the view to add waypoints
   - Robot receives commands and navigates to each waypoint

**Network Setup:**
- Viewer connects to server over TCP/IP
- Server forwards commands to Arduino via serial
- Allows remote monitoring and control

**Programmatic Control:**
```python
import serial
ser = serial.Serial('/dev/ttyACM0', 115200)
# Add waypoint at (50cm, 50cm)
ser.write(b'w\n500, 500\n')
# Get status
ser.write(b's\n')
response = ser.readline()  # Read "s: x, y, yaw, ..."
```

### Serial Communication Protocol

**Baud Rate**: 115200  
**Format**: ASCII text, newline-terminated (`\n`)

#### Commands (Host → Robot)

**Command 'w'**: Add waypoint
```
w x, y
```
- **x, y**: Coordinates in millimeters (integers)
- **Example**: 
  ```
  w 500, 300
  ```
- **Response**: `w: 500, 300, 12345\r\n` (echo with timestamp)

**Command 's'**: Get robot status
```
s
```
- **Response**: `s: x, y, yaw, timestamp, queue_full, is_idle\r\n`
  - **x, y**: Position in millimeters
  - **yaw**: Heading in milliradians
  - **queue_full**: 1 if waypoint queue is full, 0 otherwise
  - **is_idle**: 1 if no active waypoint, 0 if navigating
- **Example Response**: `s: 450, 280, 1570, 12500, 0, 0\r\n`

**Command 'k'**: Update PID gains
```
k kp_v, ki_v, kd_v, kp_dist, ki_dist, kd_dist, kp_yaw, ki_yaw, kd_yaw
```
- All gains multiplied by 1000 (to avoid floating point)
- **kp_v, ki_v, kd_v**: Velocity PID gains (per-wheel motor control)
- **kp_dist, ki_dist, kd_dist**: Distance PID gains
- **kp_yaw, ki_yaw, kd_yaw**: Heading PID gains
- **Example**: 
  ```
  k 161, 12903, 0, 1500, 0, 10, 125, 0, 10
  ```
  (Translates to: velocity Kp=0.161, Ki=12.903, Kd=0.0, etc.)
- **Response**: `k: [gains], timestamp\r\n` (echo with timestamp)

**Command 'r'**: Reset robot state
```
r
```
- Resets position to (0, 0), heading to 0, clears waypoint queue
- Motors stop immediately

#### Arduino Serial Monitor Usage

You can manually send commands from the Arduino Serial Monitor:
1. Set baud rate to **115200**
2. Set line ending to **Newline** or **Both NL & CR**
3. Type commands:
   ```
   w 500, 500
   s
   ```

**⚠️ Important for Manual Testing:**

The sketch includes a safety watchdog that stops the robot if no commands are received for 1 second. This prevents runaway behavior when connection is lost.

For manual testing with Arduino Serial Monitor, you have two options:

**Option 1: Send periodic status commands**
- Send `s` command every second to keep the watchdog happy
- Example: Add waypoint, then send `s` periodically while robot navigates

**Option 2: Disable the safety check (for testing only)**
- Comment out the watchdog check in `application.cpp`:
  ```cpp
  // Line ~121 in application.cpp
  // if (loop_count_last_hb > loop_halt_count) {
  //   left_motor.stop();
  //   right_motor.stop();
  //   return;
  // }
  ```
- Re-upload the sketch
- **Remember to re-enable it for normal use!**

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
- **Kp = 1.5**: Moderate approach speed, minimal overshoot
- **Ki = 0.0**: Kept at 0 to avoid integral windup
- **Kd = 0.01**: Small damping for stability

### Heading PID
Tuned via simulation for stable orientation control:
- **Kp = 0.125**: Smooth turning response
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

All required libraries are included with the Pololu Romi 32U4 board support:
- **Romi32U4**: Motor and encoder functions
- **Arduino.h**: Standard Arduino functions

No external libraries required!

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

**inverse_kinematics_app** - Server application that sends serial commands to this Arduino sketch and bridges communication with the viewer over network

**forward_kinematics_sim** - Forward kinematics simulation for testing and visualization:

![Forward Kinematics Simulation](../../for_kin_sim.png)

## Contributing

This is a demonstration project. Feel free to modify PID gains and control parameters for your specific robot configuration.

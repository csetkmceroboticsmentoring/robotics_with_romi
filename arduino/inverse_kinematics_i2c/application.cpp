#include "application.h"
#include "cmd_response.h"

#include <Romi32U4.h>
#include <PololuRPiSlave.h>

#include <math.h>
#include <string.h>  // For memset

// I2C Slave
PololuRPiSlave<struct Data, 25> i2c_slave;

/*
Robot state (x, y, yaw) are stored as double values in meters. The avr compiler
doesn't support float, double values printf, sprint, scanf & sscanf function.
All float, double values are multiplied by 1000.0 and converted to integer for
transmission to host. Similarly host transmits Kp, Ki, Kd and waypoint data
as integer(multiplied by 1000). These values are converted to float, double by
dividing with 1000.0.
*/

// Control loop timing and robot physical parameters
#define DELAY_MS 25                                    // PID loop runs at 40Hz (every 25ms)
constexpr double kWheelDiameter = 0.07;                // Wheel diameter in meters
constexpr double kDistanceBetweenWheels = 0.14;        // Wheelbase width in meters
constexpr uint32_t kEncoderTicksPerRotation = 1440;    // Encoder resolution

// Convert radians to degrees
double radToDeg(const double theta) {
  return (180.0*theta)/M_PI;
}

// Convert encoder ticks to linear distance traveled
double ticksToDistance(const double enc_ticks) {
  return (0.07*M_PI/kEncoderTicksPerRotation)*enc_ticks;
}

// Convert encoder tick difference (left-right) to rotation angle
double ticksToAngle(const double enc_ticks) {
  return ticksToDistance(enc_ticks)/kDistanceBetweenWheels;
}

// Convert linear distance to encoder ticks
double distanceToTicks(const double d) {
  return (kEncoderTicksPerRotation*d)/(kWheelDiameter*M_PI);
}

// Convert rotation angle to encoder tick difference needed
double angleToTicks(const double theta_rad) {
  return distanceToTicks(kDistanceBetweenWheels*theta_rad);
}

// Euclidean distance between two positions
double distance(const Position& p1, const Position& p2) {
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  return sqrt(dx*dx + dy*dy);
}

// Calculate signed angle difference between current heading and direction to destination
// Returns positive if destination is to the left, negative if to the right
double headingDiff(const double yaw, const Position& src, const Position& dest) {
  const double dist = distance(dest, src);
  const Position dest_dir = {(dest.x-src.x)/dist, (dest.y-src.y)/dist};  // Unit vector to destination
  const Position head_dir = {cos(yaw), sin(yaw)};                        // Current heading unit vector
  const double angle = acos((dest_dir.x*head_dir.x + dest_dir.y*head_dir.y));
  const double cross_prod = head_dir.x*dest_dir.y - head_dir.y*dest_dir.x;  // Determine sign
  return (cross_prod < 0.0)?(-angle):angle;
}

// Constrain value to specified range
float clamp(const float val, const float min_range, const float max_range) {
  return min(max(min_range, val), max_range);
}

// Initialize PIDs (Kp, Ki, Kd) and motor controllers with calibrated parameters
Application::Application()
  : dist_pid(0.125f, 0.0f, 0.01f),                             // Distance control PID
    heading_pid(0.0625f, 0.0f, 0.01f),                         // Heading/yaw control PID
    left_motor(MotorType::LEFT, 0.1613f, 12.9032f, 0.0f),      // Left motor velocity PID
    right_motor(MotorType::RIGHT, 0.1613f, 12.9032f, 0.0f) {}  // Right motor velocity PID

// Setup hardware and initialize robot state
void Application::init() {
  Romi32U4Buzzer::playNote(NOTE_C(4), 200, 15);  // Startup beep
  Serial.begin(115200);
  prev_time_ms = 0;
  loop_count_last_hb = 0;
  loop_count = 0;
  loop_halt_count = 1000/DELAY_MS;
  left_motor.setTargetVelocity(0.0f);
  right_motor.setTargetVelocity(0.0f);
  i2c_slave.init(20);
  Romi32U4Encoders::init();
  reset();
  memset(&i2c_slave.buffer, 0, sizeof(Data));
}

// Main control loop: processes commands and runs waypoint navigation with PID control
void Application::loop() {
  // Check for I2C commands (non-blocking)
  const char cmd = getCommand();
  switch (cmd) {
    case 's':  // Get robot status
      getStatus();
      break;
    case 'w':  // Add waypoint
      wayPoint();
      break;
    case 'r':  // Reset robot state
      reset();
      break;
  }
  
  // Rate limiting: only run control loop at fixed frequency
  unsigned long curr_time_ms = millis();
  unsigned long diff_ms = curr_time_ms - prev_time_ms;
  if (diff_ms < DELAY_MS) {
    return;
  }
  prev_time_ms = curr_time_ms;

  ++loop_count;
  ++loop_count_last_hb;
  // Safety: stop motors if no heartbeat from host
  if (loop_count_last_hb > loop_halt_count) {
    left_motor.stop();
    right_motor.stop();
    return;
  }

  float dt = ((float)diff_ms)/1000.0f;  // Time delta in seconds
  const double d_ticks = distanceToTicks(distance(curr_dest, curr_pos));
  
  // Check if reached current waypoint (within ~3cm tolerance)
  if (abs(d_ticks) < 50.0) {
    if (wps_q.isEmpty()) {
      left_motor.stop();
      right_motor.stop();
      return;
    } else {
      // Move to next waypoint
      wps_q.front(curr_dest);
      wps_q.pop();
    }
  }

  // Calculate heading error and distance to waypoint
  const double yaw_diff = headingDiff(curr_yaw, curr_pos, curr_dest);
  const double angle_ticks = angleToTicks(yaw_diff);
  
  // Compute motor velocities using cascaded PID control
  {
    // Distance PID: scaled by cos(yaw_diff) to slow down when not facing target
    const int16_t dist_corr = static_cast<int16_t>(abs(cos(yaw_diff))*dist_pid.process(-d_ticks, dt, -50.0f, 50.0f));
    // Heading PID: corrects rotation to face target
    const int16_t heading_corr = static_cast<int16_t>(heading_pid.process(-angle_ticks, dt, -50.0f, 50.0f));
    // Differential drive: add heading correction to right, subtract from left
    right_motor.setTargetVelocity(clamp(dist_corr + heading_corr, -50, +50));
    left_motor.setTargetVelocity(clamp(dist_corr - heading_corr, -50, +50));
  }

  // Run motor velocity PIDs
  int16_t v_left, pwm_left;
  left_motor.runPIDLoop(dt, v_left, pwm_left);
  int16_t v_right, pwm_right;
  right_motor.runPIDLoop(dt, v_right, pwm_right);

  // Update odometry (dead reckoning from encoder values)
  double dist = ticksToDistance((v_left + v_right)/2.0);  // Average distance
  curr_yaw += ticksToAngle(v_right - v_left);              // Differential rotation
  curr_pos.x += (dist*cos(curr_yaw));
  curr_pos.y += (dist*sin(curr_yaw));
}

// Command 's': Send robot status to host (position, yaw, queue state)
// Format: x(mm), y(mm), yaw(mrad), timestamp, queue_full
void Application::getStatus() {
  loop_count_last_hb = 0;  // Reset heartbeat watchdog
  
  // Now set all status fields explicitly to ensure no leftover data
  i2c_slave.buffer.resp_data.cmd = 's';
  i2c_slave.buffer.resp_data.cmd_id = i2c_slave.buffer.cmd_data.cmd_id;
  
  Status& status = i2c_slave.buffer.resp_data.data.status;
  status.x = static_cast<int32_t>(1000*curr_pos.x);
  status.y = static_cast<int32_t>(1000*curr_pos.y);
  status.yaw = static_cast<int32_t>(1000*curr_yaw);
  status.is_queue_full = static_cast<uint8_t>(wps_q.isFull());
  status.time_stamp = static_cast<uint32_t>(prev_time_ms);
  i2c_slave.finalizeWrites();
}

// Command 'w': Add waypoint to navigation queue
// Format: x(mm), y(mm)
void Application::wayPoint() {
  Position pos;
  loop_count_last_hb = 0;  // Reset heartbeat watchdog
  pos.x = static_cast<double>(i2c_slave.buffer.cmd_data.data.way_point.x)/1000.0;  // Convert mm to meters
  pos.y = static_cast<double>(i2c_slave.buffer.cmd_data.data.way_point.y)/1000.0;
  wps_q.push(pos);
  
  i2c_slave.buffer.resp_data.cmd = 'w';
  i2c_slave.buffer.resp_data.cmd_id = i2c_slave.buffer.cmd_data.cmd_id;
  i2c_slave.buffer.resp_data.data.way_point.x = i2c_slave.buffer.cmd_data.data.way_point.x;
  i2c_slave.buffer.resp_data.data.way_point.y = i2c_slave.buffer.cmd_data.data.way_point.y;
  i2c_slave.finalizeWrites();
}

// Command 'r': Reset robot state to origin
void Application::reset() {
  curr_yaw = 0.0;
  curr_dest.x = curr_dest.y = 0.0;
  curr_pos.x = curr_pos.y = 0.0;
  Romi32U4Motors::setSpeeds(0, 0);
  left_motor.setTargetVelocity(0.0f);
  right_motor.setTargetVelocity(0.0f);
  dist_pid.setSetPoint(0.0f);
  heading_pid.setSetPoint(0.0f);
}

char Application::getCommand() {
  i2c_slave.updateBuffer();
  const auto& buffer = i2c_slave.buffer;
  return (buffer.cmd_data.cmd_id == buffer.resp_data.cmd_id)?'\n':buffer.cmd_data.cmd;
}
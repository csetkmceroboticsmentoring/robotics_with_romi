#include "application.h"

#include <Romi32U4.h>
#include <math.h>

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
constexpr uint32_t kEncoderTicksPerRotation = 1440;   // Encoder resolution

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
  : dist_pid(1.5f, 0.0f, 0.01f),                           // Distance control PID
    heading_pid(0.125f, 0.0f, 0.01f),                       // Heading/yaw control PID
    left_motor(MotorType::LEFT, 0.1613f, 12.9032f, 0.0f),   // Left motor velocity PID
    right_motor(MotorType::RIGHT, 0.1613f, 12.9032f, 0.0f) {}  // Right motor velocity PID

// Setup hardware and initialize robot state
void Application::init() {
  Romi32U4Buzzer::playNote(NOTE_C(4), 200, 15);  // Startup beep
  Serial.begin(115200);
  while (!Serial);  // Wait for serial connection
  Serial.print("Romi robot...\r\n");
  prev_time_ms = 0;
  loop_count_last_hb = 0;
  loop_count = 0;
  left_motor.setTargetVelocity(0.0f);
  right_motor.setTargetVelocity(0.0f);
  Romi32U4Encoders::init();
  reset();
}

// Main control loop: processes commands and runs waypoint navigation with PID control
void Application::loop() {
  // Check for serial commands (non-blocking)
  const char cmd = poll();
  switch (cmd) {
    case 'k':  // Set PID gains
      setGains();
      break;
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
      is_idle = true;
      left_motor.stop();
      right_motor.stop();
      return;
    } else {
      // Move to next waypoint
      wps_q.front(curr_dest);
      wps_q.pop();
      is_idle = false;
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

// Non-blocking check for serial data
char Application::poll() {
  char ch = '\n';
  if (Serial.available()) {
    ch = Serial.read();
  }
  return ch;
}

// Blocking read of a line from serial (waits for '\n')
void Application::readLine() {
  char ch;
  int length = 0;
  while ((ch = poll()) != '\n') {
    if (ch == '\r') {
      continue;  // Skip carriage returns
    }
    line[length++] = ch; 
  }
  line[length] = '\0';  // Null-terminate string
}

// Command 'k': Update PID gains for motor, distance, and heading controllers
// Format: kp_v, ki_v, kd_v, kp_dist, ki_dist, kd_dist, kp_yaw, ki_yaw, kd_yaw (all x1000)
void Application::setGains() {
  long int kp_v, ki_v, kd_v;
  long int kp_dist, ki_dist, kd_dist;
  long int kp_yaw, ki_yaw, kd_yaw;
  readLine();
  sscanf(line, " %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld", &kp_v, 
            &ki_v, &kd_v, 
            &kp_dist, &ki_dist, &kd_dist, 
            &kp_yaw, &ki_yaw, &kd_yaw);
  loop_count_last_hb = 0;  // Reset heartbeat watchdog
  loop_halt_count = 1000/DELAY_MS;  // Stop after 1 second without heartbeat
  // Echo gains back to host
  sprintf(line, "k: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %lu\r\n", 
              kp_v, ki_v, kd_v, 
              kp_dist, ki_dist, kd_dist, 
              kp_yaw, ki_yaw, kd_yaw, 
              millis());
  Serial.print(line);
  // Apply gains (convert from int*1000 to float)
  left_motor.setGains(static_cast<float>(kp_v)/1000.0f, static_cast<float>(ki_v)/1000.0f, static_cast<float>(kd_v)/1000.0f);
  right_motor.setGains(static_cast<float>(kp_v)/1000.0f, static_cast<float>(ki_v)/1000.0f, static_cast<float>(kd_v)/1000.0f);
  dist_pid.setGains(static_cast<float>(kp_dist)/1000.0f, static_cast<float>(ki_dist)/1000.0f, static_cast<float>(kd_dist)/1000.0f);
  heading_pid.setGains(static_cast<float>(kp_yaw)/1000.0f, static_cast<float>(ki_yaw)/1000.0f, static_cast<float>(kd_yaw)/1000.0f);
  reset();
}

// Command 's': Send robot status to host (position, yaw, queue state)
// Format: x(mm), y(mm), yaw(mrad), timestamp, queue_full, is_idle
void Application::getStatus() {
  loop_count_last_hb = 0;  // Reset heartbeat watchdog
  sprintf(line, "s: %d, %d, %d, %lu, %d, %d\r\n", 
    static_cast<int>(1000*curr_pos.x),   // x in millimeters
    static_cast<int>(1000*curr_pos.y),   // y in millimeters
    static_cast<int>(1000*curr_yaw),     // yaw in milliradians
    prev_time_ms, 
    (int)wps_q.isFull(), 
    (int)is_idle);
  Serial.print(line);
}

// Command 'w': Add waypoint to navigation queue
// Format: x(mm), y(mm)
void Application::wayPoint() {
  int x, y;
  Position pos;
  readLine();
  sscanf(line, " %d, %d", &x, &y);
  pos.x = static_cast<double>(x)/1000.0;  // Convert mm to meters
  pos.y = static_cast<double>(y)/1000.0;
  wps_q.push(pos);
  // Echo waypoint back to host
  sprintf(line, "w: %d, %d, %lu\r\n", 
    static_cast<int>(1000*pos.x), 
    static_cast<int>(1000*pos.y), 
    millis());
  Serial.print(line);
}

// Command 'r': Reset robot state to origin
void Application::reset() {
  curr_yaw = 0.0;
  curr_dest.x = curr_dest.y = 0.0;
  curr_pos.x = curr_pos.y = 0.0;
  is_idle = true;
  Romi32U4Motors::setSpeeds(0, 0);
  left_motor.setTargetVelocity(0.0f);
  right_motor.setTargetVelocity(0.0f);
  dist_pid.setSetPoint(0.0f);
  heading_pid.setSetPoint(0.0f);
}

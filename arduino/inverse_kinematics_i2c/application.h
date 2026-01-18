#pragma once

#include "pid_control.h"
#include "circular_queue.h"
#include "motor_with_encoder.h"

// 2D position in robot's coordinate frame (meters)
struct Position {
  double x;  // X coordinate (forward)
  double y;  // Y coordinate (left)
};

/**
 * Main application class for Romi robot waypoint navigation.
 * Implements cascaded PID control for autonomous navigation using:
 * - Distance PID: controls forward velocity toward waypoint
 * - Heading PID: corrects rotation to face waypoint
 * - Motor PIDs: control individual wheel velocities
 * 
 * Serial protocol: 'k'=set gains, 's'=status, 'w'=waypoint, 'r'=reset
 */
class Application {
public:
  Application();
  
  void init();   // Initialize hardware and reset state
  void loop();   // Main control loop (called repeatedly)

private:
  // I2C command handlers
  void getStatus();   // 's': Report robot state
  void wayPoint();    // 'w': Add navigation waypoint
  void reset();       // 'r': Reset to origin

  char getCommand();

  // Robot state
  bool is_idle;                    // True when no waypoints to navigate
  double curr_yaw;                 // Current heading angle (radians)
  Position curr_pos;               // Current position estimate (odometry)
  Position curr_dest;              // Current target waypoint
  
  // Timing and safety watchdog
  int loop_count;                  // Total control loop iterations
  int loop_count_last_hb;          // Loops since last host communication
  int loop_halt_count;             // Max loops before safety stop
  unsigned long prev_time_ms;      // Previous loop timestamp for dt calculation

  // Serial communication buffer
  char line[200];                  // Buffer for reading serial commands

  // Cascaded PID controllers
  PidControl dist_pid;             // Controls forward velocity toward waypoint
  PidControl heading_pid;          // Controls rotation to face waypoint
  MotorWithEncoder left_motor;     // Left wheel motor + velocity PID
  MotorWithEncoder right_motor;    // Right wheel motor + velocity PID

  // Navigation
  CircularQueue<Position> wps_q;   // Queue of waypoints to visit
};

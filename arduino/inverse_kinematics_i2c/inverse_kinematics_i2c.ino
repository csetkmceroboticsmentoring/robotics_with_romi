/**
 * Inverse Kinematics Robot Control
 * 
 * This sketch implements waypoint-based navigation for a differential drive robot.
 * The robot receives waypoints over serial/network and uses PID control to navigate
 * to each waypoint sequentially using inverse kinematics.
 */

#include "application.h"

Application app;

// Initialize application (motors, encoders, communication)
void setup() {
  app.init();
}

// Main control loop (runs at ~40Hz)
void loop() {
  app.loop();
}

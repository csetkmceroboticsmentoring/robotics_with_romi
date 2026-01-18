#pragma once

#include <stdint.h>
#include "pid_control.h"

// Motor identification
enum class MotorType { LEFT, RIGHT };

/**
 * Motor controller with encoder feedback and PID velocity control.
 * Maintains target velocity using encoder tick measurements.
 */
class MotorWithEncoder {
public:
  MotorWithEncoder(const MotorType type, const float kp, const float ki, const float kd);

  // Update PID gains
  void setGains(const float kp, const float ki, const float kd);
  
  // Initialize motor and encoder pins
  void init();

  // Get current target velocity (encoder ticks per sample)
  int16_t targetVelocity();

  // Set target velocity (encoder ticks per sample)
  void setTargetVelocity(const int16_t vel);

  // Execute PID control loop, returns velocity and PWM command
  void runPIDLoop(const float dt, int16_t& velocity, int16_t& pwm);

  // Stop motor and reset state
  void stop();

private:
  int16_t encoderCount();  // Read current encoder count
  int16_t velocityAsEncoderTicks();  // Calculate velocity from encoder
  int16_t encoderCountDiff(const int16_t curr_count, const int16_t prev_count);  // Handle overflow

  const MotorType motor_type;

  int16_t prev_counts;  // Previous encoder reading
  PidControl pid_control;
};

#pragma once

/**
 * PID controller with anti-windup.
 * Calculates control output based on proportional, integral, and derivative terms.
 */
class PidControl {
public: 
  PidControl(const float kp, 
             const float ki, 
             const float kd);

  // Update PID gains
  void setGains(const float kp, const float ki, const float kd);

  // Get current setpoint (target value)
  float getSetPoint() { return set_point; };

  // Set new setpoint (target value)
  void setSetPoint(const float t);

  // Calculate PID output with anti-windup clamping
  float process(const float obs,         // Observed value
                const float dt,          // Time step (seconds)
                const float min_range,   // Minimum output limit
                const float max_range);  // Maximum output limit

private:
  float Kp;  // Proportional gain
  float Ki;  // Integral gain
  float Kd;  // Derivative gain

  float set_point;     // Target value
  float prev_err;      // Previous error for derivative
  float err_integral;  // Accumulated error for integral (with anti-windup)
};

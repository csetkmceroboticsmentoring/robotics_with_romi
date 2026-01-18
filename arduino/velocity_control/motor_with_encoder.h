#pragma once

#include <stdint.h>
#include "pid_control.h"

enum class MotorType { LEFT, RIGHT };

class MotorWithEncoder {
public:
  MotorWithEncoder(const MotorType type, const float kp, const float ki, const float kd);

  void setGains(const float kp, const float ki, const float kd);
  
  void init();

  int16_t targetVelocity();

  void setTargetVelocity(const int16_t vel);

  void runPIDLoop(const float dt, int16_t& velocity, int16_t& pwm);

  void stop();

private:
  int16_t encoderCount();
  int16_t velocityAsEncoderTicks();
  int16_t encoderCountDiff(const int16_t curr_count, const int16_t prev_count);

  const MotorType motor_type;

  int16_t prev_counts;
  PidControl pid_control;
};

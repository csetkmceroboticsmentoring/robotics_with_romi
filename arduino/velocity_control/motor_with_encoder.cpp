#include <Romi32U4.h>

#include "motor_with_encoder.h"

MotorWithEncoder::MotorWithEncoder(const MotorType type, const float kp, const float ki, const float kd)
    : motor_type(type),
      prev_counts(0),
      pid_control(kp, ki, kd) {};
  
void MotorWithEncoder::init() {
  prev_counts = encoderCount();
}

int16_t MotorWithEncoder::targetVelocity() {
  return static_cast<int16_t>(pid_control.getSetPoint());
}

void MotorWithEncoder::setTargetVelocity(const int16_t vel) {
  pid_control.setSetPoint(static_cast<float>(vel));
}

void MotorWithEncoder::setGains(const float kp, const float ki, const float kd) {
  pid_control.setGains(kp, ki, kd);
}

int16_t MotorWithEncoder::velocityAsEncoderTicks() {
  int16_t counts = encoderCount();
  int16_t v = encoderCountDiff(counts, prev_counts);
  prev_counts = counts;
  return v;
}

int16_t MotorWithEncoder::encoderCountDiff(const int16_t curr_count, const int16_t prev_count) {
  const uint16_t curr_sign = ((uint16_t)curr_count) & 0x8000;
  const uint16_t prev_sign = ((uint16_t)prev_count) & 0x8000;
  const int16_t diff = (curr_sign == prev_sign)?(curr_count - prev_count):(((uint16_t)curr_count) - prev_count);
  return diff;
}

int16_t MotorWithEncoder::encoderCount() {
  if (motor_type == MotorType::LEFT) {
    return Romi32U4Encoders::getCountsLeft();
  } else if (motor_type == MotorType::RIGHT) {
    return Romi32U4Encoders::getCountsRight();
  }
  return 0;
}

void MotorWithEncoder::stop() {
  if (motor_type == MotorType::LEFT) {
    Romi32U4Motors::setLeftSpeed(0);
  } else if (motor_type == MotorType::RIGHT) {
    Romi32U4Motors::setRightSpeed(0);
  } 
}

void MotorWithEncoder::runPIDLoop(const float dt, int16_t& velocity, int16_t& pwm) {
  velocity = velocityAsEncoderTicks();
  pwm = static_cast<int16_t>(pid_control.process(static_cast<float>(velocity), dt, -300, +300));
  if (motor_type == MotorType::LEFT) {
    Romi32U4Motors::setLeftSpeed(pwm);
  } else if (motor_type == MotorType::RIGHT) {
    Romi32U4Motors::setRightSpeed(pwm);
  }
}

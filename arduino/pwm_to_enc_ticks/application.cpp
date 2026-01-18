#include "application.h"

Application::Application()
  : direction(true),
    pwm_value(0),
    sample_count(0) {}

void Application::init() {
  Romi32U4Encoders::init();
  left_motor.init();
  right_motor.init();
}

void Application::start() {
  direction = true;
  pwm_value = 260;
  sample_count = 0;
}

void Application::spinOnce() {
  if (pwm_value == 0) {
    left_motor.setPwm(0);
    right_motor.setPwm(0);
    return;
  }

  if (sample_count == 0) {
    if (direction) {
      pwm_value -= 10;
    }
    const int16_t pwm = direction?pwm_value:(-pwm_value);
    left_motor.setPwm(pwm);
    right_motor.setPwm(pwm);
    // Change motor direction
    direction = !direction;
    // Spin for 1 second.
    sample_count = 40;
  }
  --sample_count;
  printCurrentStatus(left_motor.velocityAsEncoderTicks(), right_motor.velocityAsEncoderTicks());
}

void Application::printCurrentStatus(const int16_t left_v, const int16_t right_v) {
  int16_t pwm = direction?(-pwm_value):pwm_value;
  sprintf(line, "%hi, %hi, %hi, %hi\r\n", pwm, pwm, left_v, right_v);
  Serial.print(line);
}

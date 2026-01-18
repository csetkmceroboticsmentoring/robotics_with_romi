#pragma once

#include "motor_with_encoder.h"

class VelocityControlApp {
public:
  VelocityControlApp();

  void init();
  
  void loop();

private:
  void setGains();
  void setVelocity();

  void readLine();

  unsigned long prev_time_ms;

  char line[100];

  MotorWithEncoder left_motor;
  MotorWithEncoder right_motor;
};

#include "pid_control.h"

PidControl::PidControl(const float kp, const float ki, const float kd) {
  setGains(kp, ki, kd);
}

void PidControl::setGains(const float kp, const float ki, const float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  
  set_point = 0.0f;
  prev_err = 0.0f;
  err_integral = 0.0f;
}

void PidControl::setSetPoint(const float sp) { 
  set_point = sp; 
}

float PidControl::process(const float obs, const float dt, const float min_range, const float max_range) {
  const float err = set_point-obs; 
  const float der_err = (err - prev_err) / dt;
  prev_err = err; 
  err_integral += (err * dt); 
  float out = (Kp * err) + (Ki * err_integral) + (Kd * der_err);
  if (out < min_range) {
    out = min_range;
    err_integral -= (err*dt);
  } else if (out > max_range) {
    out = max_range;
    err_integral -= (err*dt);
  }
  return out; 
}

#pragma once

class PidControl {
public: 
  PidControl(const float kp, 
             const float ki, 
             const float kd);

  void setGains(const float kp, const float ki, const float kd);

  float getSetPoint() { return set_point; };

  void setSetPoint(const float t);

  float process(const float obs, 
                const float dt, 
                const float min_range, 
                const float max_range);

private:
  float Kp;
  float Ki;
  float Kd;

  float set_point;
  float prev_err;
  float err_integral;
};

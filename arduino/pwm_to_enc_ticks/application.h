#include "motor_with_encoder.h"

class Application {
public:
  Application();

  void init();
  
  void start();
  
  void spinOnce();

private:
  void printCurrentStatus(const int16_t left_v, const int16_t right_v);

  bool direction;
  int16_t pwm_value;
  int sample_count;

  char line[100];

  MotorWithEncoder<LEFT_WHEEL> left_motor;
  MotorWithEncoder<RIGHT_WHEEL> right_motor;
};


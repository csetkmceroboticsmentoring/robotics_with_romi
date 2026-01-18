#include <Romi32U4.h>

enum WHEEL_TYPE { LEFT_WHEEL, RIGHT_WHEEL };

template <WHEEL_TYPE WheelT>
class MotorWithEncoder {
public:
  MotorWithEncoder();
  
  void init();

  void setPwm(const int16_t pwm);

  int16_t velocityAsEncoderTicks();

private:
  int16_t encoderCount();
  int16_t encoderCountDiff(const int16_t curr_count, const int16_t prev_count);

  int16_t prev_counts;
};

template <WHEEL_TYPE WheelT>
MotorWithEncoder<WheelT>::MotorWithEncoder()
    : prev_counts(0) {};
  
template <WHEEL_TYPE WheelT>
void MotorWithEncoder<WheelT>::init() {
  prev_counts = encoderCount();
}

template <WHEEL_TYPE WheelT>
void MotorWithEncoder<WheelT>::setPwm(const int16_t pwm) {
  if (WheelT == LEFT_WHEEL) {
    Romi32U4Motors::setLeftSpeed(pwm);
  } else if (WheelT == RIGHT_WHEEL) {
    Romi32U4Motors::setRightSpeed(pwm);
  }
}

template <WHEEL_TYPE WheelT>
int16_t MotorWithEncoder<WheelT>::velocityAsEncoderTicks() {
  int16_t counts = encoderCount();
  int16_t v = encoderCountDiff(counts, prev_counts);
  prev_counts = counts;
  return v;
}

template <WHEEL_TYPE WheelT>
int16_t MotorWithEncoder<WheelT>::encoderCountDiff(const int16_t curr_count, const int16_t prev_count) {
  const uint16_t curr_sign = ((uint16_t)curr_count) & 0x8000;
  const uint16_t prev_sign = ((uint16_t)prev_count) & 0x8000;
  const int16_t diff = (curr_sign == prev_sign)?(curr_count - prev_count):(((uint16_t)curr_count) - prev_count);
  return diff;
}

template <WHEEL_TYPE WheelT>
int16_t MotorWithEncoder<WheelT>::encoderCount() {
  if (WheelT == LEFT_WHEEL) {
    return Romi32U4Encoders::getCountsLeft();
  } else if (WheelT == RIGHT_WHEEL) {
    return Romi32U4Encoders::getCountsRight();
  }
  return 0;
}

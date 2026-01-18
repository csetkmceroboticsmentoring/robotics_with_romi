#include "velocity_control_app.h"

#include <Romi32U4.h>

#define SAMPLING_TIME_MS 25

char poll() {
  char ch = '\n';
  if (Serial.available()) {
    ch = Serial.read();
  }
  return ch;
}

VelocityControlApp::VelocityControlApp()
  : left_motor(MotorType::LEFT, 0.1613f, 12.9032f, 0.0f),
    right_motor(MotorType::RIGHT, 0.1613f, 12.9032f, 0.0f) {}

void VelocityControlApp::init() {
  Romi32U4Buzzer::playNote(NOTE_C(4), 200, 15);
  Serial.begin(115200);
  while (!Serial);
  Serial.print("Romi robot...\r\n");
  Romi32U4Encoders::init();
  left_motor.init();
  right_motor.init();
  prev_time_ms = millis();
}

void VelocityControlApp::setGains() {
  left_motor.stop();
  right_motor.stop();

  readLine();
  int kp_int, ki_int, kd_int;
  sscanf(line, " %d, %d, %d", &kp_int, &ki_int, &kd_int);
  sprintf(line, "g: %d, %d, %d\r\n", kp_int, ki_int, kd_int);
  Serial.print(line);
  left_motor.setGains(static_cast<float>(kp_int)/1000.0f, static_cast<float>(ki_int)/1000.0f, static_cast<float>(kd_int)/1000.0f);
  right_motor.setGains(static_cast<float>(kp_int)/1000.0f, static_cast<float>(ki_int)/1000.0f, static_cast<float>(kd_int)/1000.0f);
  prev_time_ms = millis();
}

void VelocityControlApp::setVelocity() {
  readLine();
  int16_t left_target_vel = 0, right_target_vel = 0;
  sscanf(line, " %hi, %hi", &left_target_vel, &right_target_vel);
  sprintf(line, "v: %hi, %hi\r\n", left_target_vel, right_target_vel);
  Serial.print(line);
  left_motor.setTargetVelocity(left_target_vel);
  right_motor.setTargetVelocity(right_target_vel);
  prev_time_ms = millis();
}

void VelocityControlApp::readLine() {
  char ch;
  int length = 0;
  while ((ch = poll()) != '\n') {
    if (ch == '\r') {
      continue;
    }
    line[length++] = ch; 
  }
  line[length] = '\0';
}

void VelocityControlApp::loop() {
  const char cmd = poll();
  switch (cmd) {
    case 'g':
      setGains();
      break;
    case 'v':
      setVelocity();
      break;
  }

  unsigned long curr_time_ms = millis();
  unsigned long diff_ms = curr_time_ms - prev_time_ms;
  if (diff_ms < SAMPLING_TIME_MS) {
    return;
  }
  prev_time_ms = curr_time_ms;

  if (left_motor.targetVelocity() == 0) {
    left_motor.stop();
  }
  if (right_motor.targetVelocity() == 0) {
    right_motor.stop();
  }

  {
    int16_t left_pwm = 0, right_pwm = 0;
    int16_t left_vel = 0, right_vel = 0;
    const float dt = static_cast<float>(diff_ms)/1000.0f;
    left_motor.runPIDLoop(dt, left_vel, left_pwm);
    right_motor.runPIDLoop(dt, right_vel, right_pwm);

    if (left_motor.targetVelocity() != 0 || right_motor.targetVelocity() != 0) {
      sprintf(line, "%hi, %hi, %hi, %hi, %hi, %hi, %d\r\n",
        left_vel, left_motor.targetVelocity(), left_pwm,
        right_vel, right_motor.targetVelocity(), right_pwm,
        (int)(1000*dt));
      Serial.print(line);
    }
  }
}

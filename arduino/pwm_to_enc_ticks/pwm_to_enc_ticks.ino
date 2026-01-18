#include "application.h"

#define SAMPLING_TIME_MS 25

char poll() {
  char ch = '\n';
  if (Serial.available()) {
    ch = Serial.read();
  }
  return ch;
}

Application app;
unsigned long prev_time_ms;

void setup() {
  Romi32U4Buzzer::playNote(NOTE_C(4), 200, 15);
  Serial.begin(115200);
  while (!Serial);
  Serial.print("Romi robot...\r\n");
  
  app.init();
  prev_time_ms = millis();
}

void loop() {
  const char cmd = poll();
  switch (cmd) {
    case 's':
      app.start();
      break;
  }

  unsigned long curr_time_ms = millis();
  unsigned long diff_ms = curr_time_ms - prev_time_ms;
  if (diff_ms < SAMPLING_TIME_MS) {
    return;
  }
  prev_time_ms = curr_time_ms;

  app.spinOnce();
}

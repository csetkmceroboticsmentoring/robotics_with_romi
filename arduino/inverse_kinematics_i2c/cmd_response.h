#pragma once

#include <stdint.h>

// Pack structures to match Qt/C++ side alignment (no padding)
// Note: Arduino AVR doesn't support #pragma pack, but structures should align naturally
// We use int32_t to match the 32-bit integers on the Qt/C++ side

struct Origin {
  int32_t x;
  int32_t y;
  int32_t yaw;
};

struct Waypoint {
  int32_t x;
  int32_t y;
};

struct Status {
  int32_t x;
  int32_t y;
  int32_t yaw;
  uint8_t is_queue_full;
  uint32_t time_stamp;
};

struct CommandData {
  char cmd;
  uint8_t cmd_id;
  union CommandUnion {
    Origin origin;
    Waypoint way_point;
  } data;
};

struct ResponseData {
  char cmd;
  uint8_t cmd_id;
  union ResponseUnion {
    Origin origin;
    Waypoint way_point;
    Status status;
  } data;
};

struct Data {
  CommandData cmd_data;
  ResponseData resp_data;
};


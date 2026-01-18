#pragma once

#include <cstdint>

// Pack structures to match Arduino alignment (no padding)
#pragma pack(push, 1)

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

#pragma pack(pop)


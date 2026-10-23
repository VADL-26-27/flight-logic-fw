#ifndef LOG_H_
#define LOG_H_

#define PRELAUNCH_RECORDS 500U
#include <stdint.h>
#include "flight_state_logic.h"

typedef struct {
  uint32_t timestamp_ms;
  uint32_t sequence;

  float altitude;
  float velocity;
  float acceleration;

  State flightState;
} record_t;

typedef struct {
  record_t records[PRELAUNCH_RECORDS];
  uint16_t next_write;
  uint16_t count;
} prelaunch_buf;

#endif // LOG_H_
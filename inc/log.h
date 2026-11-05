#ifndef LOG_H_
#define LOG_H_

#include "flash_mem.h"

#define PRELAUNCH_RECORDS 3000U
#define FLIGHT_BUFFER_RECORDS 256U

#include <stdint.h>
#include "flight_state_logic.h"
#include "imu.h"

typedef struct {
  uint32_t timestamp_ms;
  IMUPacket rawPacket;
  IMUState postKf;
  State flightState;
} IMURecord;
typedef enum {
  LOG_BUF_FREE,
  LOG_BUF_FILLING,
  LOG_BUF_READY,
  LOG_BUF_WRITING
} BufferState;
typedef struct {
  IMURecord records[PRELAUNCH_RECORDS];
  uint32_t next_write;
  uint32_t count;
} PreLaunchBuf;

typedef struct {
  IMURecord records[FLIGHT_BUFFER_RECORDS];
  uint32_t count;
  BufferState state;
} FlightBuf;

void logger_init(flash_t *device);
void logger_poll(void);

bool logger_request_clear(void);
const char *logger_status_reply(void);
bool logger_request_dump(void);
/* Returns true after the last byte has been queued. */
bool logger_dump_poll(bool (*send)(const uint8_t *, uint32_t));
void logger_start(void);
void logger_finish(void);
void logger_push_ground(const IMURecord *record);
void logger_push_flight(const IMURecord *record);

#endif // LOG_H_

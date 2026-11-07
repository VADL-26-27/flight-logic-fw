#ifndef FLIGHT_STATE_LOGIC_H_
#define FLIGHT_STATE_LOGIC_H_
#include <stdint.h>
#include "latch.h"
typedef enum { STANDBY, LAUNCH, DESCENT, DEPLOYMENT, LANDED } State;

void FlightState_Update(State *current_flight_state, float altitude,
                        float velocity, float accel, float maxAltitude);

extern uint8_t deployed;
extern uint32_t deployed_time;

#endif // FLIGHT_STATE_LOGIC_H_
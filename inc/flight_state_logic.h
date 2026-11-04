#ifndef FLIGHT_STATE_LOGIC_H_
#define FLIGHT_STATE_LOGIC_H_
#include <stdint.h>
 
typedef enum { STANDBY, LAUNCH, DESCENT, DEPLOYMENT, LANDED } State;
 
void FlightState_Update(State *current_flight_state, float altitude, float velocity, float accel, float maxAltitude, float ground_altitude);
 
#endif // FLIGHT_STATE_LOGIC_H_
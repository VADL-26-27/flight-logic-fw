#ifndef flight_state_logic
#define flight_state_logic
#include <stdint.h>
 
typedef enum { STANDBY, LAUNCH, DESCENT, DEPLOYMENT, LANDED } State;
 
void FlightState_Update(State *current_flight_state, float altitude, float velocity, float accel, float maxAltitude, float ground_altitude);
 
#endif
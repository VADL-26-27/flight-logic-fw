#include <math.h>
#include <stdint.h>
#include "flight_state_logic.h"
#include "hardware.h"

static uint32_t LAUNCH_ACCEL_THRESH = 10; // 10m/s^2
static uint32_t LAUNCH_TIME_THRESH = 5;   // 5ms
uint32_t launch_start_time = 0;
uint32_t launch_time = 0;

static uint32_t DESCENT_ALTITUDE_THRESH = 10; // 10m/s^2
static uint32_t DESCENT_TIME_THRESH = 5;      // 5ms
uint32_t descent_start_time = 0;
uint32_t descent_time = 0;
uint32_t max_altitude = 0;

static uint32_t DEPLOYMENT_ALTITUDE_THRESH = 10; // 10m/s^2
static uint32_t DEPLOYMENT_TIME_THRESH = 5;      // 5ms
uint32_t deployment_start_time = 0;
uint32_t deployment_time = 0;

static uint32_t LANDED_VELOCITY_THRESH = 10; // 10m/s^2
static uint32_t LANDED_TIME_THRESH = 5;      // 5ms
uint32_t landed_start_time = 0;
uint32_t landed_time = 0;

void FlightState_Update(State *current_flight_state, float altitude, float velocity, float accel, float maxAltitude) {

    static uint8_t timer = 0;
    switch (*current_flight_state) {
      case STANDBY: //STANDBY
      //while accelaration is less than threshhold and
      //launch time less than threshold
          if (accel >= LAUNCH_ACCEL_THRESH) {

              if (timer == 0) {
                  launch_start_time = millis();
                  timer = 1;
              }

              if (millis() - launch_start_time >= LAUNCH_TIME_THRESH) {
                  launch_time = millis() - launch_start_time;
                  *current_flight_state = LAUNCH;
                  timer = 0;
              }

          } else {
              timer = 0;
          }
          break;

      case LAUNCH: //LAUNCH
      //while altitude is greater than max altitude - 500

          if (altitude < maxAltitude - DESCENT_ALTITUDE_THRESH) {

              if (timer == 0) {
                  descent_start_time = millis();
                  timer = 1;
              }

              if (millis() - descent_start_time >= DESCENT_TIME_THRESH)
              {
                  descent_time = millis() - descent_start_time;
                  *current_flight_state = DESCENT;
                  timer = 0;
              }

          } else {
              timer = 0;
          }
          break;
      case DESCENT: //DESCENT
      //while altitude is greater than deployment altitude threshold

          if (altitude < DEPLOYMENT_ALTITUDE_THRESH) {

              if (timer == 0) {
                  deployment_start_time = millis();
                  timer = 1;
              }

              if (millis() - deployment_start_time >=
              DEPLOYMENT_TIME_THRESH) {
                  deployment_time = millis() - deployment_start_time;
                  *current_flight_state = DEPLOYMENT;
                  timer = 0;
              }

          } else {
              timer = 0;
          }
          break;
      case DEPLOYMENT: //DEPLOYMENT
      //while velocity is less than velocity threshold
      //verify if the velocity will be negative or positive here
          if (velocity < LANDED_VELOCITY_THRESH) { 

              if (timer == 0) {
                  landed_start_time = millis();
                  timer = 1;
              }

              if (millis() - landed_start_time >= LANDED_TIME_THRESH) {
                  landed_time = millis() - landed_start_time;
                  *current_flight_state = LANDED;
                  timer = 0;
              }

          } else {
              timer = 0;
          }
          break;
      case LANDED: //LANDED
      //set some pins high
        break;

      }
}
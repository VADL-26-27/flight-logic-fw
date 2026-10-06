#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include <stdint.h>
#include "usart1.h"
#include "usart2.h"
#include "hardware.h"
#include "imu.h"
#include "vn_kalman.h"
#include <math.h>

#define PACKET_SIZE 36

char *imu_command = "$VNWRG,75,2,8,05,0108,0020*XX\r\n\0";
// char *imu_command = "100\0"; For UART debugging with a fake IMU (Teensy)

uint8_t imu_packet[PACKET_SIZE];
IMUState state;

vn_kf_t kalman_filter;

static uint32_t LAUNCH_ACCEL_THRESH = 10; // 10m/s^2
static uint32_t LAUNCH_TIME_THRESH = 5;   // 5ms
uint32_t accel = 0;
uint32_t launch_start_time = 0;
uint32_t launch_time = 0;

static uint32_t DESCENT_ALTITUDE_THRESH = 10; // 10m/s^2
static uint32_t DESCENT_TIME_THRESH = 5;      // 5ms
uint32_t altitude = 0;
uint32_t descent_start_time = 0;
uint32_t descent_time = 0;
uint32_t max_altitude = 0;

static uint32_t DEPLOYMENT_ALTITUDE_THRESH = 10; // 10m/s^2
static uint32_t DEPLOYMENT_TIME_THRESH = 5;      // 5ms
uint32_t deployment_start_time = 0;
uint32_t deployment_time = 0;

static uint32_t LANDED_VELOCITY_THRESH = 10; // 10m/s^2
static uint32_t LANDED_TIME_THRESH = 5;      // 5ms
uint32_t velocity = 0;
uint32_t landed_start_time = 0;
uint32_t landed_time = 0;

static float maxAltitude = 0.0f;
static float maxVelocity = 0.0f;
static float maxAccel = 0.0f;

typedef enum { STANDBY, LAUNCH, DESCENT, DEPLOYMENT, LANDED } State;

State current_flight_state = STANDBY;
uint32_t first_detection_time = 0;
uint8_t timer = 0;

uint8_t packet_count = 0;
void FlightState_Update();

int main(void) {
  vn_kf_init(&kalman_filter, 0.5f, 0.01f, 0.5f);
  hardwareInit();
  usart2_init();
  usart1_init();

  // We should have this send on a set interval until we receive our first
  // packet
  usart1_write_command(imu_command);

  while (1) {
    if (new_data) {
      new_data = 0;
      if (rx_buf.count >= PACKET_SIZE) {
        read_from_buf(&rx_buf, imu_packet, PACKET_SIZE);
        if (validate_packet(imu_packet)) {
          // kalman filter logic
          updateState(imu_packet, &state, &kalman_filter);
          ++packet_count;
          if (packet_count == 10) {
            usart2_write_packet(imu_packet);
            packet_count = 0;
          }
        }
      }
    }

    if (state.altitude > maxAltitude) {
      maxAltitude = state.altitude;
    }

    if (state.velocity > maxVelocity) {
      maxVelocity = state.velocity;
    }

    if (state.acceleration > maxAccel) {
      maxAccel = state.acceleration;
    }

    FlightState_Update();
    // TODO: real processing of rx_buffer goes here later
  }
  return 0;
}

void FlightState_Update() {

  // switch (current_flight_state) {
  //     case STANDBY: //STANDBY
  //     //while accelaration is less than threshhold and
  //     //launch time less than threshold
  //         if (accel >= LAUNCH_ACCEL_THRESH) {

  //             if (timer == 0) {
  //                 launch_start_time = HAL_GetTick();
  //                 timer = 1;
  //             }

  //             if (HAL_GetTick() - launch_start_time >= LAUNCH_TIME_THRESH) {
  //                 launch_time = HAL_GetTick() - launch_start_time;
  //                 current_flight_state = LAUNCH;
  //             }

  //         } else {
  //             timer = 0;
  //         }
  //         break;

  //     case LAUNCH: //LAUNCH
  //     //while altitude is greater than max altitude - 500

  //         if (altitude < maxAltitude - DESCENT_ALTITUDE_THRESH) {

  //             if (timer == 0) {
  //                 descent_start_time = HAL_GetTick();
  //                 timer = 1;
  //             }

  //             if (HAL_GetTick() - descent_start_time >= DESCENT_TIME_THRESH)
  //             {
  //                 descent_time = HAL_GetTick() - descent_start_time;
  //                 current_flight_state = DESCENT;
  //             }

  //         } else {
  //             timer = 0;
  //         }
  //         break;
  //     case DESCENT: //DESCENT
  //     //while altitude is greater than deployment altitude threshold

  //         if (altitude < DEPLOYMENT_ALTITUDE_THRESH) {

  //             if (timer == 0) {
  //                 deployment_start_time = HAL_GetTick();
  //                 timer = 1;
  //             }

  //             if (HAL_GetTick() - deployment_start_time >=
  //             DEPLOYMENT_TIME_THRESH) {
  //                 deployment_time = HAL_GetTick() - deployment_start_time;
  //                 current_flight_state = DEPLOYMENT;
  //             }

  //         } else {
  //             timer = 0;
  //         }
  //         break;
  //     case DEPLOYMENT: //DEPLOYMENT
  //     //while velocity is less than velocity threshold
  //         if (velocity < LANDED_VELOCITY_THRESH) {

  //             if (timer == 0) {
  //                 landed_start_time = HAL_GetTick();
  //                 timer = 1;
  //             }

  //             if (HAL_GetTick() - landed_start_time >= LANDED_TIME_THRESH) {
  //                 landed_time = HAL_GetTick() - landed_start_time;
  //                 current_flight_state = LANDED;
  //             }

  //         } else {
  //             timer = 0;
  //         }
  //         break;
  //     case LANDED: //LANDED
  //     //set some pins high

  //     }
}

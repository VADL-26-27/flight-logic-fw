#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include <stdint.h>
#include "usart1.h"
#include "usart2.h"
#include "hardware.h"
#include "flight_state_logic.h"
#include "imu.h"
#include "vn_kalman.h"
#include <math.h>

#define PACKET_SIZE 36

char *imu_command = "$VNWRG,75,2,8,05,0108,0020*XX\r\n\0";
// char *imu_command = "100\0"; For UART debugging with a fake IMU (Teensy)

uint8_t imu_packet[PACKET_SIZE];
IMUState state;

vn_kf_t kalman_filter;

static float maxAltitude = 0.0f;
static float maxVelocity = 0.0f;
static float maxAccel = 0.0f;
static float ground_altitude = 10.0f;

State current_flight_state = STANDBY;
uint32_t first_detection_time = 0;
uint8_t timer = 0;

uint8_t packet_count = 0;
uint8_t synced = 0;

int main(void) {
  vn_kf_init(&kalman_filter, 0.5f, 0.01f, 0.5f);
  hardwareInit();
  state_leds_init();     
  state_leds_all_on();
  usart2_init();
  usart1_init();

  // We should have this send on a set interval until we receive our first
  // packet
  usart1_write_command(imu_command);

  while (1) {
    if (new_data) {
      new_data = 0;

      while (!synced && rx_buf.count > 0) {
        read_from_buf(&rx_buf, imu_packet, 1); //read one packet in
        if (imu_packet[0] == 0xFA) { // looking for the header byte
          synced = 1;
        }
      }

      if (synced && rx_buf.count >= PACKET_SIZE - 1) {
        read_from_buf(&rx_buf, &imu_packet[1], PACKET_SIZE - 1);
        synced = 0; //reseting the synced value
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

    FlightState_Update(&current_flight_state, state.altitude, state.velocity, state.acceleration, maxAltitude, ground_altitude);
    //state_leds_all_on();
    state_leds_show(current_flight_state);
    // TODO: real processing of rx_buffer goes here later
  }
  return 0;
}

#include "stm32f4xx.h"
#include "stm32f413xx.h"
#include <stdint.h>
#include "hardware.h"
#include "flight_state_logic.h"
#include "imu.h"
#include "vn_kalman.h"
#include "timer.h"
#include "usart_board.h"
#include "latch.h"
#include <math.h>

const char imu_command[] = "$VNWRG,75,2,8,05,0108,0020*XX\r\n";

uint8_t imu_packet[PACKET_SIZE];
IMUState state;

vn_kf_t kalman_filter;

static float maxAltitude = 0.0f;
static float maxVelocity = 0.0f;
static float maxAccel = 0.0f;

State current_flight_state = STANDBY;
uint32_t first_detection_time = 0;
uint8_t timer = 0;

uint8_t packet_count = 0;
uint8_t synced = 0;

int main(void) {

  vn_kf_init(&kalman_filter, 0.5f, 0.01f, 0.5f);

  hardwareInit();
  board_usart_init();

  // We should have this send on a set interval until we receive our first
  // packet
  usart_write(&uart1, (const uint8_t *)imu_command, sizeof(imu_command) - 1U);
  delay_ms(500U);

  while (1) {
    while (1) {
      // Search for the packet header.
      if (!synced) {
        if (!usart_read(&uart1, &imu_packet[0], 1U)) {
          break; // No more bytes available.
        }

        if (imu_packet[0] != 0xFA) {
          continue;
        }

        synced = 1;
      }

      // Read the remaining packet once all its bytes are available.
      if (!usart_read(&uart1, &imu_packet[1], PACKET_SIZE - 1U)) {
        break; // Preserve synced until more bytes arrive.
      }

      synced = 0;

      if (validate_packet(imu_packet)) {
        updateState(imu_packet, &state, &kalman_filter);

        if (++packet_count >= 10U) {
          if (!usart_write(&uart2, imu_packet, PACKET_SIZE)) {
            // Increment a telemetry-drop counter here.
          }

          packet_count = 0;
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
    FlightState_Update(&current_flight_state, state.altitude, state.velocity,
                       state.acceleration, maxAltitude);
  }
  return 0;
}

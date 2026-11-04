#include "stm32f4xx.h"
#include "stm32f413xx.h"

#include "hardware.h"
#include "flight_state_logic.h"
#include "imu.h"
#include "vn_kalman.h"
#include "timer.h"
#include "usart_board.h"
#include "latch.h"
#include "spi_board.h"
#include "flash_mem.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

const char imu_command[] = "$VNWRG,75,2,8,05,0108,0020*XX\r\n";

uint8_t imu_packet[PACKET_SIZE];
IMUState state;
vn_kf_t kalman_filter;

static float maxAltitude;
static float maxVelocity;
static float maxAccel;

State current_flight_state = STANDBY;
uint8_t synced;

#define TEST_ADDRESS 0x00000000U

uint8_t message[] = "Hello world! 0\n";
static uint8_t readback[sizeof(message)];
static uint32_t test_number = 0U;

uint32_t last_test_finished_ms = 0U;
bool waiting_to_restart = false;

uint32_t delay = 2000U;
bool on = false;
uint32_t switch_on;
uint8_t packet_count = 0;

typedef enum {
  TEST_PROBE,
  TEST_ERASE,
  TEST_WRITE,
  TEST_READ,
  TEST_FINISHED
} flash_test_stage_t;

static flash_test_stage_t flash_test_stage;

/* Inspect these in the debugger. */
static volatile bool flash_test_done;
static volatile bool flash_test_passed;
static volatile bool flash_fault;
static volatile uint32_t detected_id;

static bool imu_started = false;
static uint32_t last_imu_command_ms;

static void flash_test_fail(void) {
  flash_fault = true;
  flash_test_done = true;
  flash_test_stage = TEST_FINISHED;
}

/* Called after spi_poll() and flash_poll(). */
static void flash_test_poll(void) {
  if (flash_test_done || flash.busy) {
    return;
  }

  if (flash.result == FLASH_RESULT_FAILED) {
    flash_ack_result(&flash);
    flash_test_fail();
    return;
  }

  if (flash.result != FLASH_RESULT_OK) {
    return;
  }

  flash_ack_result(&flash);

  switch (flash_test_stage) {
  case TEST_PROBE:
    detected_id = flash.jedec;

    /* Destructive: erase the first 4 KiB test sector. */
    if (!flash_erase_sector(&flash, TEST_ADDRESS)) {
      const uint8_t message[] = "Erase Failed!\n";
      usart_write(&uart2, (const uint8_t *)message, sizeof(message) - 1U);
      flash_test_fail();
      return;
    }

    flash_test_stage = TEST_ERASE;
    break;

  case TEST_ERASE:
    if (!flash_program(&flash, TEST_ADDRESS, message, sizeof(message))) {
      const uint8_t message[] = "Write Failed!\n";
      usart_write(&uart2, (const uint8_t *)message, sizeof(message) - 1U);
      flash_test_fail();
      return;
    }

    flash_test_stage = TEST_WRITE;
    break;

  case TEST_WRITE:
    if (!flash_read(&flash, TEST_ADDRESS, readback, sizeof(readback))) {
      const uint8_t message[] = "Read Failed!\n";
      usart_write(&uart2, (const uint8_t *)message, sizeof(message) - 1U);
      flash_test_fail();
      return;
    }

    flash_test_stage = TEST_READ;
    break;

  case TEST_READ:
    flash_test_passed = memcmp(message, readback, sizeof(message)) == 0;

    if (flash_test_passed) {
      const uint8_t message[] = "Test Passed!\n";
      usart_write(&uart2, (const uint8_t *)message, sizeof(message) - 1U);
      usart_write(&uart2, readback, sizeof(readback) - 1U);
    } else {
      const uint8_t message[] = "Test Failed!\n";
      usart_write(&uart2, (const uint8_t *)message, sizeof(message) - 1U);
    }
    flash_fault = !flash_test_passed;
    flash_test_done = true;
    flash_test_stage = TEST_FINISHED;
    break;

  default:
    flash_test_fail();
    break;
  }
}

int main(void) {
  vn_kf_init(&kalman_filter, 0.5f, 0.01f, 0.5f);

  hardwareInit();
  board_usart_init();
  board_spi_init();

  usart_write(&uart1, (const uint8_t *)imu_command, sizeof(imu_command) - 1U);
  delay_ms(500U);
  last_imu_command_ms = now_ms();

  flash_test_stage = TEST_PROBE;

  if (!flash_probe(&flash)) {
    flash_test_fail();
  }

  while (1) {
    spi_poll(&spi1);
    flash_poll(&flash);
    flash_test_poll();

    // Restart two seconds after the previous test finishes.
    if (flash_test_done) {
      if (!waiting_to_restart) {
        last_test_finished_ms = now_ms();
        waiting_to_restart = true;
      }

      if ((uint32_t)(now_ms() - last_test_finished_ms) >= 2000U &&
          !flash.busy && !spi1.busy && !spi1.error) {

        if (flash.result == FLASH_RESULT_OK ||
            flash.result == FLASH_RESULT_FAILED) {
          flash_ack_result(&flash);
        }

        flash_test_done = false;
        flash_test_passed = false;
        flash_fault = false;
        flash_test_stage = TEST_PROBE;
        waiting_to_restart = false;
        message[13] = (message[13] == '9') ? '0' : message[13] + 1;

        if (!flash_probe(&flash)) {
          flash_test_fail();
        }
      }
    }

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
        imu_started = true;
        updateState(imu_packet, &state, &kalman_filter);

        if (++packet_count >= 50U) {
          if (!usart_write(&uart2, imu_packet, PACKET_SIZE)) {
            // Increment a telemetry-drop counter here.
          }

          packet_count = 0;
        }
      }
    }

    uint32_t now = now_ms();

    if (!imu_started && (uint32_t)(now - last_imu_command_ms) >= 1000U) {

      if (usart_write(&uart1, (const uint8_t *)imu_command,
                      sizeof(imu_command) - 1U)) {
        last_imu_command_ms = now;
      }
    }

    now = now_ms();

    if ((uint32_t)(now - switch_on) >= delay) {
      switch_on = now;
      on = !on;

      if (on) {
        set_latch_high(GPIOA, SW_3);
        set_latch_high(GPIOB, SW_1);
        set_latch_high(GPIOB, SW_2);
      } else {
        set_latch_low(GPIOA, SW_3);
        set_latch_low(GPIOB, SW_1);
        set_latch_low(GPIOB, SW_2);
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
}
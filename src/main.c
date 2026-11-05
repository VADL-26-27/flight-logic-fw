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
#include "log.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

const char imu_command[] = "$VNWRG,75,2,8,05,0108,0020*XX\r\n";

uint8_t imu_packet[PACKET_SIZE];
IMUState state;
vn_kf_t kalman_filter;

static float maxAltitude;

State current_flight_state = STANDBY;
uint8_t synced;

uint32_t delay = 2000U;
bool on = false;
uint32_t switch_on;
uint8_t packet_count = 0;

static bool imu_started = false;
static uint32_t last_imu_command_ms;
static bool dump_active;
static bool dump_queued;

static bool dump_send(const uint8_t *data, uint32_t length) {
  return usart_write(&uart2, data, length);
}

static void poll_dump(void) {
  /* Discard commands sent during the binary stream. */
  uint8_t byte;
  for (uint32_t i = 0; i < BUFFER_SIZE && usart_read(&uart2, &byte, 1U); ++i) {
  }
  if (!dump_queued)
    dump_queued = logger_dump_poll(dump_send);
  if (!dump_queued || atomic_load(&uart2.tx->count) != 0U ||
      !(uart2.regs->SR & USART_SR_TC))
    return;

  /* USART1 RX IRQ is disabled. Drop queued bytes and clear hardware errors. */
  init_buf(uart1.rx);
  (void)uart1.regs->SR;
  (void)uart1.regs->DR;
  synced = 0;
  NVIC_ClearPendingIRQ(uart1.irqn);
  NVIC_EnableIRQ(uart1.irqn);
  dump_active = false;
}

static void poll_log_commands(void) {
  static char command[16];
  static uint32_t length;
  static bool overflow;
  static const char *reply;

  if (reply != NULL) {
    if (!usart_write(&uart2, (const uint8_t *)reply, strlen(reply)))
      return;
    reply = NULL;
  }

  uint8_t byte;
  for (uint32_t i = 0; i < sizeof(command) && usart_read(&uart2, &byte, 1U);
       ++i) {
    if (byte != '\n') {
      if (length < sizeof(command))
        command[length++] = (char)byte;
      else
        overflow = true;
      continue;
    }

    if (length != 0U && command[length - 1U] == '\r')
      --length;

    bool on_ground =
        current_flight_state == STANDBY || current_flight_state == LANDED;
    if (!overflow && length == 10U && memcmp(command, "LOG STATUS", 10U) == 0) {
      reply = logger_status_reply();
    } else if (!overflow && length == 9U &&
               memcmp(command, "LOG CLEAR", 9U) == 0) {
      reply = on_ground && logger_request_clear() ? "LOG CLEARING\r\n"
                                                  : "LOG CLEAR REFUSED\r\n";
    } else if (!overflow && length == 8U &&
               memcmp(command, "LOG DUMP", 8U) == 0) {
      if (on_ground && logger_request_dump()) {
        NVIC_DisableIRQ(uart1.irqn);
        dump_active = true;
        dump_queued = false;
      } else {
        reply = "\r\nLOG DUMP REFUSED\r\n";
      }
    }
    length = 0;
    overflow = false;
    if (reply != NULL || dump_active)
      return;
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

  logger_init(&flash);

  while (1) {
    spi_poll(&spi1);
    flash_poll(&flash);
    if (dump_active) {
      poll_dump();
      continue;
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

        if (state.altitude > maxAltitude)
          maxAltitude = state.altitude;

        State previous = current_flight_state;
        FlightState_Update(&current_flight_state, state.altitude,
                           state.velocity, state.acceleration, maxAltitude);

        IMURecord record = {
            .timestamp_ms = now_ms(),
            .postKf = state,
            .flightState = current_flight_state,
        };
        parsePacket(imu_packet, &record.rawPacket);

        if (previous == STANDBY && current_flight_state == LAUNCH)
          logger_start();

        if (current_flight_state == STANDBY) {
          logger_push_ground(&record);
        } else if (previous != LANDED) {
          logger_push_flight(&record);
          if (current_flight_state == LANDED)
            logger_finish();
        }
      }
    }

    poll_log_commands();
    if (dump_active)
      continue;
    logger_poll();
    uint32_t now = now_ms();

    if (!imu_started && (uint32_t)(now - last_imu_command_ms) >= 1000U) {

      if (usart_write(&uart1, (const uint8_t *)imu_command,
                      sizeof(imu_command) - 1U)) {
        last_imu_command_ms = now;
      }
    }

    if (current_flight_state == LANDED) {
      // Disable IMU interrupts upon landing detection
      NVIC_DisableIRQ(uart1.irqn);
    }
  }
}

#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include <stdint.h>
#include "usart2.h"
#include "hardware.h"
#include "imu.h"
#include "vn_kalman.h"
#include <math.h>

#define PACKET_SIZE 36


char* imu_command = "$VNWRG,75,2,8,05,0108,0020*XX\r\n\0";

uint8_t imu_packet[PACKET_SIZE];
IMUState state;

vn_kf_t kalman_filter;

uint8_t packet_count = 0;



static uint32_t LAUNCH_ACCEL_THRESH = 10; //10m/s^2
static uint32_t LAUNCH_TIME_THRESH = 5; //5ms
uint32_t accel = 0; 
uint32_t launch_start_time = 0; 
uint32_t launch_time = 0; 

static uint32_t DESCENT_ALTITUDE_THRESH = 10; //10m/s^2
static uint32_t DESCENT_TIME_THRESH = 5; //5ms
uint32_t altitude = 0; 
uint32_t descent_start_time = 0; 
uint32_t descent_time = 0; 
uint32_t max_altitude = 0; 

static uint32_t DEPLOYMENT_ALTITUDE_THRESH = 10; //10m/s^2
static uint32_t DEPLOYMENT_TIME_THRESH = 5; //5ms
uint32_t deployment_start_time = 0; 
uint32_t deployment_time = 0; 

static uint32_t LANDED_VELOCITY_THRESH = 10; //10m/s^2
static uint32_t LANDED_TIME_THRESH = 5; //5ms
uint32_t velocity = 0; 
uint32_t landed_start_time = 0; 
uint32_t landed_time = 0; 


static float maxAltitude = 0.0f;
static float maxVelocity = 0.0f;
static float maxAccel = 0.0f;

typedef enum {
    STANDBY,
    LAUNCH,
    DESCENT,
    DEPLOYMENT,
    LANDED
} State;

State current_flight_state = STANDBY;
uint32_t first_detection_time = 0;
uint8_t timer = 0;

void FlightState_Update();

static void uart_write_float(float value)
{
    // Bound the value before converting to an integer.
    if (!isfinite(value) || fabsf(value) > 1000000.0f) {
        usart2_write_command("invalid");
        return;
    }

    char text[24];
    char digits[12];
    unsigned pos = 0;
    unsigned count = 0;

    if (value < 0.0f) {
        text[pos++] = '-';
        value = -value;
    }

    uint32_t scaled = (uint32_t)(value * 1000.0f + 0.5f);
    uint32_t whole = scaled / 1000;
    uint32_t fraction = scaled % 1000;

    do {
        digits[count++] = '0' + whole % 10;
        whole /= 10;
    } while (whole);

    while (count) {
        text[pos++] = digits[--count];
    }

    text[pos++] = '.';
    text[pos++] = '0' + fraction / 100;
    text[pos++] = '0' + (fraction / 10) % 10;
    text[pos++] = '0' + fraction % 10;
    text[pos] = '\0';

    usart2_write_command(text);
}

static void send_state_packet_debug() {
    if (++packet_count >= 10) {
        packet_count = 0;

        usart2_write_command("STATE altitude=\0");
        uart_write_float(state.altitude);

        usart2_write_command(" velocity=\0");
        uart_write_float(state.velocity);

        usart2_write_command(" acceleration=\0");
        uart_write_float(state.acceleration);

        usart2_write_command("\r\n\0");
    }
}


int main(void) {
    vn_kf_init(&kalman_filter, 0.5f, 0.01f ,0.5f);
    hardwareInit();
    usart2_init();
    usart2_write_command("UART ready\r\n");
    //recieving the packet
    // Input command to VN
    usart2_write_command(imu_command);

    while (1) {
            if (new_data) {
            new_data = 0;
            if (imu_buf.count >= PACKET_SIZE) {
                read_from_buf(&imu_buf, imu_packet, PACKET_SIZE);
                if (validate_packet(imu_packet)) {
                    // kalman filter logic
                    updateState(imu_packet, &state, &kalman_filter);
                    send_state_packet_debug();
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

    //             if (HAL_GetTick() - descent_start_time >= DESCENT_TIME_THRESH) {
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

    //             if (HAL_GetTick() - deployment_start_time >= DEPLOYMENT_TIME_THRESH) {
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
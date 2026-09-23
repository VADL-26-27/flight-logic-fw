#include "stm32f4xx.h"
#include <stdint.h>
#include "main.h"

#define PACKET_SIZE 34

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;

volatile uint8_t rx_buffer[PACKET_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t new_data = 0;

uint8_t rx_byte;


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


int main(void) {


    //recieving the packet
    //need to set input timing on VectorNav
    //HAL_UART_Receive_IT(&huart2, rx_buffer, PACKET_SIZE);
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

    while (1) {
        if (new_data) {

            if (altitude > maxAltitude) {
                maxAltitude = altitude;
            }

            if (velocity > maxVelocity) {
                maxVelocity = velocity;
            }

            if (accel > maxAccel) {
                maxAccel = accel;
            }

            FlightState_Update();

            // ---- TEST: prove a packet arrived ----
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

            // TODO: real processing of rx_buffer goes here later

            new_data = 0;  // clear flag, ready for next packet
        }
    }
}


    void FlightState_Update() {

        switch (current_flight_state) {
            case STANDBY: //STANDBY
            //while accelaration is less than threshhold and 
            //launch time less than threshold
                if (accel >= LAUNCH_ACCEL_THRESH) {

                    if (timer == 0) {
                        launch_start_time = HAL_GetTick();
                        timer = 1;
                    }

                    if (HAL_GetTick() - launch_start_time >= LAUNCH_TIME_THRESH) {
                        launch_time = HAL_GetTick() - launch_start_time;
                        current_flight_state = LAUNCH;
                    }

                } else {
                    timer = 0;
                }
                break;

            case LAUNCH: //LAUNCH
            //while altitude is greater than max altitude - 500

                if (altitude < maxAltitude - DESCENT_ALTITUDE_THRESH) {

                    if (timer == 0) {
                        descent_start_time = HAL_GetTick();
                        timer = 1;
                    }

                    if (HAL_GetTick() - descent_start_time >= DESCENT_TIME_THRESH) {
                        descent_time = HAL_GetTick() - descent_start_time;
                        current_flight_state = DESCENT;
                    }

                } else {
                    timer = 0;
                }
                break;
            case DESCENT: //DESCENT
            //while altitude is greater than deployment altitude threshold

                if (altitude < DEPLOYMENT_ALTITUDE_THRESH) {

                    if (timer == 0) {
                        deployment_start_time = HAL_GetTick();
                        timer = 1;
                    }

                    if (HAL_GetTick() - deployment_start_time >= DEPLOYMENT_TIME_THRESH) {
                        deployment_time = HAL_GetTick() - deployment_start_time;
                        current_flight_state = DEPLOYMENT;
                    }

                } else {
                    timer = 0;
                }
                break;
            case DEPLOYMENT: //DEPLOYMENT
            //while velocity is less than velocity threshold
             if (velocity < LANDED_VELOCITY_THRESH) {

                    if (timer == 0) {
                        landed_start_time = HAL_GetTick();
                        timer = 1;
                    }

                    if (HAL_GetTick() - landed_start_time >= LANDED_TIME_THRESH) {
                        landed_time = HAL_GetTick() - landed_start_time;
                        current_flight_state = LANDED;
                    }

                } else {
                    timer = 0;
                }
                break;
            case LANDED: //LANDED
            //set some pins high

        }

    }

//ask about this.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
        if (huart->Instance == USART2) {

            rx_buffer[rx_index] = rx_byte;
            rx_index++;

            if (rx_index >= PACKET_SIZE) {
                if (1) { //switch to packet validation
                    rx_index = 0;

                    //kalman filter logic 
                    new_data = 1;
                } else {
                    rx_index = 0;
                }
            }

            // Re-arm for the next byte
            HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
        }
    }
    // void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    //     if (huart->Instance == USART2) {
    //         packet_ready = 1;

    //         // Re-arm to receive the next full packet
    //         HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buffer, PACKET_SIZE);
    //     }
    // }

#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include <string.h>

#define VN_SYNC_BYTE   0xFA
typedef struct {
    float yaw;
    float pitch;
    float roll;
    float ax;
    float ay;
    float az;
    float pressure;
} IMUPacket;

typedef struct {
    float altitude;
    float velocity;
    float acceleration;
} IMUState;

int validate_packet(const uint8_t* packet);
void parsePacket(const uint8_t* packet, IMUPacket* imupack);
#endif // IMU_H_
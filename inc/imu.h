#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include <string.h>

#define PACKET_SIZE 36

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
int imu_parser_feed(uint8_t* imu_packet, uint8_t* byte);
#endif // IMU_H_
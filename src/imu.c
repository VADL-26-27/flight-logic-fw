#include "imu.h"

#define GRAVITY     9.80665f
#define DEG2RAD     0.0174532925f

uint16_t vn_crc16(const uint8_t* data, uint32_t len) {
    uint16_t crc = 0;
    
    for (uint32_t i = 0; i < len; ++i) {
        crc = (crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xFF) << 5;
    }
    return crc;
}
int validate_packet(const uint8_t* packet) {
    
    if (packet[0] != VN_SYNC_BYTE) {
        return 0;
    }
     // Verify the expected groups and field masks.
    if (packet[1] != 0x05 ||
        packet[2] != 0x08 || packet[3] != 0x01 ||
        packet[4] != 0x20 || packet[5] != 0x00) {
        return 0;
    }
    // Verify CRC, excludes sync and crc16, so bytes 1-33
    const uint16_t vn_calc = vn_crc16(&packet[1], 33);
    const uint16_t recv = (uint16_t)((packet[34] << 8) | packet[35]);

    return vn_calc == recv;
}

void parsePacket(const uint8_t* packet, IMUPacket* imupack) {
    memcpy(&imupack->yaw, &packet[6], sizeof(float));
    memcpy(&imupack->pitch, &packet[10], sizeof(float));
    memcpy(&imupack->roll, &packet[14], sizeof(float));
    memcpy(&imupack->ax, &packet[18], sizeof(float));
    memcpy(&imupack->ay, &packet[22], sizeof(float));
    memcpy(&imupack->az, &packet[26], sizeof(float));
    memcpy(&imupack->pressure, &packet[30], sizeof(float));
}

// Convert pressure to altitude above the reference pressure (meters).
float pressure_to_altitude(float p_kpa, float p0_kpa)
{
    if (p_kpa <= 0.0f || p0_kpa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - powf(p_kpa / p0_kpa, 0.190295f));
}

// Rotate body-frame accel into the vertical axis and remove gravity.
// Returns acceleration in m/s^2, positive = up.
float imu_vertical_accel(const IMUPacket* pkt)
{
    float pitch = pkt->pitch * DEG2RAD;
    float roll  = pkt->roll  * DEG2RAD;

    // Down component of the measured specific force (body -> NED).
    float f_down = -sinf(pitch)               * pkt->ax
                 +  sinf(roll) * cosf(pitch)  * pkt->ay
                 +  cosf(roll) * cosf(pitch)  * pkt->az;

    // At rest f_down = -g, so this gives 0.
    return -(f_down + GRAVITY);
}
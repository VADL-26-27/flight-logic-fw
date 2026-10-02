#include "imu.h"

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
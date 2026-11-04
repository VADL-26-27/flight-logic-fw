#include <Arduino.h>
#include <string.h>
#include <stdio.h>

// Teensy Serial1 connects to VN-100 Serial2. USB Serial is diagnostics.
constexpr uint32_t IMU_BAUD = 115200;
constexpr uint16_t RATE_DIVISOR = 16; // 800/16 = 50 Hz; use 4 for 200 Hz.
constexpr size_t PACKET_SIZE = 36;
const uint8_t HEADER[] = {0xFA, 0x05, 0x08, 0x01, 0x20, 0x00};
static_assert(sizeof(float) == 4, "Requires 32-bit floats");
uint8_t packet[PACKET_SIZE];
size_t used = 0;
uint32_t rxBytes = 0, validPackets = 0, badCrc = 0, discardedBytes = 0;
uint32_t lastPacketUs = 0, maxGapUs = 0, lastReportMs = 0;
uint32_t previousBytes = 0, previousPackets = 0;
float latest[7] = {};
bool havePacket = false;

uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc = static_cast<uint16_t>((crc >> 8) | (crc << 8));
    crc ^= data[i];
    crc ^= (crc & 0xff) >> 4;
    crc ^= static_cast<uint16_t>(crc << 12);
    crc ^= static_cast<uint16_t>((crc & 0xff) << 5);
  }
  return crc;
}

float readFloatLE(const uint8_t* p) {
  uint32_t bits = uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
                  (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
  float result;
  memcpy(&result, &bits, sizeof(result));
  return result;
}

void discardFirst() {
  memmove(packet, packet + 1, --used);
  ++discardedBytes;
}

void receiveByte(uint8_t b) {
  ++rxBytes;
  packet[used++] = b;
  // Retain any partial matching header, even across UART reads.
  while (used && memcmp(packet, HEADER,
                       used < sizeof(HEADER) ? used : sizeof(HEADER)) != 0) {
    discardFirst();
  }
  if (used != PACKET_SIZE) return;
  // CRC over groups, field masks, payload AND received CRC must be zero.
  if (crc16(packet + 1, PACKET_SIZE - 1) != 0) {
    ++badCrc;
    // Shift only one byte, preserving a possible valid frame inside this data.
    discardFirst();
    while (used && memcmp(packet, HEADER,
                         used < sizeof(HEADER) ? used : sizeof(HEADER)) != 0) {
      discardFirst();
    }
    return;
  }
  const uint32_t now = micros();
  if (havePacket) {
    const uint32_t gap = now - lastPacketUs;
    if (gap > maxGapUs) maxGapUs = gap;
  }
  lastPacketUs = now;
  havePacket = true;
  ++validPackets;
  for (size_t i = 0; i < 7; ++i) latest[i] = readFloatLE(packet + 6 + 4*i);
  used = 0;
}

void configureOutput() {
  // Explicitly requested with 'r'; connect MCU TX to sensor RX2 to use it.
  // These commands change current settings but do not save them to flash.
  Serial1.print("$VNWRG,06,0,2*XX\r\n"); // Disable Serial2 ASCII output.
  char command[80];
  snprintf(command, sizeof(command), "$VNWRG,75,2,%u,05,0108,0020*XX\r\n",
           unsigned(RATE_DIVISOR));
  Serial1.print(command);
  // Confirm accepted settings separately in Control Center. Other binary
  // outputs (76/77) should already be disabled for Serial2.
}

void setup() {
  Serial.begin(115200); // Teensy USB baud selection does not set IMU UART baud.
  Serial1.begin(IMU_BAUD, SERIAL_8N1);
  // Start passively: do not alter sensor settings automatically on boot.
}

void loop() {
  while (Serial1.available() > 0) receiveByte(uint8_t(Serial1.read()));
  if (Serial.available() > 0) {
    int c = Serial.read();
    if (c == 'r' || c == 'R') configureOutput();
  }
  const uint32_t now = millis();
  const uint32_t elapsed = now - lastReportMs;
  if (elapsed < 1000) return;
  const float packetsPerSecond = (validPackets - previousPackets) * 1000.0f / elapsed;
  const float bytesPerSecond = (rxBytes - previousBytes) * 1000.0f / elapsed;
  char line[256];
  int n = snprintf(line, sizeof(line),
      "RX=%lu valid=%lu CRCbad=%lu discarded=%lu rate=%.1fHz bytes=%.0f/s "
      "age=%lums maxGap=%luus yaw=%.3f pitch=%.3f roll=%.3f P=%.3fkPa\r\n",
      (unsigned long)rxBytes, (unsigned long)validPackets,
      (unsigned long)badCrc, (unsigned long)discardedBytes,
      packetsPerSecond, bytesPerSecond,
      (unsigned long)(havePacket ? (micros()-lastPacketUs)/1000 : 0),
      (unsigned long)maxGapUs, latest[0], latest[1], latest[2], latest[6]);
  // Skip a report if USB cannot accept it immediately; keep draining UART.
  if (Serial && n > 0 && size_t(n) < sizeof(line) && Serial.availableForWrite() >= n)
    Serial.write((const uint8_t*)line, size_t(n));
  lastReportMs = now;
  previousPackets = validPackets;
  previousBytes = rxBytes;
  maxGapUs = 0;
}

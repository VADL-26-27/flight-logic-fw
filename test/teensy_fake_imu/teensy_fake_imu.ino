#include <Arduino.h>
#include <math.h>
#include <string.h>

// Serial1 connects to the STM32. Serial is USB and carries diagnostics only.
constexpr uint32_t BAUD = 115200;
constexpr uint32_t PERIOD_US = 10000;  // 100 Hz
constexpr float GRAVITY = 9.80665f;
constexpr float PEAK_HEIGHT = 20.0f;
constexpr float MOTION_SECONDS = 20.0f;
constexpr float REST_SECONDS = 5.0f;
static_assert(sizeof(float) == 4, "Packets require 32-bit floats");

bool streaming = false;
uint8_t commandPosition = 0;
uint32_t nextSampleUs = 0;
uint32_t sampleNumber = 0;

uint16_t crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc = ((crc >> 8) | (crc << 8)) & 0xFFFF;
    crc ^= data[i];
    crc ^= (crc & 0xFF) >> 4;
    crc ^= (crc << 12) & 0xFFFF;
    crc ^= ((crc & 0xFF) << 5) & 0xFFFF;
  }
  return crc;
}

void putFloatLE(uint8_t *destination, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  for (uint8_t i = 0; i < 4; ++i) {
    destination[i] = static_cast<uint8_t>(bits >> (8 * i));
  }
}

void sendSample() {
  const float t = sampleNumber * 0.01f;
  const float elapsed = fmodf(t, MOTION_SECONDS + 2 * REST_SECONDS) - REST_SECONDS;
  float height = 0.0f;
  float acceleration = 0.0f;
  if (elapsed > 0.0f && elapsed < MOTION_SECONDS) {
    const float omega = PI / MOTION_SECONDS;
    const float s = sinf(omega * elapsed);
    const float c = cosf(omega * elapsed);
    height = PEAK_HEIGHT * s * s * s * s;
    acceleration = 4 * PEAK_HEIGHT * omega * omega *
                   (3 * s * s * c * c - s * s * s * s);
  }
  const float pressure = 101.325f * powf(1.0f - height / 44330.0f,
                                       1.0f / 0.190295f);
  // Sync, group masks, yaw/pitch/roll, ax/ay/az, pressure, CRC.
  uint8_t packet[36] = {0xFA, 0x05, 0x08, 0x01, 0x20, 0x00};
  const float fields[7] = {0, 0, 0, 0, 0, -(GRAVITY + acceleration), pressure};
  for (uint8_t i = 0; i < 7; ++i) {
    putFloatLE(packet + 6 + 4 * i, fields[i]);
  }
  const uint16_t crc = crc16(packet + 1, 33);
  packet[34] = static_cast<uint8_t>(crc >> 8);
  packet[35] = static_cast<uint8_t>(crc);
  Serial1.write(packet, sizeof(packet));
  ++sampleNumber;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUD);
  // Do not wait for USB: the emulator also runs without a USB monitor.
}

void loop() {
  // STM32 sends exactly "100", with no newline or transmitted NUL byte.
  // Recognize it across separate UART reads; CR/LF after it are harmless.
  while (Serial1.available()) {
    const char byte = static_cast<char>(Serial1.read());
    const char command[] = "100";
    if (byte == command[commandPosition]) {
      ++commandPosition;
    } else {
      commandPosition = (byte == '1') ? 1 : 0;
    }
    if (commandPosition == 3) {
      commandPosition = 0;
      sampleNumber = 0;
      nextSampleUs = micros();
      streaming = true;
      Serial.println("Received 100: streaming IMU packets at 100 Hz");
      // No ASCII acknowledgement on Serial1: STM32 expects binary packets.
    }
  }

  if (streaming && static_cast<int32_t>(micros() - nextSampleUs) >= 0) {
    sendSample();
    nextSampleUs += PERIOD_US;
    // Avoid a burst of catch-up packets after an unexpected long pause.
    const uint32_t now = micros();
    if (static_cast<int32_t>(now - nextSampleUs) >= 0) {
      nextSampleUs = now + PERIOD_US;
    }
  }
}

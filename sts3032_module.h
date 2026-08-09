#pragma once

#include <Arduino.h>

namespace sts3032 {

inline void begin(HardwareSerial& serial, int pin_tx, uint32_t baud = 115200) {
  serial.begin(baud, SERIAL_8N1, -1, pin_tx);
}

inline void sendGoalPosition(HardwareSerial& serial, uint8_t id, uint16_t position, uint16_t time = 0, uint16_t speed = 0) {
  uint8_t packet[13];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 9;
  packet[4] = 0x03;
  packet[5] = 42; // goal position addr
  packet[6] = (uint8_t)(position & 0xFF);
  packet[7] = (uint8_t)((position >> 8) & 0xFF);
  packet[8] = (uint8_t)(time & 0xFF);
  packet[9] = (uint8_t)((time >> 8) & 0xFF);
  packet[10] = (uint8_t)(speed & 0xFF);
  packet[11] = (uint8_t)((speed >> 8) & 0xFF);

  uint8_t sum = 0;
  for (int i = 2; i <= 11; i++) sum += packet[i];
  packet[12] = (uint8_t)(~sum);

  serial.write(packet, sizeof(packet));
}

inline uint16_t mapSbusToPosition(uint16_t sbus_value, uint16_t pos_min = 200, uint16_t pos_max = 800) {
  float t = (sbus_value - 172.0f) / (1811.0f - 172.0f);
  t = constrain(t, 0.0f, 1.0f);
  return (uint16_t)(pos_min + t * (float)(pos_max - pos_min));
}

} // namespace sts3032

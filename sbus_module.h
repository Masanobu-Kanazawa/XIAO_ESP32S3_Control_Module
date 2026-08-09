#pragma once

#include <Arduino.h>

namespace sbus {

static constexpr uint32_t BAUD = 100000;
static constexpr uint8_t FRAME_LEN = 25;
static constexpr uint8_t HEADER = 0x0F;
static constexpr uint8_t FOOTER_NIBBLE_MASK = 0x0F;
static constexpr uint8_t FOOTER_VALID_NIBBLE = 0x04;
static constexpr uint16_t SWITCH_LOW_VALUE = 144;
static constexpr uint16_t SWITCH_MID_VALUE = 1024;
static constexpr uint16_t SWITCH_HIGH_VALUE = 1904;
static constexpr uint16_t LOG_OFF_VALUE = SWITCH_LOW_VALUE;   // CH[4]
static constexpr uint16_t LOG_ON_VALUE = SWITCH_HIGH_VALUE;   // CH[4]

enum class ThreePos : uint8_t {
  Low = 0,
  Mid,
  High
};

inline ThreePos threePosFromValue(uint16_t v) {
  static constexpr uint16_t LOW_MID_TH = (SWITCH_LOW_VALUE + SWITCH_MID_VALUE) / 2;   // 584
  static constexpr uint16_t MID_HIGH_TH = (SWITCH_MID_VALUE + SWITCH_HIGH_VALUE) / 2; // 1464
  if (v <= LOW_MID_TH) return ThreePos::Low;
  if (v >= MID_HIGH_TH) return ThreePos::High;
  return ThreePos::Mid;
}

struct Data {
  uint16_t ch[16]{};
  bool failsafe{false};
  bool frame_lost{false};
};

inline bool decodeFrame(const uint8_t* b, Data& out) {
  if (b[0] != HEADER) return false;

  out.ch[0]  = ((b[1]       | b[2]  << 8) & 0x07FF);
  out.ch[1]  = ((b[2] >> 3  | b[3]  << 5) & 0x07FF);
  out.ch[2]  = ((b[3] >> 6  | b[4]  << 2 | b[5]  << 10) & 0x07FF);
  out.ch[3]  = ((b[5] >> 1  | b[6]  << 7) & 0x07FF);
  out.ch[4]  = ((b[6] >> 4  | b[7]  << 4) & 0x07FF);
  out.ch[5]  = ((b[7] >> 7  | b[8]  << 1 | b[9]  << 9 ) & 0x07FF);
  out.ch[6]  = ((b[9] >> 2  | b[10] << 6) & 0x07FF);
  out.ch[7]  = ((b[10] >> 5 | b[11] << 3) & 0x07FF);

  out.ch[8]  = ((b[12]       | b[13] << 8) & 0x07FF);
  out.ch[9]  = ((b[13] >> 3  | b[14] << 5) & 0x07FF);
  out.ch[10] = ((b[14] >> 6  | b[15] << 2 | b[16] << 10) & 0x07FF);
  out.ch[11] = ((b[16] >> 1  | b[17] << 7) & 0x07FF);
  out.ch[12] = ((b[17] >> 4  | b[18] << 4) & 0x07FF);
  out.ch[13] = ((b[18] >> 7  | b[19] << 1 | b[20] << 9 ) & 0x07FF);
  out.ch[14] = ((b[20] >> 2  | b[21] << 6) & 0x07FF);
  out.ch[15] = ((b[21] >> 5  | b[22] << 3) & 0x07FF);

  out.frame_lost = (b[23] & 0x04) != 0;
  out.failsafe = (b[23] & 0x08) != 0;
  return true;
}

inline bool poll(HardwareSerial& serial, Data& out) {
  static uint8_t buf[FRAME_LEN];
  static uint8_t idx = 0;

  while (serial.available()) {
    uint8_t v = (uint8_t)serial.read();

    if (idx == 0 && v != HEADER) continue;
    buf[idx++] = v;

    if (idx >= FRAME_LEN) {
      idx = 0;
      const uint8_t footer = buf[FRAME_LEN - 1];
      if ((footer & FOOTER_NIBBLE_MASK) != FOOTER_VALID_NIBBLE) {
        return false;
      }
      return decodeFrame(buf, out);
    }
  }
  return false;
}

inline bool logSwitchOn(const uint16_t* ch) {
  return threePosFromValue(ch[4]) == ThreePos::High;
}

inline bool logSwitchOff(const uint16_t* ch) {
  return threePosFromValue(ch[4]) == ThreePos::Low;
}

inline bool switchOn(const uint16_t* ch) {
  // Backward-compatible alias.
  return logSwitchOn(ch);
}

} // namespace sbus

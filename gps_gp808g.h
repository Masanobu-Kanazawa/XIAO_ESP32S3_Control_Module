#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

namespace gps_gp808g {

static constexpr uint32_t DEFAULT_BAUD = 9600;

struct State {
  TinyGPSPlus gps{};
  bool present{false};
  bool fix{false};
  float lat_deg{NAN};
  float lon_deg{NAN};
  float alt_m{NAN};
  float speed_mps{NAN};
  float course_deg{NAN};
  float hdop{NAN};
  uint8_t sats{0};
  char utc[16]{};
  char date[8]{};
  bool utc_valid{false};
  bool date_valid{false};
  uint16_t year{0};
  uint8_t month{0};
  uint8_t day{0};
  uint8_t hour{0};
  uint8_t minute{0};
  uint8_t second{0};
  uint8_t centisecond{0};
  uint32_t last_sentence_ms{0};
  uint32_t last_fix_ms{0};
};

inline void updateDateTime(State& state) {
  if (state.gps.time.isValid()) {
    state.utc_valid = true;
    state.hour = state.gps.time.hour();
    state.minute = state.gps.time.minute();
    state.second = state.gps.time.second();
    state.centisecond = state.gps.time.centisecond();
    snprintf(
      state.utc,
      sizeof(state.utc),
      "%02u%02u%02u.%02u",
      (unsigned)state.hour,
      (unsigned)state.minute,
      (unsigned)state.second,
      (unsigned)state.centisecond
    );
  } else {
    state.utc_valid = false;
    state.hour = 0;
    state.minute = 0;
    state.second = 0;
    state.centisecond = 0;
    state.utc[0] = '\0';
  }

  if (state.gps.date.isValid()) {
    state.date_valid = true;
    state.year = (uint16_t)state.gps.date.year();
    state.month = state.gps.date.month();
    state.day = state.gps.date.day();
    const uint8_t yy = (uint8_t)(state.year % 100U);
    snprintf(
      state.date,
      sizeof(state.date),
      "%02u%02u%02u",
      (unsigned)state.day,
      (unsigned)state.month,
      (unsigned)yy
    );
  } else {
    state.date_valid = false;
    state.year = 0;
    state.month = 0;
    state.day = 0;
    state.date[0] = '\0';
  }
}

inline void updateNav(State& state, uint32_t now) {
  const bool has_recent_fix = state.gps.location.isValid() && state.gps.location.age() < 3000;
  state.fix = has_recent_fix;
  if (has_recent_fix) {
    state.last_fix_ms = now;
    state.lat_deg = (float)state.gps.location.lat();
    state.lon_deg = (float)state.gps.location.lng();
  } else {
    state.lat_deg = NAN;
    state.lon_deg = NAN;
  }

  state.alt_m = state.gps.altitude.isValid() ? (float)state.gps.altitude.meters() : NAN;
  state.hdop = state.gps.hdop.isValid() ? (float)state.gps.hdop.hdop() : NAN;
  state.sats = state.gps.satellites.isValid()
                 ? (uint8_t)constrain((int)state.gps.satellites.value(), 0, 99)
                 : 0;
  state.speed_mps = state.gps.speed.isValid()
                      ? (float)(state.gps.speed.knots() * 0.514444)
                      : NAN;
  state.course_deg = state.gps.course.isValid() ? (float)state.gps.course.deg() : NAN;
}

inline void begin(HardwareSerial& serial, int pin_rx, int pin_tx_dummy = -1, uint32_t baud = DEFAULT_BAUD) {
  serial.begin(baud, SERIAL_8N1, pin_rx, pin_tx_dummy);
}

inline void poll(Stream& serial, State& state) {
  bool parsed_sentence = false;
  while (serial.available()) {
    const char c = static_cast<char>(serial.read());
    if (state.gps.encode(c)) {
      parsed_sentence = true;
    }
  }

  const uint32_t now = millis();
  if (parsed_sentence) {
    state.present = true;
    state.last_sentence_ms = now;
  } else if (state.present && (now - state.last_sentence_ms > 3000)) {
    state.present = false;
  }

  updateDateTime(state);
  updateNav(state, now);
}

} // namespace gps_gp808g

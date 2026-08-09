#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace ultra_rcwl9620 {

static constexpr uint8_t I2C_ADDR = 0x57;

struct State {
  uint32_t duration_us{0}; // keeps logger/debug compatibility (stores raw 24-bit distance value)
  float cm{NAN};
  uint32_t last_ms{0};
  int last_error{0};       // 0:OK, -1 tx err, -2 timeout, -4 invalid 0mm
};

inline void initPins(int pin_sda, int pin_scl, uint32_t i2c_hz = 100000) {
  Wire1.begin(pin_sda, pin_scl, i2c_hz);
}

inline float readDistanceMmPollingValid(
  uint32_t timeout_ms = 120,
  uint32_t poll_interval_ms = 5,
  bool debug = false,
  uint32_t* out_raw = nullptr
) {
  Wire1.beginTransmission(I2C_ADDR);
  Wire1.write(0x01);
  const int tx_err = Wire1.endTransmission();
  if (tx_err != 0) {
    if (debug) {
      // Serial.print("endTransmission err = ");
      // Serial.println(tx_err);
    }
    return -1.0f;
  }

  const uint32_t t_start = millis();
  bool got3bytes_but_invalid = false;
  uint32_t request_count = 0;

  while ((millis() - t_start) < timeout_ms) {
    const int n = Wire1.requestFrom((int)I2C_ADDR, 3);
    request_count++;

    if (debug) {
      // Serial.print("req=");
      // Serial.print(request_count);
      // Serial.print(" n=");
      // Serial.println(n);
    }

    if (n == 3) {
      const uint8_t b0 = (uint8_t)Wire1.read();
      const uint8_t b1 = (uint8_t)Wire1.read();
      const uint8_t b2 = (uint8_t)Wire1.read();

      const uint32_t raw = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b2;
      float mm = (float)raw / 1000.0f;

      if (debug) {
        // Serial.print("raw=0x");
        // Serial.print(b0, HEX);
        // Serial.print(" 0x");
        // Serial.print(b1, HEX);
        // Serial.print(" 0x");
        // Serial.print(b2, HEX);
        // Serial.print(" -> ");
        // Serial.print(mm);
        // Serial.println(" mm");
      }

      if (mm <= 0.0f) {
        got3bytes_but_invalid = true;
        delay(poll_interval_ms);
        continue;
      }

      if (mm > 4500.0f) mm = 4500.0f;
      if (out_raw != nullptr) {
        *out_raw = raw;
      }
      return mm;
    }

    while (Wire1.available()) {
      (void)Wire1.read();
    }
    delay(poll_interval_ms);
  }

  if (out_raw != nullptr) {
    *out_raw = 0;
  }
  if (got3bytes_but_invalid) return -4.0f;
  return -2.0f;
}

inline void poll(
  State& state,
  uint32_t now,
  uint32_t interval_ms = 100,
  uint32_t timeout_ms = 120,
  uint32_t i2c_poll_interval_ms = 5,
  bool debug = false
) {
  if (now - state.last_ms < interval_ms) return;
  state.last_ms = now;

  uint32_t raw = 0;
  const float mm = readDistanceMmPollingValid(timeout_ms, i2c_poll_interval_ms, debug, &raw);
  state.duration_us = raw;

  if (mm >= 0.0f) {
    state.cm = mm * 0.1f;
    state.last_error = 0;
  } else {
    state.cm = NAN;
    state.last_error = (int)mm;
  }
}

} // namespace ultra_rcwl9620

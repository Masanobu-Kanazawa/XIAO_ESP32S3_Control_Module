#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace imu_lsm6dsv16x {

static constexpr uint8_t ADDR_GND = 0x6A;
static constexpr uint8_t ADDR_VCC = 0x6B;
static constexpr uint8_t WHO_AM_I = 0x0F;
static constexpr uint8_t CTRL1 = 0x10;
static constexpr uint8_t CTRL2 = 0x11;
static constexpr uint8_t CTRL6 = 0x15;
static constexpr uint8_t CTRL8 = 0x17;
static constexpr uint8_t OUTX_L_G = 0x22;

struct State {
  bool present{false};
  uint8_t addr{0};
  uint8_t whoami{0};
  float gx{NAN};
  float gy{NAN};
  float gz{NAN};
  float ax{NAN};
  float ay{NAN};
  float az{NAN};
  uint32_t last_ms{0};
  uint32_t last_reconnect_try_ms{0};
};

inline bool probe(TwoWire& wire, uint8_t addr) {
  wire.beginTransmission(addr);
  return wire.endTransmission() == 0;
}

inline bool writeReg(TwoWire& wire, uint8_t addr, uint8_t reg, uint8_t val) {
  wire.beginTransmission(addr);
  wire.write(reg);
  wire.write(val);
  return wire.endTransmission() == 0;
}

inline bool readRegs(TwoWire& wire, uint8_t addr, uint8_t start_reg, uint8_t* buf, uint8_t len) {
  wire.beginTransmission(addr);
  wire.write(start_reg);
  if (wire.endTransmission(false) != 0) return false;

  const size_t got = wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = wire.read();
  }
  return true;
}

inline bool init(TwoWire& wire, State& state) {
  uint8_t who = 0;
  if (!readRegs(wire, state.addr, WHO_AM_I, &who, 1)) return false;
  state.whoami = who;
  if (who != 0x70) return false;

  // Same register setup as the Akizuki example.
  if (!writeReg(wire, state.addr, CTRL1, 0b00001001)) return false;
  if (!writeReg(wire, state.addr, CTRL2, 0b00001001)) return false;
  if (!writeReg(wire, state.addr, CTRL6, 0b00000100)) return false;
  if (!writeReg(wire, state.addr, CTRL8, 0b10000010)) return false;
  delay(10);

  state.present = true;
  return true;
}

inline bool detectAndInit(TwoWire& wire, State& state) {
  state.addr = 0;
  state.present = false;

  if (probe(wire, ADDR_GND)) {
    state.addr = ADDR_GND;
  } else if (probe(wire, ADDR_VCC)) {
    state.addr = ADDR_VCC;
  } else {
    return false;
  }
  return init(wire, state);
}

inline bool readSample(TwoWire& wire, State& state) {
  if (!state.present || state.addr == 0) return false;

  uint8_t b[12];
  if (!readRegs(wire, state.addr, OUTX_L_G, b, 12)) return false;

  int16_t raw[6];
  for (int i = 0; i < 6; i++) {
    raw[i] = (int16_t)((uint16_t)b[i * 2 + 1] << 8 | b[i * 2]);
  }
  // Treat all-zero as invalid and trigger reconnect in poll().
  if (raw[0] == 0 && raw[1] == 0 && raw[2] == 0 &&
      raw[3] == 0 && raw[4] == 0 && raw[5] == 0) {
    return false;
  }

  // Scale factors follow the reference sample.
  state.gx = (float)raw[0] * 0.07f;
  state.gy = (float)raw[1] * 0.07f;
  state.gz = (float)raw[2] * 0.07f;
  state.ax = (float)raw[3] * 0.000244f;
  state.ay = (float)raw[4] * 0.000244f;
  state.az = (float)raw[5] * 0.000244f;
  state.last_ms = millis();
  return true;
}

inline void tryReconnect(TwoWire& wire, State& state, uint32_t now, uint32_t retry_interval_ms = 500) {
  if (now - state.last_reconnect_try_ms < retry_interval_ms) return;
  state.last_reconnect_try_ms = now;
  if (detectAndInit(wire, state)) {
    state.last_ms = now;
  }
}

inline void poll(TwoWire& wire, State& state, uint32_t now, uint32_t interval_ms = 10) {
  if (!state.present) {
    tryReconnect(wire, state, now);
    return;
  }
  if (now - state.last_ms < interval_ms) return;
  if (!readSample(wire, state)) {
    state.present = false;
    state.gx = NAN;
    state.gy = NAN;
    state.gz = NAN;
    state.ax = NAN;
    state.ay = NAN;
    state.az = NAN;
    tryReconnect(wire, state, now);
  }
}

} // namespace imu_lsm6dsv16x

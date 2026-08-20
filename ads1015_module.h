#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

namespace ads1015_module {

static constexpr uint8_t DEFAULT_ADDR = 0x48;
static constexpr uint32_t SAMPLE_INTERVAL_MS = 8; // 128 SPS = 7.8125 ms/conversion
static constexpr uint32_t RECONNECT_INTERVAL_MS = 500;

struct State {
  bool present{false};
  uint8_t addr{DEFAULT_ADDR};
  int16_t raw{0};
  float diff_volts{NAN};
  uint32_t last_sample_ms{0};
  uint32_t last_probe_ms{0};
  uint32_t last_reconnect_try_ms{0};
};

static Adafruit_ADS1015 adc;

inline bool probe(TwoWire& wire, uint8_t addr) {
  wire.beginTransmission(addr);
  return wire.endTransmission() == 0;
}

inline bool init(TwoWire& wire, State& state, uint8_t addr = DEFAULT_ADDR) {
  state.addr = addr;
  state.present = false;
  state.raw = 0;
  state.diff_volts = NAN;

  if (!probe(wire, addr)) return false;
  if (!adc.begin(addr, &wire)) return false;

  adc.setGain(GAIN_SIXTEEN);             // differential full scale: +/-0.256 V
  adc.setDataRate(RATE_ADS1015_128SPS); // conversion period: 7.8125 ms
  state.present = true;
  return true;
}

inline void tryReconnect(
  TwoWire& wire,
  State& state,
  uint32_t now,
  uint32_t retry_interval_ms = RECONNECT_INTERVAL_MS
) {
  if (now - state.last_reconnect_try_ms < retry_interval_ms) return;
  state.last_reconnect_try_ms = now;
  if (init(wire, state, state.addr)) {
    state.last_sample_ms = now;
    state.last_probe_ms = now;
  }
}

inline void poll(
  TwoWire& wire,
  State& state,
  uint32_t now,
  uint32_t sample_interval_ms = SAMPLE_INTERVAL_MS
) {
  if (!state.present) {
    tryReconnect(wire, state, now);
    return;
  }

  if (now - state.last_probe_ms >= RECONNECT_INTERVAL_MS) {
    state.last_probe_ms = now;
    if (!probe(wire, state.addr)) {
      state.present = false;
      state.raw = 0;
      state.diff_volts = NAN;
      return;
    }
  }

  if (now - state.last_sample_ms < sample_interval_ms) return;
  state.last_sample_ms = now;
  state.raw = adc.readADC_Differential_0_1();
  state.diff_volts = adc.computeVolts(state.raw);
}

} // namespace ads1015_module

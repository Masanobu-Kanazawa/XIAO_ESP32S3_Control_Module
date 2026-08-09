#pragma once

#include <Arduino.h>
#include <limits.h>

namespace hx711_bitbang {

struct Config {
  int pin_dout{41};
  int pin_sck{42};

  // Load cell constants (sample defaults).
  float out_vol{0.0004056f}; // rated output [V/V]
  float load_g{1020.0f};     // rated capacity [g]

  // HX711 analog front-end constants.
  float r1{20000.0f};
  float r2{8200.0f};
  float vbg{1.25f};
  float pga{128.0f};
};

struct State {
  long raw{LONG_MIN};
  float gram{NAN};
  float gram_avg{NAN};
  float offset{0.0f};
};

inline float avdd(const Config& cfg) {
  (void)cfg;
  return 4.2987f;
}

inline float adc1Bit(const Config& cfg) {
  return avdd(cfg) / 16777216.0f; // 2^24
}

inline float scale(const Config& cfg) {
  return (cfg.out_vol * avdd(cfg) / cfg.load_g * cfg.pga);
}

inline void initPins(const Config& cfg) {
  pinMode(cfg.pin_sck, OUTPUT);
  pinMode(cfg.pin_dout, INPUT);
  digitalWrite(cfg.pin_sck, LOW);
}

inline void reset(const Config& cfg) {
  digitalWrite(cfg.pin_sck, HIGH);
  delayMicroseconds(100);
  digitalWrite(cfg.pin_sck, LOW);
  delayMicroseconds(100);
}

inline long readRaw(const Config& cfg, uint32_t timeout_ms = 120) {
  uint32_t data = 0;

  const uint32_t t0 = millis();
  while (digitalRead(cfg.pin_dout) != LOW) {
    if (millis() - t0 > timeout_ms) return LONG_MIN;
  }

  delayMicroseconds(10);
  for (int i = 0; i < 24; i++) {
    digitalWrite(cfg.pin_sck, HIGH);
    delayMicroseconds(5);
    digitalWrite(cfg.pin_sck, LOW);
    delayMicroseconds(5);
    data = (data << 1) | (digitalRead(cfg.pin_dout) ? 1U : 0U);
  }

  // 25th pulse: gain 128 / channel A.
  digitalWrite(cfg.pin_sck, HIGH);
  delayMicroseconds(10);
  digitalWrite(cfg.pin_sck, LOW);
  delayMicroseconds(10);

  return (long)(data ^ 0x800000UL);
}

inline long averageRaw(const Config& cfg, char num) {
  if (num <= 0) return LONG_MIN;

  long sum = 0;
  int valid = 0;
  for (int i = 0; i < num; i++) {
    const long v = readRaw(cfg);
    if (v == LONG_MIN) continue;
    sum += v;
    valid++;
  }
  if (valid == 0) return LONG_MIN;
  return sum / valid;
}

inline float rawToGram(long raw, const Config& cfg) {
  return (raw * adc1Bit(cfg) / scale(cfg));
}

inline float sampleGram(const Config& cfg, char num) {
  const long avg = averageRaw(cfg, num);
  if (avg == LONG_MIN) return NAN;
  return rawToGram(avg, cfg);
}

inline bool tare(State& state, const Config& cfg, char samples = 30) {
  const float t = sampleGram(cfg, samples);
  if (isnan(t)) return false;
  state.offset = t;
  state.raw = LONG_MIN;
  state.gram = NAN;
  state.gram_avg = NAN;
  return true;
}

inline bool tareForDuration(
  State& state,
  const Config& cfg,
  uint32_t duration_ms = 3000,
  uint32_t read_timeout_ms = 120,
  uint16_t* valid_samples_out = nullptr
) {
  if (duration_ms == 0) duration_ms = 1;

  const uint32_t t0 = millis();
  int64_t sum = 0;
  uint32_t valid = 0;

  while ((millis() - t0) < duration_ms) {
    const long raw = readRaw(cfg, read_timeout_ms);
    if (raw == LONG_MIN) continue;
    sum += (int64_t)raw;
    valid++;
  }

  if (valid_samples_out) {
    *valid_samples_out = (uint16_t)constrain((int)valid, 0, 65535);
  }

  if (valid == 0) return false;

  const long avg_raw = (long)(sum / (int64_t)valid);
  state.offset = rawToGram(avg_raw, cfg);
  state.raw = LONG_MIN;
  state.gram = NAN;
  state.gram_avg = NAN;
  return true;
}

inline void poll(State& state, const Config& cfg) {
  if (digitalRead(cfg.pin_dout) != LOW) return;
  const long raw = averageRaw(cfg, 5); // equivalent to get_gram(5)-style averaging
  if (raw == LONG_MIN) return;

  state.raw = raw;
  state.gram = rawToGram(raw, cfg) - state.offset;
  state.gram_avg = state.gram; // no smoothing
}

} // namespace hx711_bitbang

#pragma once

#include <Arduino.h>

// M5 Unit Ultrasonic IO (trig/echo) non-blocking driver.
//
// Replaces the I2C variant (ultrasonic_rcwl9620.h). The echo pulse width is
// captured by a pin-change interrupt, so poll() never blocks waiting for the
// echo -- the main loop keeps running at full rate while a measurement is in
// flight. Only the 10us trigger pulse itself blocks.

namespace ultra_io {

static constexpr float SOUND_SPEED_CM_PER_US = 0.034f;

// Guard time between triggers. Firing again while the previous burst is still
// reverberating produces bogus readings. 30ms (~33Hz) is stable for the short
// range this project uses; raise it toward 60ms if values start jumping.
static constexpr uint32_t MEASURE_INTERVAL_MS = 30;

// Give up on the echo edge after this long. Some units hold echo high for
// ~38ms when nothing is detected, so this must be comfortably above that.
static constexpr uint32_t ECHO_TIMEOUT_US = 60000;

// Round-trip time still treated as a real distance. 400cm ~= 23500us; longer
// pulses mean "no obstacle", not a distance.
static constexpr uint32_t ECHO_MAX_VALID_US = 24000;

// A failed measurement does not clobber the last good reading -- it is held
// for this long instead, so a single dropout does not slam the servo back to
// neutral or write a zero into the log. Past this window the reading really is
// stale and cm goes NAN so the caller can fail safe.
static constexpr uint32_t HOLD_TIMEOUT_MS = 500;

// Median-of-3 over accepted readings kills single-sample spikes. Costs one
// sample of latency (~30ms at the default interval).
static constexpr uint8_t MEDIAN_WINDOW = 3;

struct State {
  uint32_t duration_us{0}; // echo pulse width of the held reading
  float cm{NAN};           // held + median-filtered distance -- use this for control/logging
  float raw_cm{NAN};       // most recent raw measurement, NAN if it failed
  bool valid{false};       // cm is usable (fresh, or held within HOLD_TIMEOUT_MS)
  uint32_t last_ms{0};       // last trigger time
  uint32_t last_valid_ms{0}; // last accepted measurement
  int last_error{0};         // 0:OK, -2 timeout, -3 out of range

  float hist[MEDIAN_WINDOW]{NAN, NAN, NAN};
  uint8_t hist_count{0};
  uint8_t hist_idx{0};
};

inline float median3(float a, float b, float c) {
  float t;
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { t = a; a = b; b = t; }
  return b;
}

inline void acceptReading(State& state, float cm, uint32_t width_us, uint32_t now) {
  state.raw_cm = cm;

  state.hist[state.hist_idx] = cm;
  state.hist_idx = (uint8_t)((state.hist_idx + 1) % MEDIAN_WINDOW);
  if (state.hist_count < MEDIAN_WINDOW) state.hist_count++;

  state.cm = (state.hist_count >= MEDIAN_WINDOW)
               ? median3(state.hist[0], state.hist[1], state.hist[2])
               : cm;

  state.duration_us = width_us;
  state.last_valid_ms = now;
  state.valid = true;
  state.last_error = 0;
}

inline void rejectReading(State& state, uint32_t now, int error) {
  state.raw_cm = NAN;
  state.last_error = error;

  // Keep serving the previous reading until the hold window expires.
  if (state.valid && (now - state.last_valid_ms) > HOLD_TIMEOUT_MS) {
    state.valid = false;
    state.cm = NAN;
    state.duration_us = 0;
    state.hist_count = 0;
    state.hist_idx = 0;
  }
}

// Shared with the ISR. 32-bit accesses are atomic on ESP32.
static volatile uint32_t g_rise_us = 0;
static volatile uint32_t g_pulse_us = 0;
static volatile bool g_have_rise = false;
static volatile bool g_have_pulse = false;

static int g_trig_pin = -1;
static int g_echo_pin = -1;

enum class Phase : uint8_t {
  Idle,     // waiting for the next trigger slot
  WaitEcho, // triggered, waiting for the echo pulse
};

static Phase g_phase = Phase::Idle;
static uint32_t g_trigger_us = 0;

static void IRAM_ATTR echoIsr() {
  const uint32_t now = micros();
  if (digitalRead(g_echo_pin) == HIGH) {
    g_rise_us = now;
    g_have_rise = true;
  } else if (g_have_rise) {
    g_pulse_us = now - g_rise_us;
    g_have_rise = false;
    g_have_pulse = true;
  }
}

inline void clearCapture() {
  noInterrupts();
  g_have_rise = false;
  g_have_pulse = false;
  interrupts();
}

inline bool initPins(int pin_trig, int pin_echo) {
  if (pin_trig < 0 || pin_echo < 0) return false;

  g_trig_pin = pin_trig;
  g_echo_pin = pin_echo;

  pinMode(g_trig_pin, OUTPUT);
  pinMode(g_echo_pin, INPUT);
  digitalWrite(g_trig_pin, LOW);

  clearCapture();
  g_phase = Phase::Idle;

  attachInterrupt(digitalPinToInterrupt(g_echo_pin), echoIsr, CHANGE);
  return true;
}

inline void fireTrigger(State& state, uint32_t now) {
  clearCapture();

  digitalWrite(g_trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(g_trig_pin, HIGH);
  delayMicroseconds(10); // only blocking part; harmless next to a 3ms SBUS frame
  digitalWrite(g_trig_pin, LOW);

  g_trigger_us = micros();
  state.last_ms = now;
  g_phase = Phase::WaitEcho;
}

inline void poll(State& state, uint32_t now, uint32_t interval_ms = MEASURE_INTERVAL_MS) {
  if (g_trig_pin < 0 || g_echo_pin < 0) return;

  switch (g_phase) {
    case Phase::Idle:
      if (now - state.last_ms >= interval_ms) {
        fireTrigger(state, now);
      }
      break;

    case Phase::WaitEcho: {
      if (g_have_pulse) {
        const uint32_t width = g_pulse_us;
        g_have_pulse = false;

        if (width > ECHO_MAX_VALID_US) {
          rejectReading(state, now, -3);
        } else {
          acceptReading(state, (float)width * SOUND_SPEED_CM_PER_US / 2.0f, width, now);
        }
        g_phase = Phase::Idle;
        break;
      }

      if (micros() - g_trigger_us > ECHO_TIMEOUT_US) {
        rejectReading(state, now, -2);
        clearCapture();
        g_phase = Phase::Idle;
      }
      break;
    }
  }
}

} // namespace ultra_io

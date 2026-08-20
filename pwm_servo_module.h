#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

namespace pwm_servo {

static constexpr int FREQ_HZ = 50;
static Servo myServo;
static bool g_attached = false;
static bool g_timers_allocated = false;
static uint32_t g_min_us = 1000;
static uint32_t g_max_us = 2000;

inline void ensureTimersAllocated() {
  if (g_timers_allocated) return;
  ESP32PWM::allocateTimer(0);
  g_timers_allocated = true;
}

inline uint32_t clampUs(uint32_t us) {
  if (us < g_min_us) return g_min_us;
  if (us > g_max_us) return g_max_us;
  return us;
}

inline int clampDeg(int deg) {
  if (deg < 0) return 0;
  if (deg > 180) return 180;
  return deg;
}

inline bool initUs(
  int pin,
  uint32_t initial_us = 1500,
  uint32_t min_us = 1000,
  uint32_t max_us = 2000
) {
  if (pin < 0) {
    return false;
  }
  if (min_us >= max_us) {
    return false;
  }

  ensureTimersAllocated();

  g_min_us = min_us;
  g_max_us = max_us;
  myServo.setPeriodHertz(FREQ_HZ);
  const int ch = myServo.attach(pin, (int)min_us, (int)max_us);
  if (ch < 0) return false;

  g_attached = true;
  myServo.writeMicroseconds((int)clampUs(initial_us));
  return true;
}

inline bool init(int pin, int initial_deg = 90) {
  const int deg = clampDeg(initial_deg);
  const uint32_t initial_us = 1000UL + (uint32_t)deg * 1000UL / 180UL;
  return initUs(pin, initial_us, 1000, 2000);
}

inline bool writeUs(uint32_t us) {
  if (!g_attached) {
    return false;
  }
  myServo.writeMicroseconds((int)clampUs(us));
  return true;
}

inline bool writeDeg(int deg) {
  if (!g_attached) {
    return false;
  }
  myServo.write(clampDeg(deg));
  return true;
}

} // namespace pwm_servo

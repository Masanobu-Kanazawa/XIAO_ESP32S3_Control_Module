#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

namespace pwm_servo {

static constexpr int FREQ_HZ = 50;
static Servo myServo;
static bool g_attached = false;
static bool g_timers_allocated = false;

inline void ensureTimersAllocated() {
  if (g_timers_allocated) return;
  ESP32PWM::allocateTimer(0);
  g_timers_allocated = true;
}

inline uint32_t clampUs(uint32_t us) {
  if (us < 1000UL) return 1000UL;
  if (us > 2000UL) return 2000UL;
  return us;
}

inline int clampDeg(int deg) {
  if (deg < 0) return 0;
  if (deg > 180) return 180;
  return deg;
}

inline bool init(int pin, int initial_deg = 90) {
  if (pin < 0) {
    return false;
  }

  ensureTimersAllocated();

  myServo.setPeriodHertz(FREQ_HZ);
  const int ch = myServo.attach(pin, 1000, 2000);
  if (ch < 0) return false;

  myServo.write(clampDeg(initial_deg));
  g_attached = true;
  return true;
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

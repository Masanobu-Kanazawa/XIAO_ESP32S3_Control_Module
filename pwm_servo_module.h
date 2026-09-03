#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

namespace pwm_servo {

static constexpr int FREQ_HZ = 50;
static bool g_timers_allocated = false;

inline void ensureTimersAllocated() {
  if (g_timers_allocated) return;
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  g_timers_allocated = true;
}

inline int clampDeg(int deg) {
  if (deg < 0) return 0;
  if (deg > 180) return 180;
  return deg;
}

class Channel {
public:
  bool initUs(
    int pin,
    uint32_t initial_us = 1500,
    uint32_t min_us = 1000,
    uint32_t max_us = 2000
  ) {
    if (pin < 0 || min_us >= max_us) {
      return false;
    }

    ensureTimersAllocated();

    min_us_ = min_us;
    max_us_ = max_us;
    servo_.setPeriodHertz(FREQ_HZ);
    const int ch = servo_.attach(pin, (int)min_us, (int)max_us);
    if (ch < 0) return false;

    attached_ = true;
    servo_.writeMicroseconds((int)clampUs(initial_us));
    return true;
  }

  bool init(int pin, int initial_deg = 90) {
    const int deg = clampDeg(initial_deg);
    const uint32_t initial_us = 1000UL + (uint32_t)deg * 1000UL / 180UL;
    return initUs(pin, initial_us, 1000, 2000);
  }

  bool writeUs(uint32_t us) {
    if (!attached_) {
      return false;
    }
    servo_.writeMicroseconds((int)clampUs(us));
    return true;
  }

  bool writeDeg(int deg) {
    if (!attached_) {
      return false;
    }
    servo_.write(clampDeg(deg));
    return true;
  }

private:
  uint32_t clampUs(uint32_t us) const {
    if (us < min_us_) return min_us_;
    if (us > max_us_) return max_us_;
    return us;
  }

  Servo servo_;
  bool attached_ = false;
  uint32_t min_us_ = 1000;
  uint32_t max_us_ = 2000;
};

} // namespace pwm_servo

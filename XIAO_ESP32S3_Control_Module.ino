// XIAO ESP32S3 Sense - Integrated Control Module
// Features:
//  - SBUS receive (R3008SB) on Serial1 (inverted, 100k 8E2)
//  - MG92B PWM servo control in microseconds on the former STS connector
//  - Ultrasonic distance sensor (M5Stack Unit Ultrasonic-IO / RCWL-9620)
//  - ADS1015 differential voltage read via the IMU I2C bus
//  - IMU (AE-LSM6DSV16X) via I2C
//  - GPS (SparkFun GP-808G) via UART NMEA
//  - microSD logging (CSV) on Sense expansion board

#include <Arduino.h>
#include <Wire.h>

#include "sbus_module.h"
#include "ultrasonic_rcwl9620.h"
#include "hx711_bitbang.h"
#include "imu_lsm6dsv16x.h"
#include "ads1015_module.h"
#include "gps_gp808g.h"
#include "pwm_servo_module.h"
#include "sts3032_module.h"
#include "sd_logger_module.h"

// -----------------------------
// Feature toggles
// -----------------------------
#define USE_SBUS            1
#define USE_STS3032         0
#define USE_PWM_SERVO       1
#define USE_ULTRASONIC_IO   1
#define USE_HX711           0
#define USE_IMU_LSM6DSV16X  1
#define USE_ADS1015         1
#define USE_GPS_GP808G      1
#define USE_SD_LOGGING      1
#define GPS_USE_SOFTWARE_SERIAL 1

// -----------------------------
// Pins (GPIO)
// -----------------------------
static constexpr int PIN_SBUS_RX      = 44;   // D7
static constexpr int PIN_SBUS_TX_DMY  = -1;   // unused
static constexpr int PIN_STS_TX       = 43;   // D6 (TX only)
static constexpr int PIN_PWM_SERVO    = 43;   // D6 (former STS3032 signal pin)
static constexpr int PIN_ULTRA_SDA    = 1;    // D0
static constexpr int PIN_ULTRA_SCL    = 2;    // D1
static constexpr int PIN_GPS_RX       = 4;    // D3 (GP-808G TX/white)
static constexpr int PIN_GPS_TX_DMY   = -1;   // unused
// Sense expansion board extra pins:
// D11 -> GPIO42, D12 -> GPIO41 (requires cutting J1/J2, disables onboard mic)
static constexpr int PIN_HX711_DOUT   = 41;   // D12 (expansion)
static constexpr int PIN_HX711_SCK    = 42;   // D11 (expansion)
static constexpr int PIN_IMU_SDA      = 5;    // D4
static constexpr int PIN_IMU_SCL      = 6;    // D5
static constexpr uint32_t IMU_I2C_HZ  = 100000;
static constexpr uint32_t STS_BAUD    = 1000000;

// microSD (Sense expansion board)
static constexpr int PIN_SD_CS   = 21;
static constexpr int PIN_SD_SCK  = 7;
static constexpr int PIN_SD_MISO = 8;
static constexpr int PIN_SD_MOSI = 9;

// -----------------------------
// Module instances/state
// -----------------------------
#if USE_SBUS
sbus::Data g_sbus{};
static HardwareSerial& SerialSBUS = Serial1;
static uint32_t g_lastSbusFrameUs = 0;
static uint32_t g_sbusFrameIntervalUs = 0;
#endif

#if USE_STS3032
static HardwareSerial& SerialSTS = Serial2;
#endif

#if USE_PWM_SERVO
static bool g_pwmReady = false;
#endif

#if USE_ULTRASONIC_IO
ultra_rcwl9620::State g_ultra{};
#endif

#if USE_HX711
hx711_bitbang::Config g_hx_cfg{
  PIN_HX711_DOUT,
  PIN_HX711_SCK,
  0.0004056f,  // OUT_VOL [V/V]
  1020.0f,     // LOAD [g]
  20000.0f,    // HX711_R1
  8200.0f,     // HX711_R2
  1.25f,       // HX711_VBG
  128.0f       // HX711_PGA
};
hx711_bitbang::State g_hx{};
#endif

#if USE_IMU_LSM6DSV16X
imu_lsm6dsv16x::State g_imu{};
#endif

#if USE_ADS1015
ads1015_module::State g_ads{};
#endif

#if USE_GPS_GP808G
#if GPS_USE_SOFTWARE_SERIAL
#include <SoftwareSerial.h>
static SoftwareSerial SerialGPS(PIN_GPS_RX, PIN_GPS_TX_DMY);
#else
static constexpr int GPS_UART_NUM = 0;
static HardwareSerial SerialGPS(GPS_UART_NUM);
#endif
gps_gp808g::State g_gps{};
#endif

#if USE_SD_LOGGING
sd_logger::Logger g_logger{};
static bool g_manualLogTrigger = false;
static bool g_serialLineHasData = false;
static bool g_lastSerialCR = false;
#endif

static uint32_t g_lastPrintMs = 0;
static uint32_t g_lastLogMs = 0;
static uint32_t g_lastLoopRateMs = 0;
static uint32_t g_loopCount = 0;
static uint32_t g_loopHz = 0;
static constexpr uint32_t LOG_INTERVAL_MS = 100; // 10 Hz
static constexpr uint32_t ULTRA_POLL_INTERVAL_MS = 70;    // sensor conversion is ~50-70ms in practice
static constexpr uint32_t ULTRA_I2C_TIMEOUT_MS = 90;      // cap blocking on failure path
static constexpr uint32_t ULTRA_I2C_POLL_MS = 5;          // requestFrom polling interval
static constexpr bool ULTRA_I2C_DEBUG = false; // request count/raw debug prints are intentionally off
static constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 500;  // reduce serial-print overhead
static constexpr uint32_t HX711_TARE_DURATION_MS = 3000;  // tare sampling window

// -----------------------------
// Ultrasonic mix settings
// -----------------------------
// MG92B servo:
//  - CH8 fixed-mode side: fixed angle selected by CH5 three-position switch.
//  - otherwise: ultrasonic command when available; neutral when ultrasonic is disabled.
// Mapping direction:
//  - The command uses ultrasonic range 20..30cm only.
static constexpr float ULTRA_NEAR_CM = 20.0f;
static constexpr float ULTRA_FAR_CM = 30.0f;

// MG92B published range: 700..2300us corresponds to about 150deg.
// Treat the old STS angle command as an offset from the 1500us neutral point.
// Adjust these constants after measuring the actual linkage and safe travel.
static constexpr uint16_t MG92B_MIN_US = 700;
static constexpr uint16_t MG92B_NEUTRAL_US = 1500;
static constexpr uint16_t MG92B_MAX_US = 2300;
static constexpr float MG92B_RANGE_DEG = 150.0f;
static constexpr float MG92B_US_PER_DEG =
  (float)(MG92B_MAX_US - MG92B_MIN_US) / MG92B_RANGE_DEG;
static constexpr bool MG92B_REVERSE = false;

// Preserve the former STS control behavior.
static constexpr float SERVO_ULTRA_DEG_PER_10CM = 5.0f;
static constexpr uint8_t SERVO_MODE_CH8_INDEX = 7;           // CH8
static constexpr uint8_t SERVO_FIXED_LEVEL_CH5_INDEX = 4;    // CH5
static constexpr bool SERVO_FIXED_MODE_CH8_IS_HIGH = true;   // true: CH8 upper side enables fixed mode
static constexpr float SERVO_FIXED_DEG_LOW = 0.0f;           // CH5 low
static constexpr float SERVO_FIXED_DEG_MID = 3.3f;           // CH5 mid
static constexpr float SERVO_FIXED_DEG_HIGH = 5.7f;          // CH5 high

static float g_mg92bCommandDeg = 0.0f;
static uint16_t g_mg92bTargetUs = MG92B_NEUTRAL_US;

inline uint16_t mg92bPulseFromAngleOffset(float deg) {
  const float signed_deg = MG92B_REVERSE ? -deg : deg;
  const float pulse_us = (float)MG92B_NEUTRAL_US + signed_deg * MG92B_US_PER_DEG;
  return (uint16_t)constrain((int)lroundf(pulse_us), (int)MG92B_MIN_US, (int)MG92B_MAX_US);
}

inline float servoFixedDegFromCh5(uint16_t ch5_value) {
  const sbus::ThreePos sw = sbus::threePosFromValue(ch5_value);
  if (sw == sbus::ThreePos::Low) return SERVO_FIXED_DEG_LOW;
  if (sw == sbus::ThreePos::High) return SERVO_FIXED_DEG_HIGH;
  return SERVO_FIXED_DEG_MID;
}

inline bool servoFixedModeFromCh8(uint16_t ch8_value) {
  const sbus::ThreePos sw = sbus::threePosFromValue(ch8_value);
  if (SERVO_FIXED_MODE_CH8_IS_HIGH) {
    return sw == sbus::ThreePos::High;
  }
  return sw == sbus::ThreePos::Low;
}

inline float servoDegFromUltraCm(float cm) {
  if (isnan(cm) || cm <= 0.0f) return 0.0f;
  const float cm_clamped = constrain(cm, ULTRA_NEAR_CM, ULTRA_FAR_CM);
  return (cm_clamped / 10.0f) * SERVO_ULTRA_DEG_PER_10CM;
}

#if USE_GPS_GP808G
inline void printGpsFloatOrNA(float v, int digits) {
  if (isnan(v)) {
    Serial.print("N/A");
  } else {
    Serial.print(v, digits);
  }
}

inline void printGpsDebug(const gps_gp808g::State& gps) {
  Serial.print(" gps=");
  if (!gps.present) {
    Serial.print("NOT_DETECTED");
    return;
  }

  Serial.print(gps.fix ? "FIXED" : "RECEIVING");
  Serial.print(" sats=");
  Serial.print(gps.sats);
  Serial.print(" hdop=");
  printGpsFloatOrNA(gps.hdop, 1);
  Serial.print(" utc=");
  Serial.print(gps.utc_valid ? gps.utc : "N/A");
  Serial.print(" date=");
  Serial.print(gps.date_valid ? gps.date : "N/A");
  Serial.print(" lat=");
  printGpsFloatOrNA(gps.lat_deg, 6);
  Serial.print(" lon=");
  printGpsFloatOrNA(gps.lon_deg, 6);
  Serial.print(" alt_m=");
  printGpsFloatOrNA(gps.alt_m, 1);
  Serial.print(" spd_mps=");
  printGpsFloatOrNA(gps.speed_mps, 2);
  Serial.print(" crs_deg=");
  printGpsFloatOrNA(gps.course_deg, 1);
}
#endif

#if USE_HX711
inline bool runHx711OffsetCalibration(const char* phase) {
  Serial.print("[HX711] ");
  Serial.print(phase);
  Serial.print(" tare start (");
  Serial.print(HX711_TARE_DURATION_MS);
  Serial.println(" ms)");

  uint16_t valid_samples = 0;
  const bool ok = hx711_bitbang::tareForDuration(
    g_hx,
    g_hx_cfg,
    HX711_TARE_DURATION_MS,
    120,
    &valid_samples
  );

  Serial.print("[HX711] ");
  Serial.print(phase);
  if (ok) {
    Serial.print(" tare OK n=");
    Serial.print(valid_samples);
    Serial.print(" offset=");
    Serial.print(g_hx.offset, 3);
    Serial.println(" g");
  } else {
    Serial.print(" tare failed n=");
    Serial.println(valid_samples);
  }
  return ok;
}
#endif

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("=== XIAO ESP32S3 Sense Control Module ===");

#if USE_IMU_LSM6DSV16X || USE_ADS1015
#if USE_HX711
  if (PIN_IMU_SDA == PIN_HX711_DOUT || PIN_IMU_SDA == PIN_HX711_SCK ||
      PIN_IMU_SCL == PIN_HX711_DOUT || PIN_IMU_SCL == PIN_HX711_SCK) {
    Serial.println("[IMU] warning: selected I2C pins conflict with HX711 pins");
  }
#endif
  if (PIN_IMU_SDA >= 0 && PIN_IMU_SCL >= 0) {
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
  } else {
    Wire.begin();
  }
  Wire.setClock(IMU_I2C_HZ);
#endif

#if USE_IMU_LSM6DSV16X
  if (!imu_lsm6dsv16x::detectAndInit(Wire, g_imu)) {
    if (g_imu.addr == 0) {
      Serial.println("[IMU] LSM6DSV16X not found");
    } else {
      Serial.print("[IMU] init failed (addr=0x");
      Serial.print(g_imu.addr, HEX);
      Serial.print(", who=0x");
      Serial.print(g_imu.whoami, HEX);
      Serial.println(")");
    }
  } else {
    Serial.print("[IMU] LSM6DSV16X OK addr=0x");
    Serial.print(g_imu.addr, HEX);
    Serial.print(" who=0x");
    Serial.println(g_imu.whoami, HEX);
  }
#endif

#if USE_ADS1015
  if (!ads1015_module::init(Wire, g_ads)) {
    Serial.println("[ADS1015] not found at 0x48");
  } else {
    Serial.println("[ADS1015] AIN0-AIN1 differential init @128SPS, gain x16");
  }
#endif

#if USE_GPS_GP808G
#if GPS_USE_SOFTWARE_SERIAL
  SerialGPS.begin(gps_gp808g::DEFAULT_BAUD);
  Serial.print("[GPS] GP-808G SoftwareSerial RX=");
  Serial.print(PIN_GPS_RX);
  Serial.print(" TX=");
  Serial.print(PIN_GPS_TX_DMY);
  Serial.print(" init @");
  Serial.print(gps_gp808g::DEFAULT_BAUD);
  Serial.println("bps");
#else
  gps_gp808g::begin(SerialGPS, PIN_GPS_RX, PIN_GPS_TX_DMY, gps_gp808g::DEFAULT_BAUD);
  Serial.print("[GPS] GP-808G UART");
  Serial.print(GPS_UART_NUM);
  Serial.print(" init @");
  Serial.print(gps_gp808g::DEFAULT_BAUD);
  Serial.println("bps");
#endif
#endif

#if USE_SBUS
  SerialSBUS.begin(sbus::BAUD, SERIAL_8E2, PIN_SBUS_RX, PIN_SBUS_TX_DMY, true);
  Serial.println("[SBUS] Serial1 init");
#endif

#if USE_STS3032
  sts3032::begin(SerialSTS, PIN_STS_TX, STS_BAUD);
  Serial.print("[STS3032] Serial2 TX init @");
  Serial.print(STS_BAUD);
  Serial.println("bps");
#endif

#if USE_PWM_SERVO
  g_pwmReady = pwm_servo::initUs(
    PIN_PWM_SERVO,
    MG92B_NEUTRAL_US,
    MG92B_MIN_US,
    MG92B_MAX_US
  );
  if (g_pwmReady) {
    Serial.println("[MG92B] PWM init on former STS pin, neutral=1500us");
  } else {
    Serial.println("[MG92B] PWM init FAILED");
  }
#endif

#if USE_ULTRASONIC_IO
  ultra_rcwl9620::initPins(PIN_ULTRA_SDA, PIN_ULTRA_SCL, 100000);
  Serial.println("[ULTRA] I2C init (Wire1 addr=0x57)");
#endif

#if USE_HX711
  hx711_bitbang::initPins(g_hx_cfg);
  hx711_bitbang::reset(g_hx_cfg);
  delay(200);
  runHx711OffsetCalibration("startup");
#endif

#if USE_SD_LOGGING
  if (sd_logger::init(g_logger, PIN_SD_CS, true)) {
    Serial.println("[SD] init OK");
  } else {
    Serial.println("[SD] init FAILED");
  }
#endif
}

void loop() {
  const uint32_t now = millis();
  g_loopCount++;
  if (now - g_lastLoopRateMs >= 1000) {
    const uint32_t dt = now - g_lastLoopRateMs;
    if (dt > 0) {
      g_loopHz = (g_loopCount * 1000UL) / dt;
    }
    g_loopCount = 0;
    g_lastLoopRateMs = now;
  }

#if USE_SBUS
  const bool sbusFrameUpdated = sbus::poll(SerialSBUS, g_sbus);
  if (sbusFrameUpdated) {
    const uint32_t nowUs = micros();
    if (g_lastSbusFrameUs != 0) {
      g_sbusFrameIntervalUs = nowUs - g_lastSbusFrameUs;
    }
    g_lastSbusFrameUs = nowUs;
  }
#else
  const bool sbusFrameUpdated = false;
#endif

#if USE_GPS_GP808G
  gps_gp808g::poll(SerialGPS, g_gps);
#endif

#if USE_ULTRASONIC_IO
  ultra_rcwl9620::poll(g_ultra, now, ULTRA_POLL_INTERVAL_MS, ULTRA_I2C_TIMEOUT_MS, ULTRA_I2C_POLL_MS, ULTRA_I2C_DEBUG);
#endif

#if USE_HX711
  hx711_bitbang::poll(g_hx, g_hx_cfg);
#endif

#if USE_IMU_LSM6DSV16X
  imu_lsm6dsv16x::poll(Wire, g_imu, now, 10);
#endif

#if USE_ADS1015
  ads1015_module::poll(Wire, g_ads, now);
#endif

#if USE_PWM_SERVO
  bool use_fixed_servo = false;
#if USE_SBUS
  use_fixed_servo = servoFixedModeFromCh8(g_sbus.ch[SERVO_MODE_CH8_INDEX]);
#endif
  if (use_fixed_servo) {
#if USE_SBUS
    g_mg92bCommandDeg = servoFixedDegFromCh5(g_sbus.ch[SERVO_FIXED_LEVEL_CH5_INDEX]);
#endif
  } else {
#if USE_ULTRASONIC_IO
    g_mg92bCommandDeg = servoDegFromUltraCm(g_ultra.cm);
#else
    g_mg92bCommandDeg = 0.0f;
#endif
  }
  g_mg92bTargetUs = mg92bPulseFromAngleOffset(g_mg92bCommandDeg);
  if (g_pwmReady) {
    pwm_servo::writeUs(g_mg92bTargetUs);
  }
#endif

#if USE_SD_LOGGING && USE_SBUS
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') {
      if (!g_serialLineHasData) {
        g_manualLogTrigger = !g_manualLogTrigger;
        Serial.print("[SD] manual trigger ");
        Serial.println(g_manualLogTrigger ? "ON" : "OFF");
      }
      g_serialLineHasData = false;
      g_lastSerialCR = true;
      continue;
    }
    if (c == '\n') {
      if (g_lastSerialCR) {
        g_lastSerialCR = false; // skip LF in CRLF
        continue;
      }
      if (!g_serialLineHasData) {
        g_manualLogTrigger = !g_manualLogTrigger;
        Serial.print("[SD] manual trigger ");
        Serial.println(g_manualLogTrigger ? "ON" : "OFF");
      }
      g_serialLineHasData = false;
      continue;
    }
    g_serialLineHasData = true;
    g_lastSerialCR = false;
  }

  const bool sbusLogOn = sbus::logSwitchOn(g_sbus.ch);
  const bool sbusLogOff = sbus::logSwitchOff(g_sbus.ch);
  bool wantLog = g_manualLogTrigger;
  if (sbusLogOn) {
    wantLog = true;
  } else if (sbusLogOff) {
    wantLog = false;
  }
  if (wantLog && !g_logger.logging) {
#if USE_HX711
    runHx711OffsetCalibration("log-start");
#endif
    g_logger.logging = true;
    sd_logger::open(g_logger);
  } else if (!wantLog && g_logger.logging) {
    g_logger.logging = false;
    sd_logger::close(g_logger);
    Serial.println("[SD] logging stopped");
  }

  if (g_logger.logging && (now - g_lastLogMs >= LOG_INTERVAL_MS)) {
    g_lastLogMs = now;

    sd_logger::Row row{};
    row.ms = now;
    row.sbus_ch = g_sbus.ch;

#if USE_ULTRASONIC_IO
    row.ultra_us = g_ultra.duration_us;
    row.ultra_cm = g_ultra.cm;
#endif

#if USE_ADS1015
    row.ads_diff_v = g_ads.diff_volts;
#endif

#if USE_IMU_LSM6DSV16X
    if (g_imu.present) {
      row.imu_ax = g_imu.ax;
      row.imu_ay = g_imu.ay;
      row.imu_az = g_imu.az;
      row.imu_gx = g_imu.gx;
      row.imu_gy = g_imu.gy;
      row.imu_gz = g_imu.gz;
    }
#endif

#if USE_GPS_GP808G
    if (g_gps.present) {
      row.gps_fix = g_gps.fix ? 1 : 0;
      row.gps_sats = (int)g_gps.sats;
      snprintf(row.gps_utc, sizeof(row.gps_utc), "%s", g_gps.utc);
      snprintf(row.gps_date, sizeof(row.gps_date), "%s", g_gps.date);
      row.gps_year = (int)g_gps.year;
      row.gps_month = (int)g_gps.month;
      row.gps_day = (int)g_gps.day;
      row.gps_hour = (int)g_gps.hour;
      row.gps_minute = (int)g_gps.minute;
      row.gps_second = (int)g_gps.second;
      row.gps_centisecond = (int)g_gps.centisecond;
      row.gps_hdop = g_gps.hdop;
      row.gps_lat = g_gps.lat_deg;
      row.gps_lon = g_gps.lon_deg;
      row.gps_alt_m = g_gps.alt_m;
      row.gps_speed_mps = g_gps.speed_mps;
      row.gps_course_deg = g_gps.course_deg;
    }
#endif

    sd_logger::writeRow(g_logger, row);
    sd_logger::flushIfNeeded(g_logger, now);
  }
#endif

  if (now - g_lastPrintMs >= DEBUG_PRINT_INTERVAL_MS) {
    g_lastPrintMs = now;

    Serial.print("ms=");
    Serial.print(now);
    Serial.print(" loop_hz=");
    Serial.print(g_loopHz);

#if USE_SBUS
    Serial.print(" SBUS CH(");
    for (int i = 0; i < 16; i++) {
      Serial.print(g_sbus.ch[i]);
      if (i < 15) {
        Serial.print(",");
      }
    }
    Serial.print(")");
    Serial.print(" FS=");
    Serial.print(g_sbus.failsafe);
    Serial.print(" Lost=");
    Serial.print(g_sbus.frame_lost);
    Serial.print(" dt_ms=");
    if (g_sbusFrameIntervalUs > 0) {
      Serial.print((float)g_sbusFrameIntervalUs / 1000.0f, 2);
      Serial.print(" hz=");
      Serial.print(1000000.0f / (float)g_sbusFrameIntervalUs, 1);
    } else {
      Serial.print("N/A");
    }
#endif

#if USE_ULTRASONIC_IO
    Serial.print(" ultra_raw=");
    Serial.print(g_ultra.duration_us);
    Serial.print(" cm=");
    Serial.print(g_ultra.cm, 1);
#endif

#if USE_HX711
    Serial.print(" hx_g=");
    Serial.print(g_hx.gram, 2);
    Serial.print(" hx_raw=");
    Serial.print(g_hx.raw);
#endif

#if USE_ADS1015
    Serial.print(" ads_a0_a1_v=");
    if (g_ads.present) {
      Serial.print(g_ads.diff_volts, 6);
    } else {
      Serial.print("N/A");
    }
#endif

#if USE_PWM_SERVO
    Serial.print(" mg92b_deg=");
    Serial.print(g_mg92bCommandDeg, 2);
    Serial.print(" us=");
    Serial.print(g_mg92bTargetUs);
#endif

#if USE_IMU_LSM6DSV16X
    Serial.print(" imu=");
    if (g_imu.present) {
      Serial.print("a(");
      Serial.print(g_imu.ax, 3); Serial.print(",");
      Serial.print(g_imu.ay, 3); Serial.print(",");
      Serial.print(g_imu.az, 3); Serial.print(")");
      Serial.print(" g(");
      Serial.print(g_imu.gx, 2); Serial.print(",");
      Serial.print(g_imu.gy, 2); Serial.print(",");
      Serial.print(g_imu.gz, 2); Serial.print(")");
    } else {
      Serial.print("N/A");
    }
#endif

#if USE_GPS_GP808G
    printGpsDebug(g_gps);
#endif

#if USE_SD_LOGGING
    Serial.print(" log=");
    Serial.print(g_logger.logging ? "ON" : "OFF");
#endif

    Serial.println();
  }
}

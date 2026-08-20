#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <limits.h>

namespace sd_logger {

struct Row {
  uint32_t ms{0};
  const uint16_t* sbus_ch{nullptr};
  uint32_t ultra_us{0};
  float ultra_cm{NAN};
  float ads_diff_v{NAN};
  float imu_ax{NAN};
  float imu_ay{NAN};
  float imu_az{NAN};
  float imu_gx{NAN};
  float imu_gy{NAN};
  float imu_gz{NAN};
  int gps_fix{0};
  int gps_sats{0};
  char gps_utc[16]{};
  char gps_date[8]{};
  int gps_year{0};
  int gps_month{0};
  int gps_day{0};
  int gps_hour{0};
  int gps_minute{0};
  int gps_second{0};
  int gps_centisecond{0};
  float gps_hdop{NAN};
  float gps_lat{NAN};
  float gps_lon{NAN};
  float gps_alt_m{NAN};
  float gps_speed_mps{NAN};
  float gps_course_deg{NAN};
};

struct Logger {
  File file{};
  bool logging{false};
};

inline String makeFilename() {
  const uint32_t ms = millis();
  char buf[32];
  snprintf(buf, sizeof(buf), "/log_%lu.csv", (unsigned long)ms);
  return String(buf);
}

inline void printCardInfo() {
  const uint8_t card_type = SD.cardType();
  if (card_type == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (card_type == CARD_MMC) {
    Serial.println("MMC");
  } else if (card_type == CARD_SD) {
    Serial.println("SDSC");
  } else if (card_type == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  const uint64_t card_size = SD.cardSize() / (1024ULL * 1024ULL);
  Serial.printf("SD Card Size: %lluMB\n", card_size);
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024ULL * 1024ULL));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024ULL * 1024ULL));
}

inline void listDir(fs::FS& fs, const char* dirname, uint8_t levels = 0) {
  Serial.printf("Listing directory: %s\n", dirname);
  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) listDir(fs, file.name(), levels - 1);
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

// Seeed wiki SD_Test style:
// - SD.begin(cs_pin)
// - cardType()/cardSize() checks after mount
inline bool init(Logger& logger, int cs_pin = 21, bool print_info = true) {
  (void)logger;
  if (!SD.begin(cs_pin)) {
    Serial.println("Card Mount Failed");
    return false;
  }
  if (SD.cardType() == CARD_NONE) {
    Serial.println("No SD card attached");
    return false;
  }
  if (print_info) {
    printCardInfo();
    listDir(SD, "/", 0);
  }
  return true;
}

inline bool open(Logger& logger) {
  if (logger.file) logger.file.close();

  const String name = makeFilename();
  logger.file = SD.open(name.c_str(), FILE_WRITE);
  if (!logger.file) {
    Serial.println("[SD] open failed");
    return false;
  }

  logger.file.println("ms,sbus_ch0,sbus_ch1,sbus_ch2,sbus_ch3,sbus_ch4,sbus_ch5,ultra_us,ultra_cm,ads_a0_a1_v,imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz,gps_fix,gps_sats,gps_utc,gps_date,gps_year,gps_month,gps_day,gps_hour,gps_minute,gps_second,gps_centisecond,gps_hdop,gps_lat,gps_lon,gps_alt_m,gps_speed_mps,gps_course_deg");
  logger.file.flush();
  Serial.print("[SD] logging to ");
  Serial.println(name);
  return true;
}

inline void close(Logger& logger) {
  if (!logger.file) return;
  logger.file.flush();
  logger.file.close();
}

inline void writeRow(Logger& logger, const Row& row) {
  if (!logger.file || row.sbus_ch == nullptr) return;

  char line[560];
  snprintf(line, sizeof(line),
           "%lu,%u,%u,%u,%u,%u,%u,%lu,%.2f,%.6f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%d,%s,%s,%d,%d,%d,%d,%d,%d,%d,%.2f,%.7f,%.7f,%.2f,%.3f,%.2f",
           (unsigned long)row.ms,
           row.sbus_ch[0], row.sbus_ch[1], row.sbus_ch[2], row.sbus_ch[3], row.sbus_ch[4], row.sbus_ch[5],
           (unsigned long)row.ultra_us, row.ultra_cm, row.ads_diff_v,
           row.imu_ax, row.imu_ay, row.imu_az, row.imu_gx, row.imu_gy, row.imu_gz,
           row.gps_fix, row.gps_sats, row.gps_utc, row.gps_date,
           row.gps_year, row.gps_month, row.gps_day, row.gps_hour, row.gps_minute, row.gps_second, row.gps_centisecond,
           row.gps_hdop, row.gps_lat, row.gps_lon, row.gps_alt_m, row.gps_speed_mps, row.gps_course_deg);
  logger.file.println(line);
}

inline void flushIfNeeded(Logger& logger, uint32_t now_ms) {
  if ((now_ms % 1000) < 60 && logger.file) logger.file.flush();
}

} // namespace sd_logger

// Direct-register driver for the MPU6500-compatible IMU (WHO_AM_I 0x70) on Wire.
// The Adafruit MPU6050 library rejects this chip, so it is deliberately not used.
#pragma once
#include <stdint.h>
#include "fall_detector.h"  // Sample

enum class CalResult : uint8_t { OK, MOVED, IO_FAIL };
enum class CalSource : uint8_t { FRESH, STORED, NONE };  // where the gyro bias came from

class Imu {
 public:
  // Bus-clear, init, configure and verify. Safe to call again for recovery.
  bool begin();
  // Burst-read 14 bytes and convert. Returns false on I2C error or frozen data.
  bool read(Sample& s, uint32_t now);
  // Re-read config registers; false means the IMU reset or went to sleep.
  bool verifyConfig();

  // Blocking ~2 s; the device must be still.
  CalResult calibrate();
  void saveBias();  // NVS, so a later boot can fall back to it
  bool loadBias();

  uint8_t whoAmI() const { return who_; }
  uint8_t consecutiveErrors() const { return consecErr_; }  // includes frozen-data frames
  uint32_t totalErrors() const { return errors_; }
  uint32_t saturations() const { return saturations_; }
  const char* lastError() const { return lastErr_; }
  const float* bias() const { return bias_; }

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n);
  bool readRaw(int16_t raw[7]);

  uint8_t who_ = 0;
  uint8_t consecErr_ = 0;
  uint8_t frozen_ = 0;
  uint8_t lastFrame_[14] = {0};
  uint32_t errors_ = 0;
  uint32_t saturations_ = 0;
  const char* lastErr_ = "";
  float bias_[3] = {0, 0, 0};
};

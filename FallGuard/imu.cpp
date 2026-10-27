#include "imu.h"
#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

namespace {
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;  // MPU6500/9250 only
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;   // ax ay az temp gx gy gz, big-endian
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr uint8_t WHO_MPU6050 = 0x68;
constexpr uint8_t WHO_MPU6500 = 0x70;  // what this board returns
constexpr uint8_t WHO_MPU9250 = 0x71;
constexpr uint8_t WHO_MPU9255 = 0x73;

// Free a slave that is holding SDA low (e.g. reset mid-transfer): clock out
// up to 9 bits, then issue a STOP.
void busClear() {
  Wire.end();
  pinMode(PIN_IMU_SDA, INPUT_PULLUP);
  pinMode(PIN_IMU_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_IMU_SCL, HIGH);
  delayMicroseconds(5);
  for (int i = 0; i < 9 && digitalRead(PIN_IMU_SDA) == LOW; i++) {
    digitalWrite(PIN_IMU_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_IMU_SCL, HIGH);
    delayMicroseconds(5);
  }
  pinMode(PIN_IMU_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_IMU_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_IMU_SDA, HIGH);
  delayMicroseconds(5);
}
}  // namespace

bool Imu::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool Imu::readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  if (Wire.requestFrom(IMU_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool Imu::begin() {
  busClear();
  if (!Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL, IMU_I2C_HZ)) {
    lastErr_ = "Wire.begin failed";
    return false;
  }
  Wire.setTimeOut(IMU_I2C_TIMEOUT_MS);

  if (!readRegs(REG_WHO_AM_I, &who_, 1)) {
    lastErr_ = "no ACK at 0x68";
    return false;
  }
  if (who_ != WHO_MPU6500 && who_ != WHO_MPU6050 && who_ != WHO_MPU9250 && who_ != WHO_MPU9255) {
    lastErr_ = "unknown WHO_AM_I";
    return false;
  }

  writeReg(REG_PWR_MGMT_1, 0x80);  // device reset -> known register state
  delay(100);
  bool ok = writeReg(REG_PWR_MGMT_1, 0x01)  // wake, PLL clock (more stable than 0x00 internal RC)
            && writeReg(REG_SMPLRT_DIV, 0x00)  // 1 kHz internal; DLPF does the anti-aliasing
            && writeReg(REG_CONFIG, IMU_DLPF_CFG) && writeReg(REG_GYRO_CONFIG, IMU_GYRO_CFG) &&
            writeReg(REG_ACCEL_CONFIG, IMU_ACCEL_CFG);
  if (ok && who_ != WHO_MPU6050) ok = writeReg(REG_ACCEL_CONFIG2, IMU_DLPF_CFG);
  if (!ok) {
    lastErr_ = "config write failed";
    return false;
  }
  delay(50);  // gyro PLL start-up
  consecErr_ = frozen_ = 0;
  if (!verifyConfig()) return false;
  lastErr_ = "";
  return true;
}

bool Imu::verifyConfig() {
  uint8_t r[2], pwr;
  if (!readRegs(REG_GYRO_CONFIG, r, 2) || !readRegs(REG_PWR_MGMT_1, &pwr, 1)) {
    lastErr_ = "verify read failed";
    return false;
  }
  if ((r[0] & 0x18) != (IMU_GYRO_CFG & 0x18) || (r[1] & 0x18) != (IMU_ACCEL_CFG & 0x18) || (pwr & 0x40)) {
    lastErr_ = "config lost (IMU reset/asleep)";
    return false;
  }
  return true;
}

bool Imu::readRaw(int16_t raw[7]) {
  uint8_t b[14];
  if (!readRegs(REG_ACCEL_XOUT_H, b, 14)) {
    errors_++;
    consecErr_++;
    lastErr_ = "burst read failed";
    return false;
  }
  // An awake sensor's noise always changes some LSB; identical frames mean
  // the chip is asleep or stuck while still ACKing.
  frozen_ = memcmp(b, lastFrame_, 14) == 0 ? frozen_ + 1 : 0;
  memcpy(lastFrame_, b, 14);
  if (frozen_ >= IMU_MAX_FROZEN_FRAMES) {
    errors_++;
    consecErr_++;
    lastErr_ = "data frozen";
    return false;
  }
  consecErr_ = 0;
  for (int i = 0; i < 7; i++) raw[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
  return true;
}

bool Imu::read(Sample& s, uint32_t now) {
  int16_t r[7];
  if (!readRaw(r)) return false;
  s.t_ms = now;
  s.saturated = false;
  for (int i : {0, 1, 2, 4, 5, 6})
    if (r[i] == INT16_MAX || r[i] == INT16_MIN) s.saturated = true;
  if (s.saturated) saturations_++;

  s.ax = r[0] / IMU_ACCEL_LSB_PER_G;
  s.ay = r[1] / IMU_ACCEL_LSB_PER_G;
  s.az = r[2] / IMU_ACCEL_LSB_PER_G;
  s.gx = r[4] / IMU_GYRO_LSB_PER_DPS - bias_[0];
  s.gy = r[5] / IMU_GYRO_LSB_PER_DPS - bias_[1];
  s.gz = r[6] / IMU_GYRO_LSB_PER_DPS - bias_[2];
  s.accMag = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
  s.gyroMag = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  return true;
}

CalResult Imu::calibrate() {
  double gSum[3] = {0, 0, 0}, gSq[3] = {0, 0, 0}, aSum = 0, aSq = 0;
  int n = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    int16_t r[7];
    if (readRaw(r)) {
      for (int k = 0; k < 3; k++) {
        double g = r[4 + k] / IMU_GYRO_LSB_PER_DPS;
        gSum[k] += g;
        gSq[k] += g * g;
      }
      double ax = r[0] / IMU_ACCEL_LSB_PER_G, ay = r[1] / IMU_ACCEL_LSB_PER_G, az = r[2] / IMU_ACCEL_LSB_PER_G;
      double m = sqrt(ax * ax + ay * ay + az * az);
      aSum += m;
      aSq += m * m;
      n++;
    }
    delay(CAL_INTERVAL_MS);
  }
  if (n < CAL_MIN_VALID) {
    lastErr_ = "cal: too few valid samples";
    return CalResult::IO_FAIL;
  }

  float mean[3];
  for (int k = 0; k < 3; k++) {
    mean[k] = gSum[k] / n;
    double sd = sqrt(fmax(0.0, gSq[k] / n - mean[k] * mean[k]));
    if (sd > CAL_MAX_GYRO_STD_DPS || fabsf(mean[k]) > CAL_MAX_GYRO_BIAS_DPS) {
      lastErr_ = "cal: device moved";
      return CalResult::MOVED;
    }
  }
  double aMean = aSum / n, aSd = sqrt(fmax(0.0, aSq / n - aMean * aMean));
  if (fabs(aMean - 1.0) > CAL_MAX_ACC_DEV_G || aSd > CAL_MAX_ACC_STD_G) {
    lastErr_ = "cal: accel not at rest";
    return CalResult::MOVED;
  }
  memcpy(bias_, mean, sizeof bias_);
  lastErr_ = "";
  return CalResult::OK;
}

void Imu::saveBias() {
  Preferences p;
  if (!p.begin("fallguard", false)) return;
  p.putBytes("gbias", bias_, sizeof bias_);
  p.end();
}

bool Imu::loadBias() {
  float b[3];
  Preferences p;
  if (!p.begin("fallguard", true)) return false;
  size_t n = p.getBytes("gbias", b, sizeof b);
  p.end();
  if (n != sizeof b) return false;
  for (float v : b)
    if (!isfinite(v) || fabsf(v) > CAL_MAX_GYRO_BIAS_DPS) return false;
  memcpy(bias_, b, sizeof bias_);
  return true;
}

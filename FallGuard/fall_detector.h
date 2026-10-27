// Fall-detection state machine. Pure C++ (no Arduino), so it runs unchanged in
// test/test_fall_detector.cpp for synthetic tests and CSV replay.
#pragma once
#include <stdint.h>
#include "config.h"
#include "fall_model.h"

// One calibrated IMU reading.
struct Sample {
  uint32_t t_ms;
  float ax, ay, az;  // g
  float gx, gy, gz;  // dps, bias-corrected
  float accMag;      // g
  float gyroMag;     // dps
  bool saturated;    // any axis at full-scale
};

enum class FallState : uint8_t {
  MONITORING,
  FREE_FALL,        // |a| dropped: candidate event, waiting for impact
  IMPACT,           // |a| spike: collecting peak acceleration / rotation
  POST_IMPACT,      // settling, then checking inactivity + orientation
  ALERT_PENDING,    // local alarm, cancel window running
  ALERT_CONFIRMED,  // latched until cancelled
  CANCELLED,        // re-arm delay
  SENSOR_FAULT,     // no valid IMU data: never treated as "normal"
};

enum class Cmd : uint8_t { NONE, CANCEL, SOS };
enum class Reason : uint8_t { NONE, FALL, MANUAL_SOS, SENSOR_LOST };  // why the last alert fired

// Defaults switch to ML mode automatically once fall_model.h holds a trained model.
struct DetectorConfig {
  bool useModel = FALL_MODEL_TRAINED;  // model decides instead of the still/orient/rotation rules
  float freefallG = DET_FREEFALL_G;
  float impactG = FALL_MODEL_TRAINED ? DET_ML_TRIGGER_G : DET_IMPACT_G;
  uint32_t freefallToImpactMs = DET_FREEFALL_TO_IMPACT_MS;
  uint32_t impactWindowMs = DET_IMPACT_WINDOW_MS;
  float minRotationDps = FALL_MODEL_TRAINED ? 0.0f : DET_MIN_ROTATION_DPS;
  uint32_t settleMs = DET_SETTLE_MS;
  uint32_t stillWindowMs = DET_STILL_WINDOW_MS;
  float stillAccTolG = DET_STILL_ACC_TOL_G;
  float stillGyroDps = DET_STILL_GYRO_DPS;
  float minStillFraction = DET_MIN_STILL_FRACTION;
  float minOrientChangeDeg = DET_MIN_ORIENT_CHANGE_DEG;
  float gravityAlpha = DET_GRAVITY_ALPHA;
  uint32_t cancelWindowMs = DET_CANCEL_WINDOW_MS;
  uint32_t rearmMs = DET_REARM_MS;
};

class FallDetector {
 public:
  explicit FallDetector(const DetectorConfig& cfg = DetectorConfig()) : cfg_(cfg) {}

  // Call once per sample period.
  //   s           new sample, or nullptr if none arrived this tick
  //   sensorFault true when the IMU is declared failed (not just one missed read)
  //   cmd         button / app command
  FallState update(uint32_t now, const Sample* s, bool sensorFault, Cmd cmd);

  FallState state() const { return st_; }
  uint32_t stateSince() const { return since_; }
  uint8_t countdownSec(uint32_t now) const;  // ALERT_PENDING only, else 0
  uint32_t alertCount() const { return alerts_; }

  // Why the last alert fired, and the features of the last analysed event.
  Reason reason() const { return reason_; }
  float peakG() const { return peakG_; }
  float minG() const { return minG_; }
  float peakDps() const { return peakDps_; }
  float stillFraction() const { return stillFrac_; }
  float orientChangeDeg() const { return orientDeg_; }

 private:
  void enter(FallState s, uint32_t now);
  void beginEvent();
  void raiseAlert(Reason why, uint32_t now);

  DetectorConfig cfg_;
  FallState st_ = FallState::MONITORING;
  uint32_t since_ = 0;
  uint32_t alerts_ = 0;
  Reason reason_ = Reason::NONE;

  bool haveGravity_ = false;
  float grav_[3] = {0, 0, 0};  // low-pass accel while MONITORING
  float pre_[3] = {0, 0, 0};   // gravity frozen at event start
  float post_[3] = {0, 0, 0};  // accel sum during stillness window
  uint32_t n_ = 0, stillN_ = 0;
  // Running min |a| over the last 1-2 buckets, so a direct impact still sees a preceding dip.
  float minPrev_ = 9, minCur_ = 9;
  uint32_t bucketStart_ = 0;

  float peakG_ = 0, minG_ = 0, peakDps_ = 0, stillFrac_ = 0, orientDeg_ = 0;
};

const char* stateName(FallState s);
const char* reasonName(Reason r);

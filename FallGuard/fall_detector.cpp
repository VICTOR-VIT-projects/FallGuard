#include "fall_detector.h"
#include <math.h>

namespace {
float angleDeg(const float a[3], const float b[3]) {
  float na = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  float nb = sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
  if (na < 1e-3f || nb < 1e-3f) return 0;
  float c = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb);
  c = c > 1 ? 1 : (c < -1 ? -1 : c);
  return acosf(c) * 57.29578f;
}
}  // namespace

const char* stateName(FallState s) {
  switch (s) {
    case FallState::MONITORING: return "MONITORING";
    case FallState::FREE_FALL: return "FREE_FALL";
    case FallState::IMPACT: return "IMPACT";
    case FallState::POST_IMPACT: return "POST_IMPACT";
    case FallState::ALERT_PENDING: return "ALERT_PENDING";
    case FallState::ALERT_CONFIRMED: return "ALERT_CONFIRMED";
    case FallState::CANCELLED: return "CANCELLED";
    case FallState::SENSOR_FAULT: return "SENSOR_FAULT";
  }
  return "?";
}

const char* reasonName(Reason r) {
  switch (r) {
    case Reason::FALL: return FALL_MODEL_TRAINED ? "fall (ML model)" : "impact+rotation+still";
    case Reason::MANUAL_SOS: return "manual SOS";
    case Reason::SENSOR_LOST: return "sensor lost post-impact";
    default: return "-";
  }
}

void FallDetector::enter(FallState s, uint32_t now) {
  st_ = s;
  since_ = now;
}

void FallDetector::beginEvent() {
  for (int i = 0; i < 3; i++) pre_[i] = grav_[i];
  peakG_ = peakDps_ = stillFrac_ = orientDeg_ = 0;
  minG_ = minPrev_ < minCur_ ? minPrev_ : minCur_;
  minPrev_ = minCur_ = 9;
}

void FallDetector::raiseAlert(Reason why, uint32_t now) {
  reason_ = why;
  alerts_++;
  enter(FallState::ALERT_PENDING, now);
}

uint8_t FallDetector::countdownSec(uint32_t now) const {
  if (st_ != FallState::ALERT_PENDING) return 0;
  uint32_t el = now - since_;
  if (el >= cfg_.cancelWindowMs) return 0;
  return (cfg_.cancelWindowMs - el + 999) / 1000;
}

FallState FallDetector::update(uint32_t now, const Sample* s, bool sensorFault, Cmd cmd) {
  const uint32_t dt = now - since_;
  using S = FallState;

  // 1. Commands and alert timers: these never depend on the sensor.
  if (cmd == Cmd::SOS && st_ != S::ALERT_CONFIRMED) {
    reason_ = Reason::MANUAL_SOS;
    alerts_++;
    enter(S::ALERT_CONFIRMED, now);
    return st_;
  }
  if (cmd == Cmd::CANCEL && (st_ == S::ALERT_PENDING || st_ == S::ALERT_CONFIRMED)) {
    enter(S::CANCELLED, now);
    return st_;
  }
  switch (st_) {
    case S::ALERT_PENDING:
      if (dt >= cfg_.cancelWindowMs) enter(S::ALERT_CONFIRMED, now);
      return st_;
    case S::ALERT_CONFIRMED:
      return st_;
    case S::CANCELLED:
      if (dt >= cfg_.rearmMs) enter(sensorFault ? S::SENSOR_FAULT : S::MONITORING, now);
      return st_;
    default:
      break;
  }

  // 2. Sensor failure. Losing the IMU right after an impact is suspicious
  //    (a hard fall can damage wiring), so fail safe and let the user cancel.
  if (sensorFault) {
    if (st_ == S::IMPACT || st_ == S::POST_IMPACT)
      raiseAlert(Reason::SENSOR_LOST, now);
    else if (st_ != S::SENSOR_FAULT)
      enter(S::SENSOR_FAULT, now);
    return st_;
  }
  if (st_ == S::SENSOR_FAULT) {
    haveGravity_ = false;  // orientation reference is unknown after an outage
    enter(S::MONITORING, now);
  }
  if (!s) return st_;  // single missed read: just wait for the next sample

  // 3. Motion analysis.
  const float a[3] = {s->ax, s->ay, s->az};
  switch (st_) {
    case S::MONITORING:
      if (!haveGravity_) {
        for (int i = 0; i < 3; i++) grav_[i] = a[i];
        haveGravity_ = true;
      }
      for (int i = 0; i < 3; i++) grav_[i] += cfg_.gravityAlpha * (a[i] - grav_[i]);
      if (s->accMag < minCur_) minCur_ = s->accMag;
      if (now - bucketStart_ >= DET_PRE_MIN_BUCKET_MS) {
        minPrev_ = minCur_;
        minCur_ = 9;
        bucketStart_ = now;
      }
      if (s->accMag >= cfg_.impactG) {  // not every fall has a clean free-fall phase
        beginEvent();
        peakG_ = s->accMag;
        peakDps_ = s->gyroMag;
        enter(S::IMPACT, now);
      } else if (s->accMag <= cfg_.freefallG) {
        beginEvent();
        peakDps_ = s->gyroMag;
        enter(S::FREE_FALL, now);
      }
      break;

    case S::FREE_FALL:
      if (s->gyroMag > peakDps_) peakDps_ = s->gyroMag;
      if (s->accMag < minG_) minG_ = s->accMag;
      if (s->accMag >= cfg_.impactG) {
        peakG_ = s->accMag;
        enter(S::IMPACT, now);
      } else if (dt > cfg_.freefallToImpactMs) {
        enter(S::MONITORING, now);
      }
      break;

    case S::IMPACT:
      if (s->accMag > peakG_) peakG_ = s->accMag;
      if (s->accMag < minG_) minG_ = s->accMag;
      if (s->gyroMag > peakDps_) peakDps_ = s->gyroMag;
      if (dt >= cfg_.impactWindowMs) {
        if (peakDps_ < cfg_.minRotationDps) {
          enter(S::MONITORING, now);  // big jolt without rotation: jump, stomp, bump
        } else {
          n_ = stillN_ = 0;
          post_[0] = post_[1] = post_[2] = 0;
          enter(S::POST_IMPACT, now);
        }
      }
      break;

    case S::POST_IMPACT: {
      if (dt < cfg_.settleMs) break;
      n_++;
      if (fabsf(s->accMag - 1.0f) <= cfg_.stillAccTolG && s->gyroMag <= cfg_.stillGyroDps) stillN_++;
      for (int i = 0; i < 3; i++) post_[i] += a[i];
      if (dt < cfg_.settleMs + cfg_.stillWindowMs) break;

      stillFrac_ = n_ ? (float)stillN_ / n_ : 0;
      orientDeg_ = angleDeg(pre_, post_);
      bool still = stillFrac_ >= cfg_.minStillFraction;
      bool turned = cfg_.minOrientChangeDeg <= 0 || orientDeg_ >= cfg_.minOrientChangeDeg;
      const float f[] = {peakG_, minG_, peakDps_, stillFrac_, orientDeg_};  // order: fall_model.h
      if (cfg_.useModel ? fallModelPredict(f) : (still && turned))
        raiseAlert(Reason::FALL, now);
      else
        enter(S::MONITORING, now);  // got up / normal activity
      break;
    }

    default:
      break;
  }
  return st_;
}

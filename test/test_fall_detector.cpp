// Host-side checks for the fall-detection state machine (no ESP32 needed).
//
//   g++ -std=c++14 -I../FallGuard test_fall_detector.cpp ../FallGuard/fall_detector.cpp -o fdtest
//   ./fdtest                       run synthetic scenario tests
//   ./fdtest FG_0001.csv           replay a recorded log through the detector
//   ./fdtest --features a.csv ...  per-event feature table for ml/train.py
//
// Synthetic signals only prove the LOGIC is right. They say nothing about
// real-world accuracy; that needs labelled recordings (README "Evaluation").
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include "fall_detector.h"

using S = FallState;

// The scenario tests check the hand-set rules, whether or not a model is trained.
DetectorConfig rulesCfg() {
  DetectorConfig c;
  c.useModel = false;
  c.impactG = DET_IMPACT_G;
  c.minRotationDps = DET_MIN_ROTATION_DPS;
  return c;
}

struct Sim {
  FallDetector d{rulesCfg()};
  uint32_t t = 0;

  S step(float ax, float ay, float az, float gyro, bool fault = false, Cmd c = Cmd::NONE, bool noSample = false) {
    Sample s{t, ax, ay, az, gyro, 0, 0, sqrtf(ax * ax + ay * ay + az * az), fabsf(gyro), false};
    S st = d.update(t, (fault || noSample) ? nullptr : &s, fault, c);
    t += SAMPLE_PERIOD_MS;
    return st;
  }
  S hold(uint32_t ms, float ax, float ay, float az, float gyro) {
    S st = d.state();
    for (uint32_t i = 0; i < ms / SAMPLE_PERIOD_MS; i++) st = step(ax, ay, az, gyro);
    return st;
  }
  // upright -> free fall with rotation -> impact; leaves the detector in IMPACT
  void fallToImpact(float rot = 250) {
    hold(2000, 0, 0, 1, 2);
    assert(hold(300, 0, 0.1f, 0.2f, rot) == S::FREE_FALL);
    assert(hold(50, 0, 3.0f, 2.0f, rot) == S::IMPACT);
  }
};

#define CHECK(name, cond)                    \
  do {                                       \
    if (!(cond)) {                           \
      printf("FAIL  %s\n", name);            \
      fails++;                               \
    } else {                                 \
      printf("ok    %s\n", name);            \
    }                                        \
  } while (0)

int selfTest() {
  int fails = 0;
  const uint32_t post = DET_IMPACT_WINDOW_MS + DET_SETTLE_MS + DET_STILL_WINDOW_MS + 100;

  { Sim m; CHECK("standing still stays MONITORING", m.hold(5000, 0, 0, 1, 2) == S::MONITORING); }

  {
    Sim m;
    m.fallToImpact();
    CHECK("fall + lying still -> ALERT_PENDING", m.hold(post, 1, 0, 0, 2) == S::ALERT_PENDING);
    CHECK("reason is FALL", m.d.reason() == Reason::FALL);
    CHECK("min |a| feature sees the free fall", fabsf(m.d.minG() - sqrtf(0.1f * 0.1f + 0.2f * 0.2f)) < 1e-3f);
    CHECK("countdown starts at 10 s", m.d.countdownSec(m.t) == 10);
    CHECK("no cancel -> ALERT_CONFIRMED", m.hold(DET_CANCEL_WINDOW_MS + 20, 1, 0, 0, 2) == S::ALERT_CONFIRMED);
    CHECK("confirmed alert is latched", m.hold(30000, 1, 0, 0, 2) == S::ALERT_CONFIRMED);
    CHECK("cancel clears confirmed alert", m.step(1, 0, 0, 2, false, Cmd::CANCEL) == S::CANCELLED);
    CHECK("re-arms after delay", m.hold(DET_REARM_MS + 20, 0, 0, 1, 2) == S::MONITORING);
  }

  {
    Sim m;
    m.fallToImpact();
    m.hold(post, 1, 0, 0, 2);
    CHECK("cancel during pending window", m.step(1, 0, 0, 2, false, Cmd::CANCEL) == S::CANCELLED);
  }

  {
    Sim m;
    m.fallToImpact(20);  // no rotation
    CHECK("jump (impact, no rotation) rejected", m.hold(post, 0, 0, 1, 2) == S::MONITORING);
  }

  {
    Sim m;
    m.fallToImpact();
    CHECK("moving after impact (got up) rejected", m.hold(post, 1, 0, 0, 120) == S::MONITORING);
  }

  {
    Sim m;
    m.fallToImpact();
    CHECK("still but upright (sat down hard) rejected", m.hold(post, 0, 0, 1, 2) == S::MONITORING);
  }

  {
    Sim m;
    m.hold(2000, 0, 0, 1, 2);
    m.hold(100, 0, 0, 0.7f, 30);  // mild dip, above the free-fall threshold
    assert(m.hold(50, 0, 3.0f, 2.0f, 250) == S::IMPACT);
    CHECK("impact without free-fall still detected", m.hold(post, 1, 0, 0, 2) == S::ALERT_PENDING);
    CHECK("min |a| looks back before a direct impact", fabsf(m.d.minG() - 0.7f) < 1e-3f);
  }

  {
    Sim m;
    m.hold(2000, 0, 0, 1, 2);
    assert(m.hold(200, 0, 0, 0.2f, 0) == S::FREE_FALL);  // stumble: brief low-g, no impact
    CHECK("free-fall without impact times out", m.hold(DET_FREEFALL_TO_IMPACT_MS + 100, 0, 0, 1, 2) == S::MONITORING);
  }

  {
    Sim m;
    m.fallToImpact();
    m.hold(DET_IMPACT_WINDOW_MS + 100, 1, 0, 0, 2);
    for (int i = 0; i < 3; i++) m.step(0, 0, 0, 0, false, Cmd::NONE, true);  // missed reads
    CHECK("missed reads don't abort analysis", m.d.state() == S::POST_IMPACT);
    CHECK("... and the fall is still found", m.hold(post, 1, 0, 0, 2) == S::ALERT_PENDING);
  }

  {
    Sim m;
    m.fallToImpact();
    CHECK("sensor fault after impact fails safe", m.step(0, 0, 0, 0, true) == S::ALERT_PENDING);
    CHECK("reason is SENSOR_LOST", m.d.reason() == Reason::SENSOR_LOST);
  }

  {
    Sim m;
    m.hold(1000, 0, 0, 1, 2);
    CHECK("sensor fault while idle -> SENSOR_FAULT", m.step(0, 0, 0, 0, true) == S::SENSOR_FAULT);
    CHECK("fault persists", m.step(0, 0, 0, 0, true) == S::SENSOR_FAULT);
    CHECK("valid data -> MONITORING", m.step(0, 0, 1, 2) == S::MONITORING);
  }

  {
    Sim m;
    m.hold(1000, 0, 0, 1, 2);
    m.step(0, 0, 0, 0, true);
    CHECK("SOS works even with sensor fault", m.step(0, 0, 0, 0, true, Cmd::SOS) == S::ALERT_CONFIRMED);
    CHECK("reason is MANUAL_SOS", m.d.reason() == Reason::MANUAL_SOS);
    m.step(0, 0, 0, 0, true, Cmd::CANCEL);
    S st = S::CANCELLED;
    for (uint32_t i = 0; i <= DET_REARM_MS / SAMPLE_PERIOD_MS; i++) st = m.step(0, 0, 0, 0, true);
    CHECK("re-arm with sensor still faulted -> SENSOR_FAULT", st == S::SENSOR_FAULT);
  }

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}

// Calls fn(sample, label) for every data row of a FallGuard CSV log
// (SD file or captured serial stream; '#' lines and junk are skipped).
template <class Fn>
bool forEachRow(const char* path, Fn fn) {
  std::ifstream in(path);
  if (!in) {
    fprintf(stderr, "cannot open %s\n", path);
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#' || line.rfind("timestamp", 0) == 0) continue;
    for (char& c : line)
      if (c == ',') c = ' ';
    std::istringstream f(line);
    Sample s{};
    std::string state, label = "none";
    unsigned long t;
    if (!(f >> t >> s.ax >> s.ay >> s.az >> s.gx >> s.gy >> s.gz >> s.accMag >> s.gyroMag >> state)) continue;
    f >> label;
    s.t_ms = t;
    fn(s, label);
  }
  return true;
}

// Replay a log with the firmware's current detector (rules, or the trained model).
int replay(const char* path) {
  FallDetector d;
  std::map<std::string, int> alertsByLabel, samplesByLabel;
  S last = S::MONITORING;
  long rows = 0;
  bool ok = forEachRow(path, [&](const Sample& s, const std::string& label) {
    rows++;
    samplesByLabel[label]++;
    S st = d.update(s.t_ms, &s, false, Cmd::NONE);
    if (st == last) return;
    printf("%10lu  %-15s -> %-15s label=%s", (unsigned long)s.t_ms, stateName(last), stateName(st), label.c_str());
    if (st == S::ALERT_PENDING) {
      alertsByLabel[label]++;
      printf("  peak=%.2fg min=%.2fg rot=%.0fdps still=%.0f%% orient=%.0fdeg", d.peakG(), d.minG(), d.peakDps(),
             d.stillFraction() * 100, d.orientChangeDeg());
    }
    printf("\n");
    last = st;
    // Alerts are never cancelled in a replay; auto-cancel so later events are still scored.
    if (st == S::ALERT_CONFIRMED || st == S::ALERT_PENDING) d.update(s.t_ms, &s, false, Cmd::CANCEL);
  });
  if (!ok) return 2;
  printf("\n%ld samples (%s)\n%-20s %10s %8s\n", rows, FALL_MODEL_TRAINED ? "ML model" : "rules", "label", "seconds",
         "alerts");
  for (auto& kv : samplesByLabel)
    printf("%-20s %10.1f %8d\n", kv.first.c_str(), kv.second * SAMPLE_PERIOD_MS / 1000.0, alertsByLabel[kv.first]);
  return 0;
}

// Feature table for ml/train.py. Rules are switched off and the trigger lowered
// to DET_ML_TRIGGER_G, so every impact candidate is analysed to the end and the
// model also learns from the events the rules would have rejected.
//   trial rows: one per contiguous run of the same label (duration for false alarms/hour)
//   event rows: one per candidate, tagged with the trial it started in
int features(int n, char** files) {
  DetectorConfig c;
  c.useModel = false;
  c.impactG = DET_ML_TRIGGER_G;
  c.minRotationDps = 0;
  c.minStillFraction = 0;
  c.minOrientChangeDeg = 0;
  c.rearmMs = 0;
  printf("row,file,trial,label,t_ms,duration_s,peak_g,min_g,peak_dps,still_frac,orient_deg\n");
  int trial = 0;
  for (int i = 0; i < n; i++) {
    FallDetector d(c);
    std::string runLabel, evLabel;
    uint32_t runStart = 0, lastT = 0, evT = 0;
    int evTrial = 0;
    auto closeRun = [&] {
      if (!runLabel.empty())
        printf("trial,%s,%d,%s,%lu,%.2f,,,,,\n", files[i], trial, runLabel.c_str(), (unsigned long)runStart,
               (lastT - runStart + SAMPLE_PERIOD_MS) / 1000.0);
    };
    bool ok = forEachRow(files[i], [&](const Sample& s, const std::string& label) {
      if (label != runLabel) {
        closeRun();
        runLabel = label;
        runStart = s.t_ms;
        trial++;
      }
      lastT = s.t_ms;
      S before = d.state();
      S st = d.update(s.t_ms, &s, false, Cmd::NONE);
      if (before == S::MONITORING && st != S::MONITORING) {
        evTrial = trial;
        evLabel = label;
        evT = s.t_ms;
      }
      if (st == S::ALERT_PENDING) {
        printf("event,%s,%d,%s,%lu,,%.3f,%.3f,%.1f,%.3f,%.1f\n", files[i], evTrial, evLabel.c_str(),
               (unsigned long)evT, d.peakG(), d.minG(), d.peakDps(), d.stillFraction(), d.orientChangeDeg());
        d.update(s.t_ms, &s, false, Cmd::CANCEL);
      }
    });
    if (!ok) return 2;
    closeRun();
  }
  return 0;
}

int main(int argc, char** argv) {
  if (argc > 2 && !strcmp(argv[1], "--features")) return features(argc - 2, argv + 2);
  return argc > 1 ? replay(argv[1]) : selfTest();
}

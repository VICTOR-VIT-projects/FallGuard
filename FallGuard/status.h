// Snapshot of system state, written by the sensor task and copied (under a
// spinlock in FallGuard.ino) by loop() for the display, BLE and serial output.
#pragma once
#include "imu.h"

struct Status {
  Sample s;
  bool sampleValid;
  FallState state;
  uint8_t countdown;
  uint32_t alerts;
  Reason reason;
  float peakG, peakDps, stillFrac, orientDeg;  // last analysed event

  bool imuOk;
  uint8_t whoAmI;
  CalSource cal;
  uint32_t imuErrors, saturations, missedTicks;
  const char* imuErr = "";  // printed with %s before the sensor task's first update: never null
  uint32_t uptimeMs;

  // Filled in by loop(), not the sensor task:
  bool sdOk;
  bool bleConnected;
};

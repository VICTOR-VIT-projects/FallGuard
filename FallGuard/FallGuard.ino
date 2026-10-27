// FallGuard: ESP32 wearable fall detector, offline-first.
//
// Two execution contexts:
//   sensorTask (core 1, priority 5, every 10 ms): IMU read -> buttons -> state
//       machine -> buzzer/LEDs -> log queue. Owns Wire, the detector and Io.
//   loop() (core 1, priority 1): SD writes, OLED, BLE, serial. Everything slow
//       or blocking lives here so it can never delay a sample.
// They share only `shared` (spinlock-protected copy) and the logger's queue.
#include <Arduino.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <atomic>
#include "ble.h"
#include "config.h"
#include "display.h"
#include "fall_detector.h"
#include "imu.h"
#include "io.h"
#include "logger.h"
#include "status.h"

Imu imu;
FallDetector detector;
CalSource calSource = CalSource::NONE;

portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
Status shared{};
std::atomic<uint8_t> serialCmd{0};
char bootInfo[64];

const char* resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_EXT: return "EXT";
    default: return "OTHER";
  }
}

void sensorTask(void* arg) {
  bool imuOk = arg != nullptr;
  uint32_t lastRecover = 0, lastVerify = 0, lastTick = millis(), missed = 0;

  // A hung sensor task (e.g. an I2C driver deadlock) reboots the device
  // instead of silently freezing the detector.
  esp_task_wdt_config_t wdt = {.timeout_ms = 3000, .idle_core_mask = 0, .trigger_panic = true};
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);
  esp_task_wdt_add(nullptr);

  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    esp_task_wdt_reset();
    const uint32_t now = millis();
    if (now - lastTick >= 2 * SAMPLE_PERIOD_MS) missed += (now - lastTick) / SAMPLE_PERIOD_MS - 1;
    lastTick = now;

    Sample s;
    bool got = false;
    if (imuOk) {
      got = imu.read(s, now);
      if (!got && imu.consecutiveErrors() >= IMU_MAX_CONSEC_ERRORS) imuOk = false;
      if (got && now - lastVerify >= IMU_VERIFY_INTERVAL_MS) {
        lastVerify = now;
        if (!imu.verifyConfig()) imuOk = got = false;
      }
    } else if (now - lastRecover >= IMU_RECOVER_INTERVAL_MS) {
      lastRecover = now;
      imuOk = imu.begin();  // ~150 ms; acceptable while already faulted
    }

    Cmd cmd = Io::pollButtons(now);
    if (cmd == Cmd::NONE) cmd = Ble::takeCommand();
    if (cmd == Cmd::NONE) cmd = (Cmd)serialCmd.exchange(0);

    const FallState st = detector.update(now, got ? &s : nullptr, !imuOk, cmd);
    Io::drive(st, now);
    if (got) Logger::push(s, st);

    portENTER_CRITICAL(&statusMux);
    if (got) shared.s = s;
    shared.sampleValid = imuOk;
    shared.state = st;
    shared.countdown = detector.countdownSec(now);
    shared.alerts = detector.alertCount();
    shared.reason = detector.reason();
    shared.peakG = detector.peakG();
    shared.peakDps = detector.peakDps();
    shared.stillFrac = detector.stillFraction();
    shared.orientDeg = detector.orientChangeDeg();
    shared.imuOk = imuOk;
    shared.whoAmI = imu.whoAmI();
    shared.cal = calSource;
    shared.imuErrors = imu.totalErrors();
    shared.saturations = imu.saturations();
    shared.missedTicks = missed;
    shared.imuErr = imu.lastError();
    shared.uptimeMs = now;
    portEXIT_CRITICAL(&statusMux);
  }
}

// Non-blocking line reader for USB serial commands.
void handleSerial() {
  static char line[40];
  static size_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (len < sizeof line - 1) line[len++] = c;
      continue;
    }
    if (!len) continue;
    line[len] = 0;
    len = 0;
    if (!strcmp(line, "c")) {
      serialCmd = (uint8_t)Cmd::CANCEL;
    } else if (!strcmp(line, "sos")) {
      serialCmd = (uint8_t)Cmd::SOS;
    } else if (!strcmp(line, "s")) {
      Logger::setSerialStream(!Logger::serialStream());
      if (Logger::serialStream())
        Serial.println("timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,accel_mag_g,gyro_mag_dps,state,label");
    } else if (!strncmp(line, "l ", 2)) {
      Logger::setLabel(line + 2);
      Serial.printf("# label=%s\n", line + 2);
    } else {
      Serial.println("# commands: c=cancel  sos=manual SOS  s=toggle CSV stream  l <label>=set label");
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  const char* rst = resetName(esp_reset_reason());
  Serial.printf("\n# FallGuard %s, last reset: %s\n", FW_VERSION, rst);

  Io::begin();
  Display::begin();
  char l3[32];
  snprintf(l3, sizeof l3, "last reset: %s", rst);
  Display::message("FallGuard", "v" FW_VERSION, l3);
  delay(800);

  bool imuOk = imu.begin();
  if (imuOk) {
    Serial.printf("# IMU WHO_AM_I=0x%02X\n", imu.whoAmI());
    for (int i = 1; i <= CAL_ATTEMPTS && calSource != CalSource::FRESH; i++) {
      Display::message("CALIBRATING", "Keep device still", i > 1 ? "(moved - retrying)" : "");
      CalResult r = imu.calibrate();
      if (r == CalResult::OK) {
        calSource = CalSource::FRESH;
        imu.saveBias();
      } else {
        Serial.printf("# calibration attempt %d: %s\n", i, imu.lastError());
        if (r == CalResult::IO_FAIL) break;
      }
    }
  } else {
    Serial.printf("# IMU init failed: %s (retrying in background)\n", imu.lastError());
    Display::message("IMU FAILED", imu.lastError(), "retrying...");
    delay(1500);
  }
  if (calSource != CalSource::FRESH && imu.loadBias()) calSource = CalSource::STORED;
  const float* b = imu.bias();
  Serial.printf("# gyro bias %.2f %.2f %.2f dps (%s)\n", b[0], b[1], b[2],
                calSource == CalSource::FRESH ? "fresh" : calSource == CalSource::STORED ? "from NVS" : "NONE");

  snprintf(bootInfo, sizeof bootInfo, "reset=%s who=0x%02X cal=%d", rst, imu.whoAmI(), (int)calSource);
  Logger::begin(bootInfo);
  Ble::begin();

  Display::message("READY", calSource == CalSource::NONE ? "WARNING: uncalibrated" : "monitoring", "");
  xTaskCreatePinnedToCore(sensorTask, "sensor", 4096, imuOk ? (void*)1 : nullptr, 5, nullptr, 1);
}

void loop() {
  static uint32_t lastDisp = 0, lastPrint = 0;
  static FallState lastState = FallState::MONITORING;
  const uint32_t now = millis();

  handleSerial();
  Logger::service(now);

  Status st;
  portENTER_CRITICAL(&statusMux);
  st = shared;
  portEXIT_CRITICAL(&statusMux);
  st.sdOk = Logger::ok();
  st.bleConnected = Ble::connected();

  // Diagnostic lines start with '#', so a captured serial stream is still a valid CSV.
  // (Every state is also in the per-sample log; this only shows what loop() observed.)
  if (st.state != lastState) {
    Serial.printf("# [EVENT] t=%lu %s -> %s", (unsigned long)now, stateName(lastState), stateName(st.state));
    if (st.state == FallState::ALERT_PENDING || st.state == FallState::ALERT_CONFIRMED)
      Serial.printf(" reason=%s peak=%.2fg rot=%.0fdps still=%.0f%% orient=%.0fdeg", reasonName(st.reason), st.peakG,
                    st.peakDps, st.stillFrac * 100, st.orientDeg);
    Serial.println();
    lastState = st.state;
  }

  Ble::publish(st, now);
  if (now - lastDisp >= DISPLAY_PERIOD_MS) {
    lastDisp = now;
    Display::render(st);
  }
  if (!Logger::serialStream() && now - lastPrint >= STATUS_PRINT_MS) {
    lastPrint = now;
    Serial.printf("# [STAT] %s a=%.2fg w=%.0fdps imu=%s err=%lu sat=%lu miss=%lu drop=%lu sd=%s ble=%d oled=%lums\n",
                  stateName(st.state), st.s.accMag, st.s.gyroMag, st.imuOk ? "OK" : st.imuErr,
                  (unsigned long)st.imuErrors, (unsigned long)st.saturations, (unsigned long)st.missedTicks,
                  (unsigned long)Logger::drops(), Logger::fileName(), st.bleConnected, (unsigned long)Display::lastRenderMs());
  }
  delay(1);
}

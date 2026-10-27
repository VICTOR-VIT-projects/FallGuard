#include "logger.h"
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace {
struct Record {
  Sample s;
  FallState st;
};

QueueHandle_t queue = nullptr;
portMUX_TYPE labelMux = portMUX_INITIALIZER_UNLOCKED;
char label[24] = "none";
const char* boot = "";

File file;
bool sdOk = false;
bool stream = false;
char name[16] = "";
uint32_t dropped = 0, lastFlush = 0, lastTry = 0;
char buf[4096];
size_t used = 0;

const char* CSV_HEADER =
    "timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,accel_mag_g,gyro_mag_dps,state,label\n";

void fail(const char* why) {
  Serial.printf("# [SD] %s - logging disabled, retry in %lu ms\n", why, (unsigned long)SD_RETRY_MS);
  if (file) file.close();
  SD.end();
  sdOk = false;
  used = 0;
}

bool writeOut() {
  if (!used) return true;
  bool good = file.write((const uint8_t*)buf, used) == used;
  used = 0;
  if (!good) fail("write failed");
  return good;
}

// Write a file, read it back, compare. Proves the card really works.
bool selfTest() {
  const char* path = "/fg_test.txt";
  const char* msg = "FallGuard SD test";
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  size_t n = f.print(msg);
  f.close();
  f = SD.open(path, FILE_READ);
  if (!f) return false;
  String back = f.readString();
  f.close();
  SD.remove(path);
  return n == strlen(msg) && back == msg;
}

void tryOpen() {
  lastTry = millis();
  if (!SD_ENABLED) return;
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, SPI, 4000000)) return fail("no card / init failed");
  if (!selfTest()) return fail("write/read self-test failed");

  // New file per boot (and per recovery), so a reset never corrupts old data.
  for (int i = 1; i < 10000; i++) {
    snprintf(name, sizeof name, "/FG_%04d.csv", i);
    if (!SD.exists(name)) break;
  }
  file = SD.open(name, FILE_WRITE);
  if (!file) return fail("cannot create log file");
  file.printf("# FallGuard fw=%s %s rate_hz=%lu\n", FW_VERSION, boot, (unsigned long)(1000 / SAMPLE_PERIOD_MS));
  file.printf("# thresholds (unvalidated): freefall=%.2fg impact=%.2fg rot=%.0fdps still_tol=%.2fg/%.0fdps orient=%.0fdeg\n",
              DET_FREEFALL_G, DET_IMPACT_G, DET_MIN_ROTATION_DPS, DET_STILL_ACC_TOL_G, DET_STILL_GYRO_DPS,
              DET_MIN_ORIENT_CHANGE_DEG);
  file.print(CSV_HEADER);
  file.flush();
  sdOk = true;
  Serial.printf("# [SD] logging to %s\n", name);
}
}  // namespace

namespace Logger {
void begin(const char* bootInfo) {
  boot = bootInfo;
  if (!queue) queue = xQueueCreate(LOG_QUEUE_LEN, sizeof(Record));
  tryOpen();
}

void push(const Sample& s, FallState st) {
  Record r{s, st};
  if (!queue || xQueueSend(queue, &r, 0) != pdTRUE) dropped++;
}

void service(uint32_t now) {
  if (!sdOk && now - lastTry >= SD_RETRY_MS) tryOpen();

  char lbl[sizeof label];
  portENTER_CRITICAL(&labelMux);
  memcpy(lbl, label, sizeof lbl);
  portEXIT_CRITICAL(&labelMux);

  Record r;
  char line[160];
  while (queue && xQueueReceive(queue, &r, 0) == pdTRUE) {
    if (!sdOk && !stream) continue;
    const Sample& s = r.s;
    int n = snprintf(line, sizeof line, "%lu,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.3f,%.1f,%s,%s\n",
                     (unsigned long)s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.accMag, s.gyroMag,
                     stateName(r.st), lbl);
    if (stream) Serial.write(line, n);
    if (sdOk) {
      if (used + n > sizeof buf && !writeOut()) continue;
      memcpy(buf + used, line, n);
      used += n;
    }
  }
  if (sdOk && now - lastFlush >= LOG_FLUSH_MS) {
    lastFlush = now;
    if (writeOut()) file.flush();
  }
}

void setLabel(const char* l) {
  char clean[sizeof label];
  size_t i = 0;
  for (; l[i] && i < sizeof clean - 1; i++) clean[i] = (l[i] == ',' || l[i] < ' ') ? '_' : l[i];
  clean[i] = 0;
  if (!i) strcpy(clean, "none");
  portENTER_CRITICAL(&labelMux);
  memcpy(label, clean, sizeof label);
  portEXIT_CRITICAL(&labelMux);
}

void setSerialStream(bool on) { stream = on; }
bool serialStream() { return stream; }
bool ok() { return sdOk; }
uint32_t drops() { return dropped; }
const char* fileName() { return sdOk ? name : "-"; }
}  // namespace Logger

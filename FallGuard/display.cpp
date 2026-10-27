#include "display.h"
#include <Arduino.h>
#include <U8g2lib.h>

namespace {
// Verified working constructor: SH1106, software I2C, clock=17, data=16.
U8G2_SH1106_128X64_NONAME_F_SW_I2C oled(U8G2_R0, PIN_OLED_SCL, PIN_OLED_SDA, U8X8_PIN_NONE);
uint32_t frameMs = 0;

const char* calName(CalSource c) {
  return c == CalSource::FRESH ? "CAL" : c == CalSource::STORED ? "CAL:NVS" : "UNCAL";
}

void header(const char* title, const Status& st) {
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawBox(0, 0, 128, 11);
  oled.setDrawColor(0);
  oled.drawStr(1, 9, title);
  char flags[8];
  snprintf(flags, sizeof flags, "%s%s", st.bleConnected ? "B " : "", st.sdOk ? "SD" : "");
  oled.drawStr(127 - oled.getStrWidth(flags), 9, flags);
  oled.setDrawColor(1);
}
}  // namespace

namespace Display {
void begin() {
  oled.begin();
  oled.setContrast(200);
}

void message(const char* l1, const char* l2, const char* l3) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_7x14B_tr);
  oled.drawStr(0, 16, l1);
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 36, l2);
  oled.drawStr(0, 50, l3);
  oled.sendBuffer();
}

void render(const Status& st) {
  char line[32];
  oled.clearBuffer();
  switch (st.state) {
    case FallState::ALERT_PENDING:
      header("FALL DETECTED?", st);
      oled.setFont(u8g2_font_logisoso24_tn);
      snprintf(line, sizeof line, "%u", st.countdown);
      oled.drawStr((128 - oled.getStrWidth(line)) / 2, 42, line);
      oled.setFont(u8g2_font_6x10_tf);
      oled.drawStr(0, 62, "Press CANCEL if OK");
      break;

    case FallState::ALERT_CONFIRMED:
      header("ALERT", st);
      oled.setFont(u8g2_font_helvB14_tr);
      oled.drawStr(4, 34, "NEED HELP");
      oled.setFont(u8g2_font_6x10_tf);
      oled.drawStr(0, 48, reasonName(st.reason));
      oled.drawStr(0, 62, "CANCEL to reset");
      break;

    case FallState::SENSOR_FAULT:
      header("SENSOR FAULT", st);
      oled.setFont(u8g2_font_6x10_tf);
      oled.drawStr(0, 26, st.imuErr);
      snprintf(line, sizeof line, "errors: %lu", (unsigned long)st.imuErrors);
      oled.drawStr(0, 40, line);
      oled.drawStr(0, 54, "NOT MONITORING");
      oled.drawStr(0, 64, "retrying...");
      break;

    default:
      header(stateName(st.state), st);
      oled.setFont(u8g2_font_6x10_tf);
      snprintf(line, sizeof line, "|a| %5.2f g%s", st.s.accMag, st.s.saturated ? " SAT" : "");
      oled.drawStr(0, 24, line);
      snprintf(line, sizeof line, "|w| %5.0f dps", st.s.gyroMag);
      oled.drawStr(0, 35, line);
      snprintf(line, sizeof line, "a %5.2f%6.2f%6.2f", st.s.ax, st.s.ay, st.s.az);
      oled.drawStr(0, 46, line);
      snprintf(line, sizeof line, "IMU %02X %s A:%lu", st.whoAmI, calName(st.cal), (unsigned long)st.alerts);
      oled.drawStr(0, 62, line);
      break;
  }
  uint32_t t0 = millis();
  oled.sendBuffer();
  frameMs = millis() - t0;
}

uint32_t lastRenderMs() { return frameMs; }
}  // namespace Display

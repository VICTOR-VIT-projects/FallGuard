// SH1106 128x64 OLED on software I2C (GPIO17 SCL, GPIO16 SDA) via U8g2.
// Do not switch to an SSD1306 driver: it produced artifacts on this panel.
// Rendering is slow (bit-banged), so it runs only from loop(), never the sensor task.
#pragma once
#include "status.h"

namespace Display {
void begin();
void message(const char* l1, const char* l2 = "", const char* l3 = "");
void render(const Status& st);
uint32_t lastRenderMs();  // measured frame time, for the serial status line
}

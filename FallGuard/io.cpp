#include "io.h"
#include <Arduino.h>

namespace {
struct Button {
  int pin;
  bool raw = false, pressed = false, held = false;
  uint32_t rawSince = 0, pressedSince = 0;

  // Returns true once per press ("edge"); held-time is available via pressedSince.
  bool update(uint32_t now) {
    bool r = digitalRead(pin) == LOW;  // pull-up, button to GND
    if (r != raw) {
      raw = r;
      rawSince = now;
    }
    if (raw != pressed && now - rawSince >= BUTTON_DEBOUNCE_MS) {
      pressed = raw;
      pressedSince = now;
      held = false;
      return pressed;
    }
    return false;
  }
};

Button cancelBtn{PIN_BTN_CANCEL}, sosBtn{PIN_BTN_SOS};

// true for `on` ms out of every `period` ms
bool blink(uint32_t now, uint32_t period, uint32_t on) { return now % period < on; }
}  // namespace

namespace Io {
void begin() {
  pinMode(PIN_BTN_CANCEL, INPUT_PULLUP);
  pinMode(PIN_BTN_SOS, INPUT_PULLUP);
  // Passive buzzer: needs an AC square wave, not DC. LEDC generates it in
  // hardware, so the sensor task only flips the duty cycle on/off.
  ledcAttach(PIN_BUZZER, BUZZER_FREQ_HZ, 8);
  ledcWrite(PIN_BUZZER, 0);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
}

Cmd pollButtons(uint32_t now) {
  bool cancel = cancelBtn.update(now);
  sosBtn.update(now);
  if (sosBtn.pressed && !sosBtn.held && now - sosBtn.pressedSince >= SOS_HOLD_MS) {
    sosBtn.held = true;
    return Cmd::SOS;
  }
  return cancel ? Cmd::CANCEL : Cmd::NONE;
}

void drive(FallState st, uint32_t now) {
  bool green = false, red = false, buzz = false;
  switch (st) {
    case FallState::ALERT_PENDING:
      red = blink(now, 500, 250);
      buzz = blink(now, 1000, 200);  // short beeps: "cancel me if OK"
      break;
    case FallState::ALERT_CONFIRMED:
      red = true;
      buzz = blink(now, 400, 200) && blink(now, 2000, 1200);  // urgent triple-beep bursts
      break;
    case FallState::SENSOR_FAULT:
      red = blink(now, 1000, 100);
      buzz = blink(now, 10000, 60);  // a fault must never look like normal operation
      break;
    case FallState::CANCELLED:
      green = true;
      break;
    default:  // MONITORING and analysis states look identical to the wearer
      green = blink(now, 2000, 60);  // heartbeat: firmware alive
      break;
  }
  digitalWrite(PIN_LED_GREEN, green);
  digitalWrite(PIN_LED_RED, red);
  static bool buzzing = false;
  if (buzz != buzzing) {  // only touch LEDC on changes, not every 10 ms
    buzzing = buzz;
    ledcWrite(PIN_BUZZER, buzz ? 128 : 0);  // 50% duty = loudest square wave
  }
}
}  // namespace Io

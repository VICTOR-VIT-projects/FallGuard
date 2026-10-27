// Local, Wi-Fi-independent alert hardware: buttons, buzzer, LEDs.
// Called only from the sensor task, so no locking is needed.
#pragma once
#include "fall_detector.h"

namespace Io {
void begin();
Cmd pollButtons(uint32_t now);       // debounced; SOS needs a SOS_HOLD_MS hold
void drive(FallState st, uint32_t now);  // buzzer + LED patterns per state
}

// Bluetooth LE link to the FallGuard companion app (app/index.html).
// Optional: the device alerts locally whether or not a phone is connected.
//
// GATT service fa110001-7c4e-4b8a-9e2f-3d6a5b1c0e01
//   fa110002-...  STATUS   read/notify, 20-byte BlePacket (below), 5 Hz + on state change
//   fa110003-...  COMMAND  write: 0x01 cancel | 0x02 SOS | 0x03 <ascii label>
//
// No pairing/encryption: anyone in range can connect (see README limitations).
#pragma once
#include "status.h"

struct __attribute__((packed)) BlePacket {  // little-endian, must stay <= 20 bytes (default MTU)
  uint8_t version;    // 1
  uint8_t state;      // FallState
  uint8_t flags;      // b0 imuOk, b1 sdOk, b2 calibrated, b3 saturated, b4-5 Reason
  uint8_t countdown;  // seconds left in ALERT_PENDING
  uint16_t accMg;     // |a| in milli-g
  uint16_t gyroDps;   // |w| in dps
  uint16_t peakMg;    // last event: peak |a|
  uint16_t peakDps;   // last event: peak |w|
  uint8_t stillPct;   // last event: % still samples
  uint8_t orientDeg;  // last event: orientation change
  uint16_t alerts;    // since boot
  uint32_t uptimeMs;
};
static_assert(sizeof(BlePacket) == 20, "BlePacket must fit a default-MTU notification");

namespace Ble {
void begin();
void publish(const Status& st, uint32_t now);  // loop()
bool connected();
Cmd takeCommand();  // sensor task; returns and clears the last app command
}

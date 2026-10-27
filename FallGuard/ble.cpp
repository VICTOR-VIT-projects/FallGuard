#include "ble.h"
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <atomic>
#include "logger.h"

namespace {
const char* SVC_UUID = "fa110001-7c4e-4b8a-9e2f-3d6a5b1c0e01";
const char* STATUS_UUID = "fa110002-7c4e-4b8a-9e2f-3d6a5b1c0e01";
const char* CMD_UUID = "fa110003-7c4e-4b8a-9e2f-3d6a5b1c0e01";

BLECharacteristic* statusChr = nullptr;
std::atomic<bool> isConnected{false};
std::atomic<uint8_t> pendingCmd{0};
uint32_t lastSend = 0;
FallState lastState = FallState::MONITORING;

uint16_t clamp16(float v) { return v <= 0 ? 0 : v >= 65535 ? 65535 : (uint16_t)(v + 0.5f); }

// BLE callbacks run in the Bluetooth task: only touch atomics / locked state here.
class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { isConnected = true; }
  void onDisconnect(BLEServer* s) override {
    isConnected = false;
    s->startAdvertising();
  }
};

class CmdCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    if (v.isEmpty()) return;
    switch ((uint8_t)v[0]) {
      case 0x01: pendingCmd = (uint8_t)Cmd::CANCEL; break;
      case 0x02: pendingCmd = (uint8_t)Cmd::SOS; break;
      case 0x03: Logger::setLabel(v.c_str() + 1); break;
    }
  }
};
}  // namespace

namespace Ble {
void begin() {
  BLEDevice::init("FallGuard");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCb());
  BLEService* svc = server->createService(SVC_UUID);

  statusChr = svc->createCharacteristic(STATUS_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  statusChr->addDescriptor(new BLE2902());  // CCCD; NimBLE builds ignore this
  BLECharacteristic* cmd = svc->createCharacteristic(CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  cmd->setCallbacks(new CmdCb());
  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);  // name goes in the scan response; the 128-bit UUID fills the advert
  BLEDevice::startAdvertising();
}

void publish(const Status& st, uint32_t now) {
  if (!statusChr) return;
  bool changed = st.state != lastState;
  if (!changed && now - lastSend < BLE_PERIOD_MS) return;
  lastSend = now;
  lastState = st.state;

  BlePacket p{};
  p.version = 1;
  p.state = (uint8_t)st.state;
  p.flags = (st.imuOk ? 1 : 0) | (st.sdOk ? 2 : 0) | (st.cal != CalSource::NONE ? 4 : 0) |
            (st.s.saturated ? 8 : 0) | ((uint8_t)st.reason & 3) << 4;
  p.countdown = st.countdown;
  p.accMg = st.sampleValid ? clamp16(st.s.accMag * 1000) : 0;
  p.gyroDps = st.sampleValid ? clamp16(st.s.gyroMag) : 0;
  p.peakMg = clamp16(st.peakG * 1000);
  p.peakDps = clamp16(st.peakDps);
  p.stillPct = (uint8_t)(st.stillFrac * 100 + 0.5f);
  p.orientDeg = (uint8_t)(st.orientDeg + 0.5f);
  p.alerts = st.alerts > 65535 ? 65535 : st.alerts;
  p.uptimeMs = st.uptimeMs;

  statusChr->setValue((uint8_t*)&p, sizeof p);
  if (isConnected) statusChr->notify();
}

bool connected() { return isConnected; }

Cmd takeCommand() { return (Cmd)pendingCmd.exchange(0); }
}  // namespace Ble

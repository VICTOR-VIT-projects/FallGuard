// FallGuard configuration: pins, sensor setup, timing and detector thresholds.
// Pure constants (no Arduino includes) so the host-side test can use it too.
#pragma once
#include <stdint.h>

#define FW_VERSION "2.0.0"

// ---- Pins ------------------------------------------------------------------
// Verified on hardware:
constexpr int PIN_IMU_SDA = 21;   // hardware I2C (Wire)
constexpr int PIN_IMU_SCL = 22;
constexpr int PIN_OLED_SDA = 16;  // software I2C (U8g2)
constexpr int PIN_OLED_SCL = 17;
// Planned, NOT yet hardware-verified:
constexpr bool SD_ENABLED = false;  // no SD module in the current kit: record over USB serial (ml/record.py)
constexpr int PIN_SD_CS = 5;      // boot-strap pin; move to 14 if the SD module breaks boot
constexpr int PIN_SD_SCK = 18;
constexpr int PIN_SD_MISO = 19;
constexpr int PIN_SD_MOSI = 23;
constexpr int PIN_BUZZER = 13;    // PASSIVE buzzer, driven with a PWM square wave
constexpr uint32_t BUZZER_FREQ_HZ = 2700;  // try 2000-4000: pick the loudest for your buzzer
constexpr int PIN_BTN_CANCEL = 25;  // to GND, internal pull-up
constexpr int PIN_BTN_SOS = 26;     // to GND, internal pull-up
constexpr int PIN_LED_GREEN = 32;
constexpr int PIN_LED_RED = 33;

// ---- IMU (MPU6500-compatible, WHO_AM_I = 0x70) ------------------------------
constexpr uint8_t IMU_ADDR = 0x68;
// 100 kHz / 50 ms = Arduino defaults, as in the working V1 sketch. On the bench,
// 400 kHz / 5 ms gave regular read errors and an I2C-driver hang within ~3 s.
constexpr uint32_t IMU_I2C_HZ = 100000;
constexpr uint16_t IMU_I2C_TIMEOUT_MS = 50;
// ACCEL_CONFIG 0x10 = +-8 g -> 4096 LSB/g (same as V1).
constexpr uint8_t IMU_ACCEL_CFG = 0x10;
constexpr float IMU_ACCEL_LSB_PER_G = 4096.0f;
// GYRO_CONFIG 0x10 = +-1000 dps -> 32.8 LSB/dps.
// V1 used 0x00 (+-250 dps, 131 LSB/dps); bench readings of ~240 dps were at the
// clipping limit, and fall rotations routinely exceed 250 dps.
constexpr uint8_t IMU_GYRO_CFG = 0x10;
constexpr float IMU_GYRO_LSB_PER_DPS = 32.8f;
// DLPF_CFG / A_DLPF_CFG = 3: ~41-45 Hz bandwidth, anti-aliasing for 100 Hz polling.
constexpr uint8_t IMU_DLPF_CFG = 3;

constexpr uint8_t IMU_MAX_CONSEC_ERRORS = 5;     // 50 ms of failed reads -> SENSOR_FAULT
constexpr uint8_t IMU_MAX_FROZEN_FRAMES = 25;    // identical raw frames -> device asleep/stuck
constexpr uint32_t IMU_RECOVER_INTERVAL_MS = 500;
constexpr uint32_t IMU_VERIFY_INTERVAL_MS = 1000;  // re-check config regs (catches silent IMU reset)

// Calibration (device must be still)
constexpr int CAL_SAMPLES = 200;
constexpr int CAL_MIN_VALID = 180;
constexpr uint32_t CAL_INTERVAL_MS = 10;
constexpr int CAL_ATTEMPTS = 3;
constexpr float CAL_MAX_GYRO_STD_DPS = 1.5f;   // more = device moved during calibration
constexpr float CAL_MAX_GYRO_BIAS_DPS = 20.0f;
constexpr float CAL_MAX_ACC_DEV_G = 0.10f;     // |mean |a| - 1 g|
constexpr float CAL_MAX_ACC_STD_G = 0.03f;

// ---- Timing ------------------------------------------------------------------
constexpr uint32_t SAMPLE_PERIOD_MS = 10;      // 100 Hz
constexpr uint32_t DISPLAY_PERIOD_MS = 200;    // 5 Hz
constexpr uint32_t BLE_PERIOD_MS = 200;        // 5 Hz status notifications
constexpr uint32_t STATUS_PRINT_MS = 1000;
constexpr uint32_t LOG_FLUSH_MS = 1000;        // max data lost on power cut
constexpr uint32_t SD_RETRY_MS = 5000;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t SOS_HOLD_MS = 1000;         // avoids SOS from accidental bumps
constexpr int LOG_QUEUE_LEN = 200;             // 2 s of samples buffered against SD stalls

constexpr uint32_t SERIAL_BAUD = 115200;

// ---- Fall detector defaults ------------------------------------------------------
// ALL VALUES BELOW ARE UNVALIDATED STARTING POINTS. Tune them on recorded trials
// (see README "Evaluation") and never report accuracy from the tuning set.
// Assumed placement: waist or chest. Wrist placement needs different values.
constexpr float DET_FREEFALL_G = 0.6f;          // |a| below this starts a candidate event
constexpr float DET_IMPACT_G = 2.5f;            // |a| above this = impact (V1 demo threshold)
constexpr uint32_t DET_FREEFALL_TO_IMPACT_MS = 800;
constexpr uint32_t DET_IMPACT_WINDOW_MS = 500;  // peaks collected after the impact
constexpr float DET_MIN_ROTATION_DPS = 90.0f;   // peak |w| over the event; rejects jumps/stomps
constexpr uint32_t DET_SETTLE_MS = 1000;        // ignore bounce right after impact
constexpr uint32_t DET_STILL_WINDOW_MS = 2500;  // then measure inactivity for this long
constexpr float DET_STILL_ACC_TOL_G = 0.30f;    // still if ||a|-1g| <= tol ...
constexpr float DET_STILL_GYRO_DPS = 40.0f;     // ... and |w| <= this
constexpr float DET_MIN_STILL_FRACTION = 0.70f;
constexpr float DET_MIN_ORIENT_CHANGE_DEG = 50.0f;  // upright -> lying; 0 disables the check
constexpr float DET_GRAVITY_ALPHA = 0.02f;      // pre-event gravity low-pass (~0.5 s at 100 Hz)
constexpr uint32_t DET_CANCEL_WINDOW_MS = 10000;
constexpr uint32_t DET_REARM_MS = 3000;

// ML mode (active once ml/train.py has written a trained fall_model.h).
// A lower trigger so soft falls still become candidates; the model then
// decides instead of the still/orientation/rotation rules above.
// Training data is extracted with this same value, so change both together
// (re-run ml/train.py after changing it).
constexpr float DET_ML_TRIGGER_G = 2.0f;
constexpr uint32_t DET_PRE_MIN_BUCKET_MS = 500;  // min |a| feature looks back 0.5-1 s before the event

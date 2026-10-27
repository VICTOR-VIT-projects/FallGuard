// CSV logging of every sample to microSD (and optionally USB serial).
// UNTESTED ON HARDWARE: the SD module has not passed a write/read test yet.
// The sensor task only enqueues (never blocks); loop() does the slow SD I/O.
// If the card is missing or fails, logging stops, the rest of the system keeps
// running, and initialisation is retried every SD_RETRY_MS.
#pragma once
#include "fall_detector.h"

namespace Logger {
void begin(const char* bootInfo);           // creates the queue, tries the SD card
void push(const Sample& s, FallState st);   // sensor task; drops (and counts) if full
void service(uint32_t now);                 // loop(): drain queue, write, flush, retry
void setLabel(const char* label);           // dataset label column, any task
void setSerialStream(bool on);              // also print CSV rows to Serial
bool serialStream();
bool ok();
uint32_t drops();
const char* fileName();
}

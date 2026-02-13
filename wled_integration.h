#ifndef WLED_INTEGRATION_H
#define WLED_INTEGRATION_H

#include <Arduino.h>

// Set WLED effect based on race state ("idle", "armed", "racing", "finished")
// No-op if WLED host is not configured
void setWLEDState(const char* raceState);

// Turn WLED off (for auto-sleep timer)
void setWLEDOff();

// Call from main loop() to auto-sleep WLED after inactivity
void checkWLEDTimeout();

#endif

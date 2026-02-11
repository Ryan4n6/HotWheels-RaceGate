#ifndef SPEED_TRAP_H
#define SPEED_TRAP_H

#include <Arduino.h>
#include "espnow_comm.h"

// Speed trap timing data
extern volatile uint64_t speedTrapTime1;
extern volatile uint64_t speedTrapTime2;
extern double lastTrapSpeed_mps;  // Most recent measured speed (m/s)

// Setup: init both IR sensor pins, attach ISRs
void speedTrapSetup();

// Main loop: detect beam breaks, calculate speed, send to finish gate
void speedTrapLoop();

// ESP-NOW message handler for speed trap role
void onSpeedTrapESPNow(const ESPMessage& msg, uint64_t receiveTime);

#endif

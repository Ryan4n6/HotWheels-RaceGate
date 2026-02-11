#ifndef START_GATE_H
#define START_GATE_H

#include <Arduino.h>
#include "espnow_comm.h"

void startGateSetup();
void startGateLoop();
void onStartGateESPNow(const ESPMessage& msg, uint64_t receiveTime);

#endif

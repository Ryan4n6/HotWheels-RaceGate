#include "speed_trap.h"
#include "config.h"

// Forward declaration from web_server (if this device has web server running)
extern void broadcastState();

// ============================================================================
// TIMING VARIABLES
// ============================================================================
volatile uint64_t speedTrapTime1 = 0;  // First sensor beam break timestamp
volatile uint64_t speedTrapTime2 = 0;  // Second sensor beam break timestamp
double lastTrapSpeed_mps = 0;

static unsigned long lastPingTime = 0;
static bool measurementPending = false;
static unsigned long measurementStarted = 0;

// ============================================================================
// ISRs - Capture precise timestamps when each beam is broken
// ============================================================================
void IRAM_ATTR speedTrapISR_1() {
  if (speedTrapTime1 == 0) {
    speedTrapTime1 = esp_timer_get_time();
  }
}

void IRAM_ATTR speedTrapISR_2() {
  if (speedTrapTime1 > 0 && speedTrapTime2 == 0) {
    speedTrapTime2 = esp_timer_get_time();
  }
}

// ============================================================================
// SETUP
// ============================================================================
void speedTrapSetup() {
  pinMode(cfg.sensor_pin, INPUT_PULLUP);
  pinMode(cfg.sensor_pin_2, INPUT_PULLUP);

  // Attach interrupts on both sensors
  attachInterrupt(digitalPinToInterrupt(cfg.sensor_pin), speedTrapISR_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(cfg.sensor_pin_2), speedTrapISR_2, FALLING);

  // Status LED
  pinMode(cfg.led_pin, OUTPUT);

  LOG.printf("[SPEEDTRAP] Setup complete. Sensor1=GPIO%d, Sensor2=GPIO%d, Spacing=%.3fm\n",
                cfg.sensor_pin, cfg.sensor_pin_2, cfg.sensor_spacing_m);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void speedTrapLoop() {
  // Heartbeat LED
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    digitalWrite(cfg.led_pin, !digitalRead(cfg.led_pin));
    lastBlink = millis();
  }

  // Check peer connectivity timeout
  if (peerConnected && millis() - lastPeerSeen > 10000) {
    peerConnected = false;
    LOG.println("[SPEEDTRAP] Peer disconnected");
  }

  // Ping peer
  unsigned long pingInterval = peerConnected ? 2000 : 10000;
  if (millis() - lastPingTime > pingInterval) {
    sendToPeer(MSG_PING, nowUs(), 0);
    lastPingTime = millis();
  }

  // Check if both sensors have been triggered
  if (speedTrapTime1 > 0 && speedTrapTime2 > 0) {
    // Calculate velocity
    int64_t elapsed_us = (int64_t)speedTrapTime2 - (int64_t)speedTrapTime1;

    if (elapsed_us > 0 && elapsed_us < 10000000LL) { // Sanity: < 10 seconds
      double elapsed_s = elapsed_us / 1000000.0;
      lastTrapSpeed_mps = cfg.sensor_spacing_m / elapsed_s;

      LOG.printf("[SPEEDTRAP] ===== SPEED MEASUREMENT =====\n");
      LOG.printf("[SPEEDTRAP] Elapsed: %lld us (%.4f s)\n", elapsed_us, elapsed_s);
      LOG.printf("[SPEEDTRAP] Speed: %.3f m/s (%.1f mph)\n",
                    lastTrapSpeed_mps, lastTrapSpeed_mps * 2.23694);
      LOG.printf("[SPEEDTRAP] =============================\n");

      // Send speed data to finish gate
      // We repurpose the ESPMessage: timestamp = measurement time, offset encodes speed
      // Speed is sent as microseconds-per-meter * 1000 (integer encoding for the int64 field)
      // The finish gate will decode: speed_mps = 1000000.0 / (offset / 1000.0)
      // Alternatively, we can just use the offset field to send speed * 10000 as an integer
      int64_t speed_encoded = (int64_t)(lastTrapSpeed_mps * 10000.0);
      sendToPeer(MSG_SPEED_DATA, speedTrapTime1, speed_encoded);

      // Flash LED rapidly for 1 second to indicate measurement
      for (int i = 0; i < 10; i++) {
        digitalWrite(cfg.led_pin, i % 2);
        delay(50);
      }
    } else {
      LOG.printf("[SPEEDTRAP] BAD TIMING: elapsed=%lld us\n", elapsed_us);
    }

    // Reset for next measurement
    speedTrapTime1 = 0;
    speedTrapTime2 = 0;
  }

  // Timeout: if first sensor triggered but second hasn't after 5 seconds, reset
  if (speedTrapTime1 > 0 && speedTrapTime2 == 0) {
    uint64_t now = esp_timer_get_time();
    if ((int64_t)(now - speedTrapTime1) > 5000000LL) {
      LOG.println("[SPEEDTRAP] Measurement timeout — resetting");
      speedTrapTime1 = 0;
      speedTrapTime2 = 0;
    }
  }
}

// ============================================================================
// ESP-NOW MESSAGE HANDLER
// ============================================================================
void onSpeedTrapESPNow(const ESPMessage& msg, uint64_t receiveTime) {
  switch (msg.type) {
    case MSG_PING:
      sendToPeer(MSG_PONG, nowUs(), 0);
      break;

    case MSG_SPEED_ACK:
      LOG.println("[SPEEDTRAP] Finish gate acknowledged speed data");
      break;

    case MSG_ARM_CMD:
      // Reset sensors when system is armed for new race
      speedTrapTime1 = 0;
      speedTrapTime2 = 0;
      lastTrapSpeed_mps = 0;
      LOG.println("[SPEEDTRAP] Armed — sensors reset");
      break;

    case MSG_DISARM_CMD:
      speedTrapTime1 = 0;
      speedTrapTime2 = 0;
      LOG.println("[SPEEDTRAP] Disarmed");
      break;
  }
}

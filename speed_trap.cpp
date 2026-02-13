#include "speed_trap.h"
#include "config.h"

// ============================================================================
// TIMING VARIABLES
// ============================================================================
static portMUX_TYPE speedMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint64_t speedTrapTime1 = 0;  // First sensor beam break timestamp
volatile uint64_t speedTrapTime2 = 0;  // Second sensor beam break timestamp
double lastTrapSpeed_mps = 0;

static unsigned long lastPingTime = 0;
static bool measurementPending = false;
static unsigned long measurementStarted = 0;

// Non-blocking LED flash (replaces blocking delay loop)
static unsigned long flashStartTime = 0;
static bool isFlashing = false;

// ============================================================================
// ISRs - Capture precise timestamps when each beam is broken
// ============================================================================
void IRAM_ATTR speedTrapISR_1() {
  portENTER_CRITICAL_ISR(&speedMux);
  if (speedTrapTime1 == 0) {
    speedTrapTime1 = esp_timer_get_time();
  }
  portEXIT_CRITICAL_ISR(&speedMux);
}

void IRAM_ATTR speedTrapISR_2() {
  portENTER_CRITICAL_ISR(&speedMux);
  if (speedTrapTime1 > 0 && speedTrapTime2 == 0) {
    speedTrapTime2 = esp_timer_get_time();
  }
  portEXIT_CRITICAL_ISR(&speedMux);
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
  // Heartbeat LED (only when not flashing measurement indicator)
  if (!isFlashing) {
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 1000) {
      digitalWrite(cfg.led_pin, !digitalRead(cfg.led_pin));
      lastBlink = millis();
    }
  }

  // Check peer connectivity timeout
  if (peerConnected && millis() - lastPeerSeen > PING_BACKOFF_MS) {
    peerConnected = false;
    LOG.println("[SPEEDTRAP] Peer disconnected");
  }

  // Ping peer
  unsigned long pingInterval = peerConnected ? PING_INTERVAL_MS : PING_BACKOFF_MS;
  if (millis() - lastPingTime > pingInterval) {
    sendToPeer(MSG_PING, nowUs(), 0);
    lastPingTime = millis();
  }

  // Atomic snapshot of 64-bit timing vars (shared with ISRs)
  uint64_t safeTime1, safeTime2;
  portENTER_CRITICAL(&speedMux);
  safeTime1 = speedTrapTime1;
  safeTime2 = speedTrapTime2;
  portEXIT_CRITICAL(&speedMux);

  // Check if both sensors have been triggered
  if (safeTime1 > 0 && safeTime2 > 0) {
    // Calculate velocity
    int64_t elapsed_us = (int64_t)safeTime2 - (int64_t)safeTime1;

    if (elapsed_us > 0 && elapsed_us < MAX_TRAP_DURATION_US) { // Sanity: < 10 seconds
      double elapsed_s = elapsed_us / 1000000.0;
      lastTrapSpeed_mps = cfg.sensor_spacing_m / elapsed_s;

      LOG.printf("[SPEEDTRAP] ===== SPEED MEASUREMENT =====\n");
      LOG.printf("[SPEEDTRAP] Elapsed: %lld us (%.4f s)\n", elapsed_us, elapsed_s);
      LOG.printf("[SPEEDTRAP] Speed: %.3f m/s (%.1f mph)\n",
                    lastTrapSpeed_mps, lastTrapSpeed_mps * MPS_TO_MPH);
      LOG.printf("[SPEEDTRAP] =============================\n");

      // Send speed data to finish gate
      // Offset field encodes speed as speed_mps * 10000 (fixed-point int64)
      int64_t speed_encoded = (int64_t)(lastTrapSpeed_mps * SPEED_FIXED_POINT_SCALE);
      sendToPeer(MSG_SPEED_DATA, safeTime1, speed_encoded);

      // Start non-blocking LED flash to indicate measurement
      isFlashing = true;
      flashStartTime = millis();
    } else {
      LOG.printf("[SPEEDTRAP] BAD TIMING: elapsed=%lld us\n", elapsed_us);
    }

    // Reset for next measurement
    portENTER_CRITICAL(&speedMux);
    speedTrapTime1 = 0;
    speedTrapTime2 = 0;
    portEXIT_CRITICAL(&speedMux);
  }

  // Timeout: if first sensor triggered but second hasn't after 5 seconds, reset
  if (safeTime1 > 0 && safeTime2 == 0) {
    uint64_t now = esp_timer_get_time();
    if ((int64_t)(now - safeTime1) > TRAP_SENSOR_TIMEOUT_US) {
      LOG.println("[SPEEDTRAP] Measurement timeout — resetting");
      portENTER_CRITICAL(&speedMux);
      speedTrapTime1 = 0;
      speedTrapTime2 = 0;
      portEXIT_CRITICAL(&speedMux);
    }
  }

  // Non-blocking LED flash handler (500ms rapid blink)
  if (isFlashing) {
    unsigned long elapsed = millis() - flashStartTime;
    if (elapsed < 500) {
      digitalWrite(cfg.led_pin, (elapsed / 50) % 2);
    } else {
      isFlashing = false;
      digitalWrite(cfg.led_pin, LOW);
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
      portENTER_CRITICAL(&speedMux);
      speedTrapTime1 = 0;
      speedTrapTime2 = 0;
      portEXIT_CRITICAL(&speedMux);
      lastTrapSpeed_mps = 0;
      LOG.println("[SPEEDTRAP] Armed — sensors reset");
      break;

    case MSG_DISARM_CMD:
      portENTER_CRITICAL(&speedMux);
      speedTrapTime1 = 0;
      speedTrapTime2 = 0;
      portEXIT_CRITICAL(&speedMux);
      LOG.println("[SPEEDTRAP] Disarmed");
      break;
  }
}

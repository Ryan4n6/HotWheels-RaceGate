#include "start_gate.h"
#include "config.h"
#include "audio_manager.h"
#include "lidar_sensor.h"
// NOTE: Start gate does NOT include wled_integration.h
// Only the finish gate controls WLED to avoid HTTP conflicts.

// Trigger detection
static volatile bool triggerDetected = false;
static volatile uint64_t triggerTime_us = 0;

// Timing
static unsigned long lastPingTime = 0;
static unsigned long triggeredTime = 0;
static unsigned long finishedAt = 0;
static bool waitingToReset = false;

// ============================================================================
// START TRIGGER INTERRUPT
// ============================================================================
void IRAM_ATTR startTriggerISR() {
  if (!triggerDetected) {
    triggerTime_us = esp_timer_get_time();
    triggerDetected = true;
  }
}

// ============================================================================
// SETUP
// ============================================================================
void startGateSetup() {
  pinMode(cfg.sensor_pin, INPUT_PULLUP);
  pinMode(cfg.led_pin, OUTPUT);
  // Don't attach interrupt yet - only when ARMED
  LOG.printf("[START] Setup complete. Trigger=GPIO%d, LED=GPIO%d\n",
                cfg.sensor_pin, cfg.led_pin);
}

// ============================================================================
// LED breathing effect for idle state
// ============================================================================
static void breatheLED() {
  // Simple on/off breathing using millis
  int brightness = (millis() / 10) % 512;
  if (brightness > 255) brightness = 511 - brightness;
  analogWrite(cfg.led_pin, brightness);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void startGateLoop() {
  // Check peer connectivity timeout
  if (peerConnected && millis() - lastPeerSeen > 10000) {
    peerConnected = false;
    LOG.println("[START] Peer disconnected - reducing ping rate");
  }

  // Ping finish gate - back off to 10s when disconnected
  unsigned long pingInterval = peerConnected ? 2000 : 10000;
  if (millis() - lastPingTime > pingInterval) {
    sendToPeer(MSG_PING, nowUs(), 0);
    lastPingTime = millis();
  }

  // NOTE: The FINISH gate owns clock sync (it initiates SYNC_REQ every 10s).
  // The start gate just responds to SYNC_REQ with MSG_OFFSET.

  // Non-blocking reset after FINISHED state
  if (waitingToReset && millis() - finishedAt > 2000) {
    waitingToReset = false;
    raceState = IDLE;
    LOG.println("[START] Auto-reset to IDLE");
  }

  switch (raceState) {
    case IDLE:
      breatheLED();
      // LiDAR auto-arm: if car has been staged for >1 second, auto-arm
      if (lidarAutoArmReady()) {
        raceState = ARMED;
        triggerDetected = false;
        triggerTime_us = 0;
        attachInterrupt(digitalPinToInterrupt(cfg.sensor_pin), startTriggerISR, FALLING);
        sendToPeer(MSG_ARM_CMD, nowUs(), 0);
        playSound("armed.wav");
        LOG.println("[START] AUTO-ARMED via LiDAR sensor");
      }
      break;

    case ARMED:
      // Solid LED when armed
      digitalWrite(cfg.led_pin, HIGH);

      if (triggerDetected) {
        // Beam broken - race starts!
        raceState = RACING;
        triggerDetected = false;
        triggeredTime = millis();

        // Send START with our LOCAL precise timestamp to finish gate.
        // The finish gate will convert this to its timebase using clockOffset.
        LOG.printf("[START] TRIGGERED at %llu us\n", triggerTime_us);
        sendToPeer(MSG_START, triggerTime_us, 0);

        // Play "go" sound on start gate speaker
        playSound("go.wav");

        // Detach interrupt to prevent re-trigger
        detachInterrupt(digitalPinToInterrupt(cfg.sensor_pin));

        LOG.println("[START] Race started.");
      }
      break;

    case RACING:
      // Flash LED rapidly while racing
      digitalWrite(cfg.led_pin, (millis() / 100) % 2);

      // Timeout: if no CONFIRM after 30 seconds, reset
      if (millis() - triggeredTime > 30000) {
        LOG.println("[START] Race timeout - no finish confirmation");
        raceState = IDLE;
      }
      break;

    case FINISHED:
      // Solid LED briefly then non-blocking reset
      if (!waitingToReset) {
        digitalWrite(cfg.led_pin, HIGH);
        waitingToReset = true;
        finishedAt = millis();
      }
      break;
  }
}

// ============================================================================
// ESP-NOW MESSAGE HANDLER
// ============================================================================
void onStartGateESPNow(const ESPMessage& msg, uint64_t receiveTime) {
  switch (msg.type) {
    case MSG_PING:
      // Reply with PONG
      sendToPeer(MSG_PONG, nowUs(), 0);
      break;

    case MSG_CONFIRM:
      // Finish gate confirmed race complete
      raceState = FINISHED;
      LOG.println("[START] Race confirmed complete!");
      break;

    case MSG_SYNC_REQ:
      // Finish gate is requesting clock sync - reply with our current time.
      // The finish gate will use this to compute the offset between our clocks.
      sendToPeer(MSG_OFFSET, nowUs(), 0);
      break;

    case MSG_OFFSET:
      // Finish gate owns the offset - we ignore this.
      break;

    case MSG_ARM_CMD:
      // Finish gate says to arm
      if (raceState == IDLE) {
        raceState = ARMED;
        triggerDetected = false;
        triggerTime_us = 0;
        // Attach interrupt to detect beam break
        attachInterrupt(digitalPinToInterrupt(cfg.sensor_pin), startTriggerISR, FALLING);
        // Play armed chime on start gate speaker
        playSound("armed.wav");
        LOG.println("[START] ARMED - waiting for trigger");
      }
      break;

    case MSG_DISARM_CMD:
      // Finish gate says to reset
      raceState = IDLE;
      triggerDetected = false;
      detachInterrupt(digitalPinToInterrupt(cfg.sensor_pin));
      LOG.println("[START] DISARMED");
      break;
  }
}

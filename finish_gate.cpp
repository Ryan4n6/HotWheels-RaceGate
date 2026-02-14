#include "finish_gate.h"
#include "config.h"
#include "wled_integration.h"
#include "audio_manager.h"
#include <LittleFS.h>

// Forward declaration from web_server
extern void broadcastState();

portMUX_TYPE finishTimerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint64_t startTime_us = 0;
volatile uint64_t finishTime_us = 0;

String currentCar = "Unknown";
float currentWeight = 35.0;
uint32_t totalRuns = 0;
double midTrackSpeed_mps = 0; // From speed trap node via ESP-NOW

static unsigned long lastPingTime = 0;
static unsigned long lastSyncTime = 0;

// Non-blocking auto-reset timer (replaces the old blocking delay(5000))
static bool waitingToReset = false;
static unsigned long finishedAt = 0;

// ============================================================================
// FINISH LINE INTERRUPT
// ============================================================================
void IRAM_ATTR finishISR() {
  portENTER_CRITICAL_ISR(&finishTimerMux);
  if (raceState == RACING && finishTime_us == 0) {
    finishTime_us = nowUs();
    raceState = FINISHED;
  }
  portEXIT_CRITICAL_ISR(&finishTimerMux);
}

// ============================================================================
// SETUP
// ============================================================================
void finishGateSetup() {
  pinMode(cfg.sensor_pin, INPUT_PULLUP);
  pinMode(cfg.led_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(cfg.sensor_pin), finishISR, FALLING);
  LOG.printf("[FINISH] Setup complete. Sensor=GPIO%d, LED=GPIO%d\n",
                cfg.sensor_pin, cfg.led_pin);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void finishGateLoop() {
  // Heartbeat LED blink
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    digitalWrite(cfg.led_pin, !digitalRead(cfg.led_pin));
    lastBlink = millis();
  }

  // Check peer connectivity timeout
  if (peerConnected && millis() - lastPeerSeen > PING_BACKOFF_MS) {
    peerConnected = false;
    LOG.println("[FINISH] Peer disconnected - pausing sync/ping");
  }

  // Ping peer every 2 seconds ONLY when connected.
  // When disconnected, back off to every 10 seconds to reduce radio spam.
  unsigned long pingInterval = peerConnected ? PING_INTERVAL_MS : PING_BACKOFF_MS;
  if (millis() - lastPingTime > pingInterval) {
    sendToPeer(MSG_PING, nowUs(), 0);
    lastPingTime = millis();
  }

  // Request clock sync from start gate every 10 seconds - ONLY when connected.
  // No point syncing with a peer that isn't there.
  if (peerConnected && millis() - lastSyncTime > CLOCK_SYNC_INTERVAL_MS) {
    sendToPeer(MSG_SYNC_REQ, nowUs(), 0);
    lastSyncTime = millis();
  }

  // ================================================================
  // Non-blocking auto-reset: wait 5 seconds THEN reset to IDLE
  // During this wait, WebSocket/HTTP keep running so clients see results
  // ================================================================
  if (waitingToReset && millis() - finishedAt > FINISH_RESET_DELAY_MS) {
    waitingToReset = false;
    raceState = IDLE;
    portENTER_CRITICAL(&finishTimerMux);
    startTime_us = 0;
    finishTime_us = 0;
    portEXIT_CRITICAL(&finishTimerMux);
    setWLEDState("idle");
    broadcastState();
    LOG.println("[FINISH] Auto-reset to IDLE");
  }

  // Check WLED auto-sleep timer
  checkWLEDTimeout();

  // Handle race finish (runs ONCE when ISR sets FINISHED)
  // Atomic snapshot: read both 64-bit timing vars under lock to prevent torn reads
  uint64_t safeFinish, safeStart;
  portENTER_CRITICAL(&finishTimerMux);
  safeFinish = finishTime_us;
  safeStart = startTime_us;
  portEXIT_CRITICAL(&finishTimerMux);

  if (raceState == FINISHED && safeFinish > 0 && !waitingToReset) {
    // ================================================================
    // TIMING CALCULATION
    // Use SIGNED math to catch underflows instead of wrapping to huge numbers
    // ================================================================
    int64_t elapsed_us = (int64_t)safeFinish - (int64_t)safeStart;

    LOG.println("[FINISH] ===== RACE RESULT =====");
    LOG.printf("[FINISH] finishTime_us = %llu\n", safeFinish);
    LOG.printf("[FINISH] startTime_us  = %llu\n", safeStart);
    LOG.printf("[FINISH] clockOffset   = %lld\n", clockOffset_us);
    LOG.printf("[FINISH] elapsed_us    = %lld\n", elapsed_us);

    // Sanity check: elapsed must be positive and reasonable (< 60 seconds)
    if (elapsed_us <= 0 || elapsed_us > MAX_RACE_DURATION_US) {
      LOG.printf("[FINISH] BAD TIMING! elapsed=%lld us\n", elapsed_us);
      elapsed_us = 0; // Will show as 0.000s which signals a timing error
    }

    double elapsed_s = elapsed_us / 1000000.0;
    double speed_ms = (elapsed_s > 0) ? cfg.track_length_m / elapsed_s : 0;

    LOG.printf("[FINISH] Time: %.4f s, Speed: %.1f mph\n",
                  elapsed_s, speed_ms * MPS_TO_MPH);
    LOG.println("[FINISH] =========================");

    // Save to LittleFS CSV (includes all physics data)
    double mass_kg = currentWeight / 1000.0;
    double momentum = mass_kg * speed_ms;
    double ke = 0.5 * mass_kg * speed_ms * speed_ms;

    if (!dryRunMode) {
      File file = LittleFS.open("/runs.csv", "a");
      if (file) {
        if (file.size() == 0) file.println("Run,Car,Weight(g),Time(s),Speed(mph),Scale(mph),Momentum,KE(J)");
        file.printf("%u,%s,%.1f,%.4f,%.2f,%.1f,%.4f,%.4f\n", ++totalRuns, currentCar.c_str(),
                    currentWeight, elapsed_s, speed_ms * MPS_TO_MPH,
                    speed_ms * MPS_TO_MPH * (double)cfg.scale_factor,
                    momentum, ke);
        file.close();
      }
    } else {
      LOG.println("[FINISH] Dry-run mode — CSV logging skipped");
    }

    // Send CONFIRM to start gate
    sendToPeer(MSG_CONFIRM, nowUs(), 0);

    // Trigger WLED finished effect (ONLY finish gate controls WLED)
    setWLEDState("finished");

    // Play finish sound effect
    playSound("finish.wav");

    // Broadcast results to WebSocket clients IMMEDIATELY - no delay!
    broadcastState();

    // Reset mid-track speed for next race
    midTrackSpeed_mps = 0;

    // Start the non-blocking 5-second reset timer
    waitingToReset = true;
    finishedAt = millis();
  }
}

// ============================================================================
// ESP-NOW MESSAGE HANDLER
// ============================================================================
void onFinishGateESPNow(const ESPMessage& msg, uint64_t receiveTime) {
  switch (msg.type) {
    case MSG_PING:
      // Reply with PONG
      sendToPeer(MSG_PONG, nowUs(), 0);
      break;

    case MSG_START:
      // Start gate triggered - race begins!
      if (raceState == ARMED) {
        // Convert start gate's timestamp to our local timebase.
        //
        // clockOffset_us = (start_gate_time - finish_gate_time) from last sync.
        // So if start gate clock is 500us ahead: offset = +500
        //
        // To convert start_gate_timestamp to finish_gate_time:
        //   finish_gate_equivalent = start_gate_timestamp - offset
        //
        // This way finishTime_us (local) - startTime_us (converted to local)
        // gives the actual elapsed race time.
        portENTER_CRITICAL(&finishTimerMux);
        startTime_us = msg.timestamp - clockOffset_us;
        portEXIT_CRITICAL(&finishTimerMux);

        LOG.printf("[FINISH] START received: raw_ts=%llu, offset=%lld, adjusted=%llu\n",
                      msg.timestamp, clockOffset_us, startTime_us);

        raceState = RACING;
        setWLEDState("racing");
        LOG.println("[FINISH] RACE STARTED!");
        broadcastState();
      }
      break;

    case MSG_SYNC_REQ:
      // Finish gate INITIATES sync — ignore incoming sync requests.
      // Start gate is the only responder (with MSG_OFFSET).
      break;

    case MSG_OFFSET: {
      // Clock sync response from start gate.
      // offset = start_gate_clock - finish_gate_clock
      int64_t newOffset = (int64_t)msg.timestamp - (int64_t)receiveTime;
      int64_t drift = newOffset - clockOffset_us;
      bool firstSync = (clockOffset_us == 0);
      clockOffset_us = newOffset;
      // Only log on first sync or when drift exceeds 500us to reduce console noise
      if (firstSync || drift > 500 || drift < -500) {
        LOG.printf("[FINISH] Clock sync: offset=%lld us (%.1f ms), drift=%lld us\n",
                      clockOffset_us, clockOffset_us / 1000.0, drift);
      }
      break;
    }

    case MSG_SPEED_DATA:
      // Speed trap node sent mid-track velocity
      // Encoded as speed_mps * 10000 in the offset field
      midTrackSpeed_mps = msg.offset / SPEED_FIXED_POINT_SCALE;
      LOG.printf("[FINISH] Speed trap data: %.3f m/s (%.1f mph)\n",
                    midTrackSpeed_mps, midTrackSpeed_mps * MPS_TO_MPH);
      // Acknowledge receipt
      sendToPeer(MSG_SPEED_ACK, nowUs(), 0);
      break;
  }
}

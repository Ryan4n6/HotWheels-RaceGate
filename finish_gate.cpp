#include "finish_gate.h"
#include "config.h"
#include "wled_integration.h"
#include "audio_manager.h"
#include <LittleFS.h>

// Forward declaration from web_server
extern void broadcastState();

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
  if (raceState == RACING && finishTime_us == 0) {
    finishTime_us = nowUs();
    raceState = FINISHED;
  }
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

  // Check peer connectivity timeout (10 seconds)
  if (peerConnected && millis() - lastPeerSeen > 10000) {
    peerConnected = false;
    LOG.println("[FINISH] Peer disconnected - pausing sync/ping");
  }

  // Ping peer every 2 seconds ONLY when connected.
  // When disconnected, back off to every 10 seconds to reduce radio spam.
  unsigned long pingInterval = peerConnected ? 2000 : 10000;
  if (millis() - lastPingTime > pingInterval) {
    sendToPeer(MSG_PING, nowUs(), 0);
    lastPingTime = millis();
  }

  // Request clock sync from start gate every 10 seconds - ONLY when connected.
  // No point syncing with a peer that isn't there.
  if (peerConnected && millis() - lastSyncTime > 10000) {
    sendToPeer(MSG_SYNC_REQ, nowUs(), 0);
    lastSyncTime = millis();
  }

  // ================================================================
  // Non-blocking auto-reset: wait 5 seconds THEN reset to IDLE
  // During this wait, WebSocket/HTTP keep running so clients see results
  // ================================================================
  if (waitingToReset && millis() - finishedAt > 5000) {
    waitingToReset = false;
    raceState = IDLE;
    startTime_us = 0;
    finishTime_us = 0;
    setWLEDState("idle");
    broadcastState();
    LOG.println("[FINISH] Auto-reset to IDLE");
  }

  // Check WLED auto-sleep timer
  checkWLEDTimeout();

  // Handle race finish (runs ONCE when ISR sets FINISHED)
  if (raceState == FINISHED && finishTime_us > 0 && !waitingToReset) {
    // ================================================================
    // TIMING CALCULATION
    // Use SIGNED math to catch underflows instead of wrapping to huge numbers
    // ================================================================
    int64_t elapsed_us = (int64_t)finishTime_us - (int64_t)startTime_us;

    LOG.println("[FINISH] ===== RACE RESULT =====");
    LOG.printf("[FINISH] finishTime_us = %llu\n", finishTime_us);
    LOG.printf("[FINISH] startTime_us  = %llu\n", startTime_us);
    LOG.printf("[FINISH] clockOffset   = %lld\n", clockOffset_us);
    LOG.printf("[FINISH] elapsed_us    = %lld\n", elapsed_us);

    // Sanity check: elapsed must be positive and reasonable (< 60 seconds)
    if (elapsed_us <= 0 || elapsed_us > 60000000LL) {
      LOG.printf("[FINISH] BAD TIMING! elapsed=%lld us\n", elapsed_us);
      elapsed_us = 0; // Will show as 0.000s which signals a timing error
    }

    double elapsed_s = elapsed_us / 1000000.0;
    double speed_ms = (elapsed_s > 0) ? cfg.track_length_m / elapsed_s : 0;

    LOG.printf("[FINISH] Time: %.4f s, Speed: %.1f mph\n",
                  elapsed_s, speed_ms * 2.23694);
    LOG.println("[FINISH] =========================");

    // Save to LittleFS CSV (includes all physics data)
    double mass_kg = currentWeight / 1000.0;
    double momentum = mass_kg * speed_ms;
    double ke = 0.5 * mass_kg * speed_ms * speed_ms;

    File file = LittleFS.open("/runs.csv", "a");
    if (file) {
      if (file.size() == 0) file.println("Run,Car,Weight(g),Time(s),Speed(mph),Scale(mph),Momentum,KE(J)");
      file.printf("%u,%s,%.1f,%.4f,%.2f,%.1f,%.4f,%.4f\n", ++totalRuns, currentCar.c_str(),
                  currentWeight, elapsed_s, speed_ms * 2.23694,
                  speed_ms * 2.23694 * (double)cfg.scale_factor,
                  momentum, ke);
      file.close();
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
        startTime_us = msg.timestamp - clockOffset_us;

        LOG.printf("[FINISH] START received: raw_ts=%llu, offset=%lld, adjusted=%llu\n",
                      msg.timestamp, clockOffset_us, startTime_us);

        raceState = RACING;
        setWLEDState("racing");
        LOG.println("[FINISH] RACE STARTED!");
        broadcastState();
      }
      break;

    case MSG_SYNC_REQ:
      // Start gate is requesting sync - respond with our timestamp
      sendToPeer(MSG_OFFSET, nowUs(), 0);
      break;

    case MSG_OFFSET:
      // Clock sync response from start gate.
      //
      // We sent SYNC_REQ at some point, start gate replied with its current time.
      // offset = start_gate_clock - finish_gate_clock
      //
      // Simple single-sample offset (good enough for ~1ms ESP-NOW latency)
      clockOffset_us = (int64_t)msg.timestamp - (int64_t)receiveTime;
      LOG.printf("[FINISH] Clock sync: offset=%lld us (%.1f ms)\n",
                    clockOffset_us, clockOffset_us / 1000.0);
      break;

    case MSG_SPEED_DATA:
      // Speed trap node sent mid-track velocity
      // Encoded as speed_mps * 10000 in the offset field
      midTrackSpeed_mps = msg.offset / 10000.0;
      LOG.printf("[FINISH] Speed trap data: %.3f m/s (%.1f mph)\n",
                    midTrackSpeed_mps, midTrackSpeed_mps * 2.23694);
      // Acknowledge receipt
      sendToPeer(MSG_SPEED_ACK, nowUs(), 0);
      break;
  }
}

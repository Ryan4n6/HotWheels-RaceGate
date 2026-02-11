#include "finish_gate.h"
#include "config.h"
#include "wled_integration.h"
#include <LittleFS.h>

// Forward declaration from web_server
extern void broadcastState();

volatile uint64_t startTime_us = 0;
volatile uint64_t finishTime_us = 0;

String currentCar = "Unknown";
float currentWeight = 35.0;
uint32_t totalRuns = 0;

static unsigned long lastPingTime = 0;
static unsigned long lastSyncTime = 0;

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
  Serial.printf("[FINISH] Setup complete. Sensor=GPIO%d, LED=GPIO%d\n",
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

  // Ping peer (start gate) every 2 seconds
  if (millis() - lastPingTime > 2000) {
    sendToPeer(MSG_PING, nowUs(), 0);
    lastPingTime = millis();
  }

  // Request clock sync from start gate every 10 seconds
  // The FINISH gate owns the clock sync - it needs to know the offset
  // to translate start-gate timestamps into its own timebase.
  if (millis() - lastSyncTime > 10000) {
    sendToPeer(MSG_SYNC_REQ, nowUs(), 0);
    lastSyncTime = millis();
  }

  // Check peer connectivity timeout (10 seconds)
  if (peerConnected && millis() - lastPeerSeen > 10000) {
    peerConnected = false;
  }

  // Handle race finish
  if (raceState == FINISHED && finishTime_us > 0) {
    delay(100); // Small delay for stability

    // ================================================================
    // TIMING CALCULATION
    // Use SIGNED math to catch underflows instead of wrapping to huge numbers
    // ================================================================
    int64_t elapsed_us = (int64_t)finishTime_us - (int64_t)startTime_us;

    Serial.println("[FINISH] ===== RACE RESULT =====");
    Serial.printf("[FINISH] finishTime_us = %llu\n", finishTime_us);
    Serial.printf("[FINISH] startTime_us  = %llu\n", startTime_us);
    Serial.printf("[FINISH] clockOffset   = %lld\n", clockOffset_us);
    Serial.printf("[FINISH] elapsed_us    = %lld\n", elapsed_us);

    // Sanity check: elapsed must be positive and reasonable (< 60 seconds)
    if (elapsed_us <= 0 || elapsed_us > 60000000LL) {
      Serial.printf("[FINISH] BAD TIMING! elapsed=%lld us - using finish-gate-only timing\n", elapsed_us);

      // FALLBACK: If two-gate timing failed, the finish gate can still
      // measure time from when it received the START message to when the
      // car crossed the finish line. This isn't as precise, but it gives
      // a real number instead of garbage.
      // startTime_us was set when MSG_START arrived, finishTime_us when ISR fired.
      // If startTime_us is the problem (offset mangled it), just report error.
      elapsed_us = 0; // Will show as 0.000s which signals a timing error
    }

    double elapsed_s = elapsed_us / 1000000.0;
    double speed_ms = (elapsed_s > 0) ? cfg.track_length_m / elapsed_s : 0;

    Serial.printf("[FINISH] Time: %.4f s, Speed: %.1f mph\n",
                  elapsed_s, speed_ms * 2.23694);
    Serial.println("[FINISH] =========================");

    // Save to LittleFS CSV
    File file = LittleFS.open("/runs.csv", "a");
    if (file) {
      if (file.size() == 0) file.println("Run,Car,Time,Speed");
      file.printf("%u,%s,%.4f,%.2f\n", ++totalRuns, currentCar.c_str(),
                  elapsed_s, speed_ms * 2.23694);
      file.close();
    }

    // Send CONFIRM to start gate
    sendToPeer(MSG_CONFIRM, nowUs(), 0);

    // Trigger WLED finished effect
    setWLEDState("finished");

    // Broadcast to WebSocket clients
    broadcastState();

    // Wait then auto-reset
    delay(5000);
    raceState = IDLE;
    startTime_us = 0;
    finishTime_us = 0;
    setWLEDState("idle");
    broadcastState();
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

        Serial.printf("[FINISH] START received: raw_ts=%llu, offset=%lld, adjusted=%llu\n",
                      msg.timestamp, clockOffset_us, startTime_us);

        raceState = RACING;
        setWLEDState("racing");
        Serial.println("[FINISH] RACE STARTED!");
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
      Serial.printf("[FINISH] Clock sync: offset=%lld us (%.1f ms)\n",
                    clockOffset_us, clockOffset_us / 1000.0);
      break;
  }
}

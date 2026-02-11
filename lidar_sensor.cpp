#include "lidar_sensor.h"
#include "config.h"

// TF-Luna uses UART (115200 baud, 9-byte frames)
// No external library needed — just HardwareSerial
static HardwareSerial LidarSerial(2);  // ESP32 UART2

static bool lidarInitialized = false;
static LidarState currentLidarState = LIDAR_NO_CAR;
static uint16_t lastDistance = 0;
static unsigned long lastLidarPoll = 0;
static unsigned long carStagedSince = 0;
static bool autoArmSent = false;

// TF-Luna frame buffer
static uint8_t frameBuffer[9];
static uint8_t frameIndex = 0;

// Forward declaration from web_server
extern void broadcastState();

// ============================================================================
// TF-LUNA UART PROTOCOL
// Frame format (9 bytes):
//   [0x59] [0x59] [Dist_L] [Dist_H] [Amp_L] [Amp_H] [Temp_L] [Temp_H] [Checksum]
//   - Distance: little-endian uint16 in CENTIMETERS
//   - Amplitude: signal strength (low values = unreliable reading)
//   - Checksum: low byte of sum of bytes 0-7
// ============================================================================
static bool parseTFLunaFrame(uint16_t* distanceMM, uint16_t* amplitude) {
  // Verify checksum
  uint8_t checksum = 0;
  for (int i = 0; i < 8; i++) {
    checksum += frameBuffer[i];
  }
  if (checksum != frameBuffer[8]) {
    return false; // Bad frame
  }

  // Distance in cm (little-endian) → convert to mm
  uint16_t distCm = frameBuffer[2] | (frameBuffer[3] << 8);
  *distanceMM = distCm * 10;

  // Amplitude (signal strength)
  *amplitude = frameBuffer[4] | (frameBuffer[5] << 8);

  return true;
}

// ============================================================================
// SETUP
// ============================================================================
void lidarSetup() {
  if (!cfg.lidar_enabled) return;

  // Initialize UART2 on configured pins
  // TF-Luna default baud rate: 115200
  LidarSerial.begin(115200, SERIAL_8N1, cfg.lidar_rx_pin, cfg.lidar_tx_pin);

  // Brief delay for UART to stabilize
  delay(100);

  // Flush any garbage data in the buffer
  while (LidarSerial.available()) {
    LidarSerial.read();
  }

  frameIndex = 0;
  lidarInitialized = true;
  LOG.printf("[LIDAR] TF-Luna initialized. RX=%d, TX=%d, threshold=%dmm\n",
                cfg.lidar_rx_pin, cfg.lidar_tx_pin, cfg.lidar_threshold_mm);
}

// ============================================================================
// MAIN LOOP - Poll at 10Hz (every 100ms)
// TF-Luna outputs frames at ~100Hz by default, so we process available data
// but only run the state machine at 10Hz for consistency.
// ============================================================================
void lidarLoop() {
  if (!lidarInitialized) return;

  // Read available UART bytes and extract frames
  // The TF-Luna streams data continuously, so we parse as bytes arrive
  uint16_t distMM = 0;
  uint16_t amp = 0;
  bool gotValidFrame = false;

  while (LidarSerial.available()) {
    uint8_t byte = LidarSerial.read();

    // Frame sync: look for 0x59 0x59 header
    if (frameIndex == 0) {
      if (byte == 0x59) frameIndex = 1;
      continue;
    }
    if (frameIndex == 1) {
      if (byte == 0x59) {
        frameBuffer[0] = 0x59;
        frameBuffer[1] = 0x59;
        frameIndex = 2;
      } else {
        frameIndex = 0; // Reset, wasn't a valid header
      }
      continue;
    }

    // Collecting remaining bytes
    frameBuffer[frameIndex] = byte;
    frameIndex++;

    if (frameIndex == 9) {
      // Full frame received — parse it
      if (parseTFLunaFrame(&distMM, &amp)) {
        gotValidFrame = true;
      }
      frameIndex = 0;
    }
  }

  // Run state machine at 10Hz (even though frames arrive faster)
  if (millis() - lastLidarPoll < 100) return;
  lastLidarPoll = millis();

  if (!gotValidFrame) return;

  // Filter out unreliable readings (low signal strength)
  // TF-Luna amp < 100 typically means edge-of-range or reflective issues
  if (amp < 100) {
    distMM = 9999; // Treat as no target
  }

  // Filter out zero or obviously invalid readings
  if (distMM == 0) {
    distMM = 9999;
  }

  lastDistance = distMM;

  // State machine
  LidarState newState = currentLidarState;

  switch (currentLidarState) {
    case LIDAR_NO_CAR:
      if (distMM < cfg.lidar_threshold_mm) {
        newState = LIDAR_CAR_STAGED;
        carStagedSince = millis();
        autoArmSent = false;
        LOG.printf("[LIDAR] Car detected at %dmm\n", distMM);
      }
      break;

    case LIDAR_CAR_STAGED:
      if (distMM > cfg.lidar_threshold_mm * 3) {
        // Car removed or launched (distance jumped way up)
        newState = LIDAR_CAR_LAUNCHED;
        LOG.printf("[LIDAR] Car launched! Distance jumped to %dmm\n", distMM);
      } else if (distMM >= cfg.lidar_threshold_mm) {
        // Car slowly moved away — back to no car
        newState = LIDAR_NO_CAR;
        LOG.println("[LIDAR] Car removed");
      }
      break;

    case LIDAR_CAR_LAUNCHED:
      // Auto-reset back to NO_CAR after sensor clears
      if (distMM >= cfg.lidar_threshold_mm) {
        newState = LIDAR_NO_CAR;
      }
      break;
  }

  // Broadcast state changes
  if (newState != currentLidarState) {
    currentLidarState = newState;
    broadcastState(); // Dashboard will pick up LiDAR data from state broadcast
  }
}

// ============================================================================
// PUBLIC ACCESSORS
// ============================================================================
bool isCarPresent() {
  return currentLidarState == LIDAR_CAR_STAGED;
}

uint16_t getDistanceMM() {
  return lastDistance;
}

LidarState getLidarState() {
  return currentLidarState;
}

bool lidarAutoArmReady() {
  // Returns true ONCE when car has been staged for >1 second
  // Caller should send ARM_CMD when this returns true
  if (!lidarInitialized || !cfg.lidar_enabled) return false;
  if (currentLidarState != LIDAR_CAR_STAGED) return false;
  if (autoArmSent) return false;
  if (millis() - carStagedSince > 1000) {
    autoArmSent = true;
    LOG.println("[LIDAR] Auto-arm ready — car staged for 1+ second");
    return true;
  }
  return false;
}

String getLidarJson() {
  String json = "{\"present\":";
  json += (currentLidarState == LIDAR_CAR_STAGED) ? "true" : "false";
  json += ",\"distance_mm\":";
  json += String(lastDistance);
  json += ",\"state\":\"";
  switch (currentLidarState) {
    case LIDAR_NO_CAR:      json += "no_car"; break;
    case LIDAR_CAR_STAGED:  json += "staged"; break;
    case LIDAR_CAR_LAUNCHED: json += "launched"; break;
  }
  json += "\"}";
  return json;
}

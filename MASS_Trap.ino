/*
 * M.A.S.S. TRAP — Motion Analysis & Speed System
 * Unified Firmware v2.4.0
 *
 * Single binary that runs as START GATE, FINISH GATE, or SPEED TRAP.
 * Configure via web portal on first boot (captive portal).
 *
 * WiFi connection uses the EXACT same proven pattern from the original
 * BULLETPROOF firmware: WIFI_STA -> begin -> busy-wait. Simple and reliable.
 * After WiFi connects, we switch to AP_STA for ESP-NOW coexistence.
 *
 * Features:
 *   - Zero-config ESP-NOW auto-discovery (devices find each other automatically)
 *   - Web-based configuration (WiFi, pins, track, WLED)
 *   - WLED integration for race state visual effects
 *   - Google Sheets data logging
 *   - OTA firmware updates
 *   - Full system snapshot (backup/restore config + garage + history)
 *   - Audio effects via MAX98357A I2S amplifier
 *   - LiDAR sensor (TF-Luna) for car presence detection & auto-arm
 *   - Speed trap node with dual IR sensors for mid-track velocity
 *
 * Hardware: ESP32-S3-WROOM-1 N16R8 (16MB Flash / 8MB PSRAM)
 *
 * Arduino IDE Setup:
 *   Board:       "ESP32S3 Dev Module"
 *   Flash Size:  "16MB (128Mb)"
 *   PSRAM:       "OPI PSRAM"
 *   Partition:   Any (partitions.csv in sketch folder auto-overrides)
 *   Libraries:   WebSockets, ArduinoJson (install via Library Manager)
 */

// ============================================================================
// BOARD VALIDATION — Catch misconfiguration at compile time, not at runtime
// ============================================================================
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
  #error "M.A.S.S. Trap requires an ESP32-S3 board. In Arduino IDE: Tools > Board > ESP32S3 Dev Module (or any ESP32-S3 variant)"
#endif

#if !defined(CONFIG_ESPTOOLPY_FLASHSIZE_16MB)
  #warning "M.A.S.S. Trap is designed for 16MB flash. In Arduino IDE: Tools > Flash Size > 16MB (128Mb)"
#endif

#if !defined(CONFIG_SPIRAM)
  #warning "M.A.S.S. Trap works best with PSRAM enabled. In Arduino IDE: Tools > PSRAM > OPI PSRAM"
#endif

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <DNSServer.h>

#include "config.h"
#include "espnow_comm.h"
#include "finish_gate.h"
#include "start_gate.h"
#include "speed_trap.h"
#include "wled_integration.h"
#include "audio_manager.h"
#include "lidar_sensor.h"
#include "web_server.h"

// ============================================================================
// FALLBACK WIFI - Used if configured WiFi fails.
// Set your own fallback credentials here, or leave blank to skip fallback.
// WARNING: Do NOT commit real credentials to version control!
// ============================================================================
#define FALLBACK_WIFI_SSID ""
#define FALLBACK_WIFI_PASS ""

// DNS server for captive portal in setup mode
DNSServer dnsServer;
bool setupMode = false;

// WiFi status tracking for diagnostics (/api/wifi-status)
bool wifiConnected = false;
char wifiFailReason[64] = "";

// Global log output — defaults to Serial, switched to serialTee in setup()
Print* logOutput = &Serial;

// getMacSuffix() and generateHostname() are now in config.cpp
// getRoleEmoji() returns UTF-8 emoji per role for AP SSIDs

// ============================================================================
// WiFi CONNECTION - Mirrors the proven original BULLETPROOF pattern
// ============================================================================
static bool connectWiFi(const char* ssid, const char* pass, const char* hostname) {
  // This is the EXACT pattern from HotWheels_FinishGate_BULLETPROOF.ino
  // that reliably connects every time:
  //   WiFi.mode(WIFI_STA) -> setHostname -> begin -> busy-wait
  //
  // NO disconnect(), NO persistent(), NO mode toggling, NO retry loops.
  // The original never needed any of that. Keep it simple.

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  WiFi.begin(ssid, pass);

  LOG.printf("[WIFI] Connecting to '%s'", ssid);

  // LED feedback while connecting
  pinMode(cfg.led_pin > 0 ? cfg.led_pin : 2, OUTPUT);
  uint8_t ledPin = cfg.led_pin > 0 ? cfg.led_pin : 2;

  // Wait up to 20 seconds (original had no timeout, but we add a safety net)
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(200);
    digitalWrite(ledPin, !digitalRead(ledPin));
    LOG.print(".");
  }
  LOG.println();

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(ledPin, HIGH);
    LOG.printf("[WIFI] Connected! IP: %s, RSSI: %d dBm\n",
               WiFi.localIP().toString().c_str(), WiFi.RSSI());

    // NOW switch to AP_STA so ESP-NOW can work alongside WiFi.
    // Do this AFTER successful connection to avoid confusing the driver.
    // Pin AP to the same channel as the STA connection to prevent channel-hopping.
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(hostname, NULL, WiFi.channel());
    delay(100);

    wifiConnected = true;
    memset(wifiFailReason, 0, sizeof(wifiFailReason));

    // Kick off NTP time sync (non-blocking, best-effort)
    // Uses POSIX TZ string from config for local time display in logs
    serialTee.syncNTP(cfg.timezone);
    LOG.printf("[NTP] Time sync requested (pool.ntp.org, TZ=%s)\n", cfg.timezone);

    return true;
  }

  // Translate WiFi status code to human-readable reason
  wifiConnected = false;
  int status = (int)WiFi.status();
  const char* reason = "Unknown";
  switch (status) {
    case 1:  reason = "SSID not found"; break;
    case 4:  reason = "Wrong password"; break;
    case 5:  reason = "Connection lost"; break;
    case 6:  reason = "Disconnected"; break;
    case 7:  reason = "No SSID configured"; break;
    default: reason = "Connection timeout"; break;
  }
  snprintf(wifiFailReason, sizeof(wifiFailReason), "%s (status=%d)", reason, status);
  LOG.printf("[WIFI] Failed to connect to '%s': %s\n", ssid, wifiFailReason);
  return false;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Initialize serial through the tee so all output is captured for the web console.
  // serialTee.begin() calls Serial.begin() internally, then all prints via
  // serialTee also go to the UART as normal.
  serialTee.begin(115200);
  logOutput = &serialTee;  // Redirect LOG macro to captured output
  delay(500);
  LOG.println("\n\n========================================");
  LOG.println("  " PROJECT_NAME " v" FIRMWARE_VERSION);
  LOG.println("  " PROJECT_FULL);
  LOG.println("========================================");
  LOG.println("[BOOT] Initializing Motion Analysis & Speed System...");

  // Initialize filesystem
  // IMPORTANT: Do NOT use LittleFS.begin(true) here!
  // The 'true' parameter means "format if mount fails", which silently wipes
  // all user data (config, garage, history) after OTA updates or flash glitches.
  // Instead, try mounting WITHOUT format first, retry once, and only format
  // as an absolute last resort on genuinely blank/corrupted partitions.
  bool fsOK = LittleFS.begin(false);
  if (!fsOK) {
    LOG.println("[BOOT] LittleFS mount failed on first attempt, retrying...");
    delay(100);
    fsOK = LittleFS.begin(false);
  }
  if (!fsOK) {
    // Filesystem is genuinely unformatted (first flash) or truly corrupted.
    // Format it so the device can at least enter setup mode.
    LOG.println("[BOOT] LittleFS mount failed twice — formatting (first flash or corruption)");
    fsOK = LittleFS.begin(true);
  }
  if (fsOK) {
    LOG.println("[BOOT] LittleFS mounted OK");
  } else {
    LOG.println("[BOOT] LittleFS mount FAILED even after format!");
  }

  // Load configuration
  bool configured = loadConfig();

  if (!configured) {
    // Before entering setup mode, check if we have a config file that
    // just failed to parse or has configured=false. This protects against
    // the case where uploadfs wipes LittleFS but the user re-saves config
    // from the setup page and it comes back with configured=false.
    // Also log what we found so the user can diagnose via serial.
    if (LittleFS.exists(CONFIG_FILE)) {
      LOG.println("[BOOT] Config file EXISTS but loadConfig() returned false — file may be corrupt or missing 'configured' flag");
      LOG.println("[BOOT] Attempting to read raw config for diagnostics...");
      File dbgFile = LittleFS.open(CONFIG_FILE, "r");
      if (dbgFile) {
        String raw = dbgFile.readString();
        dbgFile.close();
        LOG.printf("[BOOT] Config file size: %d bytes\n", raw.length());
        LOG.printf("[BOOT] First 200 chars: %.200s\n", raw.c_str());
      }
    } else {
      LOG.println("[BOOT] No config file on filesystem — genuinely unconfigured");
    }

    // ====================================================================
    // SETUP MODE - First boot or factory reset
    // ====================================================================
    setupMode = true;
    LOG.println("[BOOT] No config found - entering SETUP MODE");

    // Create AP with emoji SSID: "👮 MassTrap Setup XXXX"
    // Note: macOS WiFi scanner may not render UTF-8 emoji in SSIDs.
    // iOS and Android display them correctly. The SSID bytes are valid.
    char suffix[5];
    getMacSuffix(suffix, sizeof(suffix));
    char apName[48];
    snprintf(apName, sizeof(apName), "%s MassTrap Setup %s", getRoleEmoji(""), suffix);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName);
    delay(500);

    LOG.printf("[BOOT] AP started: %s\n", apName);
    LOG.printf("[BOOT] Connect to WiFi '%s' and open http://192.168.4.1\n", apName);

    // Start DNS server for captive portal
    dnsServer.start(53, "*", WiFi.softAPIP());

    // Start web server in setup mode
    initSetupServer();

    // Blink LED to indicate setup mode (use default pin 2)
    pinMode(2, OUTPUT);
  }
  else {
    // ====================================================================
    // NORMAL MODE - Configured and ready
    // ====================================================================
    LOG.printf("[BOOT] Config loaded: role=%s, hostname=%s\n", cfg.role, cfg.hostname);

    if (strcmp(cfg.network_mode, "standalone") == 0) {
      // Standalone: AP only, emoji SSID e.g. "🏁 masstrap-finish-a7b2"
      char standaloneAP[48];
      snprintf(standaloneAP, sizeof(standaloneAP), "%s %s", getRoleEmoji(cfg.role), cfg.hostname);
      WiFi.mode(WIFI_AP);
      WiFi.softAP(standaloneAP);
      LOG.printf("[BOOT] Standalone AP: %s\n", standaloneAP);
    }
    else {
      // ================================================================
      // WiFi CONNECTION - same proven pattern as original BULLETPROOF
      // ================================================================
      bool connected = false;

      // Try 1: Use config credentials (from captive portal)
      if (strlen(cfg.wifi_ssid) > 0) {
        LOG.println("[BOOT] Trying configured WiFi credentials...");
        connected = connectWiFi(cfg.wifi_ssid, cfg.wifi_pass, cfg.hostname);
      }

      // Try 2: Hardcoded fallback (if defined)
      if (!connected && strlen(FALLBACK_WIFI_SSID) > 0) {
        LOG.println("[BOOT] Config WiFi failed - trying hardcoded fallback...");
        connected = connectWiFi(FALLBACK_WIFI_SSID, FALLBACK_WIFI_PASS, cfg.hostname);
      }

      // Try 3: If all else fails, become an AP so you can still reach config
      if (!connected) {
        LOG.println("[BOOT] All WiFi failed - AP fallback mode");
        char fallbackAP[48];
        snprintf(fallbackAP, sizeof(fallbackAP), "%s %s", getRoleEmoji(cfg.role), cfg.hostname);
        WiFi.mode(WIFI_AP);
        WiFi.softAP(fallbackAP);
        LOG.printf("[BOOT] Fallback AP: %s at 192.168.4.1\n", fallbackAP);
      }
    }

    // mDNS
    if (MDNS.begin(cfg.hostname)) {
      MDNS.addService("http", "tcp", 80);
      LOG.printf("[BOOT] mDNS: http://%s.local\n", cfg.hostname);
    }

    // ESP-NOW (works in both STA and AP_STA modes)
    initESPNow();

    // Web server & WebSocket
    initWebServer();
    startWebServer();

    // OTA updates
    ArduinoOTA.setHostname(cfg.hostname);
    ArduinoOTA.setPassword(cfg.ota_password);
    ArduinoOTA.begin();
    LOG.println("[BOOT] OTA ready");

    // Audio system (optional — guarded by config flag)
    if (cfg.audio_enabled) {
      audioSetup();
      LOG.println("[BOOT] Audio system initialized");
    } else {
      LOG.println("[BOOT] Audio disabled (enable in config)");
    }

    // LiDAR sensor (optional — guarded by config flag)
    if (cfg.lidar_enabled) {
      lidarSetup();
      LOG.println("[BOOT] LiDAR sensor initialized");
    } else {
      LOG.println("[BOOT] LiDAR sensor disabled (enable in config)");
    }

    // Role-specific setup
    if (strcmp(cfg.role, "finish") == 0) {
      finishGateSetup();
    }
    else if (strcmp(cfg.role, "start") == 0) {
      startGateSetup();
    }
    else if (strcmp(cfg.role, "speedtrap") == 0) {
      speedTrapSetup();
    }
    else {
      LOG.printf("[BOOT] Role '%s' not yet implemented\n", cfg.role);
    }

    // Set initial WLED state (finish gate only controls WLED)
    if (strcmp(cfg.role, "finish") == 0) {
      setWLEDState("idle");
    }

    LOG.println("========================================");
    LOG.println("  ALL SYSTEMS OPERATIONAL");
    LOG.println("========================================");
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  if (setupMode) {
    // Setup mode: handle captive portal
    dnsServer.processNextRequest();
    server.handleClient();

    // Rapid LED blink to indicate setup mode
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 200) {
      digitalWrite(2, !digitalRead(2));
      lastBlink = millis();
    }
    return;
  }

  // Normal mode
  ArduinoOTA.handle();
  server.handleClient();
  webSocket.loop();
  processFirmwareUpdate();  // Check for scheduled firmware download (non-blocking when idle)

  // Discovery broadcasts
  discoveryLoop();

  // Audio loop (non-blocking DMA feed — guarded by config flag)
  if (cfg.audio_enabled) {
    audioLoop();
  }

  // LiDAR sensor polling (guarded by config flag)
  if (cfg.lidar_enabled) {
    lidarLoop();
  }

  // Role-specific loop
  if (strcmp(cfg.role, "finish") == 0) {
    finishGateLoop();
  }
  else if (strcmp(cfg.role, "start") == 0) {
    startGateLoop();
  }
  else if (strcmp(cfg.role, "speedtrap") == 0) {
    speedTrapLoop();
  }
}

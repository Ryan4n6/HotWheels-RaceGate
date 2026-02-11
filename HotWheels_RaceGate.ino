/*
 * HOT WHEELS RACE GATE - Unified Firmware v2.1
 *
 * Single binary that runs as either START GATE or FINISH GATE.
 * Configure via web portal on first boot (captive portal).
 *
 * WiFi connection uses the EXACT same proven pattern from the original
 * BULLETPROOF firmware: WIFI_STA -> begin -> busy-wait. Simple and reliable.
 * After WiFi connects, we switch to AP_STA for ESP-NOW coexistence.
 *
 * Features:
 *   - Web-based configuration (WiFi, pins, MAC, track, WLED)
 *   - ESP-NOW device discovery (auto-find peer devices)
 *   - WLED integration for race state visual effects
 *   - Google Sheets data logging
 *   - OTA firmware updates
 *   - Backup/restore configuration
 *
 * Hardware: ESP32 / ESP32-S3
 * Libraries: WebSockets, ArduinoJson (install via Library Manager)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <esp_mac.h>

#include "config.h"
#include "espnow_comm.h"
#include "finish_gate.h"
#include "start_gate.h"
#include "wled_integration.h"
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

// Global log output — defaults to Serial, switched to serialTee in setup()
Print* logOutput = &Serial;

// Helper: get unique 4-char hex suffix from hardware MAC
static void getMacSuffix(char* buf, size_t len) {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  snprintf(buf, len, "%02X%02X", mac[4], mac[5]);
}

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
    LOG.printf("[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());

    // NOW switch to AP_STA so ESP-NOW can work alongside WiFi.
    // Do this AFTER successful connection to avoid confusing the driver.
    WiFi.mode(WIFI_AP_STA);
    delay(100);

    return true;
  }

  LOG.printf("[WIFI] Failed to connect to '%s' (status=%d)\n", ssid, (int)WiFi.status());
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
  LOG.println("  HOT WHEELS RACE GATE v" FIRMWARE_VERSION);
  LOG.println("========================================");

  // Initialize filesystem
  if (!LittleFS.begin(true)) {
    LOG.println("[BOOT] LittleFS mount FAILED!");
  } else {
    LOG.println("[BOOT] LittleFS mounted OK");
  }

  // Load configuration
  bool configured = loadConfig();

  if (!configured) {
    // ====================================================================
    // SETUP MODE - First boot or factory reset
    // ====================================================================
    setupMode = true;
    LOG.println("[BOOT] No config found - entering SETUP MODE");

    // Create AP with unique SSID using last 2 bytes of the hardware MAC
    char suffix[5];
    getMacSuffix(suffix, sizeof(suffix));
    String apName = "HotWheels-Setup-" + String(suffix);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName.c_str());
    delay(500);

    LOG.printf("[BOOT] AP started: %s\n", apName.c_str());
    LOG.printf("[BOOT] Connect to WiFi '%s' and open http://192.168.4.1\n", apName.c_str());

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
      // Standalone: AP only, no external WiFi
      char suffix[5];
      getMacSuffix(suffix, sizeof(suffix));
      char standaloneAP[48];
      snprintf(standaloneAP, sizeof(standaloneAP), "%s-%s", cfg.hostname, suffix);
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
        char suffix[5];
        getMacSuffix(suffix, sizeof(suffix));
        char fallbackAP[48];
        snprintf(fallbackAP, sizeof(fallbackAP), "%s-%s", cfg.hostname, suffix);
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

    // Role-specific setup
    if (strcmp(cfg.role, "finish") == 0) {
      finishGateSetup();
    }
    else if (strcmp(cfg.role, "start") == 0) {
      startGateSetup();
    }
    else {
      LOG.printf("[BOOT] Role '%s' not yet implemented\n", cfg.role);
    }

    // Set initial WLED state
    setWLEDState("idle");

    LOG.println("========================================");
    LOG.println("  SYSTEM READY");
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

  // Discovery broadcasts
  discoveryLoop();

  // Role-specific loop
  if (strcmp(cfg.role, "finish") == 0) {
    finishGateLoop();
  }
  else if (strcmp(cfg.role, "start") == 0) {
    startGateLoop();
  }
}

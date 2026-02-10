/*
 * HOT WHEELS RACE GATE - Unified Firmware v2.0
 *
 * Single binary that runs as either START GATE or FINISH GATE.
 * Configure via web portal on first boot (captive portal).
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

// DNS server for captive portal in setup mode
DNSServer dnsServer;
bool setupMode = false;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n========================================");
  Serial.println("  HOT WHEELS RACE GATE v" FIRMWARE_VERSION);
  Serial.println("========================================");

  // Initialize filesystem
  if (!LittleFS.begin(true)) {
    Serial.println("[BOOT] LittleFS mount FAILED!");
  } else {
    Serial.println("[BOOT] LittleFS mounted OK");
  }

  // Load configuration
  bool configured = loadConfig();

  if (!configured) {
    // ====================================================================
    // SETUP MODE - First boot or factory reset
    // ====================================================================
    setupMode = true;
    Serial.println("[BOOT] No config found - entering SETUP MODE");

    // Create AP with unique SSID using last 2 bytes of the hardware MAC
    // We read from the efuse (burned-in MAC) because WiFi.macAddress()
    // returns 00:00:00:00:00:00 before WiFi is fully initialized
    uint8_t baseMac[6];
    esp_efuse_mac_get_default(baseMac);
    char suffix[5];
    snprintf(suffix, sizeof(suffix), "%02X%02X", baseMac[4], baseMac[5]);
    String apName = "HotWheels-Setup-" + String(suffix);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName.c_str());
    delay(500);

    Serial.printf("[BOOT] AP started: %s\n", apName.c_str());
    Serial.printf("[BOOT] Connect to WiFi '%s' and open http://192.168.4.1\n", apName.c_str());

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
    Serial.printf("[BOOT] Config loaded: role=%s, hostname=%s\n", cfg.role, cfg.hostname);

    // WiFi mode: AP_STA for WiFi + ESP-NOW coexistence
    if (strcmp(cfg.network_mode, "standalone") == 0) {
      // Standalone: AP only, no external WiFi
      WiFi.mode(WIFI_AP);
      WiFi.softAP(cfg.hostname);
      Serial.printf("[BOOT] Standalone AP: %s\n", cfg.hostname);
    }
    else {
      // Normal: connect to WiFi, keep soft AP for fallback
      // Clean disconnect first to avoid "association refused" errors
      // This is critical after reboot from AP-mode setup with WiFi scans
      WiFi.disconnect(true, true);  // disconnect + erase stored credentials
      WiFi.mode(WIFI_OFF);
      delay(200);

      WiFi.mode(WIFI_AP_STA);
      WiFi.setHostname(cfg.hostname);

      Serial.printf("[BOOT] Connecting to WiFi '%s'...", cfg.wifi_ssid);

      // Try connecting with retry logic - ESP32 sometimes needs multiple attempts
      int wifiAttempts = 0;
      const int maxAttempts = 3;
      bool wifiConnected = false;

      while (wifiAttempts < maxAttempts && !wifiConnected) {
        if (wifiAttempts > 0) {
          Serial.printf("\n[BOOT] WiFi attempt %d/%d...", wifiAttempts + 1, maxAttempts);
          WiFi.disconnect(true);
          delay(500);
        }

        WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);

        // Wait up to 10 seconds per attempt
        unsigned long wifiStart = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
          delay(200);
          Serial.print(".");
          // Blink LED while connecting
          if (cfg.led_pin > 0) {
            pinMode(cfg.led_pin, OUTPUT);
            digitalWrite(cfg.led_pin, !digitalRead(cfg.led_pin));
          }
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
          wifiConnected = true;
        }
        wifiAttempts++;
      }

      if (wifiConnected) {
        Serial.printf("[BOOT] WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
        if (cfg.led_pin > 0) digitalWrite(cfg.led_pin, HIGH);
      } else {
        Serial.printf("[BOOT] WiFi connection failed after %d attempts - running in AP fallback mode\n", maxAttempts);
        WiFi.softAP(cfg.hostname);
        Serial.printf("[BOOT] Fallback AP: %s at 192.168.4.1\n", cfg.hostname);
      }
    }

    // mDNS
    if (MDNS.begin(cfg.hostname)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("[BOOT] mDNS: http://%s.local\n", cfg.hostname);
    }

    // ESP-NOW
    initESPNow();

    // Web server & WebSocket
    initWebServer();
    startWebServer();

    // OTA updates
    ArduinoOTA.setHostname(cfg.hostname);
    ArduinoOTA.setPassword(cfg.ota_password);
    ArduinoOTA.begin();
    Serial.println("[BOOT] OTA ready");

    // Role-specific setup
    if (strcmp(cfg.role, "finish") == 0) {
      finishGateSetup();
    }
    else if (strcmp(cfg.role, "start") == 0) {
      startGateSetup();
    }
    else {
      Serial.printf("[BOOT] Role '%s' not yet implemented\n", cfg.role);
    }

    // Set initial WLED state
    setWLEDState("idle");

    Serial.println("========================================");
    Serial.println("  SYSTEM READY");
    Serial.println("========================================");
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

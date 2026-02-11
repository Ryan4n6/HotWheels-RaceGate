#include "web_server.h"
#include "config.h"
#include "espnow_comm.h"
#include "finish_gate.h"
#include "wled_integration.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_mac.h>

WebServer server(80);
WebSocketsServer webSocket(81);

// ============================================================================
// FILE SERVING HELPERS
// ============================================================================
static String getContentType(const String& path) {
  if (path.endsWith(".html")) return "text/html";
  if (path.endsWith(".css"))  return "text/css";
  if (path.endsWith(".js"))   return "application/javascript";
  if (path.endsWith(".json")) return "application/json";
  if (path.endsWith(".csv"))  return "text/csv";
  return "text/plain";
}

static void serveFile(const String& path, const String& contentType) {
  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
  } else {
    server.send(404, "text/plain", "File not found: " + path);
  }
}

// ============================================================================
// WEBSOCKET HANDLER
// ============================================================================
static void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      broadcastState();
      break;

    case WStype_TEXT: {
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, payload);
      if (error) return;

      const char* cmd = doc["cmd"];
      if (!cmd) return;

      if (strcmp(cmd, "arm") == 0) {
        raceState = ARMED;
        startTime_us = 0;
        finishTime_us = 0;
        // Tell start gate to arm too
        sendToPeer(MSG_ARM_CMD, nowUs(), 0);
        setWLEDState("armed");
        broadcastState();
      }
      else if (strcmp(cmd, "reset") == 0) {
        raceState = IDLE;
        startTime_us = 0;
        finishTime_us = 0;
        // Tell start gate to disarm
        sendToPeer(MSG_DISARM_CMD, nowUs(), 0);
        setWLEDState("idle");
        broadcastState();
      }
      else if (strcmp(cmd, "setCar") == 0) {
        currentCar = doc["name"].as<String>();
        currentWeight = doc["weight"];
      }
      else if (strcmp(cmd, "setTrack") == 0) {
        cfg.track_length_m = doc["length"];
      }
      else if (strcmp(cmd, "syncClock") == 0) {
        sendToPeer(MSG_SYNC_REQ, nowUs(), 0);
      }
      break;
    }
    default:
      break;
  }
}

// ============================================================================
// BROADCAST STATE
// ============================================================================
void broadcastState() {
  StaticJsonDocument<512> doc;

  const char* stateStr;
  switch (raceState) {
    case IDLE:     stateStr = "IDLE"; break;
    case ARMED:    stateStr = "ARMED"; break;
    case RACING:   stateStr = "RACING"; break;
    case FINISHED: stateStr = "FINISHED"; break;
    default:       stateStr = "UNKNOWN"; break;
  }

  doc["state"] = stateStr;
  doc["connected"] = peerConnected;
  doc["car"] = currentCar;
  doc["weight"] = currentWeight;
  doc["trackLength"] = cfg.track_length_m;
  doc["scaleFactor"] = cfg.scale_factor;
  doc["totalRuns"] = totalRuns;
  doc["role"] = cfg.role;
  doc["google_sheets_url"] = cfg.google_sheets_url;

  if (raceState == FINISHED && startTime_us > 0 && finishTime_us > 0) {
    // Use SIGNED math to detect underflows instead of wrapping to huge values
    int64_t elapsed_us = (int64_t)finishTime_us - (int64_t)startTime_us;

    // Sanity check: must be positive and < 60 seconds
    if (elapsed_us > 0 && elapsed_us < 60000000LL) {
      double elapsed_s = elapsed_us / 1000000.0;
      double speed_ms = cfg.track_length_m / elapsed_s;

      doc["time"] = elapsed_s;
      doc["speed_mph"] = speed_ms * 2.23694;
      doc["scale_mph"] = speed_ms * 2.23694 * (double)cfg.scale_factor;

      double mass_kg = currentWeight / 1000.0;
      doc["momentum"] = mass_kg * speed_ms;
      doc["ke"] = 0.5 * mass_kg * speed_ms * speed_ms;
    } else {
      // Timing error - report zeros instead of garbage
      doc["time"] = 0;
      doc["speed_mph"] = 0;
      doc["scale_mph"] = 0;
      doc["momentum"] = 0;
      doc["ke"] = 0;
      doc["timing_error"] = true;
    }
  }

  String output;
  serializeJson(doc, output);
  webSocket.broadcastTXT(output);
}

// ============================================================================
// CONFIG API HANDLERS
// ============================================================================
static void handleApiConfig() {
  if (server.method() == HTTP_GET) {
    server.send(200, "application/json", configToJson());
  }
  else if (server.method() == HTTP_POST) {
    String body = server.arg("plain");
    if (body.length() == 0) {
      server.send(400, "application/json", "{\"error\":\"Empty body\"}");
      return;
    }

    DeviceConfig tempCfg;
    setDefaults(tempCfg);

    // Try parsing the JSON
    String testJson = body;
    if (!configFromJson(testJson)) {
      // configFromJson modifies global cfg, so restore defaults if parse fails
      loadConfig();
      server.send(400, "application/json", "{\"error\":\"Invalid config JSON\"}");
      return;
    }

    // configFromJson already loaded into global cfg
    cfg.configured = true;

    if (!validateConfig(cfg)) {
      loadConfig(); // Restore previous
      server.send(400, "application/json", "{\"error\":\"Config validation failed\"}");
      return;
    }

    if (!saveConfig()) {
      server.send(500, "application/json", "{\"error\":\"Failed to save config\"}");
      return;
    }

    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Config saved. Rebooting...\"}");
    delay(500);
    ESP.restart();
  }
}

static void handleApiScan() {
  // Use passive scan in AP mode to avoid disrupting the radio state
  // Active scans can cause "association refused" on next WiFi.begin()
  wifi_scan_config_t scanConfig = {};
  scanConfig.scan_type = WIFI_SCAN_TYPE_PASSIVE;
  scanConfig.scan_time.passive = 300;  // 300ms per channel

  int n = WiFi.scanNetworks(false, false, false, scanConfig.scan_time.passive);

  StaticJsonDocument<1024> doc;
  JsonArray arr = doc.to<JsonArray>();

  for (int i = 0; i < n && i < 20; i++) {
    JsonObject net = arr.createNestedObject();
    net["ssid"] = WiFi.SSID(i);
    net["rssi"] = WiFi.RSSI(i);
    net["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
  }

  WiFi.scanDelete();

  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

static void handleApiMac() {
  StaticJsonDocument<128> doc;

  // WiFi.macAddress() returns STA MAC which may be 00:00:00:00:00:00 in AP-only mode
  // Use esp_efuse_mac_get_default() to always get the base MAC burned into the chip
  uint8_t baseMac[6];
  esp_efuse_mac_get_default(baseMac);
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  doc["mac"] = macStr;

  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

static void handleApiBackup() {
  String json = configToJson();
  server.sendHeader("Content-Disposition", "attachment; filename=hotwheels-config.json");
  server.send(200, "application/json", json);
}

static void handleApiRestore() {
  String body = server.arg("plain");
  if (body.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"Empty body\"}");
    return;
  }

  // Validate JSON structure
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  if (!doc.containsKey("version") || !doc.containsKey("configured")) {
    server.send(400, "application/json", "{\"error\":\"Not a valid config file\"}");
    return;
  }

  // Write directly to config file
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) {
    server.send(500, "application/json", "{\"error\":\"Failed to write config\"}");
    return;
  }
  f.print(body);
  f.close();

  server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Config restored. Rebooting...\"}");
  delay(500);
  ESP.restart();
}

static void handleApiReset() {
  server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Factory reset. Rebooting...\"}");
  delay(500);
  resetConfig();
}

static void handleApiInfo() {
  StaticJsonDocument<256> doc;
  doc["firmware"] = FIRMWARE_VERSION;
  doc["role"] = cfg.role;
  doc["hostname"] = cfg.hostname;
  doc["uptime_s"] = millis() / 1000;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["peer_connected"] = peerConnected;
  doc["ip"] = WiFi.localIP().toString();

  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

static void handleApiDiscover() {
  // Restart discovery scan
  discoveredCount = 0;
  discoveryActive = true;
  sendDiscoveryBroadcast();

  // Wait briefly for responses
  delay(2000);

  server.send(200, "application/json", getDiscoveredDevicesJson());
}

// ============================================================================
// NORMAL MODE ROUTES
// ============================================================================
void initWebServer() {
  // Main page: serve race dashboard for finish gate, status for start gate
  server.on("/", HTTP_GET, []() {
    serveFile("/index.html", "text/html");
  });

  // Config page always accessible
  server.on("/config", HTTP_GET, []() {
    serveFile("/config.html", "text/html");
  });

  // Config API
  server.on("/api/config", handleApiConfig);
  server.on("/api/scan", HTTP_GET, handleApiScan);
  server.on("/api/mac", HTTP_GET, handleApiMac);
  server.on("/api/backup", HTTP_GET, handleApiBackup);
  server.on("/api/restore", HTTP_POST, handleApiRestore);
  server.on("/api/reset", HTTP_POST, handleApiReset);
  server.on("/api/info", HTTP_GET, handleApiInfo);
  server.on("/api/discover", HTTP_GET, handleApiDiscover);

  // WLED proxy endpoints (for config page to fetch WLED data without CORS issues)
  server.on("/api/wled/info", HTTP_GET, []() {
    if (strlen(cfg.wled_host) == 0) {
      server.send(400, "application/json", "{\"error\":\"WLED not configured\"}");
      return;
    }
    HTTPClient http;
    http.begin("http://" + String(cfg.wled_host) + "/json/info");
    http.setTimeout(2000);
    int code = http.GET();
    if (code == 200) {
      server.send(200, "application/json", http.getString());
    } else {
      server.send(502, "application/json", "{\"error\":\"WLED unreachable\"}");
    }
    http.end();
  });

  server.on("/api/wled/effects", HTTP_GET, []() {
    if (strlen(cfg.wled_host) == 0) {
      server.send(400, "application/json", "{\"error\":\"WLED not configured\"}");
      return;
    }
    HTTPClient http;
    http.begin("http://" + String(cfg.wled_host) + "/json/effects");
    http.setTimeout(2000);
    int code = http.GET();
    if (code == 200) {
      server.send(200, "application/json", http.getString());
    } else {
      server.send(502, "application/json", "{\"error\":\"WLED unreachable\"}");
    }
    http.end();
  });

  // Catch-all: try to serve from LittleFS
  server.onNotFound([]() {
    String path = server.uri();
    if (LittleFS.exists(path)) {
      serveFile(path, getContentType(path));
    } else {
      server.send(404, "text/plain", "Not found");
    }
  });
}

void startWebServer() {
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("[WEB] HTTP server started on port 80");
  Serial.println("[WEB] WebSocket server started on port 81");
}

// ============================================================================
// SETUP MODE SERVER (captive portal)
// ============================================================================
void initSetupServer() {
  // In setup mode, serve config page at root
  server.on("/", HTTP_GET, []() {
    serveFile("/config.html", "text/html");
  });

  // Same config API endpoints
  server.on("/api/config", handleApiConfig);
  server.on("/api/scan", HTTP_GET, handleApiScan);
  server.on("/api/mac", HTTP_GET, handleApiMac);
  server.on("/api/info", HTTP_GET, handleApiInfo);

  // Captive portal redirect: any other request goes to root
  server.onNotFound([]() {
    String path = server.uri();
    if (LittleFS.exists(path)) {
      serveFile(path, getContentType(path));
    } else {
      server.sendHeader("Location", "/", true);
      server.send(302, "text/plain", "");
    }
  });

  server.begin();
  Serial.println("[WEB] Setup mode server started");
}

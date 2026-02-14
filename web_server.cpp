#include "web_server.h"
#include "config.h"
#include "espnow_comm.h"
#include "finish_gate.h"
#include "wled_integration.h"
#include "audio_manager.h"
#include "lidar_sensor.h"
#include "html_index.h"
#include "html_config.h"
#include "html_console.h"
#include "html_start_status.h"
#include "html_chartjs.h"
#include "html_speedtrap_status.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_mac.h>

WebServer server(80);
WebSocketsServer webSocket(81);
SerialTee serialTee;

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

// ============================================================================
// API AUTHENTICATION
// Simple API key check for destructive endpoints.
// Reuses OTA password as the key. Returns true if authorized.
// ============================================================================
static bool requireAuth() {
  if (strlen(cfg.ota_password) == 0) return true; // No password set = open
  String key = server.header("X-API-Key");
  if (key == cfg.ota_password) return true;
  server.send(401, "application/json", "{\"error\":\"Unauthorized. Provide X-API-Key header.\"}");
  return false;
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
      StaticJsonDocument<512> doc;  // 512 to fit Google Sheets URLs
      DeserializationError error = deserializeJson(doc, payload);
      if (error) return;

      const char* cmd = doc["cmd"];
      if (!cmd) return;

      if (strcmp(cmd, "arm") == 0) {
        raceState = ARMED;
        portENTER_CRITICAL(&finishTimerMux);
        startTime_us = 0;
        finishTime_us = 0;
        portEXIT_CRITICAL(&finishTimerMux);
        // Tell start gate to arm too
        sendToPeer(MSG_ARM_CMD, nowUs(), 0);
        setWLEDState("armed");
        broadcastState();
      }
      else if (strcmp(cmd, "reset") == 0) {
        raceState = IDLE;
        portENTER_CRITICAL(&finishTimerMux);
        startTime_us = 0;
        finishTime_us = 0;
        portEXIT_CRITICAL(&finishTimerMux);
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
      else if (strcmp(cmd, "setDryRun") == 0) {
        dryRunMode = doc["enabled"] | false;
        LOG.printf("[WEB] Dry-run mode %s\n", dryRunMode ? "ENABLED" : "DISABLED");
        broadcastState();
      }
      else if (strcmp(cmd, "setSheetsUrl") == 0) {
        // Update Google Sheets URL in config and save (no reboot needed)
        const char* url = doc["url"];
        if (url) {
          strncpy(cfg.google_sheets_url, url, sizeof(cfg.google_sheets_url) - 1);
          cfg.google_sheets_url[sizeof(cfg.google_sheets_url) - 1] = '\0';
          saveConfig();
          LOG.printf("[WEB] Google Sheets URL updated: %s\n", cfg.google_sheets_url);
        }
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
  StaticJsonDocument<1024> doc;

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
  doc["units"] = cfg.units;
  doc["google_sheets_url"] = cfg.google_sheets_url;
  doc["dryRun"] = dryRunMode;

  // Speed trap mid-track velocity (if available)
  if (midTrackSpeed_mps > 0) {
    doc["midTrack_mps"] = midTrackSpeed_mps;
    doc["midTrack_mph"] = midTrackSpeed_mps * MPS_TO_MPH;
    doc["midTrack_scale_mph"] = midTrackSpeed_mps * MPS_TO_MPH * (double)cfg.scale_factor;
  }

  // LiDAR sensor data (if enabled)
  if (cfg.lidar_enabled) {
    JsonObject lidar = doc.createNestedObject("lidar");
    LidarState ls = getLidarState();
    lidar["state"] = (ls == LIDAR_NO_CAR) ? "empty" :
                     (ls == LIDAR_CAR_STAGED) ? "staged" : "launched";
    lidar["distance_mm"] = getDistanceMM();
  }

  // Peer count for dashboard status indicators
  int onlinePeers = 0;
  for (int i = 0; i < peerCount; i++) {
    if (peers[i].paired && getPeerStatus(peers[i]) == PEER_ONLINE) onlinePeers++;
  }
  doc["peerCount"] = peerCount;
  doc["onlinePeers"] = onlinePeers;

  // Atomic snapshot of 64-bit timing vars (shared with ISR and ESP-NOW task)
  uint64_t bcastFinish, bcastStart;
  portENTER_CRITICAL(&finishTimerMux);
  bcastFinish = finishTime_us;
  bcastStart = startTime_us;
  portEXIT_CRITICAL(&finishTimerMux);

  if (raceState == FINISHED && bcastStart > 0 && bcastFinish > 0) {
    // Use SIGNED math to detect underflows instead of wrapping to huge values
    int64_t elapsed_us = (int64_t)bcastFinish - (int64_t)bcastStart;

    // Sanity check: must be positive and < 60 seconds
    if (elapsed_us > 0 && elapsed_us < MAX_RACE_DURATION_US) {
      double elapsed_s = elapsed_us / 1000000.0;
      double speed_ms = cfg.track_length_m / elapsed_s;

      doc["time"] = elapsed_s;
      doc["speed_mps"] = speed_ms;
      doc["speed_mph"] = speed_ms * MPS_TO_MPH;
      doc["scale_mph"] = speed_ms * MPS_TO_MPH * (double)cfg.scale_factor;

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
    if (!requireAuth()) return;
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

    // Auto-generate role-based hostname if user left it blank or default
    if (strlen(cfg.hostname) == 0 || strcmp(cfg.hostname, "masstrap") == 0) {
      char suffix[5];
      getMacSuffix(suffix, sizeof(suffix));
      generateHostname(cfg.role, suffix, cfg.hostname, sizeof(cfg.hostname));
      LOG.printf("[CONFIG] Auto-generated hostname: %s\n", cfg.hostname);
    }

    if (!validateConfig(cfg)) {
      loadConfig(); // Restore previous
      server.send(400, "application/json", "{\"error\":\"Config validation failed\"}");
      return;
    }

    if (!saveConfig()) {
      server.send(500, "application/json", "{\"error\":\"Failed to save config\"}");
      return;
    }

    // Include hostname in response so client knows where to redirect
    String resp = "{\"status\":\"ok\",\"message\":\"Config saved. Rebooting...\",\"hostname\":\"";
    resp += cfg.hostname;
    resp += "\"}";
    server.send(200, "application/json", resp);
    server.client().flush();          // Force TCP send buffer drain
    delay(1000);
    WiFi.softAPdisconnect(true);      // Kick AP clients so CNA sheet closes
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
  server.sendHeader("Content-Disposition", "attachment; filename=masstrap-config.json");
  server.send(200, "application/json", json);
}

// ============================================================================
// SYSTEM SNAPSHOT API - Full backup/restore of config + garage + history
// ============================================================================
static void handleApiSystemBackup() {
  // Read all three data files from LittleFS
  String configJson = configToJson();

  String garageJson = "[]";
  if (LittleFS.exists("/garage.json")) {
    File f = LittleFS.open("/garage.json", "r");
    garageJson = f.readString();
    f.close();
  }

  String historyJson = "[]";
  if (LittleFS.exists("/history.json")) {
    File f = LittleFS.open("/history.json", "r");
    historyJson = f.readString();
    f.close();
  }

  // Build the unified snapshot envelope
  DynamicJsonDocument doc(16384);
  doc["snapshot_version"] = 1;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["hostname"] = cfg.hostname;
  doc["role"] = cfg.role;

  // Parse config into nested object
  DynamicJsonDocument configDoc(1536);
  deserializeJson(configDoc, configJson);
  doc["config"] = configDoc.as<JsonObject>();

  // Parse garage into nested array
  DynamicJsonDocument garageDoc(4096);
  deserializeJson(garageDoc, garageJson);
  doc["garage"] = garageDoc.as<JsonArray>();

  // Parse history into nested array
  DynamicJsonDocument historyDoc(8192);
  deserializeJson(historyDoc, historyJson);
  doc["history"] = historyDoc.as<JsonArray>();

  String output;
  serializeJsonPretty(doc, output);

  server.sendHeader("Content-Disposition", "attachment; filename=masstrap-system-backup.json");
  server.send(200, "application/json", output);
  LOG.printf("[WEB] System snapshot exported (%d bytes)\n", output.length());
}

static void handleApiSystemRestore() {
  if (!requireAuth()) return;

  String body = server.arg("plain");
  if (body.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"Empty body\"}");
    return;
  }

  DynamicJsonDocument doc(16384);
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  if (!doc.containsKey("snapshot_version") || !doc.containsKey("config")) {
    server.send(400, "application/json", "{\"error\":\"Not a valid system snapshot\"}");
    return;
  }

  bool skipNetwork = server.hasArg("skip_network") && server.arg("skip_network") == "true";

  // 1. Restore config
  JsonObject configObj = doc["config"];
  if (!configObj.isNull()) {
    if (skipNetwork) {
      // Clone mode: keep this device's network identity
      configObj["network"]["wifi_ssid"] = cfg.wifi_ssid;
      configObj["network"]["wifi_pass"] = cfg.wifi_pass;
      configObj["network"]["hostname"] = cfg.hostname;
    }
    String configStr;
    serializeJson(configObj, configStr);
    File f = LittleFS.open(CONFIG_FILE, "w");
    if (f) { f.print(configStr); f.close(); }
  }

  // 2. Restore garage
  JsonArray garageArr = doc["garage"];
  if (!garageArr.isNull()) {
    String garageStr;
    serializeJson(garageArr, garageStr);
    File f = LittleFS.open("/garage.json", "w");
    if (f) { f.print(garageStr); f.close(); }
  }

  // 3. Restore history
  JsonArray historyArr = doc["history"];
  if (!historyArr.isNull()) {
    String historyStr;
    serializeJson(historyArr, historyStr);
    File f = LittleFS.open("/history.json", "w");
    if (f) { f.print(historyStr); f.close(); }
  }

  LOG.printf("[WEB] System snapshot restored (skip_network=%s). Rebooting...\n",
                skipNetwork ? "true" : "false");

  server.send(200, "application/json",
    "{\"status\":\"ok\",\"message\":\"System snapshot restored. Rebooting...\"}");
  server.client().flush();
  delay(1000);
  WiFi.softAPdisconnect(true);
  delay(500);
  ESP.restart();
}

static void handleApiRestore() {
  if (!requireAuth()) return;
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
  server.client().flush();
  delay(1000);
  WiFi.softAPdisconnect(true);
  delay(500);
  ESP.restart();
}

static void handleApiReset() {
  if (!requireAuth()) return;
  server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Factory reset. Rebooting...\"}");
  server.client().flush();
  delay(1000);
  WiFi.softAPdisconnect(true);
  delay(500);
  resetConfig();
}

static void handleApiInfo() {
  StaticJsonDocument<384> doc;
  doc["project"] = PROJECT_NAME;
  doc["firmware"] = FIRMWARE_VERSION;
  doc["role"] = cfg.role;
  doc["hostname"] = cfg.hostname;
  doc["uptime_s"] = millis() / 1000;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["peer_connected"] = peerConnected;
  doc["peer_count"] = peerCount;
  doc["ip"] = WiFi.localIP().toString();
  doc["audio_enabled"] = cfg.audio_enabled;
  doc["lidar_enabled"] = cfg.lidar_enabled;

  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

// WiFi diagnostic status — extern from MASS_Trap.ino
extern bool wifiConnected;
extern char wifiFailReason[64];

static void handleApiWifiStatus() {
  StaticJsonDocument<256> doc;
  doc["connected"] = (WiFi.status() == WL_CONNECTED);
  doc["ssid"] = cfg.wifi_ssid;
  doc["ip"] = WiFi.localIP().toString();
  doc["rssi"] = WiFi.RSSI();
  doc["mode"] = (WiFi.getMode() == WIFI_AP) ? "AP" :
                (WiFi.getMode() == WIFI_STA) ? "STA" :
                (WiFi.getMode() == WIFI_AP_STA) ? "AP_STA" : "OFF";
  if (strlen(wifiFailReason) > 0) {
    doc["fail_reason"] = wifiFailReason;
  }

  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

static void handleApiVersion() {
  StaticJsonDocument<256> doc;
  doc["firmware"] = FIRMWARE_VERSION;
  doc["web_ui"] = WEB_UI_VERSION;
  doc["build_date"] = BUILD_DATE;
  doc["build_time"] = BUILD_TIME;
#if CONFIG_IDF_TARGET_ESP32S3
  doc["board"] = "ESP32-S3";
#elif CONFIG_IDF_TARGET_ESP32
  doc["board"] = "ESP32";
#else
  doc["board"] = "Unknown";
#endif

  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

// ============================================================================
// PEER DISCOVERY API — Brother's Six Protocol
// ============================================================================
static void handleApiPeers() {
  server.send(200, "application/json", getPeersJson());
}

static void handleApiPeersForget() {
  if (!requireAuth()) return;

  String body = server.arg("plain");
  if (body.length() > 0) {
    // Forget a specific peer by MAC
    StaticJsonDocument<128> doc;
    deserializeJson(doc, body);
    const char* macStr = doc["mac"] | "";
    uint8_t mac[6];
    if (parseMacString(macStr, mac)) {
      forgetPeer(mac);
      server.send(200, "application/json", "{\"status\":\"ok\",\"action\":\"forgot_one\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"Invalid MAC\"}");
    }
  } else {
    // Forget all peers
    forgetAllPeers();
    server.send(200, "application/json", "{\"status\":\"ok\",\"action\":\"forgot_all\"}");
  }
}

// ============================================================================
// GARAGE API - Persistent car storage on ESP32 filesystem
// POST validation: must be JSON array of car objects with valid types
// ============================================================================
static void handleApiGarage() {
  if (server.method() == HTTP_GET) {
    if (LittleFS.exists("/garage.json")) {
      File f = LittleFS.open("/garage.json", "r");
      String content = f.readString();
      f.close();
      server.send(200, "application/json", content);
    } else {
      server.send(200, "application/json", "[]");
    }
  }
  else if (server.method() == HTTP_POST) {
    if (!requireAuth()) return;
    String body = server.arg("plain");
    if (body.length() == 0) {
      server.send(400, "application/json", "{\"error\":\"Empty body\"}");
      return;
    }

    // Validate JSON structure
    DynamicJsonDocument doc(8192);
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }
    if (!doc.is<JsonArray>()) {
      server.send(400, "application/json", "{\"error\":\"Must be array\"}");
      return;
    }
    JsonArray arr = doc.as<JsonArray>();
    if (arr.size() > 50) {
      server.send(400, "application/json", "{\"error\":\"Max 50 cars\"}");
      return;
    }

    // Validate each car object
    for (JsonVariant item : arr) {
      if (!item.is<JsonObject>()) {
        server.send(400, "application/json", "{\"error\":\"Array items must be objects\"}");
        return;
      }
      JsonObject car = item.as<JsonObject>();
      // name must be string
      if (car.containsKey("name") && !car["name"].is<const char*>()) {
        server.send(400, "application/json", "{\"error\":\"name must be string\"}");
        return;
      }
      // weight must be numeric if present
      if (car.containsKey("weight") && !car["weight"].is<float>() && !car["weight"].is<int>()) {
        server.send(400, "application/json", "{\"error\":\"weight must be numeric\"}");
        return;
      }
      // Validate stats sub-object if present
      if (car.containsKey("stats") && car["stats"].is<JsonObject>()) {
        JsonObject stats = car["stats"].as<JsonObject>();
        // bestTime: must be null or numeric
        if (stats.containsKey("bestTime") && !stats["bestTime"].isNull()
            && !stats["bestTime"].is<float>() && !stats["bestTime"].is<int>()) {
          server.send(400, "application/json", "{\"error\":\"bestTime must be numeric or null\"}");
          return;
        }
        // bestSpeed: must be numeric if present
        if (stats.containsKey("bestSpeed") && !stats["bestSpeed"].isNull()
            && !stats["bestSpeed"].is<float>() && !stats["bestSpeed"].is<int>()) {
          server.send(400, "application/json", "{\"error\":\"bestSpeed must be numeric\"}");
          return;
        }
      }
    }

    // Valid — write to filesystem
    File f = LittleFS.open("/garage.json", "w");
    if (!f) {
      server.send(500, "application/json", "{\"error\":\"Failed to write garage\"}");
      return;
    }
    serializeJson(doc, f);
    f.close();
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  }
}

// ============================================================================
// HISTORY API - Persistent race history on ESP32 filesystem
// POST validation: must be JSON array with valid numeric timing fields
// ============================================================================
static void handleApiHistory() {
  if (server.method() == HTTP_GET) {
    if (LittleFS.exists("/history.json")) {
      File f = LittleFS.open("/history.json", "r");
      String content = f.readString();
      f.close();
      server.send(200, "application/json", content);
    } else {
      server.send(200, "application/json", "[]");
    }
  }
  else if (server.method() == HTTP_POST) {
    if (!requireAuth()) return;
    String body = server.arg("plain");
    if (body.length() == 0) {
      server.send(400, "application/json", "{\"error\":\"Empty body\"}");
      return;
    }

    // Validate JSON structure
    DynamicJsonDocument doc(8192);
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }
    if (!doc.is<JsonArray>()) {
      server.send(400, "application/json", "{\"error\":\"Must be array\"}");
      return;
    }
    JsonArray arr = doc.as<JsonArray>();
    if (arr.size() > 100) {
      server.send(400, "application/json", "{\"error\":\"Max 100 entries\"}");
      return;
    }

    // Validate each history entry
    for (JsonVariant item : arr) {
      if (!item.is<JsonObject>()) {
        server.send(400, "application/json", "{\"error\":\"Array items must be objects\"}");
        return;
      }
      JsonObject entry = item.as<JsonObject>();
      // time must be numeric and sane
      if (entry.containsKey("time")) {
        if (!entry["time"].is<float>() && !entry["time"].is<int>()) {
          server.send(400, "application/json", "{\"error\":\"time must be numeric\"}");
          return;
        }
        float t = entry["time"].as<float>();
        if (t <= 0 || t > 60.0) {
          server.send(400, "application/json", "{\"error\":\"time out of range (0-60s)\"}");
          return;
        }
      }
      // car must be string
      if (entry.containsKey("car") && !entry["car"].is<const char*>()) {
        server.send(400, "application/json", "{\"error\":\"car must be string\"}");
        return;
      }
      // Numeric fields: reject strings
      const char* numFields[] = {"speed_mph", "speed_mps", "scale_mph", "momentum", "ke", "weight"};
      for (int i = 0; i < 6; i++) {
        if (entry.containsKey(numFields[i]) && !entry[numFields[i]].isNull()
            && !entry[numFields[i]].is<float>() && !entry[numFields[i]].is<int>()) {
          String errMsg = "{\"error\":\"" + String(numFields[i]) + " must be numeric\"}";
          server.send(400, "application/json", errMsg);
          return;
        }
      }
    }

    // Valid — write to filesystem
    File f = LittleFS.open("/history.json", "w");
    if (!f) {
      server.send(500, "application/json", "{\"error\":\"Failed to write history\"}");
      return;
    }
    serializeJson(doc, f);
    f.close();
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  }
}

// ============================================================================
// AUDIO API - List sounds, test playback, upload WAV files
// ============================================================================
static void handleApiAudioList() {
  if (!cfg.audio_enabled) {
    server.send(200, "application/json", "{\"enabled\":false,\"files\":[]}");
    return;
  }
  String json = "{\"enabled\":true,\"playing\":" + String(isPlaying() ? "true" : "false") +
                ",\"volume\":" + String(cfg.audio_volume) +
                ",\"files\":" + getAudioFileList() + "}";
  server.send(200, "application/json", json);
}

static void handleApiAudioTest() {
  if (!cfg.audio_enabled) {
    server.send(400, "application/json", "{\"error\":\"Audio not enabled\"}");
    return;
  }
  String body = server.arg("plain");
  StaticJsonDocument<128> doc;
  deserializeJson(doc, body);
  const char* file = doc["file"] | "finish.wav";
  playSound(file);
  server.send(200, "application/json", "{\"status\":\"ok\",\"playing\":\"" + String(file) + "\"}");
}

static void handleApiAudioStop() {
  stopSound();
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

static void handleApiAudioVolume() {
  String body = server.arg("plain");
  StaticJsonDocument<64> doc;
  deserializeJson(doc, body);
  uint8_t vol = doc["volume"] | cfg.audio_volume;
  if (vol > 21) vol = 21;
  cfg.audio_volume = vol;
  setVolume(vol);
  saveConfig();
  server.send(200, "application/json", "{\"status\":\"ok\",\"volume\":" + String(vol) + "}");
}

// ============================================================================
// LIDAR SENSOR API - Live readout for config page
// ============================================================================
static void handleApiLidarStatus() {
  StaticJsonDocument<128> doc;
  doc["enabled"] = cfg.lidar_enabled;
  if (cfg.lidar_enabled) {
    LidarState ls = getLidarState();
    doc["state"] = (ls == LIDAR_NO_CAR) ? "empty" :
                   (ls == LIDAR_CAR_STAGED) ? "staged" : "launched";
    doc["distance_mm"] = getDistanceMM();
    doc["threshold_mm"] = cfg.lidar_threshold_mm;
  }
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

// ============================================================================
// SERIAL LOG API - Web-viewable serial monitor
// ============================================================================
static void handleApiLog() {
  if (server.method() == HTTP_GET) {
    server.send(200, "text/plain", serialTee.getLog());
  }
  else if (server.method() == HTTP_DELETE) {
    if (!requireAuth()) return;
    serialTee.clear();
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  }
}

// ============================================================================
// FILESYSTEM API - Browse, read, and write LittleFS files from the web
// ============================================================================
static void handleApiFiles() {
  String path = server.arg("path");
  if (path.length() == 0) path = "/";

  if (server.method() == HTTP_GET) {
    // If path ends with / or is a directory, list files
    // Otherwise, return file contents
    if (path == "/" || path.endsWith("/")) {
      // List directory
      File root = LittleFS.open(path.length() > 0 ? path : "/");
      if (!root || !root.isDirectory()) {
        // Not a directory — try listing root
        root = LittleFS.open("/");
      }

      StaticJsonDocument<2048> doc;
      JsonArray arr = doc.to<JsonArray>();
      File f = root.openNextFile();
      while (f) {
        JsonObject entry = arr.createNestedObject();
        entry["name"] = String(f.name());
        entry["size"] = f.size();
        entry["isDir"] = f.isDirectory();
        f = root.openNextFile();
      }
      String output;
      serializeJson(doc, output);
      server.send(200, "application/json", output);
    } else {
      // Read specific file
      if (!LittleFS.exists(path)) {
        server.send(404, "application/json", "{\"error\":\"File not found\"}");
        return;
      }
      File f = LittleFS.open(path, "r");
      String content = f.readString();
      f.close();
      // Return as JSON with metadata
      // Use DynamicJsonDocument for potentially large file contents
      DynamicJsonDocument doc(content.length() + 256);
      doc["path"] = path;
      doc["size"] = content.length();
      doc["content"] = content;
      String output;
      serializeJson(doc, output);
      server.send(200, "application/json", output);
    }
  }
  else if (server.method() == HTTP_POST) {
    if (!requireAuth()) return;
    // Write file contents
    if (path.length() == 0 || path == "/") {
      server.send(400, "application/json", "{\"error\":\"No path specified\"}");
      return;
    }
    String body = server.arg("plain");
    File f = LittleFS.open(path, "w");
    if (!f) {
      server.send(500, "application/json", "{\"error\":\"Failed to open file for writing\"}");
      return;
    }
    f.print(body);
    f.close();
    server.send(200, "application/json", "{\"status\":\"ok\",\"size\":" + String(body.length()) + "}");
  }
  else if (server.method() == HTTP_DELETE) {
    if (!requireAuth()) return;
    // Delete file
    if (path.length() == 0 || path == "/") {
      server.send(400, "application/json", "{\"error\":\"Cannot delete root\"}");
      return;
    }
    // Protect critical files
    if (path == CONFIG_FILE) {
      server.send(400, "application/json", "{\"error\":\"Use factory reset to delete config\"}");
      return;
    }
    if (LittleFS.remove(path)) {
      server.send(200, "application/json", "{\"status\":\"ok\"}");
    } else {
      server.send(404, "application/json", "{\"error\":\"File not found or delete failed\"}");
    }
  }
}

// ============================================================================
// NORMAL MODE ROUTES
// ============================================================================
// AUTH API — Two-tier authentication (Badge Reader / Internal Affairs)
// ============================================================================
static void handleApiAuthInfo() {
  // No auth required — client uses this to decide which gates to show
  String json = "{\"hasViewerPassword\":";
  json += (strlen(cfg.viewer_password) > 0) ? "true" : "false";
  json += ",\"hasAdminPassword\":";
  json += (strlen(cfg.ota_password) > 0) ? "true" : "false";
  json += "}";
  server.send(200, "application/json", json);
}

static void handleApiAuthCheck() {
  String body = server.arg("plain");
  if (body.length() == 0) {
    server.send(400, "application/json", "{\"error\":\"Empty body\"}");
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  const char* password = doc["password"] | "";
  const char* tier = doc["tier"] | "viewer";

  if (strcmp(tier, "admin") == 0) {
    // Admin tier: check OTA password
    if (strlen(cfg.ota_password) == 0 || strcmp(password, cfg.ota_password) == 0) {
      server.send(200, "application/json", "{\"ok\":true,\"tier\":\"admin\"}");
    } else {
      server.send(200, "application/json", "{\"ok\":false}");
    }
  } else {
    // Viewer tier: check viewer password (blank = always ok)
    if (strlen(cfg.viewer_password) == 0 || strcmp(password, cfg.viewer_password) == 0) {
      server.send(200, "application/json", "{\"ok\":true,\"tier\":\"viewer\"}");
    } else {
      server.send(200, "application/json", "{\"ok\":false}");
    }
  }
}

// ============================================================================
void initWebServer() {
  // Collect X-API-Key header for authentication on protected endpoints
  const char* headerKeys[] = {"X-API-Key"};
  server.collectHeaders(headerKeys, 1);

  // Main page: serve role-appropriate page
  // v2.5.0: Prefer LittleFS files, fall back to PROGMEM if missing
  // Finish gate gets the full dashboard (garage, history, physics)
  // Start gate gets a lightweight status page (no data recording)
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    if (strcmp(cfg.role, "start") == 0) {
      if (LittleFS.exists("/start_status.html")) {
        serveFile("/start_status.html", "text/html");
      } else {
        server.send_P(200, "text/html", START_STATUS_HTML);
      }
    } else if (strcmp(cfg.role, "speedtrap") == 0) {
      if (LittleFS.exists("/speedtrap_status.html")) {
        serveFile("/speedtrap_status.html", "text/html");
      } else {
        server.send_P(200, "text/html", SPEEDTRAP_STATUS_HTML);
      }
    } else {
      if (LittleFS.exists("/dashboard.html")) {
        serveFile("/dashboard.html", "text/html");
      } else {
        server.send_P(200, "text/html", INDEX_HTML);
      }
    }
  });

  // Dashboard alias (direct URL access)
  server.on("/dashboard.html", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    if (LittleFS.exists("/dashboard.html")) {
      serveFile("/dashboard.html", "text/html");
    } else {
      server.send_P(200, "text/html", INDEX_HTML);
    }
  });

  // Chart.js library: prefer LittleFS, fall back to PROGMEM
  server.on("/chart.min.js", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "public, max-age=86400"); // Cache 24h
    if (LittleFS.exists("/chart.min.js")) {
      serveFile("/chart.min.js", "application/javascript");
    } else {
      server.send_P(200, "application/javascript", CHARTJS_MIN);
    }
  });

  // Config page: prefer LittleFS system.html, fall back to PROGMEM
  server.on("/config", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    if (LittleFS.exists("/system.html")) {
      serveFile("/system.html", "text/html");
    } else {
      server.send_P(200, "text/html", CONFIG_HTML);
    }
  });

  // Auth API
  server.on("/api/auth/info", HTTP_GET, handleApiAuthInfo);
  server.on("/api/auth/check", HTTP_POST, handleApiAuthCheck);

  // Config API
  server.on("/api/config", handleApiConfig);
  server.on("/api/scan", HTTP_GET, handleApiScan);
  server.on("/api/mac", HTTP_GET, handleApiMac);
  server.on("/api/backup", HTTP_GET, handleApiBackup);
  server.on("/api/restore", HTTP_POST, handleApiRestore);
  server.on("/api/system/backup", HTTP_GET, handleApiSystemBackup);
  server.on("/api/system/restore", HTTP_POST, handleApiSystemRestore);
  server.on("/api/reset", HTTP_POST, handleApiReset);
  server.on("/api/info", HTTP_GET, handleApiInfo);
  server.on("/api/wifi-status", HTTP_GET, handleApiWifiStatus);
  server.on("/api/version", HTTP_GET, handleApiVersion);
  server.on("/api/peers", HTTP_GET, handleApiPeers);
  server.on("/api/peers/forget", HTTP_POST, handleApiPeersForget);
  server.on("/api/garage", HTTP_GET, handleApiGarage);
  server.on("/api/garage", HTTP_POST, handleApiGarage);
  server.on("/api/history", HTTP_GET, handleApiHistory);
  server.on("/api/history", HTTP_POST, handleApiHistory);

  // Audio API
  server.on("/api/audio/list", HTTP_GET, handleApiAudioList);
  server.on("/api/audio/test", HTTP_POST, handleApiAudioTest);
  server.on("/api/audio/stop", HTTP_POST, handleApiAudioStop);
  server.on("/api/audio/volume", HTTP_POST, handleApiAudioVolume);

  // LiDAR sensor API
  server.on("/api/lidar/status", HTTP_GET, handleApiLidarStatus);

  // Serial log & filesystem
  server.on("/api/log", HTTP_GET, handleApiLog);
  server.on("/api/log", HTTP_DELETE, handleApiLog);
  server.on("/api/files", HTTP_GET, handleApiFiles);
  server.on("/api/files", HTTP_POST, handleApiFiles);
  server.on("/api/files", HTTP_DELETE, handleApiFiles);

  // Console page: prefer LittleFS, fall back to PROGMEM
  server.on("/console", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    if (LittleFS.exists("/console.html")) {
      serveFile("/console.html", "text/html");
    } else {
      server.send_P(200, "text/html", CONSOLE_HTML);
    }
  });

  // WLED proxy endpoints (for config page to fetch WLED data without CORS issues)
  server.on("/api/wled/info", HTTP_GET, []() {
    if (strlen(cfg.wled_host) == 0) {
      server.send(400, "application/json", "{\"error\":\"WLED not configured\"}");
      return;
    }
    HTTPClient http;
    http.begin("http://" + String(cfg.wled_host) + "/json/info");
    http.setTimeout(1000); // 1s for user-facing config page
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
    http.setTimeout(1000); // 1s for user-facing config page
    int code = http.GET();
    if (code == 200) {
      server.send(200, "application/json", http.getString());
    } else {
      server.send(502, "application/json", "{\"error\":\"WLED unreachable\"}");
    }
    http.end();
  });

  // Static CSS/JS assets from LittleFS with cache headers
  server.on("/style.css", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "public, max-age=3600"); // Cache 1h
    serveFile("/style.css", "text/css");
  });

  server.on("/main.js", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "public, max-age=3600"); // Cache 1h
    serveFile("/main.js", "application/javascript");
  });

  // Evidence Log page (new v2.5.0 page)
  server.on("/history.html", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    serveFile("/history.html", "text/html");
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
  LOG.println("[WEB] HTTP server started on port 80");
  LOG.println("[WEB] WebSocket server started on port 81");
}

// ============================================================================
// SETUP MODE SERVER (captive portal)
// ============================================================================
void initSetupServer() {
  // Collect X-API-Key header for authentication
  const char* headerKeys[] = {"X-API-Key"};
  server.collectHeaders(headerKeys, 1);

  // In setup mode, serve config page from PROGMEM at root
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send_P(200, "text/html", CONFIG_HTML);
  });

  // Same config API endpoints
  server.on("/api/config", handleApiConfig);
  server.on("/api/scan", HTTP_GET, handleApiScan);
  server.on("/api/mac", HTTP_GET, handleApiMac);
  server.on("/api/info", HTTP_GET, handleApiInfo);
  server.on("/api/wifi-status", HTTP_GET, handleApiWifiStatus);

  // ---- Captive portal detection handlers ----
  // Explicit handlers for OS-level probe URLs ensure reliable detection.
  // Using full URL in Location header (not relative "/") and no-cache headers
  // prevents CNA caching issues that cause "forget network and try again."

  // Apple (iOS/macOS) probe
  server.on("/hotspot-detect.html", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/html", "");
  });

  // Android probe
  server.on("/generate_204", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/html", "");
  });

  // Windows probe
  server.on("/connecttest.txt", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/html", "");
  });

  // Additional known probe paths
  server.on("/redirect", HTTP_GET, []() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/html", "");
  });
  server.on("/success.txt", HTTP_GET, []() {
    server.send(200, "text/plain", "");  // Empty body != expected → triggers portal
  });
  server.on("/fwlink", HTTP_GET, []() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/html", "");
  });

  // Catch-all: redirect any other request to the config page
  server.onNotFound([]() {
    String path = server.uri();
    if (LittleFS.exists(path)) {
      serveFile(path, getContentType(path));
    } else {
      server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
      server.sendHeader("Location", "http://192.168.4.1/", true);
      server.send(302, "text/html", "");
    }
  });

  server.begin();
  LOG.println("[WEB] Setup mode server started");
}

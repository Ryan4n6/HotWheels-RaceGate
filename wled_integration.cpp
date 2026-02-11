#include "wled_integration.h"
#include "config.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>

// Auto-sleep timer: turn off WLED after 5 minutes of inactivity
static unsigned long lastWLEDActivity = 0;
static bool wledActive = false;
static const unsigned long WLED_TIMEOUT_MS = 5 * 60 * 1000; // 5 minutes

void setWLEDState(const char* raceState) {
  if (strlen(cfg.wled_host) == 0) return; // WLED not configured

  uint8_t effectId;
  if (strcmp(raceState, "idle") == 0)          effectId = cfg.wled_effect_idle;
  else if (strcmp(raceState, "armed") == 0)    effectId = cfg.wled_effect_armed;
  else if (strcmp(raceState, "racing") == 0)   effectId = cfg.wled_effect_racing;
  else if (strcmp(raceState, "finished") == 0) effectId = cfg.wled_effect_finished;
  else return;

  HTTPClient http;
  String url = "http://" + String(cfg.wled_host) + "/json/state";
  http.begin(url);
  http.setTimeout(100); // 100ms max - LAN is fast, don't block race timing
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<128> doc;
  doc["on"] = true;
  doc["bri"] = 255;
  JsonArray seg = doc.createNestedArray("seg");
  JsonObject s = seg.createNestedObject();
  s["fx"] = effectId;
  s["sx"] = 128;
  s["ix"] = 128;

  String body;
  serializeJson(doc, body);

  int httpCode = http.POST(body);
  if (httpCode > 0) {
    LOG.printf("[WLED] Effect %d set (state: %s)\n", effectId, raceState);
  } else {
    LOG.printf("[WLED] Request failed: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();

  // Reset activity timer
  lastWLEDActivity = millis();
  wledActive = true;
}

void setWLEDOff() {
  if (strlen(cfg.wled_host) == 0) return;

  HTTPClient http;
  String url = "http://" + String(cfg.wled_host) + "/json/state";
  http.begin(url);
  http.setTimeout(100);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.POST("{\"on\":false}");
  if (httpCode > 0) {
    LOG.println("[WLED] Turned off (auto-sleep)");
  }
  http.end();
  wledActive = false;
}

void resetWLEDActivity() {
  lastWLEDActivity = millis();
  // If WLED was asleep, wake it up to idle state
  if (!wledActive && strlen(cfg.wled_host) > 0) {
    wledActive = true;
    setWLEDState("idle");
  }
}

void checkWLEDTimeout() {
  if (!wledActive) return;
  if (strlen(cfg.wled_host) == 0) return;
  if (millis() - lastWLEDActivity > WLED_TIMEOUT_MS) {
    LOG.println("[WLED] Inactivity timeout - turning off");
    setWLEDOff();
  }
}

bool testWLEDConnection() {
  if (strlen(cfg.wled_host) == 0) return false;

  HTTPClient http;
  String url = "http://" + String(cfg.wled_host) + "/json/info";
  http.begin(url);
  http.setTimeout(2000); // Config page call - 2s is fine here

  int httpCode = http.GET();
  http.end();

  return (httpCode == 200);
}

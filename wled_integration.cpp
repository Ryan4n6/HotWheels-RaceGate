#include "wled_integration.h"
#include "config.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>

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
  http.setTimeout(500); // 500ms max - don't block race timing
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
}

bool testWLEDConnection() {
  if (strlen(cfg.wled_host) == 0) return false;

  HTTPClient http;
  String url = "http://" + String(cfg.wled_host) + "/json/info";
  http.begin(url);
  http.setTimeout(2000);

  int httpCode = http.GET();
  http.end();

  return (httpCode == 200);
}

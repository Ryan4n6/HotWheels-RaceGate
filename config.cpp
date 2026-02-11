#include "config.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <esp_mac.h>

DeviceConfig cfg;

void setDefaults(DeviceConfig& c) {
  c.configured = false;
  c.version = CONFIG_VERSION;

  memset(c.wifi_ssid, 0, sizeof(c.wifi_ssid));
  memset(c.wifi_pass, 0, sizeof(c.wifi_pass));
  strncpy(c.hostname, "masstrap", sizeof(c.hostname) - 1);
  strncpy(c.network_mode, "wifi", sizeof(c.network_mode) - 1);

  strncpy(c.role, "finish", sizeof(c.role) - 1);
  c.device_id = 1;

  c.sensor_pin = 4;
  c.sensor_pin_2 = 5;    // Speed trap second sensor
  c.led_pin = 2;

  // Audio (MAX98357A I2S) — disabled by default
  c.audio_enabled = false;
  c.i2s_bclk_pin = 15;
  c.i2s_lrc_pin = 16;
  c.i2s_dout_pin = 17;
  c.audio_volume = 10;

  // LiDAR Sensor (TF-Luna UART) — disabled by default
  c.lidar_enabled = false;
  c.lidar_rx_pin = 39;
  c.lidar_tx_pin = 38;
  c.lidar_threshold_mm = 50;

  // Speed Trap
  c.sensor_spacing_m = 0.10f;

  memset(c.peer_mac, 0, 6);

  c.track_length_m = 2.0f;
  c.scale_factor = 64;

  memset(c.google_sheets_url, 0, sizeof(c.google_sheets_url));
  memset(c.wled_host, 0, sizeof(c.wled_host));
  c.wled_effect_idle = 0;
  c.wled_effect_armed = 28;
  c.wled_effect_racing = 49;
  c.wled_effect_finished = 11;

  strncpy(c.ota_password, "admin", sizeof(c.ota_password) - 1);
}

bool loadConfig() {
  setDefaults(cfg);

  if (!LittleFS.exists(CONFIG_FILE)) {
    LOG.println("[CONFIG] No config file found, using defaults");
    return false;
  }

  File file = LittleFS.open(CONFIG_FILE, "r");
  if (!file) {
    LOG.println("[CONFIG] Failed to open config file");
    return false;
  }

  String json = file.readString();
  file.close();

  if (!configFromJson(json)) {
    // configFromJson returns cfg.configured, but after an OTA update the
    // config file survives on LittleFS even though the "configured" flag
    // may be missing or false (e.g. older firmware versions, manual edits).
    // If the file parsed successfully AND has a non-empty role, treat it
    // as a valid config so the device doesn't drop into setup mode.
    if (strlen(cfg.role) > 0 && strlen(cfg.hostname) > 0) {
      LOG.println("[CONFIG] Config file valid but 'configured' flag was false — auto-recovering");
      cfg.configured = true;
      saveConfig();  // Persist the fix so next boot is clean
      return true;
    }
    return false;
  }

  return true;
}

bool saveConfig() {
  String json = configToJson();

  File file = LittleFS.open(CONFIG_FILE, "w");
  if (!file) {
    LOG.println("[CONFIG] Failed to open config file for writing");
    return false;
  }

  file.print(json);
  file.close();
  LOG.println("[CONFIG] Config saved successfully");
  return true;
}

bool isValidGPIO(uint8_t pin) {
  if (pin > 48) return false;
  for (int i = 0; i < GPIO_BLACKLIST_SIZE; i++) {
    if (pin == GPIO_BLACKLIST[i]) return false;
  }
  return true;
}

bool validateConfig(const DeviceConfig& c) {
  if (!isValidGPIO(c.sensor_pin)) {
    LOG.printf("[CONFIG] Invalid sensor pin: %d\n", c.sensor_pin);
    return false;
  }
  if (!isValidGPIO(c.led_pin)) {
    LOG.printf("[CONFIG] Invalid LED pin: %d\n", c.led_pin);
    return false;
  }
  if (c.sensor_pin == c.led_pin) {
    LOG.println("[CONFIG] Sensor and LED pins cannot be the same");
    return false;
  }
  if (c.device_id == 0) {
    LOG.println("[CONFIG] Device ID must be > 0");
    return false;
  }
  if (c.track_length_m <= 0 || c.track_length_m > 100) {
    LOG.println("[CONFIG] Track length must be 0-100m");
    return false;
  }
  if (c.scale_factor < 1 || c.scale_factor > 1000) {
    LOG.println("[CONFIG] Scale factor must be 1-1000");
    return false;
  }
  if (strlen(c.hostname) == 0) {
    LOG.println("[CONFIG] Hostname cannot be empty");
    return false;
  }
  if (strcmp(c.role, "start") != 0 && strcmp(c.role, "finish") != 0 &&
      strcmp(c.role, "speedtrap") != 0 && strcmp(c.role, "display") != 0 &&
      strcmp(c.role, "judge") != 0 && strcmp(c.role, "lights") != 0) {
    LOG.printf("[CONFIG] Invalid role: %s\n", c.role);
    return false;
  }
  return true;
}

String configToJson() {
  StaticJsonDocument<1536> doc;

  doc["configured"] = cfg.configured;
  doc["version"] = cfg.version;

  JsonObject network = doc.createNestedObject("network");
  network["wifi_ssid"] = cfg.wifi_ssid;
  network["wifi_pass"] = cfg.wifi_pass;
  network["hostname"] = cfg.hostname;
  network["mode"] = cfg.network_mode;

  JsonObject device = doc.createNestedObject("device");
  device["role"] = cfg.role;
  device["id"] = cfg.device_id;

  JsonObject pins = doc.createNestedObject("pins");
  pins["sensor_pin"] = cfg.sensor_pin;
  pins["sensor_pin_2"] = cfg.sensor_pin_2;
  pins["led_pin"] = cfg.led_pin;

  JsonObject audio = doc.createNestedObject("audio");
  audio["enabled"] = cfg.audio_enabled;
  audio["bclk_pin"] = cfg.i2s_bclk_pin;
  audio["lrc_pin"] = cfg.i2s_lrc_pin;
  audio["dout_pin"] = cfg.i2s_dout_pin;
  audio["volume"] = cfg.audio_volume;

  JsonObject lidar = doc.createNestedObject("lidar");
  lidar["enabled"] = cfg.lidar_enabled;
  lidar["rx_pin"] = cfg.lidar_rx_pin;
  lidar["tx_pin"] = cfg.lidar_tx_pin;
  lidar["threshold_mm"] = cfg.lidar_threshold_mm;

  JsonObject peer = doc.createNestedObject("peer");
  peer["mac"] = formatMac(cfg.peer_mac);

  JsonObject track = doc.createNestedObject("track");
  track["length_m"] = cfg.track_length_m;
  track["scale_factor"] = cfg.scale_factor;
  track["sensor_spacing_m"] = cfg.sensor_spacing_m;

  JsonObject integrations = doc.createNestedObject("integrations");
  integrations["google_sheets_url"] = cfg.google_sheets_url;
  integrations["wled_host"] = cfg.wled_host;
  JsonObject wled_fx = integrations.createNestedObject("wled_effects");
  wled_fx["idle"] = cfg.wled_effect_idle;
  wled_fx["armed"] = cfg.wled_effect_armed;
  wled_fx["racing"] = cfg.wled_effect_racing;
  wled_fx["finished"] = cfg.wled_effect_finished;

  JsonObject ota = doc.createNestedObject("ota");
  ota["password"] = cfg.ota_password;

  String output;
  serializeJsonPretty(doc, output);
  return output;
}

bool configFromJson(const String& json) {
  StaticJsonDocument<1536> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    LOG.printf("[CONFIG] JSON parse error: %s\n", err.c_str());
    return false;
  }

  cfg.configured = doc["configured"] | false;
  cfg.version = doc["version"] | CONFIG_VERSION;

  JsonObject network = doc["network"];
  if (network) {
    strncpy(cfg.wifi_ssid, network["wifi_ssid"] | "", sizeof(cfg.wifi_ssid) - 1);
    strncpy(cfg.wifi_pass, network["wifi_pass"] | "", sizeof(cfg.wifi_pass) - 1);
    strncpy(cfg.hostname, network["hostname"] | "masstrap", sizeof(cfg.hostname) - 1);
    strncpy(cfg.network_mode, network["mode"] | "wifi", sizeof(cfg.network_mode) - 1);
  }

  JsonObject device = doc["device"];
  if (device) {
    strncpy(cfg.role, device["role"] | "finish", sizeof(cfg.role) - 1);
    cfg.device_id = device["id"] | 1;
  }

  JsonObject pins = doc["pins"];
  if (pins) {
    cfg.sensor_pin = pins["sensor_pin"] | 4;
    cfg.sensor_pin_2 = pins["sensor_pin_2"] | 5;
    cfg.led_pin = pins["led_pin"] | 2;
  }

  JsonObject audio = doc["audio"];
  if (audio) {
    cfg.audio_enabled = audio["enabled"] | false;
    cfg.i2s_bclk_pin = audio["bclk_pin"] | 15;
    cfg.i2s_lrc_pin = audio["lrc_pin"] | 16;
    cfg.i2s_dout_pin = audio["dout_pin"] | 17;
    cfg.audio_volume = audio["volume"] | 10;
  }

  JsonObject lidar = doc["lidar"];
  if (!lidar) lidar = doc["tof"]; // Backwards-compatible with old config files
  if (lidar) {
    cfg.lidar_enabled = lidar["enabled"] | false;
    cfg.lidar_rx_pin = lidar["rx_pin"] | lidar["sda_pin"] | 39;
    cfg.lidar_tx_pin = lidar["tx_pin"] | lidar["scl_pin"] | 38;
    cfg.lidar_threshold_mm = lidar["threshold_mm"] | 50;
  }

  JsonObject peer = doc["peer"];
  if (peer) {
    const char* macStr = peer["mac"] | "00:00:00:00:00:00";
    parseMacString(macStr, cfg.peer_mac);
  }

  JsonObject track = doc["track"];
  if (track) {
    cfg.track_length_m = track["length_m"] | 2.0f;
    cfg.scale_factor = track["scale_factor"] | 64;
    cfg.sensor_spacing_m = track["sensor_spacing_m"] | 0.10f;
  }

  JsonObject integrations = doc["integrations"];
  if (integrations) {
    strncpy(cfg.google_sheets_url, integrations["google_sheets_url"] | "", sizeof(cfg.google_sheets_url) - 1);
    strncpy(cfg.wled_host, integrations["wled_host"] | "", sizeof(cfg.wled_host) - 1);
    JsonObject wled_fx = integrations["wled_effects"];
    if (wled_fx) {
      cfg.wled_effect_idle = wled_fx["idle"] | 0;
      cfg.wled_effect_armed = wled_fx["armed"] | 28;
      cfg.wled_effect_racing = wled_fx["racing"] | 49;
      cfg.wled_effect_finished = wled_fx["finished"] | 11;
    }
  }

  JsonObject ota = doc["ota"];
  if (ota) {
    strncpy(cfg.ota_password, ota["password"] | "admin", sizeof(cfg.ota_password) - 1);
  }

  if (cfg.configured) {
    LOG.printf("[CONFIG] Loaded: role=%s, hostname=%s, wifi=%s\n",
                  cfg.role, cfg.hostname, cfg.wifi_ssid);
  }

  return cfg.configured;
}

void resetConfig() {
  LOG.println("[CONFIG] Factory reset - deleting config and rebooting");
  LittleFS.remove(CONFIG_FILE);
  LittleFS.remove("/runs.csv");
  delay(500);
  ESP.restart();
}

bool parseMacString(const char* macStr, uint8_t* macOut) {
  if (!macStr || strlen(macStr) < 17) {
    memset(macOut, 0, 6);
    return false;
  }
  int values[6];
  int matched = sscanf(macStr, "%x:%x:%x:%x:%x:%x",
                        &values[0], &values[1], &values[2],
                        &values[3], &values[4], &values[5]);
  if (matched != 6) {
    memset(macOut, 0, 6);
    return false;
  }
  for (int i = 0; i < 6; i++) {
    macOut[i] = (uint8_t)values[i];
  }
  return true;
}

String formatMac(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

void getMacSuffix(char* buf, size_t len) {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  snprintf(buf, len, "%02X%02X", mac[4], mac[5]);
}

void generateHostname(const char* role, const char* macSuffix, char* outBuf, size_t outLen) {
  // Abbreviate "speedtrap" to "speed" for compact hostnames
  const char* abbrev = role;
  if (strcmp(role, "speedtrap") == 0) abbrev = "speed";

  if (abbrev && strlen(abbrev) > 0) {
    snprintf(outBuf, outLen, "masstrap-%s-%s", abbrev, macSuffix);
  } else {
    snprintf(outBuf, outLen, "masstrap-%s", macSuffix);
  }

  // Force lowercase (mDNS convention)
  for (size_t i = 0; i < strlen(outBuf); i++) {
    outBuf[i] = tolower(outBuf[i]);
  }
}

const char* getRoleEmoji(const char* role) {
  if (strcmp(role, "finish") == 0)    return "\xF0\x9F\x8F\x81";  // 🏁 checkered flag
  if (strcmp(role, "start") == 0)     return "\xF0\x9F\x9A\xA6";  // 🚦 traffic light
  if (strcmp(role, "speedtrap") == 0) return "\xF0\x9F\x93\xA1";  // 📡 satellite dish
  return "\xF0\x9F\x9A\x94";                                      // 🚔 police car (setup/unknown)
}

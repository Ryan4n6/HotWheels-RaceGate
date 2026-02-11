#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define CONFIG_FILE "/config.json"
#define CONFIG_VERSION 1
#define FIRMWARE_VERSION "2.3.0"
#define WEB_UI_VERSION  "2.3.0"
#define BUILD_DATE      __DATE__
#define BUILD_TIME      __TIME__

// Global log output — all Serial.printf calls should use LOG.printf instead
// This captures output for the web serial monitor (/console)
// Set to &serialTee in setup(), falls back to Serial before that
extern Print* logOutput;
#define LOG (*logOutput)

// GPIO blacklist - pins unsafe for general use on ESP32/ESP32-S3
// GPIO 0: boot button, 6-11: flash SPI (ESP32),
// Adjust if using different board variants
static const uint8_t GPIO_BLACKLIST[] = {0, 6, 7, 8, 9, 10, 11};
static const int GPIO_BLACKLIST_SIZE = sizeof(GPIO_BLACKLIST) / sizeof(GPIO_BLACKLIST[0]);

struct DeviceConfig {
  bool configured;
  int version;

  // Network
  char wifi_ssid[33];
  char wifi_pass[65];
  char hostname[32];
  char network_mode[16]; // "wifi" or "standalone"

  // Device
  char role[16]; // "start", "finish", "display", "judge", "lights"
  uint8_t device_id;

  // Pins
  uint8_t sensor_pin;
  uint8_t led_pin;

  // Peer
  uint8_t peer_mac[6];

  // Track
  float track_length_m;
  int scale_factor;

  // Integrations
  char google_sheets_url[256];
  char wled_host[64];
  uint8_t wled_effect_idle;
  uint8_t wled_effect_armed;
  uint8_t wled_effect_racing;
  uint8_t wled_effect_finished;

  // OTA
  char ota_password[32];
};

// Global config instance
extern DeviceConfig cfg;

// Load config from LittleFS. Returns true if valid config found.
bool loadConfig();

// Save current config to LittleFS. Returns true on success.
bool saveConfig();

// Validate config values. Returns true if valid.
bool validateConfig(const DeviceConfig& c);

// Check if a GPIO pin is safe to use
bool isValidGPIO(uint8_t pin);

// Set default values on the config struct
void setDefaults(DeviceConfig& c);

// Serialize config to JSON string (for API and backup)
String configToJson();

// Deserialize JSON string into config struct. Returns true on success.
bool configFromJson(const String& json);

// Delete config file and reboot (factory reset)
void resetConfig();

// Parse MAC string "XX:XX:XX:XX:XX:XX" into uint8_t[6]. Returns true on success.
bool parseMacString(const char* macStr, uint8_t* macOut);

// Format uint8_t[6] MAC to "XX:XX:XX:XX:XX:XX" string
String formatMac(const uint8_t* mac);

#endif

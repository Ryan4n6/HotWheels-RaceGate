#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define CONFIG_FILE "/config.json"
#define CONFIG_VERSION 2
#define FIRMWARE_VERSION "2.5.0"
#define WEB_UI_VERSION  "2.5.0"
#define BUILD_DATE      __DATE__
#define BUILD_TIME      __TIME__
#define PROJECT_NAME    "M.A.S.S. Trap"
#define PROJECT_FULL    "Motion Analysis & Speed System"

// GitHub firmware update — closed-circuit download from official releases only
#define GITHUB_REPO           "Ryan4n6/MASS-Trap"
#define GITHUB_RELEASES_URL   "https://github.com/Ryan4n6/MASS-Trap/releases"
#define GITHUB_API_LATEST     "https://api.github.com/repos/Ryan4n6/MASS-Trap/releases/latest"
#define GITHUB_ASSET_PREFIX_1 "https://github.com/"
#define GITHUB_ASSET_PREFIX_2 "https://objects.githubusercontent.com/"
#define MAX_FIRMWARE_SIZE     0x300000  // 3MB, matches app0/app1 partition size

// ============================================================================
// NAMED CONSTANTS — Replaces magic numbers scattered across the codebase
// ============================================================================

// Unit conversion
#define MPS_TO_MPH              2.23694     // metres/second → miles/hour
#define MPS_TO_KPH              3.6         // metres/second → kilometres/hour
#define METERS_TO_FEET          3.28084     // metres → feet

// ESP-NOW speed data encoding (fixed-point in int64_t offset field)
#define SPEED_FIXED_POINT_SCALE 10000.0

// Race timing sanity limits (microseconds)
#define MAX_RACE_DURATION_US    60000000LL  // 60 seconds — reject anything longer
#define MAX_TRAP_DURATION_US    10000000LL  // 10 seconds — speed trap max
#define TRAP_SENSOR_TIMEOUT_US  5000000LL   // 5 seconds  — single sensor timeout

// Auto-reset delays after FINISHED state (milliseconds)
#define FINISH_RESET_DELAY_MS   5000        // Finish gate: 5s display then IDLE
#define START_RESET_DELAY_MS    2000        // Start gate: 2s then IDLE

// Race timeout (milliseconds)
#define RACE_TIMEOUT_MS         30000       // 30s — abort if no finish

// ESP-NOW peer health intervals (milliseconds)
#define PING_INTERVAL_MS        2000        // Keepalive when peer is online
#define PING_BACKOFF_MS         10000       // Keepalive when peer is offline
#define CLOCK_SYNC_INTERVAL_MS  30000       // Finish→start time sync request (30s)
#define PEER_HEALTH_CHECK_MS    5000        // Peer status scan interval

// ESP-NOW discovery (milliseconds)
#define BEACON_INTERVAL_MS      3000        // Broadcast "I'm here"
#define PEER_ONLINE_THRESH_MS   15000       // <15s since last heard = ONLINE
#define PEER_STALE_THRESH_MS    60000       // <60s = STALE, >60s = OFFLINE
#define PEER_SAVE_DEBOUNCE_MS   2000        // Delay before writing /peers.json

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
  char role[16]; // "start", "finish", "speedtrap", "display", "judge", "lights"
  uint8_t device_id;

  // Pins
  uint8_t sensor_pin;
  uint8_t sensor_pin_2;  // Second sensor (speed trap dual-IR)
  uint8_t led_pin;

  // Audio (MAX98357A I2S)
  bool audio_enabled;
  uint8_t i2s_bclk_pin;
  uint8_t i2s_lrc_pin;
  uint8_t i2s_dout_pin;
  uint8_t audio_volume;  // 0-21

  // LiDAR Sensor (Benewake TF-Luna, UART)
  bool lidar_enabled;
  uint8_t lidar_rx_pin;
  uint8_t lidar_tx_pin;
  uint16_t lidar_threshold_mm;

  // Speed Trap
  float sensor_spacing_m; // Distance between speed trap sensors

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

  // Regional / Display preferences
  char units[12];       // "imperial" (mph, ft) or "metric" (km/h, m)
  char timezone[40];    // POSIX TZ string, e.g. "EST5EDT,M3.2.0,M11.1.0" or "UTC"

  // OTA
  char ota_password[32];

  // Viewer authentication (Badge Reader)
  char viewer_password[32];  // Blank = open access (no viewer gate)
};

// Global config instance
extern DeviceConfig cfg;

// Load config from LittleFS. Returns true if valid config found.
// Falls back to NVS backup if LittleFS config is missing/corrupt.
bool loadConfig();

// Save current config to LittleFS. Returns true on success.
// Also saves critical boot fields (ssid, pass, role, hostname) to NVS
// as a backup that survives LittleFS wipes (e.g. uploadfs).
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

// Get 4-char hex suffix from hardware MAC (e.g., "A7B2")
void getMacSuffix(char* buf, size_t len);

// Generate role-based hostname: "masstrap-finish-a7b2"
// Abbreviates "speedtrap" to "speed". Forces lowercase.
void generateHostname(const char* role, const char* macSuffix, char* outBuf, size_t outLen);

// Returns UTF-8 emoji string for a role's AP SSID personality
// finish → 🏁  start → 🚦  speedtrap → 🚓  default/setup → 👮
const char* getRoleEmoji(const char* role);

#endif

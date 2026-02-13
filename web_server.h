#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include <time.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

extern WebServer server;
extern WebSocketsServer webSocket;

// Initialize web server routes and WebSocket
void initWebServer();

// Start the servers (call after WiFi is connected)
void startWebServer();

// Broadcast current race state to all WebSocket clients
void broadcastState();

// Setup mode: minimal server with config endpoints only
void initSetupServer();

// ============================================================================
// SERIAL LOG CAPTURE - Ring buffer that tees Serial output for web viewing
// ============================================================================
#define SERIAL_LOG_SIZE 8192  // 8KB ring buffer

class SerialTee : public Print {
public:
  char buffer[SERIAL_LOG_SIZE];
  volatile size_t head = 0;    // Write position
  volatile size_t count = 0;   // Total bytes in buffer
  Print* hw;
  bool atLineStart = true;     // Track whether next char starts a new line
  bool ntpSynced = false;      // True once NTP provides valid wall-clock time

  SerialTee() : hw(&Serial) {}

  void begin(unsigned long baud) {
    Serial.begin(baud);  // Call begin() on Serial directly (works for both HardwareSerial and HWCDC)
    memset(buffer, 0, SERIAL_LOG_SIZE);
    head = 0;
    count = 0;
    atLineStart = true;
    ntpSynced = false;
  }

  // Try NTP sync — call after WiFi is connected. Non-blocking, best-effort.
  // Uses the POSIX TZ string from config (e.g. "EST5EDT,M3.2.0,M11.1.0")
  void syncNTP(const char* tz = nullptr) {
    const char* tzStr = (tz && strlen(tz) > 0) ? tz : "EST5EDT,M3.2.0,M11.1.0";
    configTzTime(tzStr, "pool.ntp.org", "time.nist.gov");
    // Don't block — just fire off the request. We'll check time validity in writeTimestamp().
  }

  // Format timestamp: [HH:MM:SS.mmm] using NTP wall-clock if available, uptime if not
  void writeTimestamp() {
    char ts[24];  // "[HH:MM:SS.mmm] " max = 16 chars; "[+HH:MM:SS.mmm] " max = 18

    // Try NTP wall-clock time first
    if (!ntpSynced) {
      struct tm ti;
      if (getLocalTime(&ti, 0)) {  // 0 = don't block
        ntpSynced = (ti.tm_year > 100);  // year > 2000 means NTP succeeded
      }
    }

    if (ntpSynced) {
      struct tm ti;
      if (getLocalTime(&ti, 0)) {
        unsigned int frac = millis() % 1000;
        snprintf(ts, sizeof(ts), "[%02d:%02d:%02d.%03u] ",
                 ti.tm_hour, ti.tm_min, ti.tm_sec, frac);
      } else {
        // Fallback if getLocalTime fails unexpectedly
        goto uptime_fallback;
      }
    } else {
      uptime_fallback:
      // Pre-NTP: use uptime with '+' prefix to distinguish from wall-clock
      unsigned long ms = millis();
      unsigned long totalSec = ms / 1000;
      unsigned int frac = ms % 1000;
      unsigned int sec = totalSec % 60;
      unsigned int mins = (totalSec / 60) % 60;
      unsigned int hrs = (totalSec / 3600);

      if (hrs > 0) {
        snprintf(ts, sizeof(ts), "[+%u:%02u:%02u.%03u] ", hrs, mins, sec, frac);
      } else {
        snprintf(ts, sizeof(ts), "[+%02u:%02u.%03u] ", mins, sec, frac);
      }
    }

    // Write timestamp to ring buffer only (not UART — UART already has its own timing)
    for (int i = 0; ts[i] != '\0'; i++) {
      buffer[head] = ts[i];
      head = (head + 1) % SERIAL_LOG_SIZE;
      if (count < SERIAL_LOG_SIZE) count = count + 1;
    }
  }

  // Store one byte to ring buffer (no UART echo)
  inline void storeByte(char c) {
    buffer[head] = c;
    head = (head + 1) % SERIAL_LOG_SIZE;
    if (count < SERIAL_LOG_SIZE) count = count + 1;
  }

  size_t write(uint8_t c) override {
    hw->write(c);  // Always send to real UART
    // Inject timestamp at start of each new line in the ring buffer
    if (atLineStart && c != '\n' && c != '\r') {
      writeTimestamp();
      atLineStart = false;
    }
    storeByte((char)c);
    if (c == '\n') atLineStart = true;
    return 1;
  }

  size_t write(const uint8_t* buf, size_t size) override {
    hw->write(buf, size);  // Real UART
    // Process each byte for timestamp injection in ring buffer
    for (size_t i = 0; i < size; i++) {
      char c = (char)buf[i];
      if (atLineStart && c != '\n' && c != '\r') {
        writeTimestamp();
        atLineStart = false;
      }
      storeByte(c);
      if (c == '\n') atLineStart = true;
    }
    return size;
  }

  // Read the buffer in chronological order
  String getLog() {
    if (count == 0) return "";
    String result;
    result.reserve(count);
    if (count < SERIAL_LOG_SIZE) {
      // Buffer hasn't wrapped — read from 0 to head
      for (size_t i = 0; i < head; i++) {
        result += buffer[i];
      }
    } else {
      // Buffer wrapped — read from head to end, then start to head
      for (size_t i = head; i < SERIAL_LOG_SIZE; i++) {
        result += buffer[i];
      }
      for (size_t i = 0; i < head; i++) {
        result += buffer[i];
      }
    }
    return result;
  }

  void clear() {
    head = 0;
    count = 0;
    atLineStart = true;
    memset(buffer, 0, SERIAL_LOG_SIZE);
  }
};

extern SerialTee serialTee;

#endif

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
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
  HardwareSerial* hw;

  SerialTee() : hw(&Serial) {}

  void begin(unsigned long baud) {
    hw->begin(baud);
    memset(buffer, 0, SERIAL_LOG_SIZE);
    head = 0;
    count = 0;
  }

  size_t write(uint8_t c) override {
    hw->write(c);  // Always send to real UART
    buffer[head] = (char)c;
    head = (head + 1) % SERIAL_LOG_SIZE;
    if (count < SERIAL_LOG_SIZE) count++;
    return 1;
  }

  size_t write(const uint8_t* buf, size_t size) override {
    hw->write(buf, size);  // Real UART
    for (size_t i = 0; i < size; i++) {
      buffer[head] = (char)buf[i];
      head = (head + 1) % SERIAL_LOG_SIZE;
    }
    if (count + size >= SERIAL_LOG_SIZE) count = SERIAL_LOG_SIZE;
    else count += size;
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
    memset(buffer, 0, SERIAL_LOG_SIZE);
  }
};

extern SerialTee serialTee;

#endif

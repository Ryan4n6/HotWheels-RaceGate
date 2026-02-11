#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <Arduino.h>
#include <esp_now.h>

// Message types
#define MSG_PING        0
#define MSG_START       1
#define MSG_CONFIRM     2
#define MSG_PONG        3
#define MSG_SYNC_REQ    4
#define MSG_OFFSET      5
#define MSG_ARM_CMD     6
#define MSG_DISARM_CMD  7
#define MSG_DISCOVER    8
#define MSG_DISCOVER_ACK 9
#define MSG_SPEED_DATA  10  // Speed trap → finish: mid-track velocity (offset = speed_mps * 10000)
#define MSG_SPEED_ACK   11  // Finish → speed trap: acknowledge receipt

// Race states shared by both roles
enum RaceState { IDLE, ARMED, RACING, FINISHED };

// ESP-NOW message structure
typedef struct {
  uint8_t type;
  uint8_t senderId;
  uint64_t timestamp;
  int64_t offset;
  char role[16];
  char hostname[32];
} ESPMessage;

// Discovered device info
struct DiscoveredDevice {
  uint8_t mac[6];
  char role[16];
  char hostname[32];
  unsigned long lastSeen;
};

#define MAX_DISCOVERED_DEVICES 10

// Global state
extern volatile RaceState raceState;
extern int64_t clockOffset_us;
extern bool peerConnected;
extern unsigned long lastPeerSeen;

// Discovery state
extern DiscoveredDevice discoveredDevices[];
extern int discoveredCount;
extern bool discoveryActive;

// Microsecond timer
uint64_t nowUs();

// Initialize ESP-NOW, register callbacks, add peer from config
void initESPNow();

// Send an ESP-NOW message to the configured peer
void sendToPeer(uint8_t type, uint64_t timestamp, int64_t offset);

// Send a discovery broadcast
void sendDiscoveryBroadcast();

// Send a discovery acknowledgement to a specific MAC
void sendDiscoveryAck(const uint8_t* targetMac);

// Called from finish_gate, start_gate, or speed_trap when ESP-NOW message arrives
// These are implemented in their respective .cpp files
extern void onFinishGateESPNow(const ESPMessage& msg, uint64_t receiveTime);
extern void onStartGateESPNow(const ESPMessage& msg, uint64_t receiveTime);
extern void onSpeedTrapESPNow(const ESPMessage& msg, uint64_t receiveTime);

// Run periodic discovery (call from loop)
void discoveryLoop();

// Get discovered devices as JSON string
String getDiscoveredDevicesJson();

#endif

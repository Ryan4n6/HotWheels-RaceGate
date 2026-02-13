#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <Arduino.h>
#include <esp_now.h>

// ============================================================================
// ESP-NOW MESSAGE TYPES
// ============================================================================
#define MSG_PING        0
#define MSG_START       1
#define MSG_CONFIRM     2
#define MSG_PONG        3
#define MSG_SYNC_REQ    4
#define MSG_OFFSET      5
#define MSG_ARM_CMD     6
#define MSG_DISARM_CMD  7
#define MSG_BEACON      8    // Periodic "I'm here" broadcast (replaces MSG_DISCOVER)
#define MSG_BEACON_ACK  9    // Direct reply to a beacon (replaces MSG_DISCOVER_ACK)
#define MSG_SPEED_DATA  10   // Speed trap → finish: mid-track velocity
#define MSG_SPEED_ACK   11   // Finish → speed trap: acknowledge receipt
#define MSG_PAIR_REQ    12   // "I want to pair with you" (role-aware)
#define MSG_PAIR_ACK    13   // "Pairing accepted"

// Race states shared by all roles
enum RaceState { IDLE, ARMED, RACING, FINISHED };

// ============================================================================
// ESP-NOW MESSAGE STRUCTURE
// ============================================================================
typedef struct {
  uint8_t type;
  uint8_t senderId;
  uint64_t timestamp;
  int64_t offset;
  char role[16];         // "start", "finish", "speedtrap"
  char hostname[32];     // mDNS hostname for display
} ESPMessage;

// ============================================================================
// PEER REGISTRY — The "Brother's Six" System
//
// Every M.A.S.S. Trap device continuously broadcasts beacons on ESP-NOW.
// When a compatible peer is heard, it's automatically added to the registry
// and paired for direct communication — no manual MAC entry needed.
//
// Pairing rules (role-aware):
//   start    ↔ finish     (bidirectional — timing link)
//   speedtrap → finish    (unidirectional — speed data to dashboard)
//   start    ↔ speedtrap  (not paired — no direct communication needed)
//
// Peers are persisted to /peers.json on LittleFS so they survive reboots.
// ============================================================================

// Peer status
enum PeerStatus {
  PEER_ONLINE,       // Heard within last 15 seconds
  PEER_STALE,        // Heard within last 60 seconds
  PEER_OFFLINE       // Not heard for >60 seconds (but still in registry)
};

// A known peer device
struct KnownPeer {
  uint8_t mac[6];
  char role[16];
  char hostname[32];
  uint8_t deviceId;
  unsigned long lastSeen;      // millis() of last beacon/message
  bool espnowRegistered;       // true if esp_now_add_peer() was called
  bool paired;                 // true if mutual pairing confirmed
};

#define MAX_PEERS 8

// ============================================================================
// GLOBAL STATE
// ============================================================================
extern volatile RaceState raceState;
extern int64_t clockOffset_us;
extern bool peerConnected;           // Legacy: true if PRIMARY peer is online
extern unsigned long lastPeerSeen;   // Legacy: millis() of last primary peer msg

// Peer registry
extern KnownPeer peers[];
extern int peerCount;

// ============================================================================
// MICROSECOND TIMER
// ============================================================================
uint64_t nowUs();  // Definition in .cpp is IRAM_ATTR (ISR-safe)

// ============================================================================
// INITIALIZATION
// ============================================================================
// Initialize ESP-NOW, register callbacks, load persisted peers, start beaconing
void initESPNow();

// ============================================================================
// MESSAGING
// ============================================================================
// Send to a specific MAC address
void sendToMac(const uint8_t* mac, uint8_t type, uint64_t timestamp, int64_t offset);

// Send to the primary peer (legacy — sends to first paired complementary role)
void sendToPeer(uint8_t type, uint64_t timestamp, int64_t offset);

// ============================================================================
// PEER MANAGEMENT
// ============================================================================
// Find a peer by MAC address. Returns index or -1.
int findPeerByMac(const uint8_t* mac);

// Find the first paired peer with a given role. Returns index or -1.
int findPeerByRole(const char* role);

// Get peer status based on lastSeen
PeerStatus getPeerStatus(const KnownPeer& peer);

// Check if we have at least one online paired peer (for legacy peerConnected flag)
bool hasOnlinePeer();

// Forget a specific peer by MAC
void forgetPeer(const uint8_t* mac);

// Forget ALL peers (factory reset peers)
void forgetAllPeers();

// Get peers list as JSON string (for web API)
String getPeersJson();

// ============================================================================
// PEER PERSISTENCE
// ============================================================================
void loadPeers();   // Load /peers.json from LittleFS
void savePeers();   // Save /peers.json to LittleFS

// ============================================================================
// DISCOVERY LOOP — Call from main loop()
// ============================================================================
// Handles beacon broadcasting, peer timeout tracking, and auto-pairing
void discoveryLoop();

// ============================================================================
// ROLE-SPECIFIC ESP-NOW HANDLERS
// Implemented in start_gate.cpp, finish_gate.cpp, speed_trap.cpp
// ============================================================================
extern void onFinishGateESPNow(const ESPMessage& msg, uint64_t receiveTime);
extern void onStartGateESPNow(const ESPMessage& msg, uint64_t receiveTime);
extern void onSpeedTrapESPNow(const ESPMessage& msg, uint64_t receiveTime);

#endif

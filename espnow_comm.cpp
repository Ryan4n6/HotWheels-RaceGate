// ============================================================================
// TODO: AUTODISCOVERY PEER EXCHANGE (Feature Earmark)
// ============================================================================
// Currently, peer MAC addresses must be manually configured via the config UI.
// The discovery broadcast (MSG_DISCOVER / MSG_DISCOVER_ACK) already exchanges
// MAC addresses, roles, and hostnames between nodes.
//
// Planned enhancement: When a device discovers a peer via broadcast, it should:
//   1. Auto-store the peer's MAC in its own config (cfg.peer_mac) and persist
//      via saveConfig() — creating a "mesh registry" so all nodes know each
//      other without manual MAC entry.
//   2. The responding device should ALSO store the requester's MAC (from
//      info->src_addr in the MSG_DISCOVER handler) — making it bidirectional.
//   3. Add a "known_peers" array to DeviceConfig (config.h) to support
//      future multi-lane setups where more than 2 nodes need to communicate.
//   4. On boot, if cfg.peer_mac is 00:00:00:00:00:00, automatically run
//      discovery for 30s and auto-pair with the first compatible peer
//      (e.g., start gate auto-pairs with finish gate and vice versa).
//   5. Add a "FORGET PEERS" button in config.html to clear stored MACs.
//   6. Consider storing peers in a separate /peers.json on LittleFS rather
//      than in config.json, to avoid requiring a reboot when peers change.
//
// Key data already available in discovery:
//   - info->src_addr: requester's MAC (6 bytes)
//   - msg.role: "start" or "finish"
//   - msg.hostname: e.g. "hotwheels-A1B2"
//   - msg.senderId: device ID number
// ============================================================================

#include "espnow_comm.h"
#include "config.h"
#include <ArduinoJson.h>
#include <WiFi.h>

volatile RaceState raceState = IDLE;
int64_t clockOffset_us = 0;
bool peerConnected = false;
unsigned long lastPeerSeen = 0;

DiscoveredDevice discoveredDevices[MAX_DISCOVERED_DEVICES];
int discoveredCount = 0;
bool discoveryActive = false;
static unsigned long discoveryStartTime = 0;
static unsigned long lastDiscoveryBroadcast = 0;

uint64_t nowUs() {
  return esp_timer_get_time();
}

// Internal: ESP-NOW receive callback - dispatches to role-specific handler
static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(ESPMessage)) return;

  ESPMessage msg;
  memcpy(&msg, data, sizeof(msg));
  uint64_t receiveTime = nowUs();

  // Handle discovery messages regardless of role
  if (msg.type == MSG_DISCOVER) {
    // Someone is looking for devices - reply with our info
    sendDiscoveryAck(info->src_addr);
    return;
  }

  if (msg.type == MSG_DISCOVER_ACK) {
    // Add to discovered devices list
    bool found = false;
    for (int i = 0; i < discoveredCount; i++) {
      if (memcmp(discoveredDevices[i].mac, info->src_addr, 6) == 0) {
        // Update existing entry
        strncpy(discoveredDevices[i].role, msg.role, sizeof(discoveredDevices[i].role) - 1);
        strncpy(discoveredDevices[i].hostname, msg.hostname, sizeof(discoveredDevices[i].hostname) - 1);
        discoveredDevices[i].lastSeen = millis();
        found = true;
        break;
      }
    }
    if (!found && discoveredCount < MAX_DISCOVERED_DEVICES) {
      memcpy(discoveredDevices[discoveredCount].mac, info->src_addr, 6);
      strncpy(discoveredDevices[discoveredCount].role, msg.role, sizeof(discoveredDevices[discoveredCount].role) - 1);
      strncpy(discoveredDevices[discoveredCount].hostname, msg.hostname, sizeof(discoveredDevices[discoveredCount].hostname) - 1);
      discoveredDevices[discoveredCount].lastSeen = millis();
      discoveredCount++;
      Serial.printf("[DISCOVER] Found device: %s (%s) at %02X:%02X:%02X:%02X:%02X:%02X\n",
                    msg.hostname, msg.role,
                    info->src_addr[0], info->src_addr[1], info->src_addr[2],
                    info->src_addr[3], info->src_addr[4], info->src_addr[5]);
    }
    return;
  }

  // Track peer connectivity from PING/PONG
  if (msg.type == MSG_PING || msg.type == MSG_PONG) {
    peerConnected = true;
    lastPeerSeen = millis();
  }

  // Dispatch to role-specific handler
  if (strcmp(cfg.role, "finish") == 0) {
    onFinishGateESPNow(msg, receiveTime);
  } else if (strcmp(cfg.role, "start") == 0) {
    onStartGateESPNow(msg, receiveTime);
  }
}

void initESPNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init failed!");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  // Add configured peer if MAC is non-zero
  bool hasValidMac = false;
  for (int i = 0; i < 6; i++) {
    if (cfg.peer_mac[i] != 0) { hasValidMac = true; break; }
  }

  if (hasValidMac) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, cfg.peer_mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
      Serial.printf("[ESP-NOW] Peer added: %s\n", formatMac(cfg.peer_mac).c_str());
    }
  }

  // Add broadcast peer for discovery
  esp_now_peer_info_t broadcastPeer = {};
  memset(broadcastPeer.peer_addr, 0xFF, 6);
  broadcastPeer.channel = 0;
  broadcastPeer.encrypt = false;
  esp_now_add_peer(&broadcastPeer);

  // Start initial discovery
  discoveryActive = true;
  discoveryStartTime = millis();

  Serial.println("[ESP-NOW] Initialized");
}

void sendToPeer(uint8_t type, uint64_t timestamp, int64_t offset) {
  ESPMessage msg;
  msg.type = type;
  msg.senderId = cfg.device_id;
  msg.timestamp = timestamp;
  msg.offset = offset;
  strncpy(msg.role, cfg.role, sizeof(msg.role) - 1);
  msg.role[sizeof(msg.role) - 1] = '\0';
  strncpy(msg.hostname, cfg.hostname, sizeof(msg.hostname) - 1);
  msg.hostname[sizeof(msg.hostname) - 1] = '\0';

  esp_now_send(cfg.peer_mac, (uint8_t*)&msg, sizeof(msg));
}

void sendDiscoveryBroadcast() {
  ESPMessage msg;
  msg.type = MSG_DISCOVER;
  msg.senderId = cfg.device_id;
  msg.timestamp = nowUs();
  msg.offset = 0;
  strncpy(msg.role, cfg.role, sizeof(msg.role) - 1);
  msg.role[sizeof(msg.role) - 1] = '\0';
  strncpy(msg.hostname, cfg.hostname, sizeof(msg.hostname) - 1);
  msg.hostname[sizeof(msg.hostname) - 1] = '\0';

  uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(broadcastAddr, (uint8_t*)&msg, sizeof(msg));
}

void sendDiscoveryAck(const uint8_t* targetMac) {
  // Temporarily add peer if not already known
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, targetMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer); // OK if already exists

  ESPMessage msg;
  msg.type = MSG_DISCOVER_ACK;
  msg.senderId = cfg.device_id;
  msg.timestamp = nowUs();
  msg.offset = 0;
  strncpy(msg.role, cfg.role, sizeof(msg.role) - 1);
  msg.role[sizeof(msg.role) - 1] = '\0';
  strncpy(msg.hostname, cfg.hostname, sizeof(msg.hostname) - 1);
  msg.hostname[sizeof(msg.hostname) - 1] = '\0';

  esp_now_send(targetMac, (uint8_t*)&msg, sizeof(msg));
}

void discoveryLoop() {
  if (!discoveryActive) return;

  // Broadcast every 5 seconds during discovery
  if (millis() - lastDiscoveryBroadcast > 5000) {
    sendDiscoveryBroadcast();
    lastDiscoveryBroadcast = millis();
  }

  // Stop after 30 seconds
  if (millis() - discoveryStartTime > 30000) {
    discoveryActive = false;
    Serial.printf("[DISCOVER] Scan complete. Found %d device(s)\n", discoveredCount);
  }
}

String getDiscoveredDevicesJson() {
  StaticJsonDocument<1024> doc;
  JsonArray arr = doc.to<JsonArray>();

  for (int i = 0; i < discoveredCount; i++) {
    JsonObject dev = arr.createNestedObject();
    dev["mac"] = formatMac(discoveredDevices[i].mac);
    dev["role"] = discoveredDevices[i].role;
    dev["hostname"] = discoveredDevices[i].hostname;
    dev["lastSeen"] = (millis() - discoveredDevices[i].lastSeen) / 1000; // seconds ago
  }

  String output;
  serializeJson(doc, output);
  return output;
}

# Hot Wheels Race Gate - Physics Lab

A real-time race timing and physics measurement system for Hot Wheels (and other diecast) tracks, built on ESP32/ESP32-S3 microcontrollers. Designed as an educational STEM tool for science fairs, classrooms, and backyard racing.

Two IR break-beam sensors (start gate + finish gate) communicate over ESP-NOW to deliver microsecond-precision timing, real-time speed calculations, and physics data — all viewable on a live web dashboard from any device on your network.

## Features

### Race Timing
- **Microsecond-precision timing** via hardware interrupts on IR break-beam sensors
- **ESP-NOW peer-to-peer communication** between start and finish gates (sub-millisecond latency)
- **Clock synchronization** between devices for accurate cross-gate timing
- **Automatic race detection** — arm the system, release the car, results appear instantly

### Physics Calculations (Real-Time)
- Elapsed time (seconds)
- Real speed (mph)
- Scale speed (configurable ratio, default 1:64)
- Momentum (kg*m/s)
- Kinetic energy (Joules)

### Web Dashboard
- **Live updating** via WebSocket — no page refresh needed
- **Race state banner** — IDLE / ARMED / RACING / FINISHED with animations
- **Car Garage** — persistent database of cars with name, color, weight, and per-car stats (runs, best time, best speed)
- **Race History** — last 100 races with full physics data
- **CSV Export** — download race data for spreadsheets
- **Google Sheets Integration** — auto-upload results after each race
- **Version badge** — firmware and UI version displayed on every page

### Device Configuration
- **Web-based setup** — captive portal on first boot, no code editing required
- **WiFi or Standalone mode** — connect to your router, or create its own network
- **Pin configuration** — choose sensor and LED GPIOs via the web UI
- **Peer discovery** — automatically find other race gate devices on the network
- **WLED integration** — control LED strips for race state visual effects (idle, armed, racing, finished)
- **OTA updates** — flash new firmware over WiFi, no USB cable needed
- **Backup/Restore** — download and upload configuration as JSON

### Debug Console
- **Web-based serial monitor** — view device logs over WiFi at `/console`
- **Ring buffer capture** — 8KB of recent serial output, auto-refreshing
- **File browser** — inspect and manage files on the device filesystem
- **Device info** — IP, uptime, free memory, peer status at a glance

### Architecture
- **Unified firmware** — single codebase runs as start gate OR finish gate (configured via web UI)
- **Role-appropriate UI** — finish gate serves the full dashboard; start gate serves a lightweight status page
- **Embedded web pages** — HTML/CSS/JS compiled into firmware via PROGMEM (no LittleFS upload needed for web UI)
- **Persistent data** — garage, history, and config survive OTA firmware updates (stored in LittleFS)
- **ESP32 and ESP32-S3** compatible

## Hardware Requirements

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32 or ESP32-S3 dev board | 2 | One for start gate, one for finish gate |
| IR break-beam sensor | 2 | 5V transmitter + receiver pairs |
| Hot Wheels track | 1 | Any configuration |
| USB cable | 1 | For initial flash only (OTA after that) |
| Power supply | 2 | USB power for each ESP32 |

### Optional
| Component | Notes |
|-----------|-------|
| WLED-compatible LED strip | For race state lighting effects |
| Kitchen/gram scale | To weigh cars for physics calculations |

### Wiring

```
IR Sensor Signal → GPIO 4 (default, configurable)
IR Sensor VCC    → 5V / 3.3V (check your sensor)
IR Sensor GND    → GND
Status LED       → GPIO 2 (default, built-in LED)
```

## Installation

### Prerequisites
1. **Arduino IDE 2.x** (or PlatformIO)
2. **ESP32 Board Package** — Add `https://espressif.github.io/arduino-esp32/package_esp32_index.json` to Board Manager URLs
3. **Required Libraries** (install via Library Manager):
   - `WebSockets` by Markus Sattler
   - `ArduinoJson` by Benoit Blanchon

### Flash Firmware

1. Clone this repository:
   ```bash
   git clone https://github.com/YOUR_USERNAME/HotWheels_RaceGate.git
   ```

2. Open `HotWheels_RaceGate.ino` in Arduino IDE

3. Select your board:
   - **Board:** `ESP32S3 Dev Module` (or `ESP32 Dev Module`)
   - **Partition Scheme:** `Default 4MB with spiffs` (or larger)
   - **Upload Speed:** `921600`

4. Click **Upload**

5. **No LittleFS upload needed** — web pages are embedded in the firmware

### First Boot Setup

1. After flashing, the device creates a WiFi network: `HotWheels-Setup-XXXX`
2. Connect to it from your phone/laptop
3. A captive portal opens automatically (or navigate to `http://192.168.4.1`)
4. Configure:
   - **Network tab:** Select your WiFi network and enter the password
   - **Device tab:** Choose role (Start Gate or Finish Gate)
   - **Peer tab:** Note this device's MAC address; enter the partner's MAC
   - **Track tab:** Set track length in meters
5. Click **SAVE CONFIGURATION** — device reboots and connects to your WiFi

6. Repeat for the second device (opposite role)

7. Access the dashboard at `http://hotwheels.local` (or the device's IP address)

### OTA Updates (After Initial Flash)

Once configured, you never need a USB cable again:

1. In Arduino IDE, go to **Tools → Port** and select the network port (e.g., `hotwheels.local`)
2. Click **Upload** — firmware flashes over WiFi
3. Web pages update automatically (they're embedded in the firmware)

The OTA password defaults to `admin` — change it in the config page for security.

## Usage

### Racing

1. Open `http://hotwheels.local` on any device (phone, tablet, laptop)
2. **Add cars** to the Garage with name, color, and weight
3. **Select a car** by tapping it in the garage
4. Click **ARM SYSTEM**
5. Place car at the start gate
6. Release the car — timing starts automatically when the beam breaks
7. Results appear instantly on the dashboard

### Web Pages

| URL | Page | Description |
|-----|------|-------------|
| `/` | Dashboard | Live race data, garage, history (finish gate) or status page (start gate) |
| `/config` | Configuration | WiFi, pins, peer, track, WLED, OTA settings |
| `/console` | Debug Console | Serial log viewer, file browser, device info |

### API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/info` | GET | Device info (role, IP, uptime, heap) |
| `/api/version` | GET | Firmware version, build date, board type |
| `/api/config` | GET/POST | Read or write device configuration |
| `/api/garage` | GET/POST | Read or write car garage data |
| `/api/history` | GET/POST | Read or write race history |
| `/api/scan` | GET | Scan for WiFi networks |
| `/api/mac` | GET | Get device MAC address |
| `/api/discover` | GET | Discover peer devices on network |
| `/api/log` | GET/DELETE | Read or clear serial log buffer |
| `/api/files` | GET/POST/DELETE | File browser (list, read, write, delete) |
| `/api/backup` | GET | Download full config backup as JSON |
| `/api/restore` | POST | Restore config from backup JSON |
| `/api/reset` | POST | Factory reset (deletes config, reboots) |
| `/api/wled/info` | GET | Proxy: get WLED controller info |
| `/api/wled/effects` | GET | Proxy: list WLED effects |

## Project Structure

```
HotWheels_RaceGate/
├── HotWheels_RaceGate.ino   # Main entry point, WiFi, OTA, boot logic
├── config.h                  # Configuration struct, version constants
├── config.cpp                # Config load/save/validate, JSON serialization
├── web_server.h              # Web server + SerialTee ring buffer class
├── web_server.cpp            # HTTP routes, WebSocket, API handlers
├── espnow_comm.h             # ESP-NOW message types and protocol
├── espnow_comm.cpp           # ESP-NOW init, send, receive, clock sync
├── finish_gate.h             # Finish gate declarations
├── finish_gate.cpp           # Finish gate logic, timing, race results
├── start_gate.h              # Start gate declarations
├── start_gate.cpp            # Start gate logic, trigger detection
├── wled_integration.h        # WLED HTTP API declarations
├── wled_integration.cpp      # WLED effect control
├── html_index.h              # Dashboard HTML (PROGMEM embedded)
├── html_config.h             # Config page HTML (PROGMEM embedded)
├── html_console.h            # Console page HTML (PROGMEM embedded)
├── html_start_status.h       # Start gate status page (PROGMEM embedded)
├── data/                     # Source HTML files (edit these, then rebuild headers)
│   ├── index.html            # Dashboard source
│   ├── config.html           # Config page source
│   └── console.html          # Console page source
└── .gitignore
```

### How Web Pages Work

The HTML files in `data/` are the **source files** you edit. They get wrapped into C header files (`html_*.h`) as PROGMEM string literals and compiled directly into the firmware binary. This means:

- **OTA updates deliver code AND web UI** in one shot
- **No LittleFS upload needed** for HTML files
- **Runtime data** (config, garage, history) still lives in LittleFS and persists across updates

## Data Persistence

| File | Storage | Survives OTA? | Purpose |
|------|---------|---------------|---------|
| Web pages | PROGMEM (firmware) | Updated with firmware | Dashboard, config, console UI |
| `/config.json` | LittleFS | Yes | Device configuration |
| `/garage.json` | LittleFS | Yes | Car database |
| `/history.json` | LittleFS | Yes | Race history (last 100) |
| `/runs.csv` | LittleFS | Yes | Race log with full physics data |

## Security Notes

- **Change the default OTA password** (`admin`) in the config page before deploying
- **WiFi credentials** are stored in `/config.json` on the device filesystem
- The fallback WiFi defines in `HotWheels_RaceGate.ino` are blank by default — set them for your network if desired, but **never commit real credentials**
- The web interface has no authentication — it's designed for local network use

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Can't find device | Check `http://hotwheels.local` or find the IP in your router's DHCP table |
| Peer not connecting | Verify MAC addresses match on both devices (check Peer tab in config) |
| Timing seems wrong | Run clock sync from dashboard, check track length setting |
| OTA upload fails | Verify OTA password matches, ensure device is on same network |
| Web page not loading | The HTML is embedded in firmware — just reflash via OTA |
| Setup mode won't exit | Ensure you selected a WiFi network and the password is correct |

## Roadmap

### In Progress
- [ ] **Hub Scoreboard Device** — Waveshare ESP32 display as the central data authority with SD card storage
- [ ] **NFC Car Tagging** — Tap an NFC tag on each car to auto-select it before racing (non-destructive garage field addition)

### Planned
- [ ] **Testing Playlists** — Pre-defined science fair test protocols (e.g., "Weight vs Speed", "Angle vs Distance") with step-by-step prompts on screen
- [ ] **GitHub Version Check** — Compare running firmware against latest release, notify when updates are available
- [ ] **Multi-Lane Support** — Multiple finish sensors for parallel lane timing
- [ ] **Tournament Bracket Mode** — Head-to-head elimination bracket with automatic advancement
- [ ] **Audio Announcements** — Race countdown, results readout via connected speaker
- [ ] **Data Visualization** — Built-in charts and graphs for science fair presentations
- [ ] **Single Source of Truth Architecture** — All databases migrate to hub device; gate nodes become pure sensors

### Completed (v2.2.0)
- [x] Unified firmware (start + finish from one codebase)
- [x] Web-based captive portal configuration
- [x] ESP-NOW communication with clock synchronization
- [x] Embedded web UI (PROGMEM, no LittleFS upload needed)
- [x] Version tracking across firmware and web UI
- [x] `/api/version` endpoint for future update checking
- [x] Role-appropriate web pages (full dashboard vs status page)
- [x] Smart peer connection backoff (reduces radio spam when alone)
- [x] Full physics data in CSV log (including Joules)
- [x] WLED integration for visual race effects
- [x] Google Sheets auto-upload
- [x] OTA firmware updates
- [x] Backup/restore configuration
- [x] Web-based serial console with file browser

## License

MIT License — use it, modify it, teach with it.

## Credits

Built for the science fair by a family of racers, engineers, and curious kids.

Firmware assistance by Claude (Anthropic).

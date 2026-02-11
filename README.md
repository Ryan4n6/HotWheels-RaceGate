# The M.A.S.S. Trap

### **M**otion **A**nalysis & **S**peed **S**ystem

> *"Respect the Laws of Physics."*

**The M.A.S.S. Trap** is a forensic-grade physics laboratory disguised as a Hot Wheels track speedometer. It utilizes commercial LiDAR and enterprise IoT hardware to enforce the laws of **Mass**, **Momentum**, and **Kinetic Energy** on 1:64 scale traffic.

---

## The Case File (Lore)

This project was engineered by a former **Police Detective** and **Digital Forensics Instructor** to bridge the gap between law enforcement precision and childhood wonder.

* **The Name:** A nod to the creator's past life enforcing speed limits -- and his family name, **Mass**feller. It is a literal "Speed Trap" for Hot Wheels.
* **The Legacy:** Inspired by a grandfather who raced **Formula Vee** in the 70s/80s. In Formula Vee, raw horsepower didn't win races; **efficiency** and **momentum conservation** did.
* **The Mission:** To teach the next generation that **Science is Cool**, **Math is Power**, and while you can't break the laws of physics... you can certainly measure them.

---

## "Interceptor" Architecture (Hardware)

Unlike civilian-grade Arduino projects, The M.A.S.S. Trap runs on the **Interceptor Spec** -- a modified, high-performance stack designed for real-time telemetry, heavy data logging, and audio synthesis.

| Component | Spec | Role |
| --- | --- | --- |
| **The Engine** | **ESP32-S3 (N16R8)** | Modified 16MB Flash / 8MB PSRAM. Dual-core 240MHz "Interceptor" Class. |
| **The LiDAR** | **Benewake TF-Luna** | **UART Solid-State LiDAR.** Performs "Tech Inspection" staging and auto-arms the trap when a vehicle is detected. |
| **The Siren** | **MAX98357A I2S** | Digital Audio Amp. Broadcasts race audio effects -- arm chime, go tone, finish fanfare, new record alert. |
| **The Trap** | **Dual IR Break-beam** | Measures mid-track velocity delta to calculate drag coefficients. |
| **The Comms** | **ESP-NOW** | Encrypted, ultra-low latency (<1ms) tactical link between Start and Finish gates. |

---

## Command Center (The Dashboard)

The system hosts its own web app (no internet required), turning any phone, tablet, or laptop into the **M.A.S.S. Trap Command Center**.

* **Tacticool UI:** A dark mode interface styled with a police shield badge (Navy + Gold color scheme).
* **Real-Time Telemetry:** Instant display of Scale MPH, Actual MPH, Finish Time (ms), Momentum, KE, G-Force.
* **Suspect Tracking:**
  * **"Staged":** LiDAR confirms vehicle presence at the start gate.
  * **"Clocked":** Speed trap sensors capture mid-track velocity.
  * **"Booked":** Finish gate logs the final time and physics data.
* **The "Lead Sled" Leaderboard:** Tracks new records with an animated podium ceremony.
* **Mechanic's Log:** A digital notebook to correlate car modifications (graphite, weight) with performance gains.
* **Chart.js Visualization:** Speed vs Run and KE vs Weight scatter plots, auto-updating after each race.
* **Ghost Car Comparison:** Personal best delta display with new record/slower indicators.
* **Physics Explainer:** Live formula cards showing actual race values.
* **Speed Profile:** Start/mid/finish velocity comparison when speed trap data is available.

---

## Features

### Race Timing & Physics
- **Microsecond-precision timing** via hardware interrupts on IR break-beam sensors
- **ESP-NOW peer-to-peer communication** between gates (sub-millisecond latency)
- **Clock synchronization** between devices for accurate cross-gate timing
- **Automatic race detection** -- arm the system, release the car, results appear instantly
- **Real-time physics calculations:** elapsed time, speed (mph), scale speed, momentum (kg*m/s), kinetic energy (Joules), G-force
- **Mid-track velocity profiling** via optional speed trap node (demonstrates acceleration/deceleration)

### LiDAR Staging (Benewake TF-Luna)
- **Automatic car staging** -- solid-state LiDAR detects car at start gate
- **Auto-arm** -- car present > 1 second triggers automatic system arming
- **Dashboard indicator** -- live "LIDAR TARGET ACQUIRED" status on Command Center
- **Configurable threshold** -- adjustable detection distance via web UI
- **UART interface** -- 115200 baud, 9-byte frames with checksum validation
- **Signal strength filtering** -- rejects low-confidence readings
- **Fully optional** -- disabled by default, zero overhead when off

### Audio System (MAX98357A I2S)
- **Non-blocking WAV playback** via ESP32 I2S DMA ring buffer
- **Race event sounds** -- arm chime, countdown, go tone, finish fanfare, new record alert
- **Web-based upload** -- drag-and-drop WAV files to device via config page
- **Volume control** -- adjustable 0-21 levels via web UI
- **Fully optional** -- disabled by default, zero overhead when off

### Speed Trap Node (3rd ESP32)
- **Mid-track velocity measurement** -- dual IR sensors ~10cm apart
- **Microsecond ISR timing** -- hardware interrupt on both sensors
- **ESP-NOW integration** -- sends velocity data to finish gate automatically
- **Dedicated status page** -- shows last speed, peer status, sensor config

### Device Configuration
- **Web-based setup** -- captive portal on first boot, no code editing required
- **WiFi or Standalone mode** -- connect to your router, or create its own network
- **Pin configuration** -- all GPIO assignments configurable via web UI
- **Peer discovery** -- automatically find other M.A.S.S. Trap devices on the network
- **WLED integration** -- LED strip effects for race states (idle, armed, racing, finished)
- **OTA updates** -- flash new firmware over WiFi, no USB cable needed
- **Full system snapshot** -- one-click backup/restore of config + garage + history
- **Clone mode** -- restore snapshot to a new device (strips network identity)

### Debug Console
- **Web-based serial monitor** -- view device logs over WiFi at `/console`
- **Ring buffer capture** -- 8KB of recent serial output, auto-refreshing
- **File browser** -- inspect, edit, and manage files on the device filesystem
- **Device info** -- IP, uptime, free memory, peer status at a glance

### Architecture
- **Unified firmware** -- single codebase runs as start gate, finish gate, or speed trap
- **Role-appropriate UI** -- finish gate serves full dashboard; start gate and speed trap serve lightweight status pages
- **Embedded web pages** -- HTML/CSS/JS compiled into firmware via PROGMEM (no LittleFS upload needed for web UI)
- **Custom 16MB partition** -- 3MB app (with OTA) + 10MB LittleFS for data and audio files
- **Graceful degradation** -- audio, LiDAR, and speed trap are optional; existing gates work without them

---

## Deployment Protocols

### 1. Hardware Prep

- **Target Board:** ESP32-S3 Dev Module (Enable `USB CDC On Boot`)
- **Flash Size:** Select **16MB (128Mb)**
- **PSRAM:** Select **OPI PSRAM**

### 2. Partitioning (Critical)

The "Interceptor" requires a custom memory map to fit the audio assets and logs.

- Place `partitions.csv` in the sketch root
- Select **"Huge APP (3MB No OTA/1MB SPIFFS)"** in Arduino IDE (the `.csv` file overrides this to provide **~10MB** for the filesystem)

### 3. Prerequisites

1. **Arduino IDE 2.x** (or PlatformIO)
2. **ESP32 Board Package** -- Add `https://espressif.github.io/arduino-esp32/package_esp32_index.json` to Board Manager URLs
3. **Required Libraries** (install via Library Manager):
   - `WebSockets` by Markus Sattler
   - `ArduinoJson` by Benoit Blanchon

> No external library needed for the TF-Luna LiDAR -- it uses the built-in HardwareSerial UART driver.

### 4. Wiring

**IR Sensor (Start / Finish Gate)**
```
IR Sensor Signal  ->  GPIO 4 (default, configurable)
IR Sensor VCC     ->  5V / 3.3V (check your sensor)
IR Sensor GND     ->  GND
Status LED        ->  GPIO 48 (NeoPixel on ESP32-S3 DevKit)
```

**Benewake TF-Luna LiDAR (optional)**
```
VCC  ->  5V
GND  ->  GND
TX   ->  ESP32 GPIO 39 (RX) (default, configurable)
RX   ->  ESP32 GPIO 38 (TX) (default, configurable)
```
*Note: The TF-Luna runs in UART mode at 115200 baud.*

**MAX98357A I2S Amplifier (optional)**
```
BCLK  ->  GPIO 15 (default, configurable)
LRC   ->  GPIO 16 (default, configurable)
DIN   ->  GPIO 17 (default, configurable)
VIN   ->  5V
GND   ->  GND
```

**Speed Trap Node (dual IR sensors)**
```
Sensor 1 Signal  ->  GPIO 4 (default, configurable)
Sensor 2 Signal  ->  GPIO 5 (default, configurable)
Sensor spacing: ~10cm apart on track
```

### 5. Flash & Race

1. Clone this repository:
   ```bash
   git clone https://github.com/Ryan4n6/HotWheels-RaceGate.git
   ```
2. Open `HotWheels_RaceGate.ino` in Arduino IDE
3. Upload Firmware
4. After flashing, connect to WiFi SSID: `MASSTrap-Setup-XXXX`
5. Configure role, WiFi, pins via the captive portal at `http://192.168.4.1`
6. Access the Command Center at `http://masstrap.local`
7. **Respect the laws of physics.**

### OTA Updates (After Initial Flash)

Once configured, you never need a USB cable again:

1. In Arduino IDE, go to **Tools -> Port** and select the network port (e.g., `masstrap.local`)
2. Click **Upload** -- firmware flashes over WiFi
3. Web pages update automatically (they're embedded in the firmware)

The OTA password defaults to `admin` -- change it in the config page for security.

---

## Web Pages

| URL | Page | Description |
|-----|------|-------------|
| `/` | Command Center | Live race data, garage, history (finish gate) or status page (start/speed trap) |
| `/config` | Configuration | WiFi, pins, peer, track, audio, LiDAR, WLED, OTA settings |
| `/console` | Debug Console | Serial log viewer, file browser, device info |

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/info` | GET | Device info (role, IP, uptime, heap, peer status) |
| `/api/version` | GET | Firmware version, build date, board type |
| `/api/config` | GET/POST | Read or write device configuration |
| `/api/garage` | GET/POST | Read or write car garage data |
| `/api/history` | GET/POST | Read or write race history |
| `/api/scan` | GET | Scan for WiFi networks |
| `/api/mac` | GET | Get device MAC address |
| `/api/discover` | GET | Discover peer devices on network |
| `/api/log` | GET/DELETE | Read or clear serial log buffer |
| `/api/files` | GET/POST/DELETE | File browser (list, read, write, delete) |
| `/api/system/backup` | GET | Full system snapshot (config + garage + history) |
| `/api/system/restore` | POST | Restore system snapshot (with optional clone mode) |
| `/api/lidar/status` | GET | Live LiDAR readout (state, distance, threshold) |
| `/api/audio/list` | GET | List audio files on device |
| `/api/audio/upload` | POST | Upload WAV file to device |
| `/api/audio/test` | POST | Play a test sound |
| `/api/wled/info` | GET | Proxy: get WLED controller info |
| `/api/wled/effects` | GET | Proxy: list WLED effects |
| `/api/backup` | GET | Legacy single-config backup |
| `/api/restore` | POST | Legacy single-config restore |
| `/api/reset` | POST | Factory reset (deletes config, reboots) |

## Project Structure

```
HotWheels_RaceGate/
├── HotWheels_RaceGate.ino    # Main entry point, WiFi, OTA, boot logic
├── partitions.csv             # Custom 16MB partition table (3MB app + 10MB LittleFS)
├── config.h                   # Configuration struct, version constants, PROJECT_NAME
├── config.cpp                 # Config load/save/validate, JSON serialization
├── web_server.h               # Web server + SerialTee ring buffer class
├── web_server.cpp             # HTTP routes, WebSocket, API handlers
├── espnow_comm.h              # ESP-NOW message types and protocol (12 message types)
├── espnow_comm.cpp            # ESP-NOW init, send, receive, clock sync, discovery
├── finish_gate.h              # Finish gate declarations
├── finish_gate.cpp            # Finish gate logic, timing, race results, speed data
├── start_gate.h               # Start gate declarations
├── start_gate.cpp             # Start gate logic, trigger, LiDAR auto-arm
├── speed_trap.h               # Speed trap declarations
├── speed_trap.cpp             # Dual ISR velocity measurement, ESP-NOW send
├── lidar_sensor.h             # LiDAR sensor declarations (TF-Luna)
├── lidar_sensor.cpp           # TF-Luna UART init, frame parsing, presence state machine
├── audio_manager.h            # Audio playback declarations
├── audio_manager.cpp          # I2S init, WAV loading, non-blocking DMA playback
├── wled_integration.h         # WLED HTTP API declarations
├── wled_integration.cpp       # WLED effect control, auto-sleep
├── html_index.h               # Dashboard HTML (PROGMEM embedded)
├── html_config.h              # Config page HTML (PROGMEM embedded)
├── html_console.h             # Console page HTML (PROGMEM embedded)
├── html_start_status.h        # Start gate status page (PROGMEM embedded)
├── html_speedtrap_status.h    # Speed trap status page (PROGMEM embedded)
├── html_chartjs.h             # Chart.js v4.4.7 library (PROGMEM embedded)
├── data/                      # Source HTML files (edit these, then rebuild headers)
│   ├── index.html             # Command Center dashboard source
│   ├── config.html            # Config page source
│   ├── console.html           # Console page source
│   └── speedtrap_status.html  # Speed trap status page source
├── README.md
├── CHANGELOG.md
└── .gitignore
```

## Data Persistence

| File | Storage | Survives OTA? | Purpose |
|------|---------|---------------|---------|
| Web pages | PROGMEM (firmware) | Updated with firmware | Dashboard, config, console UI |
| `/config.json` | LittleFS | Yes | Device configuration |
| `/garage.json` | LittleFS | Yes | Car database with mechanic's notes |
| `/history.json` | LittleFS | Yes | Race history (last 100) |
| `/runs.csv` | LittleFS | Yes | Race log with full physics data |
| `/*.wav` | LittleFS | Yes | Audio effect files |

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Can't find device | Check `http://masstrap.local` or find the IP in your router's DHCP table |
| Peer not connecting | Verify MAC addresses match on both devices (check Peer tab in config) |
| Timing seems wrong | Run clock sync from dashboard, check track length setting |
| OTA upload fails | Verify OTA password matches, ensure device is on same network |
| Firmware too large | Ensure Flash Size is set to 16MB -- the custom partitions.csv gives 3MB per app slot |
| No LiDAR readings | Enable LiDAR in config, verify UART pins (TX/RX), check 5V power to TF-Luna |
| No audio output | Enable audio in config, verify I2S pins, upload WAV files |
| Speed trap no data | Verify speed trap node is connected (check peer status), check sensor spacing |

## Roadmap

### Planned
- [ ] **Hub Scoreboard Device** -- Waveshare ESP32 display as the central data authority with SD card storage
- [ ] **NFC Car Tagging** -- Tap an NFC tag on each car to auto-select it before racing
- [ ] **Testing Playlists** -- Pre-defined science fair test protocols (e.g., "Weight vs Speed", "Angle vs Distance")
- [ ] **GitHub Version Check** -- Compare running firmware against latest release, notify when updates available
- [ ] **Multi-Lane Support** -- Multiple finish sensors for parallel lane timing
- [ ] **Tournament Bracket Mode** -- Head-to-head elimination bracket with automatic advancement

### Completed (v2.4.0)
- [x] **M.A.S.S. Trap Rebrand** -- Full project rebrand with police shield/badge themed Command Center
- [x] **Custom 16MB Partition Table** -- 3MB app (with OTA) + 10MB LittleFS for audio and data
- [x] **Full System Snapshot** -- One-click backup/restore of config + garage + history with clone mode
- [x] **TF-Luna LiDAR** -- UART solid-state LiDAR for car staging detection and auto-arm
- [x] **Audio System** -- MAX98357A I2S amplifier with non-blocking WAV playback via DMA
- [x] **Speed Trap Node** -- 3rd ESP32 role with dual IR sensors for mid-track velocity measurement
- [x] **Leaderboard Podium Overlay** -- Animated ranking display after each race
- [x] **Mechanic's Log** -- Per-car notes with timestamps for tracking modifications
- [x] **Speed Profile** -- Start/mid/finish velocity comparison when speed trap data available

### Completed (v2.3.0)
- [x] Chart.js Data Visualization, Ghost Car Comparison, Physics Explainer Panel
- [x] LED Visualizer Bar, API Authentication, WLED Auto-Sleep, XSS Protection

### Completed (v2.2.0)
- [x] Unified firmware, Captive portal config, ESP-NOW, Embedded web UI
- [x] WLED integration, Google Sheets, OTA, Backup/restore, Web console

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for detailed release notes.

## License

MIT License -- use it, modify it, teach with it.

---

### Dedication

*Built for my sons. May you always find the thrill in the data.*

*In memory of the Formula Vee days. We are still racing, Dad.*

---

Firmware assistance by Claude (Anthropic).

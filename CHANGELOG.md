# Changelog

All notable changes to The M.A.S.S. Trap (Motion Analysis & Speed System) will be documented in this file.

## [2.4.0] - 2026-02-11

### The M.A.S.S. Trap Rebrand
The project has been renamed from "Hot Wheels Race Gate" to **The M.A.S.S. Trap** (Motion Analysis & Speed System). The name carries three meanings: "speed trap" from law enforcement, "mass" from physics, and "Massfeller" -- the family name. The entire UI has been redesigned with a police shield/badge authority aesthetic using navy + gold colors.

### Added
- **PlatformIO Support** -- Full `platformio.ini` configuration for the pioarduino platform (Arduino Core 3.x / ESP-IDF 5.x). One-command build, flash, OTA, and LittleFS upload. All board settings (flash size, PSRAM, partition table) are defined in config — no manual dropdown hunting.
- **Custom 16MB Partition Table** (`partitions.csv`) -- Fixes the v2.3.0 compile blocker where firmware (1.53MB) exceeded the default 1.25MB partition. New layout: 3MB per app slot (with OTA) + ~9.9MB LittleFS for data and audio files + 64KB coredump partition for crash diagnostics. PlatformIO handles this automatically; Arduino IDE auto-detects the CSV when Flash Size is set to 16MB.
- **Full System Snapshot** -- One-click backup/restore of the entire device state:
  - `GET /api/system/backup` returns a JSON envelope with config, garage, and history
  - `POST /api/system/restore` restores all three files and reboots
  - Clone mode (`?skip_network=true`) strips WiFi/hostname for flashing to a new device
  - UI in config page with Export/Import/Clone buttons
- **Audio System** (`audio_manager.h/.cpp`) -- MAX98357A I2S amplifier integration:
  - Non-blocking WAV playback via ESP32 I2S DMA ring buffer
  - Format: 8-bit unsigned, 16kHz mono WAV files stored in LittleFS
  - Sound slots: `armed.wav`, `go.wav`, `finish.wav`, `record.wav`, `countdown3/2/1.wav`
  - Web upload: `POST /api/audio/upload`, list: `GET /api/audio/list`, test: `POST /api/audio/test`
  - Configurable pins (BCLK, LRC, DOUT) and volume (0-21) via web UI
  - Disabled by default -- zero overhead when `audio_enabled = false`
- **LiDAR Car Presence Detection** (`lidar_sensor.h/.cpp`) -- Benewake TF-Luna solid-state LiDAR:
  - UART interface at 115200 baud, 9-byte frames with checksum validation
  - Signal strength filtering rejects low-confidence readings
  - State machine: NO_CAR -> CAR_STAGED -> CAR_LAUNCHED
  - Auto-arm: car present > 1 second at start gate triggers automatic ARM
  - WebSocket broadcast: `{"lidar": {"state": "staged", "distance_mm": 32}}`
  - Dashboard indicator: "LIDAR TARGET ACQUIRED" near ARM button
  - Configurable UART pins (RX=GPIO39, TX=GPIO38) and threshold distance
  - Disabled by default -- no UART init when `lidar_enabled = false`
  - No external library needed -- uses ESP32 built-in HardwareSerial
- **Speed Trap Node** (`speed_trap.h/.cpp`) -- New "speedtrap" ESP32 role:
  - Dual IR break-beam sensors ~10cm apart for mid-track velocity measurement
  - Microsecond ISR timing on both sensor pins
  - Calculates instantaneous velocity and sends to finish gate via ESP-NOW
  - New ESP-NOW message types: `MSG_SPEED_DATA` (10) and `MSG_SPEED_ACK` (11)
  - Dedicated status page at `/` showing last speed, peer status, sensor config
  - Dashboard "Speed Profile" card comparing start/mid/finish velocity
  - New PROGMEM page: `html_speedtrap_status.h`
- **Leaderboard Podium Overlay** -- Animated CSS overlay after each race:
  - Shows car ranking among all tested cars
  - "SUSPECT RANKED #1 -- NEW TRACK RECORD" style callouts
  - Slide-in animation, 3-second hold, fade out
- **Mechanic's Log** -- Per-car notes with timestamps:
  - Each car in garage JSON can have a `notes` array
  - Expandable accordion under each car in the garage section
  - "Add Note" button for tracking modifications (weight changes, lube, etc.)
  - Correlate modifications with performance trends
- **Speed Profile Card** -- Start/mid/finish velocity comparison visualization when speed trap data is available

### Changed
- **Full UI Rebrand** -- Every web page updated with M.A.S.S. Trap branding:
  - Dashboard: "M.A.S.S. TRAP COMMAND CENTER" with CSS shield badge
  - Color scheme: Dark navy (#1a1a2e) + gold (#d4af37) + white accents
  - State banners: AWAITING SUSPECTS / TRAP SET / IN PURSUIT / SUSPECT CLOCKED
  - Config page: "M.A.S.S. Trap - Configuration"
  - Console page: "M.A.S.S. Trap - Console"
  - Start gate: "M.A.S.S. TRAP SYSTEM"
- **Boot sequence** -- Prints M.A.S.S. Trap banner with version and "ALL SYSTEMS OPERATIONAL"
- **Default hostname** -- `masstrap` (was `hotwheels`), mDNS at `http://masstrap.local`
- **AP naming** -- `MASSTrap-Setup-XXXX` (was `HotWheels-Setup-XXXX`)
- **Config struct** -- 13 new fields for audio, LiDAR, and speed trap configuration
- **ESP-NOW protocol** -- Extended from 9 to 12 message types (added speed data, speed ack, reserved)
- **DeviceConfig role** -- Now supports `"start"`, `"finish"`, and `"speedtrap"`
- **Version bump** -- Firmware and Web UI: 2.3.0 -> 2.4.0
- **All PROGMEM headers rebuilt** -- `html_index.h`, `html_config.h`, `html_console.h`, `html_start_status.h`, `html_speedtrap_status.h`

### Changed (Repo Structure)
- **Flattened repository** -- All source files moved from `MASS_Trap/` subfolder to repo root. Clone and build directly — no nested directories.
- **Git tags for releases** -- Version history tracked via git tags (e.g., `v2.4.0`) and GitHub Releases instead of version folders.
- **Improved .gitignore** -- Added PlatformIO directories, `*.env` for secrets.

### Technical Notes
- **PlatformIO is the recommended build system.** `platformio.ini` pins all board settings (`board_build.flash_size = 16MB`, `qio_opi` memory type, partition table) — eliminates the class of Arduino IDE misconfiguration bugs where wrong dropdown selections cause flash detection failures.
- All new hardware features (audio, LiDAR, speed trap) are **off by default** and guarded by config flags. Existing two-gate setups work identically to v2.3.0 with no configuration changes needed.
- Custom partition table includes a 64KB coredump partition for crash diagnostics.
- Changing partitions erases LittleFS -- use system snapshot to backup before upgrading from a different partition layout.
- LiDAR sensor uses UART (HardwareSerial) instead of I2C -- no external library required. TF-Luna 9-byte frame protocol with checksum validation and signal strength filtering.
- Audio uses ESP32 built-in I2S driver (`driver/i2s.h`) -- no external library required.
- Only 2 external libraries needed: `WebSockets` and `ArduinoJson` (was 3 in early development -- removed `Adafruit_VL53L0X` dependency).
- Config JSON uses `"lidar"` key (backwards-compatible: reads old `"tof"` key if present).
- Resource budget: PROGMEM ~340KB (of 3MB), LittleFS ~250KB with audio (of ~9.9MB), heap ~130KB free (of 320KB).

---

## [2.3.0] - 2026-02-11

### Added
- **Chart.js Data Visualization** -- Two live-updating charts on the dashboard:
  - Speed vs Run Number (line chart) -- tracks speed progression across races
  - Kinetic Energy vs Weight (scatter plot) -- visualizes KE = 1/2 mv^2 relationship
  - Charts populate from history on page load and update in real time after each race
  - Chart.js v4.4.7 embedded as PROGMEM (~206KB) -- works offline in standalone mode
- **Ghost Car Comparison** -- After each race, shows time delta vs the active car's personal best:
  - Green "NEW PERSONAL BEST!" with negative delta for new records
  - Red "+X.XXXs" for slower runs
  - "FIRST RUN!" indicator for a car's inaugural race
- **Physics Explainer Panel** -- Collapsible "HOW THE MATH WORKS" section with four formula cards:
  - Speed: v = d/t
  - Momentum: p = mv
  - Kinetic Energy: KE = 1/2 mv^2
  - G-Force: a = v/t / 9.8
  - Each formula shows actual values from the last race, substituted live
- **LED Visualizer Bar** -- Animated CSS bar below the state banner mirrors WLED states:
  - Idle: Breathing green glow
  - Armed: Fast-pulsing red (F1 start lights)
  - Racing: Yellow chase effect
  - Finished: Rainbow gradient flow
- **API Authentication** -- Simple `X-API-Key` header check on destructive endpoints:
  - Protected: POST `/api/config`, POST `/api/restore`, POST `/api/reset`, POST/DELETE `/api/files`, DELETE `/api/log`
  - Reuses existing OTA password as the API key
  - Read-only endpoints and dashboard data (garage/history) remain open
  - Config page auto-includes key from OTA password field
  - Console page prompts for key on first destructive action, caches in localStorage
- **WLED Auto-Sleep Timer** -- Turns off WLED after 5 minutes of inactivity
  - Automatically wakes WLED back to idle state on next race activity
  - New functions: `setWLEDOff()`, `checkWLEDTimeout()`, `resetWLEDActivity()`

### Changed
- **WLED Timeout Reduction** -- `setWLEDState()` HTTP timeout reduced from 500ms to 100ms (LAN is fast; don't block race timing). WLED proxy endpoints reduced from 2000ms to 1000ms.
- **Version bump** -- Firmware and Web UI version: 2.2.0 -> 2.3.0

### Fixed
- **XSS Vulnerability** -- Car names rendered via `innerHTML` could execute injected scripts. All user-generated content now uses a safe `setCurrentCarDisplay()` helper with `textContent` and DOM API.

### Technical Notes
- Total PROGMEM flash usage: ~315KB (up from ~94KB in v2.2.0). ESP32-S3 with 8MB flash has ample headroom.
- Chart.js served at `/chart.min.js` with 24-hour cache header for browser caching efficiency.
- `window.onload` converted to async to ensure history is loaded before chart initialization.
- Ghost comparison reads previous best BEFORE `updateGarageStats()` overwrites it (correct sequencing).

---

## [2.2.0] - 2026-02-10

### Added
- Unified firmware -- single codebase runs as start gate OR finish gate (configured via web UI)
- Web-based captive portal configuration on first boot
- ESP-NOW peer-to-peer communication with clock synchronization
- Embedded web UI via PROGMEM -- OTA updates deliver code AND web pages in one shot
- Version tracking: `FIRMWARE_VERSION`, `WEB_UI_VERSION`, `BUILD_DATE`, `BUILD_TIME`
- `/api/version` endpoint for future GitHub update checking
- Role-appropriate web pages -- finish gate serves full dashboard; start gate serves lightweight status page
- Smart peer connection backoff -- ping rate drops from 2s to 10s when peer is disconnected
- Full physics data in CSV log (Run, Car, Weight, Time, Speed, Scale Speed, Momentum, KE)
- WLED integration for visual race state effects (idle, armed, racing, finished)
- Google Sheets auto-upload after each race via webhook
- OTA firmware updates with configurable password
- Backup/restore device configuration as JSON
- Web-based serial console with file browser at `/console`
- Car Garage with persistent database (name, color, weight, per-car stats)
- Race History with last 100 races and full physics data
- CSV export for spreadsheet analysis

### Fixed
- ESP32-S3 compile error (`HWCDC*` to `HardwareSerial*`) -- changed to `Print*` base class
- WiFi mode dropdown defaulting issue in captive portal
- Clock sync spam when peer disconnected
- Start gate serving full dashboard (now has its own lightweight status page)
- Duplicate race history entries (only finish gate records data now)
- Joules column missing from history table display

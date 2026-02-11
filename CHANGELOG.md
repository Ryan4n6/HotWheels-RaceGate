# Changelog

All notable changes to the Hot Wheels Race Gate project will be documented in this file.

## [2.3.0] - 2026-02-11

### Added
- **Chart.js Data Visualization** — Two live-updating charts on the dashboard:
  - Speed vs Run Number (line chart) — tracks speed progression across races
  - Kinetic Energy vs Weight (scatter plot) — visualizes KE = ½mv² relationship
  - Charts populate from history on page load and update in real time after each race
  - Chart.js v4.4.7 embedded as PROGMEM (~206KB) — works offline in standalone mode
- **Ghost Car Comparison** — After each race, shows time delta vs the active car's personal best:
  - Green "NEW PERSONAL BEST!" with negative delta for new records
  - Red "+X.XXXs" for slower runs
  - "FIRST RUN!" indicator for a car's inaugural race
- **Physics Explainer Panel** — Collapsible "HOW THE MATH WORKS" section with four formula cards:
  - Speed: v = d/t
  - Momentum: p = mv
  - Kinetic Energy: KE = ½mv²
  - G-Force: a = v/t ÷ 9.8
  - Each formula shows actual values from the last race, substituted live
- **LED Visualizer Bar** — Animated CSS bar below the state banner mirrors WLED states:
  - Idle: Breathing green glow
  - Armed: Fast-pulsing red (F1 start lights)
  - Racing: Yellow chase effect
  - Finished: Rainbow gradient flow
- **API Authentication** — Simple `X-API-Key` header check on destructive endpoints:
  - Protected: POST `/api/config`, POST `/api/restore`, POST `/api/reset`, POST/DELETE `/api/files`, DELETE `/api/log`
  - Reuses existing OTA password as the API key
  - Read-only endpoints and dashboard data (garage/history) remain open
  - Config page auto-includes key from OTA password field
  - Console page prompts for key on first destructive action, caches in localStorage
- **WLED Auto-Sleep Timer** — Turns off WLED after 5 minutes of inactivity
  - Automatically wakes WLED back to idle state on next race activity
  - New functions: `setWLEDOff()`, `checkWLEDTimeout()`, `resetWLEDActivity()`

### Changed
- **WLED Timeout Reduction** — `setWLEDState()` HTTP timeout reduced from 500ms to 100ms (LAN is fast; don't block race timing). WLED proxy endpoints reduced from 2000ms to 1000ms.
- **Version bump** — Firmware and Web UI version: 2.2.0 → 2.3.0

### Fixed
- **XSS Vulnerability** — Car names rendered via `innerHTML` could execute injected scripts. All user-generated content now uses a safe `setCurrentCarDisplay()` helper with `textContent` and DOM API.

### Technical Notes
- Total PROGMEM flash usage: ~315KB (up from ~94KB in v2.2.0). ESP32-S3 with 8MB flash has ample headroom.
- Chart.js served at `/chart.min.js` with 24-hour cache header for browser caching efficiency.
- `window.onload` converted to async to ensure history is loaded before chart initialization.
- Ghost comparison reads previous best BEFORE `updateGarageStats()` overwrites it (correct sequencing).

---

## [2.2.0] - 2026-02-10

### Added
- Unified firmware — single codebase runs as start gate OR finish gate (configured via web UI)
- Web-based captive portal configuration on first boot
- ESP-NOW peer-to-peer communication with clock synchronization
- Embedded web UI via PROGMEM — OTA updates deliver code AND web pages in one shot
- Version tracking: `FIRMWARE_VERSION`, `WEB_UI_VERSION`, `BUILD_DATE`, `BUILD_TIME`
- `/api/version` endpoint for future GitHub update checking
- Role-appropriate web pages — finish gate serves full dashboard; start gate serves lightweight status page
- Smart peer connection backoff — ping rate drops from 2s to 10s when peer is disconnected
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
- ESP32-S3 compile error (`HWCDC*` to `HardwareSerial*`) — changed to `Print*` base class
- WiFi mode dropdown defaulting issue in captive portal
- Clock sync spam when peer disconnected
- Start gate serving full dashboard (now has its own lightweight status page)
- Duplicate race history entries (only finish gate records data now)
- Joules column missing from history table display

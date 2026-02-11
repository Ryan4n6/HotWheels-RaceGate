#ifndef HTML_CONFIG_H
#define HTML_CONFIG_H

#include <Arduino.h>

const char CONFIG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <meta name="fw-version" content="2.2.0">
  <title>Hot Wheels Race Gate - Setup</title>
  <style>
    :root {
      --hw-orange: #FF4400;
      --hw-blue: #007ACC;
      --hw-yellow: #FFCC00;
      --hw-green: #28a745;
      --hw-red: #dc3545;
      --bg: #1a1a1a;
      --card: #ffffff;
      --text: #333;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, var(--hw-blue) 0%, #005599 100%);
      min-height: 100vh;
      padding: 15px;
      color: var(--text);
    }
    .container { max-width: 800px; margin: 0 auto; }
    .header {
      text-align: center;
      color: white;
      margin-bottom: 20px;
      text-shadow: 2px 2px 0px #000;
    }
    .header h1 { font-size: 2rem; font-weight: 900; font-style: italic; }
    .header p { color: var(--hw-yellow); font-weight: bold; margin-top: 5px; }

    /* Tabs */
    .tabs {
      display: flex;
      flex-wrap: wrap;
      gap: 4px;
      margin-bottom: 20px;
    }
    .tab {
      padding: 10px 16px;
      background: rgba(255,255,255,0.2);
      color: white;
      border: none;
      border-radius: 8px 8px 0 0;
      cursor: pointer;
      font-weight: 700;
      font-size: 0.85rem;
    }
    .tab.active {
      background: white;
      color: var(--hw-blue);
    }
    .tab-content {
      display: none;
      background: white;
      border-radius: 0 12px 12px 12px;
      padding: 25px;
      box-shadow: 0 6px 0 rgba(0,0,0,0.1);
    }
    .tab-content.active { display: block; }

    /* Form elements */
    label {
      display: block;
      font-weight: 700;
      font-size: 0.85rem;
      text-transform: uppercase;
      color: #666;
      margin-bottom: 6px;
      margin-top: 16px;
    }
    label:first-child { margin-top: 0; }
    input, select {
      width: 100%;
      padding: 12px;
      border: 2px solid #ddd;
      border-radius: 6px;
      font-size: 1rem;
      font-family: inherit;
    }
    input:focus, select:focus {
      outline: none;
      border-color: var(--hw-blue);
    }
    .hint { font-size: 0.8rem; color: #999; margin-top: 4px; }

    /* Role cards */
    .role-cards { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin-top: 10px; }
    .role-card {
      border: 3px solid #ddd;
      border-radius: 12px;
      padding: 20px;
      text-align: center;
      cursor: pointer;
      transition: all 0.2s;
    }
    .role-card:hover { border-color: var(--hw-blue); transform: translateY(-2px); }
    .role-card.selected { border-color: var(--hw-green); background: #f0fff4; border-width: 4px; }
    .role-card h3 { font-size: 1.3rem; margin-bottom: 8px; }
    .role-card p { font-size: 0.85rem; color: #666; }
    .role-icon { font-size: 2.5rem; margin-bottom: 10px; }

    /* Buttons */
    .btn {
      padding: 14px 28px;
      border: none;
      border-radius: 8px;
      font-size: 1.1rem;
      font-weight: 900;
      cursor: pointer;
      text-transform: uppercase;
      box-shadow: 0 4px 0 rgba(0,0,0,0.2);
      transition: all 0.1s;
      width: 100%;
      margin-top: 20px;
    }
    .btn:active { transform: translateY(4px); box-shadow: none; }
    .btn-primary { background: var(--hw-blue); color: white; }
    .btn-success { background: var(--hw-green); color: white; }
    .btn-danger { background: var(--hw-red); color: white; }
    .btn-warning { background: var(--hw-yellow); color: black; }
    .btn-sm { padding: 8px 16px; font-size: 0.85rem; width: auto; margin-top: 10px; }

    /* Discovered devices */
    .device-list { margin-top: 10px; }
    .device-item {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 10px;
      border: 2px solid #eee;
      border-radius: 8px;
      margin-bottom: 8px;
      cursor: pointer;
    }
    .device-item:hover { border-color: var(--hw-green); background: #f8fff8; }
    .device-role { font-weight: 700; color: var(--hw-blue); }
    .device-mac { font-family: monospace; font-size: 0.9rem; color: #666; }

    /* Info box */
    .info-box {
      background: #e8f4fd;
      border: 2px solid var(--hw-blue);
      border-radius: 8px;
      padding: 12px;
      font-size: 0.9rem;
      margin-top: 10px;
    }
    .info-box.warning {
      background: #fff3cd;
      border-color: var(--hw-yellow);
    }

    /* Grid */
    .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }

    /* Status bar */
    .status-bar {
      position: fixed;
      bottom: 0;
      left: 0;
      right: 0;
      background: rgba(0,0,0,0.9);
      color: white;
      padding: 12px 20px;
      text-align: center;
      font-weight: 700;
      z-index: 1000;
      display: none;
    }
    .status-bar.show { display: block; }
    .status-bar.success { background: var(--hw-green); }
    .status-bar.error { background: var(--hw-red); }

    .version-badge {
      position: fixed;
      bottom: 8px;
      right: 8px;
      background: rgba(0,0,0,0.7);
      color: #888;
      padding: 4px 10px;
      border-radius: 6px;
      font-size: 0.7rem;
      z-index: 1001;
      font-family: monospace;
    }

    @media (max-width: 600px) {
      .role-cards { grid-template-columns: 1fr; }
      .grid-2 { grid-template-columns: 1fr; }
      .tabs { gap: 2px; }
      .tab { padding: 8px 10px; font-size: 0.75rem; }
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>HOT WHEELS RACE GATE</h1>
      <p>DEVICE CONFIGURATION</p>
      <div style="margin-top:8px;">
        <a href="/" style="color:var(--hw-yellow); text-decoration:none; margin-right:15px; font-weight:700;">Dashboard</a>
        <a href="/console" style="color:var(--hw-yellow); text-decoration:none; font-weight:700;">Debug Console</a>
      </div>
    </div>

    <!-- Tabs -->
    <div class="tabs">
      <button class="tab active" onclick="showTab('network')">Network</button>
      <button class="tab" onclick="showTab('device')">Device</button>
      <button class="tab" onclick="showTab('pins')">Pins</button>
      <button class="tab" onclick="showTab('peer')">Peer</button>
      <button class="tab" onclick="showTab('track')">Track</button>
      <button class="tab" onclick="showTab('integrations')">Integrations</button>
      <button class="tab" onclick="showTab('maintenance')">Maintenance</button>
    </div>

    <!-- Tab: Network -->
    <div id="tab-network" class="tab-content active">
      <label>WiFi Network</label>
      <div style="display:flex; gap:8px;">
        <select id="wifiScan" style="flex:1;" onchange="document.getElementById('wifiSSID').value=this.value; if(this.value) document.getElementById('networkMode').value='wifi';">
          <option value="">-- Scan for networks --</option>
        </select>
        <button class="btn btn-primary btn-sm" onclick="scanWifi()">Scan</button>
      </div>
      <input type="text" id="wifiSSID" placeholder="WiFi SSID (or type manually)">

      <label>WiFi Password</label>
      <div style="display:flex; gap:8px;">
        <input type="password" id="wifiPass" placeholder="WiFi Password" style="flex:1;">
        <button class="btn btn-warning btn-sm" onclick="togglePass('wifiPass')">Show</button>
      </div>

      <label>Hostname</label>
      <input type="text" id="hostname" placeholder="hotwheels" value="hotwheels">
      <div class="hint">Access device at http://[hostname].local</div>

      <label>Network Mode</label>
      <select id="networkMode">
        <option value="wifi" selected>Connect to WiFi (recommended)</option>
        <option value="standalone">Standalone AP (no router needed)</option>
      </select>
      <div class="hint">Standalone mode creates its own WiFi network. Use when no router is available.</div>
    </div>

    <!-- Tab: Device -->
    <div id="tab-device" class="tab-content">
      <label>Device Role</label>
      <div class="role-cards">
        <div class="role-card" id="role-start" onclick="selectRole('start')">
          <div class="role-icon" id="icon-start">&#x1F7E2;</div>
          <h3>START GATE</h3>
          <p>Detects car release and sends start signal to finish gate</p>
        </div>
        <div class="role-card selected" id="role-finish" onclick="selectRole('finish')">
          <div class="role-icon" id="icon-finish">&#x1F3C1;</div>
          <h3>FINISH GATE</h3>
          <p>Detects car crossing finish line, calculates speed and physics</p>
        </div>
      </div>
      <input type="hidden" id="role" value="finish">

      <label>Device ID</label>
      <input type="number" id="deviceId" value="1" min="1" max="254">
      <div class="hint">Unique number for this device (1-254). Use different IDs for each device.</div>
    </div>

    <!-- Tab: Pins -->
    <div id="tab-pins" class="tab-content">
      <div class="info-box">
        <strong>ESP32-S3 Safe GPIOs:</strong> 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 35-48<br>
        <strong>Avoid:</strong> GPIO 0 (boot), 6-11 (flash SPI)
      </div>

      <label id="sensorLabel">IR Sensor Pin (Finish Line)</label>
      <select id="sensorPin">
        <option value="4" selected>GPIO 4 (default)</option>
        <option value="1">GPIO 1</option>
        <option value="2">GPIO 2</option>
        <option value="3">GPIO 3</option>
        <option value="5">GPIO 5</option>
        <option value="12">GPIO 12</option>
        <option value="13">GPIO 13</option>
        <option value="14">GPIO 14</option>
        <option value="15">GPIO 15</option>
        <option value="16">GPIO 16</option>
        <option value="17">GPIO 17</option>
        <option value="18">GPIO 18</option>
        <option value="19">GPIO 19</option>
        <option value="20">GPIO 20</option>
        <option value="21">GPIO 21</option>
      </select>
      <div class="hint">Connect your IR break-beam sensor signal wire to this pin</div>

      <label>Status LED Pin</label>
      <select id="ledPin">
        <option value="2" selected>GPIO 2 (default, built-in LED)</option>
        <option value="1">GPIO 1</option>
        <option value="3">GPIO 3</option>
        <option value="4">GPIO 4</option>
        <option value="5">GPIO 5</option>
        <option value="12">GPIO 12</option>
        <option value="13">GPIO 13</option>
        <option value="14">GPIO 14</option>
        <option value="15">GPIO 15</option>
        <option value="16">GPIO 16</option>
        <option value="17">GPIO 17</option>
        <option value="18">GPIO 18</option>
      </select>
    </div>

    <!-- Tab: Peer -->
    <div id="tab-peer" class="tab-content">
      <label>This Device's MAC Address</label>
      <input type="text" id="myMac" readonly style="background:#f0f0f0; font-family:monospace;">
      <div class="hint">Give this MAC address to your partner device</div>

      <label>Partner Device MAC Address</label>
      <input type="text" id="peerMac" placeholder="XX:XX:XX:XX:XX:XX" style="font-family:monospace;">
      <div class="hint">Enter the MAC address of the other gate, or use discovery below</div>

      <button class="btn btn-primary btn-sm" onclick="discoverDevices()">Scan for Devices</button>
      <div id="deviceList" class="device-list">
        <div class="hint">Click "Scan for Devices" to find nearby race gate devices</div>
      </div>
    </div>

    <!-- Tab: Track -->
    <div id="tab-track" class="tab-content">
      <label>Track Length (meters)</label>
      <input type="number" id="trackLength" value="2.0" step="0.01" min="0.1" max="100">
      <div class="hint">Measure from start gate sensor to finish gate sensor</div>

      <label>Scale Factor</label>
      <input type="number" id="scaleFactor" value="64" min="1" max="1000">
      <div class="hint">1:64 for standard Hot Wheels. Scale speed = real speed x this factor.</div>
    </div>

    <!-- Tab: Integrations -->
    <div id="tab-integrations" class="tab-content">
      <label>Google Sheets Webhook URL</label>
      <input type="text" id="sheetsUrl" placeholder="https://script.google.com/macros/s/.../exec">
      <div class="hint">Optional: race data auto-uploads to Google Sheets after each run</div>

      <hr style="margin: 20px 0; border: 1px solid #eee;">

      <h3 style="color: var(--hw-blue); margin-bottom: 10px;">WLED Integration</h3>
      <label>WLED Controller Host</label>
      <div style="display:flex; gap:8px;">
        <input type="text" id="wledHost" placeholder="wled-1.local" style="flex:1;">
        <button class="btn btn-primary btn-sm" onclick="testWled()">Test</button>
      </div>
      <div class="hint">Hostname or IP of your WLED controller</div>

      <div id="wledStatus" class="info-box" style="display:none;"></div>

      <button class="btn btn-warning btn-sm" onclick="loadWledEffects()">Load Effects from WLED</button>

      <div class="grid-2">
        <div>
          <label>Idle Effect</label>
          <select id="wledIdle" class="wled-effect"><option value="0">0 - Solid</option></select>
        </div>
        <div>
          <label>Armed Effect</label>
          <select id="wledArmed" class="wled-effect"><option value="28">28 - Breathe</option></select>
        </div>
        <div>
          <label>Racing Effect</label>
          <select id="wledRacing" class="wled-effect"><option value="49">49 - Chase Rainbow</option></select>
        </div>
        <div>
          <label>Finished Effect</label>
          <select id="wledFinished" class="wled-effect"><option value="11">11 - Rainbow</option></select>
        </div>
      </div>
    </div>

    <!-- Tab: Maintenance -->
    <div id="tab-maintenance" class="tab-content">
      <div id="deviceInfo" class="info-box">Loading device info...</div>

      <label>OTA Update Password</label>
      <div style="display:flex; gap:8px;">
        <input type="password" id="otaPassword" placeholder="Default: admin" style="flex:1;">
        <button class="btn btn-warning btn-sm" onclick="togglePass('otaPassword')">Show</button>
      </div>
      <div class="hint">Used for over-the-air firmware updates. Change from the default for security.</div>

      <div class="grid-2" style="margin-top: 20px;">
        <button class="btn btn-primary" onclick="downloadBackup()">Download Backup</button>
        <button class="btn btn-warning" onclick="document.getElementById('restoreFile').click()">
          Restore Backup
        </button>
        <input type="file" id="restoreFile" accept=".json" style="display:none;" onchange="restoreBackup(this)">
      </div>

      <button class="btn btn-danger" onclick="factoryReset()" style="margin-top: 20px;">
        Factory Reset
      </button>
      <div class="hint">Deletes all configuration and race data. Device will reboot into setup mode.</div>
    </div>

    <!-- Save Button -->
    <button class="btn btn-success" onclick="saveConfig()">
      SAVE CONFIGURATION
    </button>
  </div>

  <!-- Status Bar -->
  <div id="statusBar" class="status-bar"></div>
  <div class="version-badge" id="versionBadge">v--</div>

  <script>
    // ====================================================================
    // TAB NAVIGATION
    // ====================================================================
    function showTab(name) {
      document.querySelectorAll('.tab-content').forEach(t => t.classList.remove('active'));
      document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
      document.getElementById('tab-' + name).classList.add('active');
      event.target.classList.add('active');
    }

    // ====================================================================
    // ROLE SELECTION
    // ====================================================================
    let selectedRole = 'finish';
    function selectRole(role) {
      selectedRole = role;
      document.getElementById('role').value = role;
      document.querySelectorAll('.role-card').forEach(c => c.classList.remove('selected'));
      document.getElementById('role-' + role).classList.add('selected');
      // Update pin label
      const label = document.getElementById('sensorLabel');
      label.textContent = role === 'start' ? 'Trigger Pin (Break-Beam Sensor)' : 'IR Sensor Pin (Finish Line)';
    }

    // ====================================================================
    // WIFI SCAN
    // ====================================================================
    async function scanWifi() {
      const sel = document.getElementById('wifiScan');
      sel.innerHTML = '<option value="">Scanning...</option>';
      try {
        const resp = await fetch('/api/scan');
        const networks = await resp.json();
        sel.innerHTML = '<option value="">-- Select network --</option>';
        networks.forEach(n => {
          const sec = n.secure ? ' (secured)' : ' (open)';
          sel.innerHTML += `<option value="${n.ssid}">${n.ssid} (${n.rssi}dBm)${sec}</option>`;
        });
      } catch (e) {
        sel.innerHTML = '<option value="">Scan failed - try again</option>';
      }
    }

    // ====================================================================
    // PASSWORD TOGGLE
    // ====================================================================
    function togglePass(id) {
      const input = document.getElementById(id);
      input.type = input.type === 'password' ? 'text' : 'password';
    }

    // ====================================================================
    // DEVICE DISCOVERY
    // ====================================================================
    async function discoverDevices() {
      const list = document.getElementById('deviceList');
      list.innerHTML = '<div class="hint">Scanning for devices... (this takes a few seconds)</div>';
      try {
        const resp = await fetch('/api/discover');
        const devices = await resp.json();
        if (devices.length === 0) {
          list.innerHTML = '<div class="hint">No devices found. Make sure other devices are powered on.</div>';
          return;
        }
        list.innerHTML = devices.map(d => `
          <div class="device-item" onclick="document.getElementById('peerMac').value='${d.mac}'">
            <div>
              <span class="device-role">${d.role.toUpperCase()}</span>
              <span> - ${d.hostname}</span>
            </div>
            <span class="device-mac">${d.mac}</span>
          </div>
        `).join('');
      } catch (e) {
        list.innerHTML = '<div class="hint">Discovery failed. Try again.</div>';
      }
    }

    // ====================================================================
    // WLED
    // ====================================================================
    async function testWled() {
      const host = document.getElementById('wledHost').value.trim();
      const status = document.getElementById('wledStatus');
      if (!host) { status.style.display = 'block'; status.textContent = 'Enter a WLED host first'; return; }
      status.style.display = 'block';
      status.textContent = 'Testing connection...';
      try {
        const resp = await fetch('/api/wled/info');
        if (resp.ok) {
          const info = await resp.json();
          status.textContent = 'Connected! WLED v' + (info.ver || '?') + ' - ' + (info.name || 'unnamed');
          status.className = 'info-box';
        } else {
          status.textContent = 'Could not reach WLED controller';
          status.className = 'info-box warning';
        }
      } catch (e) {
        status.textContent = 'Connection failed: ' + e.message;
        status.className = 'info-box warning';
      }
    }

    async function loadWledEffects() {
      try {
        const resp = await fetch('/api/wled/effects');
        if (!resp.ok) { alert('Could not load effects. Test WLED connection first.'); return; }
        const effects = await resp.json();
        const selects = document.querySelectorAll('.wled-effect');
        selects.forEach(sel => {
          const currentVal = sel.value;
          sel.innerHTML = effects.map((name, idx) =>
            `<option value="${idx}" ${idx == currentVal ? 'selected' : ''}>${idx} - ${name}</option>`
          ).join('');
        });
        showStatus('Effects loaded from WLED!', 'success');
      } catch (e) {
        alert('Failed to load effects: ' + e.message);
      }
    }

    // ====================================================================
    // SAVE CONFIG
    // ====================================================================
    async function saveConfig() {
      const sensorPin = parseInt(document.getElementById('sensorPin').value);
      const ledPin = parseInt(document.getElementById('ledPin').value);
      if (sensorPin === ledPin) {
        alert('Sensor pin and LED pin cannot be the same!');
        return;
      }

      // Ensure network mode has a valid value (some captive portal WebViews
      // don't properly default select elements that were never interacted with)
      const networkMode = document.getElementById('networkMode').value || 'wifi';

      const config = {
        configured: true,
        version: 1,
        network: {
          wifi_ssid: document.getElementById('wifiSSID').value.trim(),
          wifi_pass: document.getElementById('wifiPass').value,
          hostname: document.getElementById('hostname').value.trim() || 'hotwheels',
          mode: networkMode
        },
        device: {
          role: selectedRole,
          id: parseInt(document.getElementById('deviceId').value) || 1
        },
        pins: {
          sensor_pin: sensorPin,
          led_pin: ledPin
        },
        peer: {
          mac: document.getElementById('peerMac').value.trim() || '00:00:00:00:00:00'
        },
        track: {
          length_m: parseFloat(document.getElementById('trackLength').value) || 2.0,
          scale_factor: parseInt(document.getElementById('scaleFactor').value) || 64
        },
        integrations: {
          google_sheets_url: document.getElementById('sheetsUrl').value.trim(),
          wled_host: document.getElementById('wledHost').value.trim(),
          wled_effects: {
            idle: parseInt(document.getElementById('wledIdle').value) || 0,
            armed: parseInt(document.getElementById('wledArmed').value) || 28,
            racing: parseInt(document.getElementById('wledRacing').value) || 49,
            finished: parseInt(document.getElementById('wledFinished').value) || 11
          }
        },
        ota: {
          password: document.getElementById('otaPassword').value || 'admin'
        }
      };

      try {
        showStatus('Saving configuration...', '');
        const resp = await fetch('/api/config', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(config)
        });
        const result = await resp.json();
        if (resp.ok) {
          showStatus('Config saved! Device rebooting...', 'success');
          // Try to reconnect after reboot
          setTimeout(() => {
            const host = config.network.hostname || 'hotwheels';
            showStatus('Trying to reach http://' + host + '.local ...', '');
            setTimeout(() => {
              window.location.href = 'http://' + host + '.local/';
            }, 5000);
          }, 3000);
        } else {
          showStatus('Error: ' + (result.error || 'Save failed'), 'error');
        }
      } catch (e) {
        showStatus('Error: ' + e.message, 'error');
      }
    }

    // ====================================================================
    // BACKUP / RESTORE / RESET
    // ====================================================================
    function downloadBackup() {
      window.location.href = '/api/backup';
    }

    async function restoreBackup(input) {
      const file = input.files[0];
      if (!file) return;
      try {
        const text = await file.text();
        JSON.parse(text); // Validate JSON
        showStatus('Restoring configuration...', '');
        const resp = await fetch('/api/restore', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: text
        });
        if (resp.ok) {
          showStatus('Config restored! Device rebooting...', 'success');
        } else {
          showStatus('Restore failed', 'error');
        }
      } catch (e) {
        alert('Invalid JSON file');
      }
    }

    async function factoryReset() {
      if (!confirm('This will delete ALL configuration and race data. Continue?')) return;
      if (!confirm('Are you sure? Type YES to confirm.')) return;
      try {
        await fetch('/api/reset', { method: 'POST' });
        showStatus('Factory reset! Device rebooting into setup mode...', 'success');
      } catch (e) {
        showStatus('Reset failed', 'error');
      }
    }

    // ====================================================================
    // STATUS BAR
    // ====================================================================
    function showStatus(msg, type) {
      const bar = document.getElementById('statusBar');
      bar.textContent = msg;
      bar.className = 'status-bar show' + (type ? ' ' + type : '');
      if (type) setTimeout(() => bar.classList.remove('show'), 5000);
    }

    // ====================================================================
    // LOAD EXISTING CONFIG
    // ====================================================================
    async function loadExistingConfig() {
      try {
        // Load MAC address
        const macResp = await fetch('/api/mac');
        const macData = await macResp.json();
        document.getElementById('myMac').value = macData.mac;

        // Load device info
        const infoResp = await fetch('/api/info');
        const info = await infoResp.json();
        // Update Chrome tab title with device role
        if (info.role && info.role !== 'unconfigured') {
          const roleName = info.role.charAt(0).toUpperCase() + info.role.slice(1);
          document.title = roleName + ' Gate - Setup';
        }
        document.getElementById('deviceInfo').innerHTML =
          `<strong>Firmware:</strong> v${info.firmware || '?'}<br>
           <strong>Role:</strong> ${(info.role || 'unconfigured').toUpperCase()}<br>
           <strong>IP:</strong> ${info.ip || 'N/A'}<br>
           <strong>Uptime:</strong> ${info.uptime_s || 0}s<br>
           <strong>Free Heap:</strong> ${info.free_heap || 0} bytes`;

        // Load existing config
        const cfgResp = await fetch('/api/config');
        const cfg = await cfgResp.json();
        if (cfg.configured) {
          // Populate form with existing values
          if (cfg.network) {
            document.getElementById('wifiSSID').value = cfg.network.wifi_ssid || '';
            document.getElementById('wifiPass').value = cfg.network.wifi_pass || '';
            document.getElementById('hostname').value = cfg.network.hostname || 'hotwheels';
            document.getElementById('networkMode').value = cfg.network.mode || 'wifi';
          }
          if (cfg.device) {
            selectRole(cfg.device.role || 'finish');
            document.getElementById('deviceId').value = cfg.device.id || 1;
          }
          if (cfg.pins) {
            document.getElementById('sensorPin').value = cfg.pins.sensor_pin || 4;
            document.getElementById('ledPin').value = cfg.pins.led_pin || 2;
          }
          if (cfg.peer) {
            document.getElementById('peerMac').value = cfg.peer.mac || '';
          }
          if (cfg.track) {
            document.getElementById('trackLength').value = cfg.track.length_m || 2.0;
            document.getElementById('scaleFactor').value = cfg.track.scale_factor || 64;
          }
          if (cfg.integrations) {
            document.getElementById('sheetsUrl').value = cfg.integrations.google_sheets_url || '';
            document.getElementById('wledHost').value = cfg.integrations.wled_host || '';
            if (cfg.integrations.wled_effects) {
              document.getElementById('wledIdle').value = cfg.integrations.wled_effects.idle || 0;
              document.getElementById('wledArmed').value = cfg.integrations.wled_effects.armed || 28;
              document.getElementById('wledRacing').value = cfg.integrations.wled_effects.racing || 49;
              document.getElementById('wledFinished').value = cfg.integrations.wled_effects.finished || 11;
            }
          }
          if (cfg.ota) {
            document.getElementById('otaPassword').value = cfg.ota.password || 'admin';
          }
        }
      } catch (e) {
        console.log('Could not load config (may be first boot):', e);
      }
    }

    // Fetch and display firmware version
    async function loadVersion() {
      try {
        const resp = await fetch('/api/version');
        const v = await resp.json();
        document.getElementById('versionBadge').textContent = 'FW v' + v.firmware + ' | UI v' + v.web_ui;
      } catch (e) {
        const meta = document.querySelector('meta[name="fw-version"]');
        if (meta) document.getElementById('versionBadge').textContent = 'v' + meta.content;
      }
    }

    // Initialize
    window.onload = () => {
      loadExistingConfig();
      loadVersion();
      scanWifi();
    };
  </script>
</body>
</html>

)rawliteral";

#endif // HTML_CONFIG_H

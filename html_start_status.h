#ifndef HTML_START_STATUS_H
#define HTML_START_STATUS_H

#include <Arduino.h>

// Minimal status page for the START GATE device.
// No garage, no history, no race data recording — just connection
// status and device info. The finish gate is the single source of truth.
static const char START_STATUS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <meta name="fw-version" content="2.2.0">
  <title>Start Gate - Hot Wheels</title>
  <style>
    :root {
      --hw-orange: #FF4400;
      --hw-blue: #007ACC;
      --hw-yellow: #FFCC00;
      --hw-green: #28a745;
      --hw-red: #dc3545;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, var(--hw-green) 0%, #1a7a3a 100%);
      min-height: 100vh;
      padding: 15px;
      color: #333;
    }
    .container { max-width: 600px; margin: 0 auto; }
    .header {
      text-align: center;
      color: white;
      margin-bottom: 20px;
      text-shadow: 3px 3px 0px #000;
    }
    .header h1 { font-size: 2.5rem; font-weight: 900; font-style: italic; }
    .header p { color: var(--hw-yellow); font-weight: bold; margin-top: 5px; font-size: 1.1rem; }
    .nav-links {
      text-align: center;
      margin-bottom: 15px;
    }
    .nav-links a {
      color: var(--hw-yellow);
      text-decoration: none;
      font-weight: 700;
      margin: 0 10px;
    }

    .state-banner {
      background: #000;
      color: var(--hw-yellow);
      padding: 25px;
      border-radius: 10px;
      text-align: center;
      font-size: 2.5rem;
      font-weight: 900;
      border: 4px solid var(--hw-green);
      margin-bottom: 20px;
      text-transform: uppercase;
      letter-spacing: 3px;
      transition: all 0.3s;
    }
    .state-banner.armed { background: var(--hw-green); color: white; animation: pulse 1s infinite; }
    .state-banner.racing { background: var(--hw-red); color: white; animation: flash 0.5s infinite; }
    .state-banner.finished { background: white; color: var(--hw-green); }

    @keyframes pulse { 0%,100% { transform: scale(1); } 50% { transform: scale(1.03); } }
    @keyframes flash { 0%,100% { opacity: 1; } 50% { opacity: 0.7; } }

    .card {
      background: white;
      border-radius: 12px;
      padding: 20px;
      margin-bottom: 15px;
      box-shadow: 0 4px 0 rgba(0,0,0,0.1);
      border-left: 6px solid var(--hw-green);
    }
    .card h3 { color: var(--hw-green); font-size: 0.85rem; text-transform: uppercase; margin-bottom: 10px; }
    .info-row { display: flex; justify-content: space-between; padding: 6px 0; border-bottom: 1px solid #f0f0f0; }
    .info-row:last-child { border-bottom: none; }
    .info-label { color: #888; font-weight: 600; font-size: 0.85rem; }
    .info-value { font-weight: 700; font-family: monospace; }
    .info-value.connected { color: var(--hw-green); }
    .info-value.disconnected { color: var(--hw-red); }

    .message {
      background: rgba(0,0,0,0.15);
      color: white;
      border-radius: 10px;
      padding: 20px;
      text-align: center;
      font-size: 0.95rem;
      line-height: 1.6;
    }
    .message strong { color: var(--hw-yellow); }

    .version-badge {
      position: fixed;
      bottom: 8px;
      right: 8px;
      background: rgba(0,0,0,0.7);
      color: #888;
      padding: 4px 10px;
      border-radius: 6px;
      font-size: 0.7rem;
      z-index: 1000;
      font-family: monospace;
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>START GATE</h1>
      <p>HOT WHEELS RACE SYSTEM</p>
    </div>

    <div class="nav-links">
      <a href="/config">Settings</a>
      <a href="/console">Debug Console</a>
    </div>

    <div id="stateBanner" class="state-banner">INITIALIZING...</div>

    <div class="card">
      <h3>Device Status</h3>
      <div class="info-row">
        <span class="info-label">Finish Gate</span>
        <span class="info-value" id="peerStatus">--</span>
      </div>
      <div class="info-row">
        <span class="info-label">State</span>
        <span class="info-value" id="raceState">--</span>
      </div>
      <div class="info-row">
        <span class="info-label">IP Address</span>
        <span class="info-value" id="ipAddr">--</span>
      </div>
      <div class="info-row">
        <span class="info-label">Uptime</span>
        <span class="info-value" id="uptime">--</span>
      </div>
      <div class="info-row">
        <span class="info-label">Free Memory</span>
        <span class="info-value" id="freeHeap">--</span>
      </div>
    </div>

    <div class="message">
      This is the <strong>Start Gate</strong> sensor node.<br>
      Race data, garage, and history are managed by the <strong>Finish Gate</strong>.<br>
      Open the Finish Gate dashboard to view race results.
    </div>
  </div>

  <div class="version-badge" id="versionBadge">v--</div>

  <script>
    // WebSocket for live state updates
    let ws = null;
    function connectWS() {
      ws = new WebSocket('ws://' + window.location.hostname + ':81');
      ws.onopen = () => {};
      ws.onclose = () => setTimeout(connectWS, 2000);
      ws.onmessage = (e) => {
        try {
          const d = JSON.parse(e.data);
          // Update state banner
          const banner = document.getElementById('stateBanner');
          banner.className = 'state-banner ' + (d.state || '').toLowerCase();
          banner.textContent = d.state || 'UNKNOWN';
          document.getElementById('raceState').textContent = d.state || '--';
          // Peer status
          const peer = document.getElementById('peerStatus');
          if (d.connected) {
            peer.textContent = 'CONNECTED';
            peer.className = 'info-value connected';
          } else {
            peer.textContent = 'DISCONNECTED';
            peer.className = 'info-value disconnected';
          }
        } catch (err) {}
      };
    }

    // Fetch device info
    async function loadInfo() {
      try {
        const resp = await fetch('/api/info');
        const info = await resp.json();
        document.getElementById('ipAddr').textContent = info.ip || '--';
        document.getElementById('uptime').textContent = (info.uptime_s || 0) + 's';
        document.getElementById('freeHeap').textContent = (info.free_heap || 0) + ' bytes';
      } catch (e) {}
    }

    // Version
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

    window.onload = () => {
      connectWS();
      loadInfo();
      loadVersion();
      // Refresh device info every 30 seconds
      setInterval(loadInfo, 30000);
    };
  </script>
</body>
</html>
)rawliteral";

#endif

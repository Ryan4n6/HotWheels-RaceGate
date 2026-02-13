/* =================================================================
   M.A.S.S. TRAP — UNIFIED JAVASCRIPT v2.5.0
   Motion Analysis & Speed System
   Shared logic: Theme, WebSocket, API, Navigation
   ================================================================= */

// ====================================================================
// CONSTANTS (mirrors firmware config.h for UI-side consistency)
// ====================================================================
var MPS_TO_MPH = 2.23694;
var MPS_TO_KPH = 3.6;
var METERS_TO_FEET = 3.28084;
var MAX_RACE_DURATION_US = 60000000;  // 60s sanity check

// ====================================================================
// UNIT SYSTEM — "imperial" (mph, ft) or "metric" (km/h, m)
// Loaded from firmware config via WebSocket or /api/config
// ====================================================================
var massUnits = localStorage.getItem('mass_units') || 'imperial';

function setUnits(u) {
  massUnits = u;
  localStorage.setItem('mass_units', u);
}

// Speed: convert m/s to display unit
function formatSpeed(mps, decimals) {
  if (mps == null || isNaN(mps)) return '--';
  decimals = (decimals !== undefined) ? decimals : 1;
  if (massUnits === 'metric') return (mps * MPS_TO_KPH).toFixed(decimals);
  return (mps * MPS_TO_MPH).toFixed(decimals);
}

// Speed unit label
function speedUnit() {
  return massUnits === 'metric' ? 'km/h' : 'mph';
}

// Distance: always stored in meters, display in ft or m
function formatDistance(meters, decimals) {
  if (meters == null || isNaN(meters)) return '--';
  decimals = (decimals !== undefined) ? decimals : 2;
  if (massUnits === 'metric') return meters.toFixed(decimals);
  return (meters * METERS_TO_FEET).toFixed(decimals);
}

// Distance unit label
function distanceUnit() {
  return massUnits === 'metric' ? 'm' : 'ft';
}

// Distance input: display-unit value → meters (for saving to config)
function distanceToMeters(displayVal) {
  if (massUnits === 'metric') return displayVal;
  return displayVal / METERS_TO_FEET;  // ft → m
}

// Distance input: meters → display-unit value (for showing in form)
function metersToDisplay(meters) {
  if (massUnits === 'metric') return meters;
  return meters * METERS_TO_FEET;  // m → ft
}

// Convert mph value from WS to current unit (when mps not available)
function mphToDisplaySpeed(mph) {
  if (mph == null || isNaN(mph)) return '--';
  if (massUnits === 'metric') return (mph * 1.60934).toFixed(1);  // mph → km/h
  return mph.toFixed(1);
}

// ====================================================================
// THEME ENGINE
// ====================================================================
function getTheme() {
  return localStorage.getItem('mass_theme') || 'interceptor';
}

function setTheme(theme) {
  document.documentElement.setAttribute('data-theme', theme);
  localStorage.setItem('mass_theme', theme);
  var sel = document.getElementById('themePicker');
  if (sel) sel.value = theme;
}

function initTheme() {
  setTheme(getTheme());
}

// ====================================================================
// NAVIGATION — Mark active link based on current path
// ====================================================================
function initNav() {
  var path = window.location.pathname;
  var links = document.querySelectorAll('.mass-nav-list a');
  for (var i = 0; i < links.length; i++) {
    var href = links[i].getAttribute('href');
    if (href === path || (href === '/' && (path === '' || path === '/index.html' || path === '/dashboard.html'))) {
      links[i].classList.add('active');
    } else {
      links[i].classList.remove('active');
    }
  }
}

// ====================================================================
// WEBSOCKET CONNECTION (shared across dashboard + status pages)
// ====================================================================
var ws = null;
var wsReconnectTimer = null;
var wsConnected = false;
var wsMessageHandlers = [];

function connectWebSocket() {
  var wsUrl = 'ws://' + window.location.hostname + ':81';
  ws = new WebSocket(wsUrl);

  ws.onopen = function() {
    wsConnected = true;
    updateConnectionBadge(true);
    clearTimeout(wsReconnectTimer);
  };

  ws.onclose = function() {
    wsConnected = false;
    updateConnectionBadge(false);
    wsReconnectTimer = setTimeout(connectWebSocket, 2000);
  };

  ws.onerror = function() {};

  ws.onmessage = function(event) {
    try {
      var data = JSON.parse(event.data);
      for (var i = 0; i < wsMessageHandlers.length; i++) {
        wsMessageHandlers[i](data);
      }
    } catch (e) {
      console.error('WS parse error:', e);
    }
  };
}

function onWsMessage(handler) {
  wsMessageHandlers.push(handler);
}

function wsSend(data) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(data));
  } else {
    alert('Not connected to M.A.S.S. Trap!');
  }
}

function updateConnectionBadge(connected) {
  var el = document.getElementById('connBadge');
  if (!el) return;
  el.className = 'conn-badge ' + (connected ? 'connected' : 'disconnected');
  el.textContent = connected ? 'LINKED' : 'OFFLINE';
}

// ====================================================================
// API AUTHENTICATION HELPER
// ====================================================================
function getApiKey() {
  if (!window._apiKey) {
    window._apiKey = localStorage.getItem('mass_api_key') || '';
  }
  if (!window._apiKey) {
    window._apiKey = prompt('Enter API key (OTA password):') || '';
    if (window._apiKey) localStorage.setItem('mass_api_key', window._apiKey);
  }
  return window._apiKey;
}

function authHeaders(extra) {
  return Object.assign({ 'X-API-Key': getApiKey() }, extra || {});
}

// ====================================================================
// VERSION BADGE
// ====================================================================
function loadVersion() {
  fetch('/api/version').then(function(r) { return r.json(); }).then(function(v) {
    var badge = document.getElementById('versionBadge');
    if (badge) badge.textContent = 'FW v' + v.firmware + ' | UI v' + v.web_ui;
  }).catch(function() {
    var meta = document.querySelector('meta[name="fw-version"]');
    var badge = document.getElementById('versionBadge');
    if (meta && badge) badge.textContent = 'v' + meta.content;
  });
}

// ====================================================================
// UTILITY HELPERS
// ====================================================================
function formatBytes(b) {
  if (!b) return '--';
  if (b > 1024 * 1024) return (b / 1024 / 1024).toFixed(1) + ' MB';
  if (b > 1024) return (b / 1024).toFixed(1) + ' KB';
  return b + ' B';
}

function formatUptime(s) {
  if (!s && s !== 0) return '--';
  var h = Math.floor(s / 3600);
  var m = Math.floor((s % 3600) / 60);
  var sec = s % 60;
  if (h > 0) return h + 'h ' + m + 'm ' + sec + 's';
  if (m > 0) return m + 'm ' + sec + 's';
  return sec + 's';
}

function escHtml(s) {
  return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

// ====================================================================
// NAV HTML SNIPPET GENERATOR
// Returns the <nav> HTML string to be inserted via JS or inline.
// Each page includes this and calls initNav() after DOM ready.
// ====================================================================
function getNavHTML() {
  return '<nav class="mass-nav"><ul class="mass-nav-list">' +
    '<li><a href="/"><span class="mass-nav-icon">&#x1F3C1;</span><span class="mass-nav-label">Live Monitor</span></a></li>' +
    '<li><a href="/history.html"><span class="mass-nav-icon">&#x1F4CB;</span><span class="mass-nav-label">Evidence Log</span></a></li>' +
    '<li><a href="/config"><span class="mass-nav-icon">&#x2699;&#xFE0F;</span><span class="mass-nav-label">System Config</span></a></li>' +
    '<li><a href="/console"><span class="mass-nav-icon">&#x1F4DF;</span><span class="mass-nav-label">Terminal</span></a></li>' +
    '</ul></nav>';
}

// ====================================================================
// COMMON INIT — Call from each page's onload
// ====================================================================
function massInit(opts) {
  opts = opts || {};
  initTheme();
  initNav();
  loadVersion();
  if (opts.ws !== false) connectWebSocket();
}

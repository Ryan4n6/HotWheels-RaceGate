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
var wsReconnectAttempts = 0;

function connectWebSocket() {
  // Protocol-aware: use wss:// when loaded over HTTPS (e.g. Tailscale Funnel)
  var proto = (location.protocol === 'https:') ? 'wss://' : 'ws://';
  var wsUrl = proto + window.location.hostname + ':81';

  try {
    ws = new WebSocket(wsUrl);
  } catch (e) {
    console.error('[WS] Constructor failed:', e);
    wsReconnectAttempts++;
    updateConnectionBadge(false);
    wsReconnectTimer = setTimeout(connectWebSocket, 3000);
    return;
  }

  ws.onopen = function() {
    wsConnected = true;
    wsReconnectAttempts = 0;
    updateConnectionBadge(true);
    clearTimeout(wsReconnectTimer);
    console.log('[WS] Connected to ' + wsUrl);
  };

  ws.onclose = function(evt) {
    wsConnected = false;
    wsReconnectAttempts++;
    updateConnectionBadge(false);
    console.log('[WS] Closed (code=' + evt.code + ', reason=' + (evt.reason || 'none') + '). Reconnect #' + wsReconnectAttempts);
    wsReconnectTimer = setTimeout(connectWebSocket, Math.min(2000 * wsReconnectAttempts, 10000));
  };

  ws.onerror = function(evt) {
    console.error('[WS] Error on ' + wsUrl, evt);
  };

  ws.onmessage = function(event) {
    try {
      var data = JSON.parse(event.data);
      for (var i = 0; i < wsMessageHandlers.length; i++) {
        wsMessageHandlers[i](data);
      }
    } catch (e) {
      console.error('[WS] Parse error:', e);
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
  var wasConnected = el.classList.contains('connected');
  el.className = 'conn-badge ' + (connected ? 'connected' : 'disconnected');
  if (connected) {
    el.textContent = 'LINKED';
  } else {
    el.textContent = wsReconnectAttempts > 1 ? 'OFFLINE (' + wsReconnectAttempts + ')' : 'OFFLINE';
  }
  // Dispatcher: announce connection changes (not initial load)
  if (wasConnected !== connected && _announceEl) {
    announce(connected ? 'Dispatch online. Link established.' : 'Signal lost. Reconnecting.');
  }
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
// VERSION BADGE — with GitHub update check (24hr cache)
// ====================================================================
var GITHUB_API_LATEST = 'https://api.github.com/repos/Ryan4n6/MASS-Trap/releases/latest';
var GITHUB_RELEASES_URL = 'https://github.com/Ryan4n6/MASS-Trap/releases';
var UPDATE_CACHE_KEY = 'mass_update_cache';
var UPDATE_CACHE_TTL = 24 * 60 * 60 * 1000; // 24 hours in ms

function compareSemver(a, b) {
  // Returns: 1 if a > b, -1 if a < b, 0 if equal
  // Strips pre-release suffixes (-rc1, -beta2, etc.) before comparing
  var pa = String(a).replace(/^v/i, '').replace(/-.*$/, '').split('.').map(Number);
  var pb = String(b).replace(/^v/i, '').replace(/-.*$/, '').split('.').map(Number);
  for (var i = 0; i < 3; i++) {
    var va = pa[i] || 0;
    var vb = pb[i] || 0;
    if (va > vb) return 1;
    if (va < vb) return -1;
  }
  return 0;
}

function applyVersionBadge(badge, currentFw, releaseInfo) {
  if (!badge) return;
  if (releaseInfo && releaseInfo.tag_name) {
    var latest = releaseInfo.tag_name.replace(/^v/i, '');
    if (compareSemver(latest, currentFw) > 0) {
      // Update available — breathing red badge with link
      badge.classList.add('update-available');
      badge.innerHTML = '<a href="' + GITHUB_RELEASES_URL + '" target="_blank" rel="noopener" title="Update available! Click to view release.">' +
        'v' + currentFw + ' → v' + latest + ' ⬆</a>';
      return;
    }
  }
  // Current or check failed — simple link to releases
  badge.classList.remove('update-available');
  badge.innerHTML = '<a href="' + GITHUB_RELEASES_URL + '" target="_blank" rel="noopener" title="View releases on GitHub">' +
    'FW v' + currentFw + '</a>';
}

function checkGitHubRelease(forceRefresh) {
  // Check localStorage cache first (unless forced)
  if (!forceRefresh) {
    try {
      var cached = JSON.parse(localStorage.getItem(UPDATE_CACHE_KEY));
      if (cached && cached.ts && (Date.now() - cached.ts) < UPDATE_CACHE_TTL) {
        return Promise.resolve(cached.data);
      }
    } catch (e) { /* ignore parse errors */ }
  }

  // Fetch fresh from GitHub API
  return fetch(GITHUB_API_LATEST).then(function(r) {
    if (!r.ok) throw new Error('GitHub API ' + r.status);
    return r.json();
  }).then(function(data) {
    // Cache the result with timestamp
    try {
      localStorage.setItem(UPDATE_CACHE_KEY, JSON.stringify({ data: data, ts: Date.now() }));
    } catch (e) { /* localStorage full — ignore */ }
    return data;
  }).catch(function(err) {
    // Network error or rate limit — return cached data if available, else null
    try {
      var cached = JSON.parse(localStorage.getItem(UPDATE_CACHE_KEY));
      if (cached && cached.data) return cached.data;
    } catch (e) { /* ignore */ }
    return null;
  });
}

function loadVersion() {
  fetch('/api/version').then(function(r) { return r.json(); }).then(function(v) {
    var badge = document.getElementById('versionBadge');
    var currentFw = v.firmware || '0.0.0';

    // Start with basic version display immediately
    if (badge) {
      badge.innerHTML = '<a href="' + GITHUB_RELEASES_URL + '" target="_blank" rel="noopener">' +
        'FW v' + currentFw + '</a>';
    }

    // Then check GitHub for updates (uses 24hr cache)
    checkGitHubRelease(false).then(function(releaseInfo) {
      applyVersionBadge(badge, currentFw, releaseInfo);
    });
  }).catch(function() {
    var meta = document.querySelector('meta[name="fw-version"]');
    var badge = document.getElementById('versionBadge');
    if (meta && badge) badge.textContent = 'v' + meta.content;
  });
}

// ====================================================================
// UTILITY HELPERS
// ====================================================================

// Safe .toFixed() — crash-proof for non-numeric values (data validation)
function safeFixed(val, dec) {
  var n = parseFloat(val);
  return isNaN(n) ? '--' : n.toFixed(dec || 2);
}

// Kiosk mode check — used by hotkey guard and auth integration
function isKioskMode() {
  return document.body.classList.contains('kiosk');
}

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
  initAnnouncer();
  initHotkeys();
  if (opts.ws !== false) connectWebSocket();
}

// ====================================================================
// DISPATCHER — Screen Reader Announcements ("Radio Comms")
// Tactical Police Dispatcher voice for aria-live announcements
// ====================================================================
var _announceEl = null;
var _lastAnnouncement = '';

function initAnnouncer() {
  if (_announceEl) return;
  _announceEl = document.createElement('div');
  _announceEl.id = 'massAnnouncer';
  _announceEl.className = 'sr-only';
  _announceEl.setAttribute('aria-live', 'assertive');
  _announceEl.setAttribute('aria-atomic', 'true');
  _announceEl.setAttribute('role', 'status');
  document.body.appendChild(_announceEl);
}

function announce(text, priority) {
  priority = priority || 'assertive';
  if (!_announceEl) initAnnouncer();
  _announceEl.setAttribute('aria-live', priority);
  _lastAnnouncement = text;
  // Clear then set after tick — forces screen readers to re-announce
  _announceEl.textContent = '';
  setTimeout(function() { _announceEl.textContent = text; }, 100);
}

// ====================================================================
// TACTICAL HOTKEYS — Keyboard shortcuts for blind users & excited kids
// Guard: only fire when not typing in a form field
// ====================================================================
function initHotkeys() {
  document.addEventListener('keydown', function(e) {
    // Skip if user is typing in a form field
    var tag = (document.activeElement || {}).tagName || '';
    if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;

    // Skip if modifier keys held (allow browser shortcuts)
    if (e.ctrlKey || e.metaKey || e.altKey) return;

    // Skip all hotkeys in kiosk mode (display-only)
    if (isKioskMode()) return;

    var key = e.key;

    // SPACE — Arm system (only on dashboard with armRace available)
    if (key === ' ' && typeof armRace === 'function') {
      e.preventDefault();
      armRace();
      return;
    }

    // R — Reset
    if ((key === 'r' || key === 'R') && typeof resetRace === 'function') {
      e.preventDefault();
      resetRace();
      announce('System reset. Standing by.');
      return;
    }

    // L — Loop (re-announce last message)
    if (key === 'l' || key === 'L') {
      e.preventDefault();
      if (_lastAnnouncement) {
        announce(_lastAnnouncement);
      } else {
        announce('No recent dispatch.');
      }
      return;
    }

    // G — Announce active car (dashboard-specific)
    if ((key === 'g' || key === 'G') && typeof activeCar !== 'undefined') {
      e.preventDefault();
      if (activeCar) {
        var stats = activeCar.stats || {};
        var best = (stats.bestTime != null && isFinite(stats.bestTime)) ? stats.bestTime.toFixed(3) + ' seconds' : 'no runs';
        announce('Active unit: ' + activeCar.name + '. ' +
          activeCar.weight + ' grams. ' +
          (stats.runs || 0) + ' runs. Best: ' + best + '.');
      } else {
        announce('No active unit. Select a vehicle from the garage.');
      }
      return;
    }

    // T — Announce last race time (dashboard-specific)
    if ((key === 't' || key === 'T') && typeof _lastRaceTime !== 'undefined') {
      e.preventDefault();
      if (_lastRaceTime) {
        announce('Last time: ' + _lastRaceTime + '.');
      } else {
        announce('No race data. Arm the system and run a vehicle.');
      }
      return;
    }

    // ? — Help (list hotkeys)
    if (key === '?') {
      e.preventDefault();
      announce('Hotkeys. Space to arm. R to reset. L to repeat. G for active car. T for last time. Question mark for help.');
      return;
    }
  });
}

// ====================================================================
// ARIA TABS — Reusable tab widget initialization (WAI-ARIA pattern)
// ====================================================================
function initAriaTabs(tablistSelector, tabSelector, panelPrefix) {
  var tablist = document.querySelector(tablistSelector);
  if (!tablist) return;
  tablist.setAttribute('role', 'tablist');

  var tabs = tablist.querySelectorAll(tabSelector);
  for (var i = 0; i < tabs.length; i++) {
    var tab = tabs[i];
    var panelId = tab.getAttribute('data-tab');
    if (!panelId) continue;

    tab.setAttribute('role', 'tab');
    tab.setAttribute('aria-controls', 'tab-' + panelId);
    if (!tab.id) tab.id = 'tabBtn-' + panelId;
    tab.setAttribute('aria-selected', tab.classList.contains('active') ? 'true' : 'false');
    tab.setAttribute('tabindex', tab.classList.contains('active') ? '0' : '-1');

    var panel = document.getElementById('tab-' + panelId);
    if (panel) {
      panel.setAttribute('role', 'tabpanel');
      panel.setAttribute('aria-labelledby', tab.id);
      panel.setAttribute('tabindex', '0');
    }
  }

  // Arrow key navigation within tablist
  tablist.addEventListener('keydown', function(e) {
    var current = document.activeElement;
    if (!current || current.getAttribute('role') !== 'tab') return;

    var allTabs = tablist.querySelectorAll(tabSelector);
    var idx = -1;
    for (var j = 0; j < allTabs.length; j++) {
      if (allTabs[j] === current) { idx = j; break; }
    }
    if (idx === -1) return;

    var newIdx = -1;
    if (e.key === 'ArrowRight' || e.key === 'ArrowDown') {
      newIdx = (idx + 1) % allTabs.length;
    } else if (e.key === 'ArrowLeft' || e.key === 'ArrowUp') {
      newIdx = (idx - 1 + allTabs.length) % allTabs.length;
    } else if (e.key === 'Home') {
      newIdx = 0;
    } else if (e.key === 'End') {
      newIdx = allTabs.length - 1;
    }

    if (newIdx >= 0) {
      e.preventDefault();
      allTabs[newIdx].focus();
      allTabs[newIdx].click();
    }
  });
}

function updateAriaTabState(activeTab, tabSelector) {
  var tabs = document.querySelectorAll(tabSelector);
  for (var i = 0; i < tabs.length; i++) {
    var isActive = tabs[i] === activeTab;
    tabs[i].setAttribute('aria-selected', isActive ? 'true' : 'false');
    tabs[i].setAttribute('tabindex', isActive ? '0' : '-1');
  }
}

// ====================================================================
// AUTH GATE — Badge Reader (viewer) & Internal Affairs (admin)
// Creates full-screen overlay requiring password to dismiss.
// sessionStorage-based: persists across reloads, expires on tab close.
// ====================================================================
function checkAuthGate(tier, opts) {
  opts = opts || {};
  var sessionKey = tier === 'admin' ? 'mass_admin_auth' : 'mass_viewer_auth';
  var apiTier = tier === 'admin' ? 'admin' : 'viewer';

  // Already authenticated this session?
  if (sessionStorage.getItem(sessionKey) === 'true') return;

  fetch('/api/auth/info').then(function(r) { return r.json(); }).then(function(info) {
    var needed = tier === 'admin' ? info.hasAdminPassword : info.hasViewerPassword;
    if (!needed) return; // No password set — open access

    // Build overlay
    var overlay = document.createElement('div');
    overlay.id = 'authGate';
    overlay.style.cssText = 'position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.92);z-index:10000;display:flex;align-items:center;justify-content:center;';

    var isAdmin = tier === 'admin';
    var title = isAdmin ? 'INTERNAL AFFAIRS' : 'BADGE READER';
    var subtitle = isAdmin ? 'Administrative access required' : 'Scan your badge to proceed';
    var btnText = isAdmin ? 'Authenticate' : 'Scan Badge';
    var accentColor = isAdmin ? '#dc3545' : 'var(--accent, #d4af37)';

    var box = document.createElement('div');
    box.style.cssText = 'background:var(--bg-card,#16213e);border:2px solid ' + accentColor + ';border-radius:12px;padding:40px;text-align:center;max-width:360px;width:90%;';
    box.innerHTML =
      '<div style="font-size:2.5rem;margin-bottom:12px;">' + (isAdmin ? '&#x1F6E1;' : '&#x1F6E1;') + '</div>' +
      '<h2 style="color:' + accentColor + ';font-family:var(--font-body);letter-spacing:2px;margin:0 0 8px;">' + title + '</h2>' +
      '<p style="color:var(--text-muted,#888);font-size:0.85rem;margin:0 0 24px;">' + subtitle + '</p>' +
      '<input type="password" id="authGateInput" placeholder="Password" style="width:100%;padding:12px;border:1px solid var(--border,#2a2a4a);border-radius:6px;background:var(--bg-input,#0f0f1e);color:var(--text,#e0e0e0);font-size:1rem;margin-bottom:12px;text-align:center;font-family:var(--font-data);" autocomplete="current-password">' +
      '<div id="authGateError" style="color:var(--danger,#dc3545);font-size:0.85rem;margin-bottom:12px;min-height:1.2em;"></div>' +
      '<button id="authGateBtn" style="width:100%;padding:12px;background:' + accentColor + ';color:#000;border:none;border-radius:6px;font-weight:900;font-size:1rem;cursor:pointer;text-transform:uppercase;letter-spacing:1px;">' + btnText + '</button>';

    overlay.appendChild(box);
    document.body.appendChild(overlay);

    var input = document.getElementById('authGateInput');
    var errEl = document.getElementById('authGateError');
    var btn = document.getElementById('authGateBtn');

    function tryAuth() {
      var pw = input.value;
      btn.disabled = true;
      btn.textContent = 'Checking...';
      fetch('/api/auth/check', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ password: pw, tier: apiTier })
      }).then(function(r) { return r.json(); }).then(function(result) {
        if (result.ok) {
          sessionStorage.setItem(sessionKey, 'true');
          overlay.remove();
          if (typeof announce === 'function') {
            announce(isAdmin ? 'Internal Affairs clearance granted.' : 'Badge accepted. Access granted.');
          }
          if (typeof opts.onSuccess === 'function') opts.onSuccess();
        } else {
          errEl.textContent = 'ACCESS DENIED';
          input.value = '';
          input.focus();
          box.style.animation = 'none';
          void box.offsetWidth; // Reflow
          box.style.animation = 'shake 0.4s ease';
        }
        btn.disabled = false;
        btn.textContent = btnText;
      }).catch(function() {
        errEl.textContent = 'Connection error';
        btn.disabled = false;
        btn.textContent = btnText;
      });
    }

    btn.onclick = tryAuth;
    input.onkeydown = function(e) { if (e.key === 'Enter') tryAuth(); };
    setTimeout(function() { input.focus(); }, 100);

    // Add shake animation CSS if not present
    if (!document.getElementById('authShakeCSS')) {
      var style = document.createElement('style');
      style.id = 'authShakeCSS';
      style.textContent = '@keyframes shake{0%,100%{transform:translateX(0)}25%{transform:translateX(-8px)}75%{transform:translateX(8px)}}';
      document.head.appendChild(style);
    }
  }).catch(function(e) {
    console.log('[AUTH] Could not fetch auth info, proceeding without gate:', e);
  });
}

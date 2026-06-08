// autoJetsonBot — Main App Entrypoint
import { TelemetryManager } from './telemetry.js';
import { ControlsManager } from './controls.js';
import { NavigationManager } from './navigation.js';
import { LidarRenderer } from './lidar.js';
import { MapManager } from './map.js';

class RobotController {
  constructor() {
    this.ros = null;

    // Control state
    this.isConnected   = false;
    this.isControlling = false;
    this.currentLinear = 0.0;
    this.currentAngular = 0.0;
    this.linearScale   = 0.5;
    this.angularScale  = 0.5;
    this.publishInterval = null;

    // Reconnect backoff
    this._reconnectTimer   = null;
    this._reconnectDelay   = 1000;
    this._userDisconnected = false;

    // Configuration — topic names are read from sidebar inputs on connect
    this.config = {
      rosbridgeUrl:  'ws://localhost:9090',
      publishRate:   100,
      maxLinearVel:  1.0,
      maxAngularVel: 2.0,
      cameraUrl:     'http://localhost:8080/stream?topic=/camera/image_raw&type=mjpeg&width=320&height=240',
      cameraUrlFallback: 'http://localhost:8080/stream?topic=/camera/camera/image_raw&type=mjpeg&width=320&height=240',
      topics: {
        cmdVel:      '/cmd_vel',
        jointStates: '/joint_states',
        scan:        '/scan',
        amclPose:    '/amcl_pose',
        odom:        '/odom',
      },
    };

    // Shared state variables across modules
    this.metrics = {
      leftRpm: 0, rightRpm: 0,
      linearVel: 0, angularVel: 0,
      batteryLevel: 0,
      cpu: null, memory: null, temperature: null,
    };

    this._mapPose     = { x: 0, y: 0, yaw: 0 }; // AMCL Pose in map frame
    this._odomPose    = { x: 0, y: 0, yaw: 0 }; // Odom Pose (fallback)
    this._hasAmcl     = false;                  // AMCL status flag
    this._goalPreview = null; // { x, y, yaw } in map frame
    this._activeGoal  = null; // { x, y, yaw } in map frame
    this._currentPath = [];   // Array of {x, y} poses from global planner (map frame)

    this._previewResetTimer = null;
    this._lastScanWarn = 0;

    // Instantiate sub-managers
    this.telemetry  = new TelemetryManager(this);
    this.controls   = new ControlsManager(this);
    this.navigation = new NavigationManager(this);
    this.lidar      = new LidarRenderer(this);
    this.map        = new MapManager(this);

    this.init();
  }

  // ── Init ─────────────────────────────────────────────────────────────

  init() {
    this._loadSettings();
    this._syncAllSettingsFromDOM(); // Sync initial values to config
    this._applyTheme(localStorage.getItem('theme') || 'dark');
    this.setupEventListeners();
    this.setupUI();
    this.autoDetectROSBridge();
    this.fetchConfig();
    this.startMetricsUpdate();
    this.lidar.start();
    this.map.start();
    this.log('Robot Controller initialized', 'info');
  }

  async fetchConfig() {
    try {
      const res = await fetch('config.json');
      if (!res.ok) { this.log('No remote config, using defaults', 'warning'); return; }
      const cfg = await res.json();
      if (cfg.map_name) this.config.map_name = cfg.map_name;
      if (cfg.map_dir)  this.config.map_dir  = cfg.map_dir;
      if (cfg.mode)     this._setModeBadge(cfg.mode, cfg.use_sim);
      this.log(`Config loaded — mode: ${cfg.mode || '?'}, map: ${cfg.map_name || '?'}`, 'info');
    } catch (e) {
      this.log(`Config fetch failed: ${e}`, 'warning');
    }
  }

  _syncAllSettingsFromDOM() {
    this.config.rosbridgeUrl = document.getElementById('rosbridgeUrl').value || 'ws://localhost:9090';
    this.config.maxLinearVel  = parseFloat(document.getElementById('maxLinearVel').value) || 1.0;
    this.config.maxAngularVel = parseFloat(document.getElementById('maxAngularVel').value) || 2.0;
    const hz = parseFloat(document.getElementById('publishRate').value) || 10;
    this.config.publishRate   = Math.round(1000 / hz);
    this._syncTopicConfig();
  }

  // ── Theme ─────────────────────────────────────────────────────────────

  _applyTheme(theme) {
    document.body.setAttribute('data-theme', theme);
    const btn = document.getElementById('themeToggle');
    if (btn) btn.textContent = theme === 'dark' ? '🌙' : '☀️';
    localStorage.setItem('theme', theme);
  }

  _toggleTheme() {
    const current = document.body.getAttribute('data-theme') || 'dark';
    this._applyTheme(current === 'dark' ? 'light' : 'dark');
  }

  // ── Mode badge ────────────────────────────────────────────────────────

  _setModeBadge(mode, isSim) {
    const badge = document.getElementById('modeBadge');
    if (!badge) return;
    badge.textContent = `${isSim ? 'SIM' : 'HW'} · ${mode.toUpperCase()}`;
    badge.style.display = 'inline-flex';
  }

  // ── Settings persistence ──────────────────────────────────────────────

  _saveSettings() {
    const s = {
      rosbridgeUrl:     document.getElementById('rosbridgeUrl').value,
      maxLinearVel:     document.getElementById('maxLinearVel').value,
      maxAngularVel:    document.getElementById('maxAngularVel').value,
      publishRate:      document.getElementById('publishRate').value,
      cmdVelTopic:      document.getElementById('cmdVelTopic').value,
      jointStatesTopic: document.getElementById('jointStatesTopic').value,
      scanTopic:        document.getElementById('scanTopic').value,
    };
    localStorage.setItem('robotSettings', JSON.stringify(s));
  }

  _loadSettings() {
    try {
      const s = JSON.parse(localStorage.getItem('robotSettings') || '{}');
      const set = (id, val) => { const el = document.getElementById(id); if (el && val !== undefined) el.value = val; };
      set('rosbridgeUrl',      s.rosbridgeUrl);
      set('maxLinearVel',      s.maxLinearVel);
      set('maxAngularVel',     s.maxAngularVel);
      set('publishRate',       s.publishRate);
      set('cmdVelTopic',       s.cmdVelTopic);
      set('jointStatesTopic',  s.jointStatesTopic);
      set('scanTopic',         s.scanTopic);
    } catch (_) {}
  }

  // ── Event listeners ───────────────────────────────────────────────────

  setupEventListeners() {
    document.getElementById('connectBtn').addEventListener('click',    () => { this._userDisconnected = false; this.connect(); });
    document.getElementById('disconnectBtn').addEventListener('click', () => { this._userDisconnected = true;  this.disconnect(); });

    // delegate controls-specific event listeners
    this.controls.setupEventListeners();

    // generic UI event listeners
    document.getElementById('emergencyStop').addEventListener('click', () => this.controls.emergencyStop());
    document.getElementById('resetOdometry').addEventListener('click', () => this.controls.resetOdometry());
    document.getElementById('saveMap').addEventListener('click',       () => this.controls.saveMap());
    
    document.getElementById('previewGoalBtn').addEventListener('click', () => this.navigation.previewNavGoal());
    document.getElementById('sendGoalBtn').addEventListener('click',   () => this.navigation.sendNavGoal());
    document.getElementById('cancelGoalBtn').addEventListener('click', () => this.navigation.cancelNavGoal());
    
    document.getElementById('resetMapView').addEventListener('click',  () => this.map.resetView());
    
    document.getElementById('cameraToggleBtn').addEventListener('click', () => this._toggleCamera());
    document.getElementById('themeToggle').addEventListener('click',   () => this._toggleTheme());
    document.getElementById('exportLogBtn').addEventListener('click',  () => this._exportLog());
    document.getElementById('rosbridgeUrl').addEventListener('change', () => this._saveSettings());

    // Topic input changes
    ['cmdVelTopic', 'jointStatesTopic', 'scanTopic'].forEach(id => {
      document.getElementById(id).addEventListener('change', () => {
        this._syncTopicConfig();
        this._saveSettings();
        if (this.isConnected) {
          this._teardownROSTopics();
          this.setupROSTopics();
          this.log('Topics reconfigured', 'info');
        }
      });
    });
  }

  _syncTopicConfig() {
    this.config.topics.cmdVel      = document.getElementById('cmdVelTopic').value      || '/cmd_vel';
    this.config.topics.jointStates = document.getElementById('jointStatesTopic').value || '/joint_states';
    this.config.topics.scan        = document.getElementById('scanTopic').value         || '/scan';
  }

  // ── UI setup ──────────────────────────────────────────────────────────

  setupUI() {
    document.getElementById('linearValue').textContent  = this.linearScale.toFixed(2);
    document.getElementById('angularValue').textContent = this.angularScale.toFixed(2);
    this.updateConnectionStatus('disconnected');
    this.navigation._setNav2Badge('idle');
    document.getElementById('cancelGoalBtn').style.display = 'none';
  }

  autoDetectROSBridge() {
    const host = window.location.hostname;
    const el   = document.getElementById('rosbridgeUrl');
    if (!localStorage.getItem('robotSettings')) {
      el.value = `ws://${host}:9090`;
    }
    this.config.rosbridgeUrl = el.value;
  }

  updateConnectionStatus(status) {
    const indicator = document.querySelector('.status-indicator');
    const text = document.getElementById('connectionStatusText');
    const connectBtn = document.getElementById('connectBtn');
    const disconnectBtn = document.getElementById('disconnectBtn');
    if (!indicator || !text || !connectBtn || !disconnectBtn) return;

    indicator.className = `status-indicator status-${status}`;
    text.textContent = {
      connected: 'Connected',
      connecting: 'Connecting…',
      disconnected: 'Disconnected'
    }[status] || status;

    connectBtn.style.display = status === 'connected' ? 'none' : 'inline-block';
    disconnectBtn.style.display = status === 'connected' ? 'inline-block' : 'none';
  }

  // ── Connection ────────────────────────────────────────────────────────

  connect() {
    if (this._reconnectTimer) { clearTimeout(this._reconnectTimer); this._reconnectTimer = null; }
    const url = document.getElementById('rosbridgeUrl').value;
    this.config.rosbridgeUrl = url;
    this._syncTopicConfig();
    this.updateConnectionStatus('connecting');
    this.log(`Connecting to ${url}…`, 'info');
    if (this.ros) { try { this.ros.close(); } catch (_) {} }
    
    this.ros = new ROSLIB.Ros({ url });
    
    this.ros.on('connection', () => {
      this.isConnected = true;
      this._reconnectDelay = 1000;
      this.updateConnectionStatus('connected');
      this.setupROSTopics();
      this.log('Connected to robot', 'info');
    });
    
    this.ros.on('error', (err) => {
      this.isConnected = false;
      this.updateConnectionStatus('disconnected');
      this.log(`Connection error: ${err}`, 'error');
      this._scheduleReconnect();
    });
    
    this.ros.on('close', () => {
      this.isConnected = false;
      this.updateConnectionStatus('disconnected');
      this.log('Connection closed', 'warning');
      this._scheduleReconnect();
    });
  }

  disconnect() {
    this._userDisconnected = true;
    if (this._reconnectTimer) { clearTimeout(this._reconnectTimer); this._reconnectTimer = null; }
    this._teardownROSTopics();
    if (this.ros) { try { this.ros.close(); } catch (_) {} }
    this.isConnected = false;
    this.updateConnectionStatus('disconnected');
    this.log('Disconnected', 'info');
  }

  _scheduleReconnect() {
    if (this._userDisconnected) return;
    this.log(`Reconnecting in ${this._reconnectDelay / 1000}s…`, 'warning');
    this._reconnectTimer = setTimeout(() => this.connect(), this._reconnectDelay);
    this._reconnectDelay = Math.min(this._reconnectDelay * 2, 30000);
  }

  // ── ROS Topics Coordinate Orchestration ─────────────────────────────────

  setupROSTopics() {
    this.controls.setupTopics();
    this.telemetry.setupTopics();
    this.lidar.setupTopics();
    this.navigation.setupTopics();
    this.map.setupTopics();
    this.log('ROS topics initialized', 'info');
  }

  _teardownROSTopics() {
    this.controls.teardownTopics();
    this.telemetry.teardownTopics();
    this.lidar.teardownTopics();
    this.navigation.teardownTopics();
    this.map.teardownTopics();
  }

  // ── Camera feed toggler ──────────────────────────────────────────────────

  _toggleCamera() {
    const img = document.getElementById('cameraImg');
    const placeholder = document.getElementById('cameraPlaceholder');
    const btn = document.getElementById('cameraToggleBtn');
    const statusTxt = placeholder.querySelector('p:last-child');
    if (img.src && img.src !== window.location.href) {
      img.src = ''; img.style.display = 'none';
      placeholder.style.display = 'flex';
      if (statusTxt) statusTxt.textContent = 'Camera feed paused';
      btn.textContent = '▶ Start Feed';
      return;
    }
    const tryUrl = (url, isFallback) => {
      img.src = url; img.style.display = 'block'; placeholder.style.display = 'none';
      btn.textContent = '⏸ Pause Feed';
      img.onerror = () => {
        if (!isFallback && this.config.cameraUrlFallback) tryUrl(this.config.cameraUrlFallback, true);
        else {
          img.src = ''; img.style.display = 'none'; placeholder.style.display = 'flex';
          if (statusTxt) statusTxt.textContent = '📷 Stream unavailable';
          btn.textContent = '▶ Start Feed';
        }
      };
    };
    tryUrl(this.config.cameraUrl, false);
  }

  // ── Metrics loop ─────────────────────────────────────────────────────────

  startMetricsUpdate() {
    setInterval(() => this.telemetry.updateMetricsDisplay(), 1000);
  }

  // ── Log management ───────────────────────────────────────────────────────

  log(message, type = 'info') {
    const container = document.getElementById('logContainer');
    if (!container) return;
    const el = document.createElement('div');
    el.className = `log-entry log-${type}`;
    el.innerHTML = `<span style="color:#64748b">[${new Date().toLocaleTimeString()}]</span> ${message}`;
    container.appendChild(el);
    container.scrollTop = container.scrollHeight;
    
    // Cap log at 100 entries
    while (container.children.length > 100) container.removeChild(container.firstChild);
  }

  _exportLog() {
    const logContainer = document.getElementById('logContainer');
    if (!logContainer) return;
    const lines = [...logContainer.children].map(el => el.textContent.trim()).join('\n');
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([lines], { type: 'text/plain' }));
    a.download = `robot_log_${Date.now()}.txt`;
    a.click();
  }
}

document.addEventListener('DOMContentLoaded', () => {
  window.robotController = new RobotController();
});

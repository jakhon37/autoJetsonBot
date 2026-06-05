// autoJetsonBot — Robot Controller
// Phase 1: Fix broken inputs, wire telemetry, dark mode
// Phase 2: Nav2 goal send/cancel, status badge, lidar canvas, camera feed
class RobotController {
  constructor() {
    this.ros = null;

    // ROS topic/service handles
    this.cmdVelTopic       = null;
    this.jointStatesTopic  = null;
    this.scanTopic         = null;
    this.batteryTopic      = null;
    this.systemTopic       = null;
    this.navStatusTopic    = null;
    this._activeGoalId     = null;

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
      // Gazebo libgazebo_ros_camera.so publishes under /camera/image_raw by default.
      // web_video_server auto-discovers topics — try both common namespaces.
      cameraUrl:     'http://localhost:8080/stream?topic=/camera/image_raw&type=mjpeg&width=320&height=240',
      cameraUrlFallback: 'http://localhost:8080/stream?topic=/camera/camera/image_raw&type=mjpeg&width=320&height=240',
      topics: {
        cmdVel:      '/cmd_vel',
        jointStates: '/joint_states',
        scan:        '/scan',
      },
    };
    
    // Metrics
    this.metrics = {
      leftRpm: 0, rightRpm: 0,
      linearVel: 0, angularVel: 0,
      batteryLevel: 0,
      cpu: null, memory: null, temperature: null,
    };

    // Lidar canvas state
    this._lidarRanges   = [];
    this._lidarAngleMin = 0;
    this._lidarAngleInc = 0;

    // Goal preview state
    this._goalPreview = null; // { x, y, yaw } in metres, robot-relative

    this.init();
  }

  // ── Init ─────────────────────────────────────────────────────────────

  init() {
    this._loadSettings();
    this._applyTheme(localStorage.getItem('theme') || 'dark');
    this.setupEventListeners();
    this.setupUI();
    this.autoDetectROSBridge();
    this.fetchConfig();
    this.startMetricsUpdate();
    this._startLidarRender();
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

    document.getElementById('linearScale').addEventListener('input', (e) => {
      this.linearScale = parseFloat(e.target.value);
      document.getElementById('linearValue').textContent = this.linearScale.toFixed(2);
    });
    document.getElementById('angularScale').addEventListener('input', (e) => {
      this.angularScale = parseFloat(e.target.value);
      document.getElementById('angularValue').textContent = this.angularScale.toFixed(2);
    });

    document.getElementById('maxLinearVel').addEventListener('change',  (e) => { this.config.maxLinearVel  = parseFloat(e.target.value) || 1.0; this._saveSettings(); });
    document.getElementById('maxAngularVel').addEventListener('change', (e) => { this.config.maxAngularVel = parseFloat(e.target.value) || 2.0; this._saveSettings(); });
    document.getElementById('publishRate').addEventListener('change',   (e) => { this.config.publishRate   = Math.round(1000 / (parseFloat(e.target.value) || 10)); this._saveSettings(); });

    // Topic inputs — FIX: now actually update config and reconnect topics
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

    this.setupDPadControls();
    this.setupKeyboardControls();

    document.getElementById('emergencyStop').addEventListener('click', () => this.emergencyStop());
    document.getElementById('resetOdometry').addEventListener('click', () => this.resetOdometry());
    document.getElementById('saveMap').addEventListener('click',       () => this.saveMap());
    document.getElementById('previewGoalBtn').addEventListener('click', () => this.previewNavGoal());
    document.getElementById('sendGoalBtn').addEventListener('click',   () => this.sendNavGoal());
    document.getElementById('cancelGoalBtn').addEventListener('click', () => this.cancelNavGoal());
    document.getElementById('cameraToggleBtn').addEventListener('click', () => this._toggleCamera());
    document.getElementById('themeToggle').addEventListener('click',   () => this._toggleTheme());
    document.getElementById('exportLogBtn').addEventListener('click',  () => this._exportLog());
    document.getElementById('rosbridgeUrl').addEventListener('change', () => this._saveSettings());
  }

  _syncTopicConfig() {
    this.config.topics.cmdVel      = document.getElementById('cmdVelTopic').value      || '/cmd_vel';
    this.config.topics.jointStates = document.getElementById('jointStatesTopic').value || '/joint_states';
    this.config.topics.scan        = document.getElementById('scanTopic').value         || '/scan';
  }

  // ── D-pad & keyboard ──────────────────────────────────────────────────

  setupDPadControls() {
    ['up', 'down', 'left', 'right'].forEach(dir => {
      const btn = document.getElementById(`${dir}Button`);
      btn.addEventListener('mousedown',  (e) => this.startDirection(e, dir));
      btn.addEventListener('mouseup',    (e) => this.stopDirection(e));
      btn.addEventListener('mouseleave', (e) => this.stopDirection(e));
      btn.addEventListener('touchstart', (e) => this.startDirection(e, dir));
      btn.addEventListener('touchend',   (e) => this.stopDirection(e));
      btn.addEventListener('contextmenu',(e) => e.preventDefault());
    });
  }

  setupKeyboardControls() {
    const keyMap = { 'w':'up','arrowup':'up','s':'down','arrowdown':'down','a':'left','arrowleft':'left','d':'right','arrowright':'right' };
    document.addEventListener('keydown', (e) => {
      if (e.repeat) return;
      const dir = keyMap[e.key.toLowerCase()];
      if (dir) { this.startDirection(e, dir); return; }
      if (e.key === ' ')              { this.emergencyStop(); e.preventDefault(); }
      if (e.key.toLowerCase() === 'g') document.getElementById('sendGoalBtn').click();
      if (e.key.toLowerCase() === 'c') document.getElementById('cancelGoalBtn').click();
      if (e.key.toLowerCase() === 'm') document.getElementById('saveMap').click();
    });
    document.addEventListener('keyup', (e) => {
      if (keyMap[e.key.toLowerCase()]) this.stopDirection(e);
    });
  }

  // ── UI setup ──────────────────────────────────────────────────────────

  setupUI() {
    document.getElementById('linearValue').textContent  = this.linearScale.toFixed(2);
    document.getElementById('angularValue').textContent = this.angularScale.toFixed(2);
    this.updateConnectionStatus('disconnected');
    this._setNav2Badge('idle');
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

  // ── ROS Topics ────────────────────────────────────────────────────────

  _teardownROSTopics() {
    [this.jointStatesTopic, this.scanTopic, this.batteryTopic,
     this.systemTopic, this.navStatusTopic].forEach(t => {
      if (t) { try { t.unsubscribe(); } catch (_) {} }
    });
    this.cmdVelTopic = this.jointStatesTopic = this.scanTopic =
    this.batteryTopic = this.systemTopic = this.navStatusTopic = null;
  }

  setupROSTopics() {
    this.cmdVelTopic = new ROSLIB.Topic({ ros: this.ros, name: this.config.topics.cmdVel, messageType: 'geometry_msgs/Twist' });

    this.jointStatesTopic = new ROSLIB.Topic({ ros: this.ros, name: this.config.topics.jointStates, messageType: 'sensor_msgs/JointState' });
    this.jointStatesTopic.subscribe((msg) => this.updateJointStates(msg));

    this.scanTopic = new ROSLIB.Topic({ ros: this.ros, name: this.config.topics.scan, messageType: 'sensor_msgs/LaserScan', throttle_rate: 100 });
    this.scanTopic.subscribe((msg) => this._onScan(msg));

    this.batteryTopic = new ROSLIB.Topic({ ros: this.ros, name: '/battery_state', messageType: 'sensor_msgs/BatteryState' });
    this.batteryTopic.subscribe((msg) => { this.metrics.batteryLevel = Math.round(msg.percentage * 100); });

    // System telemetry — CPU / Memory / Temperature (published by telemetry_node.py)
    this.systemTopic = new ROSLIB.Topic({ ros: this.ros, name: '/telemetry/system', messageType: 'std_msgs/String' });
    this.systemTopic.subscribe((msg) => {
      try {
        const d = JSON.parse(msg.data);
        this.metrics.cpu         = d.cpu;
        this.metrics.memory      = d.memory;
        this.metrics.temperature = d.temperature;
        if (d.battery !== undefined) this.metrics.batteryLevel = d.battery;
      } catch (_) {}
    });

    // Nav2 goal status
    this.navStatusTopic = new ROSLIB.Topic({ ros: this.ros, name: '/navigate_to_pose/_action/status', messageType: 'action_msgs/GoalStatusArray', throttle_rate: 500 });
    this.navStatusTopic.subscribe((msg) => this._onNavStatus(msg));

    this.log('ROS topics initialized', 'info');
  }

  // ── Movement ──────────────────────────────────────────────────────────

  startDirection(event, direction) {
    event.preventDefault();
    if (!this.isConnected || this.isControlling) return;
    this.isControlling = true;
    const map = { up:[1,0], down:[-1,0], left:[0,1], right:[0,-1] };
    [this.currentLinear, this.currentAngular] = map[direction];
    document.getElementById(`${direction}Button`).classList.add('active');
    this.publishInterval = setInterval(() => this.publishVelocity(), this.config.publishRate);
    this.log(`Moving ${direction}`, 'info');
  }

  stopDirection(event) {
    event.preventDefault();
    if (!this.isControlling) return;
    this.isControlling = false;
    this.currentLinear = this.currentAngular = 0;
    if (this.publishInterval) { clearInterval(this.publishInterval); this.publishInterval = null; }
    ['up','down','left','right'].forEach(d => document.getElementById(`${d}Button`).classList.remove('active'));
    this.publishVelocity();
    this.log('Stopped', 'info');
  }

  emergencyStop() {
    this.stopDirection(new Event('emergency'));
    this.log('EMERGENCY STOP', 'warning');
    const btn = document.getElementById('emergencyStop');
    btn.style.background = '#ef4444';
    setTimeout(() => { btn.style.background = ''; }, 1000);
  }

  publishVelocity() {
    if (!this.cmdVelTopic || !this.isConnected) return;
    const twist = new ROSLIB.Message({
      linear:  { x: this.currentLinear  * this.linearScale  * this.config.maxLinearVel,  y: 0, z: 0 },
      angular: { x: 0, y: 0, z: this.currentAngular * this.angularScale * this.config.maxAngularVel },
    });
    this.cmdVelTopic.publish(twist);
    this.metrics.linearVel  = twist.linear.x;
    this.metrics.angularVel = twist.angular.z;
  }

  // ── Odometry / Map ────────────────────────────────────────────────────

  resetOdometry() {
    if (!this.isConnected) { this.log('Not connected', 'warning'); return; }
    const t = new ROSLIB.Topic({ ros: this.ros, name: '/initialpose', messageType: 'geometry_msgs/PoseWithCovarianceStamped' });
    t.publish(new ROSLIB.Message({ header: { frame_id: 'map' }, pose: { pose: { position: {x:0,y:0,z:0}, orientation: {x:0,y:0,z:0,w:1} } } }));
    this.log('Odometry reset → /initialpose', 'info');
  }

  saveMap() {
    if (!this.isConnected) { this.log('Not connected', 'warning'); return; }
    const mapName  = this.config.map_name || 'lab_map';
    const mapDir   = this.config.map_dir  || '/autonomous_ROS/src/jetson_bot_bringup/worlds';
    const fullPath = `${mapDir}/${mapName}`;
    this.log(`Saving map → ${fullPath}`, 'info');
    const svc = new ROSLIB.Service({ ros: this.ros, name: '/slam_toolbox/save_map', serviceType: 'slam_toolbox/srv/SaveMap' });
    svc.callService(
      new ROSLIB.ServiceRequest({ name: { data: fullPath } }),
      ()  => this.log(`✅ Map saved: ${fullPath}`, 'info'),
      (e) => {
        this.log(`❌ Map save failed: ${e} — retrying with flat string`, 'error');
        svc.callService(new ROSLIB.ServiceRequest({ name: fullPath }),
          () => this.log(`✅ Map saved (fallback): ${fullPath}`, 'info'),
          (e2) => this.log(`❌ Map save failed again: ${e2}`, 'error'));
      }
    );
  }

  // ── Nav2 goal ─────────────────────────────────────────────────────────

  previewNavGoal() {
    const x   = parseFloat(document.getElementById('goalX').value)   || 0;
    const y   = parseFloat(document.getElementById('goalY').value)   || 0;
    const yaw = parseFloat(document.getElementById('goalYaw').value) || 0;

    this._goalPreview = { x, y, yaw };

    // Visual feedback on button
    const btn = document.getElementById('previewGoalBtn');
    btn.textContent = '👁 Previewing…';
    btn.style.background = 'var(--warning-color)';
    // Reset after 3s if user doesn't send
    clearTimeout(this._previewResetTimer);
    this._previewResetTimer = setTimeout(() => {
      this._goalPreview = null;
      btn.textContent = '👁 Preview';
      btn.style.background = '';
    }, 8000);

    this.log(`Preview goal → x:${x} y:${y} yaw:${yaw}° (shown on lidar canvas)`, 'info');
  }

  sendNavGoal() {
    if (!this.isConnected) { this.log('Not connected', 'warning'); return; }
    const x   = parseFloat(document.getElementById('goalX').value)   || 0;
    const y   = parseFloat(document.getElementById('goalY').value)   || 0;
    const yaw = (parseFloat(document.getElementById('goalYaw').value) || 0) * Math.PI / 180;
    const qz  = Math.sin(yaw / 2);
    const qw  = Math.cos(yaw / 2);
    const t   = new ROSLIB.Topic({ ros: this.ros, name: '/goal_pose', messageType: 'geometry_msgs/PoseStamped' });
    t.publish(new ROSLIB.Message({ header: { frame_id: 'map' }, pose: { position: {x,y,z:0}, orientation: {x:0,y:0,z:qz,w:qw} } }));
    this.log(`Nav goal → x:${x} y:${y} yaw:${(yaw*180/Math.PI).toFixed(1)}°`, 'info');
    document.getElementById('cancelGoalBtn').style.display = 'inline-block';

    // Clear preview
    this._goalPreview = null;
    clearTimeout(this._previewResetTimer);
    const prevBtn = document.getElementById('previewGoalBtn');
    prevBtn.textContent = '👁 Preview';
    prevBtn.style.background = '';
  }

  cancelNavGoal() {
    if (!this.isConnected) return;
    const t = new ROSLIB.Topic({ ros: this.ros, name: '/goal_pose', messageType: 'geometry_msgs/PoseStamped' });
    t.publish(new ROSLIB.Message({ header: { frame_id: 'map' }, pose: { position: {x:0,y:0,z:0}, orientation: {x:0,y:0,z:0,w:1} } }));
    this.log('Navigation cancelled', 'warning');
    document.getElementById('cancelGoalBtn').style.display = 'none';
    this._setNav2Badge('idle');
  }

  _onNavStatus(msg) {
    if (!msg.status_list || msg.status_list.length === 0) {
      this._setNav2Badge('idle');
      document.getElementById('cancelGoalBtn').style.display = 'none';
      return;
    }
    const code  = msg.status_list[msg.status_list.length - 1].status;
    const map   = { 1:'executing', 2:'executing', 3:'recovering', 4:'succeeded', 5:'idle', 6:'failed' };
    this._setNav2Badge(map[code] || 'idle');
    document.getElementById('cancelGoalBtn').style.display = (code === 1 || code === 2) ? 'inline-block' : 'none';
  }

  _setNav2Badge(state) {
    const badge = document.getElementById('nav2Badge');
    if (!badge) return;
    const labels = { idle:'⬜ Nav: Idle', executing:'🟢 Nav: Executing', recovering:'🟡 Nav: Recovering', succeeded:'🔵 Nav: Succeeded', failed:'🔴 Nav: Failed' };
    badge.textContent = labels[state] || '⬜ Nav: Idle';
    badge.className   = `nav2-badge ${state}`;
  }

  // ── Lidar canvas ──────────────────────────────────────────────────────

  _onScan(msg) {
    this._lidarRanges   = msg.ranges;
    this._lidarAngleMin = msg.angle_min;
    this._lidarAngleInc = msg.angle_increment;
    const minR = Math.min(...msg.ranges.filter(r => r > 0 && isFinite(r)));
    if (minR < 0.5 && (!this._lastScanWarn || Date.now() - this._lastScanWarn > 5000)) {
      this.log(`⚠️ Obstacle at ${minR.toFixed(2)}m`, 'warning');
      this._lastScanWarn = Date.now();
    }
  }

  _startLidarRender() {
    const canvas = document.getElementById('lidarCanvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    const cx = W / 2, cy = H / 2;
    const maxR = Math.min(cx, cy) - 4;
    const scale = maxR / 4.0; // 4m = full radius

    const render = () => {
      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = '#0a0f1e';
      ctx.fillRect(0, 0, W, H);

      // Range rings
      [1,2,3,4].forEach(m => {
        ctx.beginPath(); ctx.arc(cx, cy, m * scale, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(59,130,246,0.2)'; ctx.lineWidth = 1; ctx.stroke();
        ctx.fillStyle = 'rgba(148,163,184,0.4)'; ctx.font = '9px monospace';
        ctx.fillText(`${m}m`, cx + m * scale + 2, cy - 2);
      });

      // Cross-hairs
      ctx.strokeStyle = 'rgba(59,130,246,0.15)';
      ctx.beginPath(); ctx.moveTo(cx,0); ctx.lineTo(cx,H); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0,cy); ctx.lineTo(W,cy); ctx.stroke();

      // Scan points
      this._lidarRanges.forEach((r, i) => {
        if (!isFinite(r) || r <= 0 || r > 4) return;
        const angle = this._lidarAngleMin + i * this._lidarAngleInc;
        const px = cx + r * scale * Math.cos(angle);
        const py = cy - r * scale * Math.sin(angle);
        ctx.beginPath(); ctx.arc(px, py, 2, 0, Math.PI * 2);
        ctx.fillStyle = r < 0.5 ? '#ef4444' : r < 1.0 ? '#f59e0b' : '#10b981';
        ctx.fill();
      });

      // Robot dot
      ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI * 2);
      ctx.fillStyle = '#3b82f6'; ctx.fill();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();

      // Goal preview marker
      if (this._goalPreview) {
        const { x, y, yaw } = this._goalPreview;
        // Map coords: canvas X = right (+x), canvas Y = up (+y inverted)
        const gx = cx + x * scale;
        const gy = cy - y * scale;
        const yawRad = yaw * Math.PI / 180;

        // Dashed line from robot to goal
        ctx.setLineDash([4, 4]);
        ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(gx, gy);
        ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 1.5; ctx.stroke();
        ctx.setLineDash([]);

        // Goal circle
        ctx.beginPath(); ctx.arc(gx, gy, 8, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(245,158,11,0.25)'; ctx.fill();
        ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 2; ctx.stroke();

        // Yaw arrow
        const arrowLen = 18;
        const ax = gx + arrowLen * Math.cos(yawRad);
        const ay = gy - arrowLen * Math.sin(yawRad);
        ctx.beginPath(); ctx.moveTo(gx, gy); ctx.lineTo(ax, ay);
        ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 2; ctx.stroke();
        // Arrowhead
        const headLen = 6, headAngle = Math.PI / 6;
        const angle = Math.atan2(gy - ay, gx - ax);
        ctx.beginPath();
        ctx.moveTo(ax, ay);
        ctx.lineTo(ax + headLen * Math.cos(angle - headAngle), ay + headLen * Math.sin(angle - headAngle));
        ctx.lineTo(ax + headLen * Math.cos(angle + headAngle), ay + headLen * Math.sin(angle + headAngle));
        ctx.closePath();
        ctx.fillStyle = '#f59e0b'; ctx.fill();

        // Label
        ctx.fillStyle = '#f59e0b';
        ctx.font = 'bold 10px monospace';
        ctx.fillText(`(${x},${y})`, gx + 10, gy - 10);
      }

      requestAnimationFrame(render);
    };
    requestAnimationFrame(render);
  }

  // ── Camera ────────────────────────────────────────────────────────────

  _toggleCamera() {
    const img         = document.getElementById('cameraImg');
    const placeholder = document.getElementById('cameraPlaceholder');
    const btn         = document.getElementById('cameraToggleBtn');
    const statusTxt   = placeholder.querySelector('p:last-child');

    if (img.src && img.src !== window.location.href) {
      // Pause
      img.src = ''; img.style.display = 'none';
      placeholder.style.display = 'flex';
      statusTxt.textContent = 'Camera feed paused';
      btn.textContent = '▶ Start Feed';
      return;
    }

    // Try primary URL, fall back to alternate topic namespace
    const tryUrl = (url, isFallback) => {
      img.src = url;
      img.style.display = 'block';
      placeholder.style.display = 'none';
      btn.textContent = '⏸ Pause Feed';
      statusTxt.textContent = 'Camera feed paused';

      img.onerror = () => {
        if (!isFallback && this.config.cameraUrlFallback) {
          this.log('Camera: primary topic failed, trying fallback…', 'warning');
          tryUrl(this.config.cameraUrlFallback, true);
        } else {
          img.src = ''; img.style.display = 'none';
          placeholder.style.display = 'flex';
          statusTxt.textContent = '📷 Stream unavailable — is web_video_server running?';
          btn.textContent = '▶ Start Feed';
          this.log('Camera stream unavailable. Ensure web_video_server is launched and Gazebo camera is active.', 'error');
        }
      };
    };

    tryUrl(this.config.cameraUrl, false);
  }

  // ── Joint states ──────────────────────────────────────────────────────

  updateJointStates(msg) {
    const li = msg.name.indexOf('left_wheel_joint');
    const ri = msg.name.indexOf('right_wheel_joint');
    if (li !== -1) this.metrics.leftRpm  = (msg.velocity[li] * 60) / (2 * Math.PI);
    if (ri !== -1) this.metrics.rightRpm = (msg.velocity[ri] * 60) / (2 * Math.PI);
  }

  // ── Metrics display ───────────────────────────────────────────────────

  startMetricsUpdate() {
    setInterval(() => this.updateMetricsDisplay(), 1000);
  }

  updateMetricsDisplay() {
    const set = (id, val) => { const el = document.getElementById(id); if (el) el.textContent = val; };
    set('leftRpm',    this.metrics.leftRpm.toFixed(1));
    set('rightRpm',   this.metrics.rightRpm.toFixed(1));
    set('linearVel',  this.metrics.linearVel.toFixed(2));
    set('angularVel', this.metrics.angularVel.toFixed(2));
    // Battery — label + bar
    const bat = this.metrics.batteryLevel;
    set('batteryLevel', `${bat}%`);
    const batBar = document.getElementById('batBar');
    if (batBar) {
      batBar.style.width = `${Math.min(bat, 100)}%`;
      batBar.className   = 'metric-bar-fill' + (bat < 20 ? ' crit' : bat < 40 ? ' warn' : '');
      batBar.style.background = bat >= 40 ? 'var(--success-color)' : '';
    }
    this._updateBar('cpuBar', 'cpuUsage',  this.metrics.cpu);
    this._updateBar('memBar', 'memUsage',  this.metrics.memory);
    const temp = this.metrics.temperature;
    set('temperature', temp !== null && temp !== undefined ? `${temp}` : 'N/A');
  }

  _updateBar(barId, labelId, value) {
    const bar = document.getElementById(barId), label = document.getElementById(labelId);
    if (!bar || !label) return;
    if (value === null || value === undefined) { label.textContent = 'N/A'; bar.style.width = '0%'; return; }
    label.textContent = `${value.toFixed(1)}%`;
    bar.style.width   = `${Math.min(value, 100)}%`;
    bar.className     = 'metric-bar-fill' + (value > 90 ? ' crit' : value > 70 ? ' warn' : '');
  }

  // ── Connection status ─────────────────────────────────────────────────

  updateConnectionStatus(status) {
    const indicator     = document.querySelector('.status-indicator');
    const text          = document.getElementById('connectionStatusText');
    const connectBtn    = document.getElementById('connectBtn');
    const disconnectBtn = document.getElementById('disconnectBtn');
    indicator.className = `status-indicator status-${status}`;
    text.textContent    = { connected:'Connected', connecting:'Connecting…', disconnected:'Disconnected' }[status] || status;
    connectBtn.style.display    = status === 'connected' ? 'none' : 'inline-block';
    disconnectBtn.style.display = status === 'connected' ? 'inline-block' : 'none';
  }

  // ── Log ───────────────────────────────────────────────────────────────

  log(message, type = 'info') {
    const container = document.getElementById('logContainer');
    const el = document.createElement('div');
    el.className = `log-entry log-${type}`;
    el.innerHTML = `<span style="color:#64748b">[${new Date().toLocaleTimeString()}]</span> ${message}`;
    container.appendChild(el);
    container.scrollTop = container.scrollHeight;
    while (container.children.length > 100) container.removeChild(container.firstChild);
    console.log(`[${type.toUpperCase()}] ${message}`);
  }

  _exportLog() {
    const lines = [...document.getElementById('logContainer').children].map(el => el.textContent.trim()).join('\n');
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([lines], { type: 'text/plain' }));
    a.download = `robot_log_${Date.now()}.txt`;
    a.click();
  }
}

// ── Bootstrap ─────────────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
  window.robotController = new RobotController();

  const keyMap = { KeyW:'upButton', KeyS:'downButton', KeyA:'leftButton', KeyD:'rightButton' };
  document.addEventListener('keydown', (e) => { const id = keyMap[e.code]; if (id) document.getElementById(id).style.transform = 'scale(0.95)'; });
  document.addEventListener('keyup',   (e) => { const id = keyMap[e.code]; if (id) document.getElementById(id).style.transform = ''; });
  document.addEventListener('keydown', (e) => {
    if (e.key === 'F11') { e.preventDefault(); document.fullscreenElement ? document.exitFullscreen() : document.documentElement.requestFullscreen(); }
  });
});
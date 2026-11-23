// autoJetsonBot — Controls Manager
export class ControlsManager {
  constructor(app) {
    this.app = app;
    this.cmdVelTopic = null;
  }

  setupTopics() {
    this.cmdVelTopic = new ROSLIB.Topic({
      ros: this.app.ros,
      name: this.app.config.topics.cmdVel,
      messageType: 'geometry_msgs/Twist'
    });
  }

  teardownTopics() {
    this.cmdVelTopic = null;
  }

  setupEventListeners() {
    // Scaling sliders
    document.getElementById('linearScale').addEventListener('input', (e) => {
      this.app.linearScale = parseFloat(e.target.value);
      document.getElementById('linearValue').textContent = this.app.linearScale.toFixed(2);
    });
    document.getElementById('angularScale').addEventListener('input', (e) => {
      this.app.angularScale = parseFloat(e.target.value);
      document.getElementById('angularValue').textContent = this.app.angularScale.toFixed(2);
    });

    // Settings inputs live update
    document.getElementById('maxLinearVel').addEventListener('change', (e) => {
      this.app.config.maxLinearVel = parseFloat(e.target.value) || 1.0;
      this.app._saveSettings();
    });
    document.getElementById('maxAngularVel').addEventListener('change', (e) => {
      this.app.config.maxAngularVel = parseFloat(e.target.value) || 2.0;
      this.app._saveSettings();
    });
    document.getElementById('publishRate').addEventListener('change', (e) => {
      const hz = parseFloat(e.target.value) || 10;
      this.app.config.publishRate = Math.round(1000 / hz);
      this.app._saveSettings();
    });

    this.setupDPadControls();
    this.setupKeyboardControls();
  }

  setupDPadControls() {
    ['up', 'down', 'left', 'right'].forEach(dir => {
      const btn = document.getElementById(`${dir}Button`);
      if (!btn) return;
      btn.addEventListener('mousedown',  (e) => this.startDirection(e, dir));
      btn.addEventListener('mouseup',    (e) => this.stopDirection(e));
      btn.addEventListener('mouseleave', (e) => this.stopDirection(e));
      btn.addEventListener('touchstart', (e) => this.startDirection(e, dir));
      btn.addEventListener('touchend',   (e) => this.stopDirection(e));
      btn.addEventListener('contextmenu',(e) => e.preventDefault());
    });
  }

  setupKeyboardControls() {
    const keyMap = {
      'w': 'up', 'arrowup': 'up',
      's': 'down', 'arrowdown': 'down',
      'a': 'left', 'arrowleft': 'left',
      'd': 'right', 'arrowright': 'right'
    };
    
    document.addEventListener('keydown', (e) => {
      // Ignore shortcut keys if user is typing in form fields
      if (document.activeElement.tagName === 'INPUT') {
        return;
      }
      if (e.repeat) return;
      const dir = keyMap[e.key.toLowerCase()];
      if (dir) {
        this.startDirection(e, dir);
        return;
      }
      if (e.key === ' ') {
        this.emergencyStop();
        e.preventDefault();
      }
      if (e.key.toLowerCase() === 'g') {
        document.getElementById('sendGoalBtn').click();
      }
      if (e.key.toLowerCase() === 'c') {
        document.getElementById('cancelGoalBtn').click();
      }
      if (e.key.toLowerCase() === 'm') {
        document.getElementById('saveMap').click();
      }
    });

    document.addEventListener('keyup', (e) => {
      if (keyMap[e.key.toLowerCase()]) {
        this.stopDirection(e);
      }
    });
  }

  startDirection(event, direction) {
    if (event) event.preventDefault();
    if (!this.app.isConnected || this.app.isControlling) return;
    this.app.isControlling = true;
    const map = { up: [1, 0], down: [-1, 0], left: [0, 1], right: [0, -1] };
    [this.app.currentLinear, this.app.currentAngular] = map[direction];
    
    const btn = document.getElementById(`${direction}Button`);
    if (btn) btn.classList.add('active');
    
    this.app.publishInterval = setInterval(() => this.publishVelocity(), this.app.config.publishRate);
    this.app.log(`Moving ${direction}`, 'info');
  }

  stopDirection(event) {
    if (event) event.preventDefault();
    if (!this.app.isControlling) return;
    this.app.isControlling = false;
    this.app.currentLinear = this.app.currentAngular = 0;
    if (this.app.publishInterval) {
      clearInterval(this.app.publishInterval);
      this.app.publishInterval = null;
    }
    ['up', 'down', 'left', 'right'].forEach(d => {
      const b = document.getElementById(`${d}Button`);
      if (b) b.classList.remove('active');
    });
    this.publishVelocity();
    this.app.log('Stopped', 'info');
  }

  emergencyStop() {
    this.stopDirection(new Event('emergency'));
    this.app.navigation.cancelNavGoal();
    this.app.log('EMERGENCY STOP — All movement terminated', 'warning');
    const btn = document.getElementById('emergencyStop');
    if (btn) {
      btn.style.background = '#ef4444';
      setTimeout(() => { btn.style.background = ''; }, 1000);
    }
  }

  publishVelocity() {
    if (!this.cmdVelTopic || !this.app.isConnected) return;
    const twist = new ROSLIB.Message({
      linear:  { x: this.app.currentLinear  * this.app.linearScale  * this.app.config.maxLinearVel,  y: 0, z: 0 },
      angular: { x: 0, y: 0, z: this.app.currentAngular * this.app.angularScale * this.app.config.maxAngularVel },
    });
    this.cmdVelTopic.publish(twist);
    this.app.metrics.linearVel  = twist.linear.x;
    this.app.metrics.angularVel = twist.angular.z;
  }

  resetOdometry() {
    if (!this.app.isConnected) {
      this.app.log('Not connected', 'warning');
      return;
    }
    const t = new ROSLIB.Topic({
      ros: this.app.ros,
      name: '/initialpose',
      messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    });
    t.publish(new ROSLIB.Message({
      header: { frame_id: 'map' },
      pose: {
        pose: {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 }
        }
      }
    }));
    this.app.log('Odometry reset → /initialpose', 'info');
  }

  async saveMap() {
    if (!this.app.isConnected) {
      this.app.log('Not connected', 'warning');
      return;
    }
    if (this._savingMap) return;  // the M key, or a second click
    this._savingMap = true;
    const btn = document.getElementById('saveMap');
    if (btn) btn.disabled = true;
    try {
      await this._saveMap();
    } finally {
      this._savingMap = false;
      if (btn) btn.disabled = false;
    }
  }

  async _saveMap() {
    // Read the launch's config again: the page may be older than the launch now running.
    await this.app.fetchConfig();
    const { map_name: mapName, map_dir: mapDir, mode } = this.app.config;
    if (!mapName || !mapDir) {
      this.app.log('❌ Map NOT saved: no map name, because config.json did not load', 'error');
      return;
    }
    const fullPath = `${mapDir}/${mapName}`;
    this.app.log(`Saving map → ${fullPath}…`, 'info');
    const started = performance.now();

    // Nav2's map saver, which mapping mode keeps running (main.launch.py). It answers
    // result: false when the save fails. The thresholds are left out, so it uses its own:
    // free_thresh 0.196 keeps unknown cells unknown when the map is loaded.
    const request = new ROSLIB.ServiceRequest({
      map_topic: 'map', map_url: fullPath, image_format: 'pgm', map_mode: 'trinary'
    });
    // On a connection of its own: rosbridge handles each connection's messages in turn, so on
    // the teleop connection a save waiting for /map held drive and stop commands until it ended.
    const url = this.app.config.rosbridgeUrl;
    const ros = new ROSLIB.Ros({ url });
    const answer = await new Promise((resolve) => {
      // roslib calls no callback if the connection drops, so stop waiting after 60 s.
      const timer = setTimeout(() => resolve({}), 60000);
      const finish = (result) => { clearTimeout(timer); resolve(result); };
      ros.on('error', () => finish({ err: `no connection to ${url}` }));
      ros.on('close', () => finish({ err: 'the connection closed before an answer' }));
      ros.on('connection', () => {
        const svc = new ROSLIB.Service({
          ros, name: '/map_saver/save_map', serviceType: 'nav2_msgs/srv/SaveMap'
        });
        svc.callService(request, (res) => finish({ res }), (err) => finish({ err: String(err) }));
      });
    });
    try { ros.close(); } catch (_) {}
    const secs = ((performance.now() - started) / 1000).toFixed(1);
    if (answer.res && answer.res.result === true) {
      this.app.log(`✅ Map saved: ${fullPath}.yaml and .pgm (${secs} s)`, 'info');
    } else if (answer.res) {
      this.app.log(`❌ Map NOT saved: the map saver failed after ${secs} s ` +
                   '(./robot.sh logs says why)', 'error');
    } else if (answer.err !== undefined) {
      const hint = mode && mode !== 'mapping' ? ` (it runs in mapping mode, and this is ${mode})` : '';
      this.app.log(`❌ Map NOT saved: ${answer.err}${hint}`, 'error');
    } else {
      this.app.log(`❌ No answer about the save in ${secs} s: check the map's files before using it`,
                   'error');
    }
  }
}

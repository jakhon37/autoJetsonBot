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

  saveMap() {
    if (!this.app.isConnected) {
      this.app.log('Not connected', 'warning');
      return;
    }
    const mapName  = this.app.config.map_name || 'lab_map';
    const mapDir   = this.app.config.map_dir  || '/autonomous_ROS/src/jetson_bot_bringup/worlds';
    const fullPath = `${mapDir}/${mapName}`;
    this.app.log(`Saving map → ${fullPath}`, 'info');
    
    const svc = new ROSLIB.Service({
      ros: this.app.ros,
      name: '/slam_toolbox/save_map',
      serviceType: 'slam_toolbox/srv/SaveMap'
    });
    
    svc.callService(
      new ROSLIB.ServiceRequest({ name: { data: fullPath } }),
      ()  => this.app.log(`✅ Map saved: ${fullPath}`, 'info'),
      (e) => {
        this.app.log(`❌ Map save failed: ${e} — retrying with flat string`, 'error');
        svc.callService(
          new ROSLIB.ServiceRequest({ name: fullPath }),
          () => this.app.log(`✅ Map saved (fallback): ${fullPath}`, 'info'),
          (e2) => this.app.log(`❌ Map save failed again: ${e2}`, 'error')
        );
      }
    );
  }
}

// autoJetsonBot — Telemetry Manager
export class TelemetryManager {
  constructor(app) {
    this.app = app;
    this.jointStatesTopic = null;
    this.batteryTopic = null;
    this.systemTopic = null;
    this.odomTopic = null;
  }

  setupTopics() {
    const ros = this.app.ros;
    
    // Joint States (Wheel RPM)
    this.jointStatesTopic = new ROSLIB.Topic({
      ros: ros,
      name: this.app.config.topics.jointStates,
      messageType: 'sensor_msgs/JointState'
    });
    this.jointStatesTopic.subscribe((msg) => this.updateJointStates(msg));

    // Battery State
    this.batteryTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/battery_state',
      messageType: 'sensor_msgs/BatteryState'
    });
    this.batteryTopic.subscribe((msg) => {
      this.app.metrics.batteryLevel = Math.round(msg.percentage * 100);
    });

    // System Telemetry
    this.systemTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/telemetry/system',
      messageType: 'std_msgs/String'
    });
    this.systemTopic.subscribe((msg) => {
      try {
        const d = JSON.parse(msg.data);
        this.app.metrics.cpu         = d.cpu;
        this.app.metrics.memory      = d.memory;
        this.app.metrics.temperature = d.temperature;
        if (d.battery !== undefined) {
          this.app.metrics.batteryLevel = d.battery;
        }
      } catch (_) {}
    });

    // Odometry (Velocity metrics and pose fallback)
    this.odomTopic = new ROSLIB.Topic({
      ros: ros,
      name: this.app.config.topics.odom,
      messageType: 'nav_msgs/Odometry',
      throttle_rate: 100
    });
    this.odomTopic.subscribe((msg) => {
      // Metrics
      this.app.metrics.linearVel  = msg.twist.twist.linear.x;
      this.app.metrics.angularVel = msg.twist.twist.angular.z;
      
      // Pose tracking (as fallback for map-to-robot frame)
      this.app._odomPose.x = msg.pose.pose.position.x;
      this.app._odomPose.y = msg.pose.pose.position.y;
      const q = msg.pose.pose.orientation;
      this.app._odomPose.yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    });
  }

  teardownTopics() {
    [this.jointStatesTopic, this.batteryTopic, this.systemTopic, this.odomTopic].forEach(t => {
      if (t) {
        try { t.unsubscribe(); } catch (_) {}
      }
    });
    this.jointStatesTopic = this.batteryTopic = this.systemTopic = this.odomTopic = null;
  }

  updateJointStates(msg) {
    const li = msg.name.indexOf('left_wheel_joint');
    const ri = msg.name.indexOf('right_wheel_joint');
    if (li !== -1) {
      this.app.metrics.leftRpm = (msg.velocity[li] * 60) / (2 * Math.PI);
    }
    if (ri !== -1) {
      this.app.metrics.rightRpm = (msg.velocity[ri] * 60) / (2 * Math.PI);
    }
  }

  updateMetricsDisplay() {
    const set = (id, val) => {
      const el = document.getElementById(id);
      if (el) el.textContent = val;
    };

    set('leftRpm',    this.app.metrics.leftRpm.toFixed(1));
    set('rightRpm',   this.app.metrics.rightRpm.toFixed(1));
    set('linearVel',  this.app.metrics.linearVel.toFixed(2));
    set('angularVel', this.app.metrics.angularVel.toFixed(2));

    const bat = this.app.metrics.batteryLevel;
    set('batteryLevel', `${bat}%`);
    
    const batBar = document.getElementById('batBar');
    if (batBar) {
      batBar.style.width = `${Math.min(bat, 100)}%`;
      batBar.className   = 'metric-bar-fill' + (bat < 20 ? ' crit' : bat < 40 ? ' warn' : '');
      batBar.style.background = bat >= 40 ? 'var(--success-color)' : '';
    }

    this._updateBar('cpuBar', 'cpuUsage',  this.app.metrics.cpu);
    this._updateBar('memBar', 'memUsage',  this.app.metrics.memory);
    set('temperature', this.app.metrics.temperature !== null ? `${this.app.metrics.temperature}` : 'N/A');
  }

  _updateBar(barId, labelId, value) {
    const bar = document.getElementById(barId);
    const label = document.getElementById(labelId);
    if (!bar || !label) return;
    if (value === null) {
      label.textContent = 'N/A';
      bar.style.width = '0%';
      return;
    }
    label.textContent = `${value.toFixed(1)}%`;
    bar.style.width = `${Math.min(value, 100)}%`;
    bar.className = 'metric-bar-fill' + (value > 90 ? ' crit' : value > 70 ? ' warn' : '');
  }
}

// Modern Robot Controller JavaScript
class RobotController {
  constructor() {
    this.ros = null;
    this.cmdVel = null;
    this.jointStatesTopic = null;
    this.scanTopic = null;
    
    // Control state
    this.isConnected = false;
    this.isControlling = false;
    this.currentLinear = 0.0;
    this.currentAngular = 0.0;
    this.linearScale = 0.5;
    this.angularScale = 0.5;
    this.publishInterval = null;
    
    // Configuration
    this.config = {
      rosbridgeUrl: 'ws://localhost:9090',
      publishRate: 100, // ms
      maxLinearVel: 1.0,
      maxAngularVel: 2.0,
      topics: {
        cmdVel: '/diff_cont/cmd_vel_unstamped',
        jointStates: '/joint_states',
        scan: '/scan'
      }
    };
    
    // Metrics
    this.metrics = {
      leftRpm: 0,
      rightRpm: 0,
      linearVel: 0,
      angularVel: 0,
      batteryLevel: 100,
      cpuUsage: 0,
      memoryUsage: 0,
      lastUpdate: Date.now()
    };
    
    this.init();
  }
  
  init() {
    this.setupEventListeners();
    this.setupUI();
    this.autoDetectROSBridge();
    this.startMetricsUpdate();
    this.log('Robot Controller initialized', 'info');
  }
  
  setupEventListeners() {
    // Connection controls
    document.getElementById('connectBtn').addEventListener('click', () => this.connect());
    document.getElementById('disconnectBtn').addEventListener('click', () => this.disconnect());
    
    // Speed controls
    document.getElementById('linearScale').addEventListener('input', (e) => {
      this.linearScale = parseFloat(e.target.value);
      document.getElementById('linearValue').textContent = this.linearScale.toFixed(2);
    });
    
    document.getElementById('angularScale').addEventListener('input', (e) => {
      this.angularScale = parseFloat(e.target.value);
      document.getElementById('angularValue').textContent = this.angularScale.toFixed(2);
    });
    
    // D-pad controls
    this.setupDPadControls();
    
    // Keyboard controls
    this.setupKeyboardControls();
    
    // Emergency stop
    document.getElementById('emergencyStop').addEventListener('click', () => this.emergencyStop());
  }
  
  setupDPadControls() {
    const directions = ['up', 'down', 'left', 'right'];
    
    directions.forEach(direction => {
      const button = document.getElementById(`${direction}Button`);
      
      // Mouse events
      button.addEventListener('mousedown', (e) => this.startDirection(e, direction));
      button.addEventListener('mouseup', (e) => this.stopDirection(e));
      button.addEventListener('mouseleave', (e) => this.stopDirection(e));
      
      // Touch events
      button.addEventListener('touchstart', (e) => this.startDirection(e, direction));
      button.addEventListener('touchend', (e) => this.stopDirection(e));
      
      // Prevent context menu
      button.addEventListener('contextmenu', (e) => e.preventDefault());
    });
  }
  
  setupKeyboardControls() {
    document.addEventListener('keydown', (e) => {
      if (e.repeat) return;
      
      switch(e.key.toLowerCase()) {
        case 'w':
        case 'arrowup':
          this.startDirection(e, 'up');
          break;
        case 's':
        case 'arrowdown':
          this.startDirection(e, 'down');
          break;
        case 'a':
        case 'arrowleft':
          this.startDirection(e, 'left');
          break;
        case 'd':
        case 'arrowright':
          this.startDirection(e, 'right');
          break;
        case ' ':
          this.emergencyStop();
          e.preventDefault();
          break;
      }
    });
    
    document.addEventListener('keyup', (e) => {
      switch(e.key.toLowerCase()) {
        case 'w':
        case 's':
        case 'a':
        case 'd':
        case 'arrowup':
        case 'arrowdown':
        case 'arrowleft':
        case 'arrowright':
          this.stopDirection(e);
          break;
      }
    });
  }
  
  setupUI() {
    // Initialize UI elements
    document.getElementById('linearValue').textContent = this.linearScale.toFixed(2);
    document.getElementById('angularValue').textContent = this.angularScale.toFixed(2);
    document.getElementById('rosbridgeUrl').value = this.config.rosbridgeUrl;
    
    this.updateConnectionStatus('disconnected');
  }
  
  autoDetectROSBridge() {
    const hostname = window.location.hostname;
    const possibleUrls = [
      `ws://${hostname}:9090`,
      'ws://localhost:9090',
      'ws://127.0.0.1:9090',
      'ws://192.168.219.150:9090',
      'ws://192.168.219.151:9090'
    ];
    
    document.getElementById('rosbridgeUrl').value = possibleUrls[0];
    this.config.rosbridgeUrl = possibleUrls[0];
  }
  
  connect() {
    const url = document.getElementById('rosbridgeUrl').value;
    this.config.rosbridgeUrl = url;
    
    this.updateConnectionStatus('connecting');
    this.log(`Connecting to ${url}...`, 'info');
    
    if (this.ros) {
      this.ros.close();
    }
    
    this.ros = new ROSLIB.Ros({
      url: url
    });
    
    this.ros.on('connection', () => {
      this.isConnected = true;
      this.updateConnectionStatus('connected');
      this.setupROSTopics();
      this.log('Connected to robot successfully', 'info');
    });
    
    this.ros.on('error', (error) => {
      this.isConnected = false;
      this.updateConnectionStatus('disconnected');
      this.log(`Connection error: ${error}`, 'error');
    });
    
    this.ros.on('close', () => {
      this.isConnected = false;
      this.updateConnectionStatus('disconnected');
      this.log('Connection closed', 'warning');
    });
  }
  
  disconnect() {
    if (this.ros) {
      this.ros.close();
    }
    this.isConnected = false;
    this.updateConnectionStatus('disconnected');
    this.log('Disconnected from robot', 'info');
  }
  
  setupROSTopics() {
    // Command velocity publisher
    this.cmdVel = new ROSLIB.Topic({
      ros: this.ros,
      name: this.config.topics.cmdVel,
      messageType: 'geometry_msgs/Twist'
    });
    
    // Joint states subscriber
    this.jointStatesTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: this.config.topics.jointStates,
      messageType: 'sensor_msgs/JointState'
    });
    
    this.jointStatesTopic.subscribe((message) => {
      this.updateJointStates(message);
    });
    
    // Scan subscriber
    this.scanTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: this.config.topics.scan,
      messageType: 'sensor_msgs/LaserScan'
    });
    
    this.scanTopic.subscribe((message) => {
      this.updateScanData(message);
    });
    
    this.log('ROS topics initialized', 'info');
  }
  
  startDirection(event, direction) {
    event.preventDefault();
    
    if (!this.isConnected || this.isControlling) return;
    
    this.isControlling = true;
    
    switch (direction) {
      case 'up':
        this.currentLinear = 1.0;
        this.currentAngular = 0.0;
        break;
      case 'down':
        this.currentLinear = -1.0;
        this.currentAngular = 0.0;
        break;
      case 'left':
        this.currentLinear = 0.0;
        this.currentAngular = 1.0;
        break;
      case 'right':
        this.currentLinear = 0.0;
        this.currentAngular = -1.0;
        break;
    }
    
    // Visual feedback
    document.getElementById(`${direction}Button`).classList.add('active');
    
    // Start publishing
    this.publishInterval = setInterval(() => this.publishVelocity(), this.config.publishRate);
    
    this.log(`Started moving ${direction}`, 'info');
  }
  
  stopDirection(event) {
    event.preventDefault();
    
    if (!this.isControlling) return;
    
    this.isControlling = false;
    this.currentLinear = 0.0;
    this.currentAngular = 0.0;
    
    // Clear interval
    if (this.publishInterval) {
      clearInterval(this.publishInterval);
      this.publishInterval = null;
    }
    
    // Remove visual feedback
    ['up', 'down', 'left', 'right'].forEach(dir => {
      document.getElementById(`${dir}Button`).classList.remove('active');
    });
    
    // Send stop command
    this.publishVelocity();
    
    this.log('Stopped movement', 'info');
  }
  
  emergencyStop() {
    this.stopDirection(new Event('emergency'));
    this.log('EMERGENCY STOP activated', 'warning');
    
    // Visual feedback
    const stopBtn = document.getElementById('emergencyStop');
    stopBtn.style.background = '#ef4444';
    setTimeout(() => {
      stopBtn.style.background = '';
    }, 1000);
  }
  
  publishVelocity() {
    if (!this.cmdVel || !this.isConnected) return;
    
    const twist = new ROSLIB.Message({
      linear: {
        x: this.currentLinear * this.linearScale * this.config.maxLinearVel,
        y: 0.0,
        z: 0.0
      },
      angular: {
        x: 0.0,
        y: 0.0,
        z: this.currentAngular * this.angularScale * this.config.maxAngularVel
      }
    });
    
    this.cmdVel.publish(twist);
    
    // Update metrics
    this.metrics.linearVel = twist.linear.x;
    this.metrics.angularVel = twist.angular.z;
    this.metrics.lastUpdate = Date.now();
  }
  
  updateJointStates(message) {
    const leftIdx = message.name.indexOf('left_wheel_joint');
    const rightIdx = message.name.indexOf('right_wheel_joint');
    
    if (leftIdx !== -1 && rightIdx !== -1) {
      this.metrics.leftRpm = (message.velocity[leftIdx] * 60) / (2 * Math.PI);
      this.metrics.rightRpm = (message.velocity[rightIdx] * 60) / (2 * Math.PI);
    }
  }
  
  updateScanData(message) {
    // Process laser scan data for obstacle detection
    const ranges = message.ranges;
    const minRange = Math.min(...ranges.filter(r => r > 0));
    
    // Update obstacle warning if too close
    if (minRange < 0.5) {
      this.log(`Obstacle detected at ${minRange.toFixed(2)}m`, 'warning');
    }
  }
  
  updateConnectionStatus(status) {
    const indicator = document.querySelector('.status-indicator');
    const statusText = document.getElementById('connectionStatusText');
    const connectBtn = document.getElementById('connectBtn');
    const disconnectBtn = document.getElementById('disconnectBtn');
    
    indicator.className = `status-indicator status-${status}`;
    
    switch (status) {
      case 'connected':
        statusText.textContent = 'Connected';
        connectBtn.style.display = 'none';
        disconnectBtn.style.display = 'inline-block';
        break;
      case 'connecting':
        statusText.textContent = 'Connecting...';
        connectBtn.style.display = 'none';
        disconnectBtn.style.display = 'inline-block';
        break;
      case 'disconnected':
        statusText.textContent = 'Disconnected';
        connectBtn.style.display = 'inline-block';
        disconnectBtn.style.display = 'none';
        break;
    }
  }
  
  startMetricsUpdate() {
    setInterval(() => {
      this.updateMetricsDisplay();
    }, 1000);
  }
  
  updateMetricsDisplay() {
    document.getElementById('leftRpm').textContent = this.metrics.leftRpm.toFixed(1);
    document.getElementById('rightRpm').textContent = this.metrics.rightRpm.toFixed(1);
    document.getElementById('linearVel').textContent = this.metrics.linearVel.toFixed(2);
    document.getElementById('angularVel').textContent = this.metrics.angularVel.toFixed(2);
    document.getElementById('batteryLevel').textContent = this.metrics.batteryLevel;
    
    // Simulate system metrics (in real implementation, get from robot)
    this.metrics.cpuUsage = Math.random() * 30 + 20;
    this.metrics.memoryUsage = Math.random() * 40 + 30;
    
    document.getElementById('cpuUsage').textContent = this.metrics.cpuUsage.toFixed(1);
    document.getElementById('memoryUsage').textContent = this.metrics.memoryUsage.toFixed(1);
  }
  
  log(message, type = 'info') {
    const logContainer = document.getElementById('logContainer');
    const timestamp = new Date().toLocaleTimeString();
    
    const logEntry = document.createElement('div');
    logEntry.className = `log-entry log-${type}`;
    logEntry.innerHTML = `<span style="color: #64748b;">[${timestamp}]</span> ${message}`;
    
    logContainer.appendChild(logEntry);
    logContainer.scrollTop = logContainer.scrollHeight;
    
    // Keep only last 50 entries
    while (logContainer.children.length > 50) {
      logContainer.removeChild(logContainer.firstChild);
    }
    
    console.log(`[${type.toUpperCase()}] ${message}`);
  }
}

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
  window.robotController = new RobotController();
});
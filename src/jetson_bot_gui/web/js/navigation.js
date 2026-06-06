// autoJetsonBot — Navigation Manager
export class NavigationManager {
  constructor(app) {
    this.app = app;
    this.pathTopic = null;
    this.navStatusTopic = null;
    this.navActionClient = null;
  }

  setupTopics() {
    const ros = this.app.ros;

    // Plan / Path topic
    this.pathTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/plan',
      messageType: 'nav_msgs/Path',
      throttle_rate: 1000
    });
    this.pathTopic.subscribe((msg) => {
      this.app._currentPath = msg.poses.map(p => ({
        x: p.pose.position.x,
        y: p.pose.position.y
      }));
    });

    // Nav2 Goal Status topic
    this.navStatusTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/navigate_to_pose/_action/status',
      messageType: 'action_msgs/GoalStatusArray',
      throttle_rate: 500
    });
    this.navStatusTopic.subscribe((msg) => this._onNavStatus(msg));

    // Nav2 Action Client
    this.navActionClient = new ROSLIB.ActionClient({
      ros: ros,
      serverName: '/navigate_to_pose',
      actionName: 'nav2_msgs/action/NavigateToPose'
    });
  }

  teardownTopics() {
    [this.pathTopic, this.navStatusTopic].forEach(t => {
      if (t) {
        try { t.unsubscribe(); } catch (_) {}
      }
    });
    this.pathTopic = this.navStatusTopic = this.navActionClient = null;
  }

  previewNavGoal() {
    const x   = parseFloat(document.getElementById('goalX').value)   || 0;
    const y   = parseFloat(document.getElementById('goalY').value)   || 0;
    const yaw = parseFloat(document.getElementById('goalYaw').value) || 0;
    this.app._goalPreview = { x, y, yaw };
    
    const btn = document.getElementById('previewGoalBtn');
    if (btn) {
      btn.textContent = '👁 Previewing…';
      btn.style.background = 'var(--warning-color)';
    }

    clearTimeout(this.app._previewResetTimer);
    this.app._previewResetTimer = setTimeout(() => {
      this.app._goalPreview = null;
      if (btn) {
        btn.textContent = '👁 Preview';
        btn.style.background = '';
      }
    }, 8000);

    this.app.log(`Preview goal → x:${x} y:${y} yaw:${yaw}°`, 'info');
  }

  sendNavGoal() {
    if (!this.app.isConnected) {
      this.app.log('Not connected', 'warning');
      return;
    }
    const x   = parseFloat(document.getElementById('goalX').value)   || 0;
    const y   = parseFloat(document.getElementById('goalY').value)   || 0;
    const yaw = (parseFloat(document.getElementById('goalYaw').value) || 0) * Math.PI / 180;
    const qz  = Math.sin(yaw / 2);
    const qw  = Math.cos(yaw / 2);
    
    this.app._activeGoal = {
      x, y, yaw: (parseFloat(document.getElementById('goalYaw').value) || 0)
    };
    
    const t = new ROSLIB.Topic({
      ros: this.app.ros,
      name: '/goal_pose',
      messageType: 'geometry_msgs/PoseStamped'
    });
    t.publish(new ROSLIB.Message({
      header: { frame_id: 'map' },
      pose: {
        position: { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: qz, w: qw }
      }
    }));
    
    this.app.log(`Nav goal → x:${x} y:${y} yaw:${(yaw * 180 / Math.PI).toFixed(1)}°`, 'info');
    
    const cancelBtn = document.getElementById('cancelGoalBtn');
    if (cancelBtn) cancelBtn.style.display = 'inline-block';
    
    // Clear preview state
    this.app._goalPreview = null;
    clearTimeout(this.app._previewResetTimer);
    const prevBtn = document.getElementById('previewGoalBtn');
    if (prevBtn) {
      prevBtn.textContent = '👁 Preview';
      prevBtn.style.background = '';
    }
  }

  cancelNavGoal() {
    if (!this.app.isConnected) return;
    
    // Properly cancel Nav2 action goal
    if (this.navActionClient) {
      try { this.navActionClient.cancel(); } catch (_) {}
    }
    
    // Also send zero velocity as safety fallback
    if (this.app.controls && this.app.controls.cmdVelTopic) {
      this.app.controls.cmdVelTopic.publish(new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      }));
    }
    
    this.app._activeGoal = null;
    this.app._currentPath = [];
    this.app.log('Navigation cancelled', 'warning');
    
    const cancelBtn = document.getElementById('cancelGoalBtn');
    if (cancelBtn) cancelBtn.style.display = 'none';
    
    this._setNav2Badge('idle');
  }

  _onNavStatus(msg) {
    if (!msg.status_list || msg.status_list.length === 0) {
      this._setNav2Badge('idle');
      const cancelBtn = document.getElementById('cancelGoalBtn');
      if (cancelBtn) cancelBtn.style.display = 'none';
      return;
    }
    const lastStatus = msg.status_list[msg.status_list.length - 1];
    const code  = lastStatus.status;
    const map   = { 1: 'executing', 2: 'executing', 3: 'recovering', 4: 'succeeded', 5: 'idle', 6: 'failed' };
    const state = map[code] || 'idle';
    
    this._setNav2Badge(state);
    
    if (state === 'succeeded' || state === 'failed') {
      this.app._activeGoal = null;
      this.app._currentPath = [];
    }
    
    const cancelBtn = document.getElementById('cancelGoalBtn');
    if (cancelBtn) {
      cancelBtn.style.display = (code === 1 || code === 2) ? 'inline-block' : 'none';
    }
  }

  _setNav2Badge(state) {
    const badge = document.getElementById('nav2Badge');
    if (!badge) return;
    const labels = {
      idle: '⬜ Nav: Idle',
      executing: '🟢 Nav: Executing',
      recovering: '🟡 Nav: Recovering',
      succeeded: '🔵 Nav: Succeeded',
      failed: '🔴 Nav: Failed'
    };
    badge.textContent = labels[state] || '⬜ Nav: Idle';
    badge.className   = `nav2-badge ${state}`;
  }
}

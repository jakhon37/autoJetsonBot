// autoJetsonBot — Lidar & Canvas Renderer
export class LidarRenderer {
  constructor(app) {
    this.app = app;
    this.scanTopic = null;
    this.amclPoseTopic = null;
    this._lidarRanges = [];
    this._lidarAngleMin = 0;
    this._lidarAngleInc = 0;
    this._animationFrameId = null;
    this._frontDist = null;
  }

  setupTopics() {
    const ros = this.app.ros;

    // Laser Scan Topic
    this.scanTopic = new ROSLIB.Topic({
      ros: ros,
      name: this.app.config.topics.scan,
      messageType: 'sensor_msgs/LaserScan',
      throttle_rate: 100
    });
    this.scanTopic.subscribe((msg) => this._onScan(msg));

    // AMCL Pose (Robot Pose in Map Frame)
    this.amclPoseTopic = new ROSLIB.Topic({
      ros: ros,
      name: this.app.config.topics.amclPose,
      messageType: 'geometry_msgs/PoseWithCovarianceStamped',
      throttle_rate: 50
    });
    this.amclPoseTopic.subscribe((msg) => {
      this.app._mapPose.x = msg.pose.pose.position.x;
      this.app._mapPose.y = msg.pose.pose.position.y;
      const q = msg.pose.pose.orientation;
      this.app._mapPose.yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      // Flag that AMCL is active
      this.app._hasAmcl = true;
    });
  }

  teardownTopics() {
    [this.scanTopic, this.amclPoseTopic].forEach(t => {
      if (t) {
        try { t.unsubscribe(); } catch (_) {}
      }
    });
    this.scanTopic = this.amclPoseTopic = null;
    this.app._hasAmcl = false;
  }

  _onScan(msg) {
    this._lidarRanges   = msg.ranges;
    this._lidarAngleMin = msg.angle_min;
    this._lidarAngleInc = msg.angle_increment;
    
    // Proximity warnings & Front distance calculation
    const validRanges = msg.ranges.filter(r => r > 0 && isFinite(r));
    if (validRanges.length > 0) {
      const minR = Math.min(...validRanges);
      if (minR < 0.5 && (Date.now() - this.app._lastScanWarn > 5000)) {
        this.app.log(`⚠️ Obstacle at ${minR.toFixed(2)}m`, 'warning');
        this.app._lastScanWarn = Date.now();
      }

      // Calculate Front Distance (Average of center +/- 10 degrees)
      // Standard Lidar: 0 rad is Forward. Some start at -PI.
      const centerIdx = Math.floor((0 - msg.angle_min) / msg.angle_increment);
      const span = Math.floor((10 * Math.PI / 180) / msg.angle_increment);
      const frontSector = msg.ranges.slice(Math.max(0, centerIdx - span), Math.min(msg.ranges.length, centerIdx + span));
      const validFront = frontSector.filter(r => r > 0 && isFinite(r));
      this._frontDist = validFront.length > 0 ? Math.min(...validFront) : null;
    }
  }

  start() {
    this._startLidarRender();
  }

  stop() {
    if (this._animationFrameId) {
      cancelAnimationFrame(this._animationFrameId);
      this._animationFrameId = null;
    }
  }

  _startLidarRender() {
    const canvas = document.getElementById('lidarCanvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    const render = () => {
      const W = canvas.width, H = canvas.height;
      const cx = W / 2, cy = H / 2;
      const maxR = Math.min(cx, cy) - 10; // Extra padding for labels
      const scale = maxR / 5.0; // 5 metres fits the canvas

      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = '#0a0f1e';
      ctx.fillRect(0, 0, W, H);

      // ── Background Grid (Square Tactical View) ──
      ctx.strokeStyle = 'rgba(59,130,246,0.05)';
      ctx.lineWidth = 1;
      for (let i = -5; i <= 5; i++) {
        ctx.beginPath(); ctx.moveTo(cx + i * scale, 0); ctx.lineTo(cx + i * scale, H); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, cy + i * scale); ctx.lineTo(W, cy + i * scale); ctx.stroke();
      }

      // ── Radar Lines (30/60 degrees) ──
      ctx.strokeStyle = 'rgba(59,130,246,0.1)';
      [30, 60, 120, 150, 210, 240, 300, 330].forEach(deg => {
        const rad = deg * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + Math.cos(rad) * maxR, cy - Math.sin(rad) * maxR);
        ctx.stroke();
      });

      // ── Range Rings with Meter Labels ──
      [1,2,3,4,5].forEach(m => {
        ctx.beginPath(); ctx.arc(cx, cy, m * scale, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(59,130,246,0.2)'; ctx.lineWidth = 1; ctx.stroke();
        ctx.fillStyle = 'rgba(148,163,184,0.6)'; ctx.font = '10px monospace';
        ctx.fillText(`${m}m`, cx + m * scale + 2, cy - 2);
      });

      // ── Center Cross-hairs ──
      ctx.strokeStyle = 'rgba(59,130,246,0.4)';
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(W, cy); ctx.stroke();

      // ── Front Distance Label ──
      if (this._frontDist !== null) {
        ctx.fillStyle = this._frontDist < 0.5 ? '#ef4444' : this._frontDist < 1.0 ? '#f59e0b' : '#10b981';
        ctx.font = 'bold 12px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`FRONT: ${this._frontDist.toFixed(2)}m`, cx, 20);
        ctx.textAlign = 'left';
      }

      // ── Scan Points ──
      if (this._lidarRanges && this._lidarRanges.length > 0) {
        this._lidarRanges.forEach((r, i) => {
          if (!isFinite(r) || r <= 0 || r > 6) return;
          const angle = this._lidarAngleMin + i * this._lidarAngleInc;
          const px = cx + r * scale * Math.cos(angle);
          const py = cy - r * scale * Math.sin(angle);
          if (px >= 0 && px <= W && py >= 0 && py <= H) {
            ctx.beginPath(); ctx.arc(px, py, 2, 0, Math.PI * 2);
            ctx.fillStyle = r < 0.5 ? '#ef4444' : r < 1.0 ? '#f59e0b' : '#10b981';
            ctx.fill();
          }
        });
      }

      // Helper: Map Frame -> Robot Frame (Local) 
      // Falling back to Odom if AMCL is not yet converged/active
      const toRobotFrame = (mx, my) => {
        const pose = this.app._hasAmcl ? this.app._mapPose : this.app._odomPose;
        const dx = mx - pose.x;
        const dy = my - pose.y;
        const angle = -pose.yaw;
        return {
          x: dx * Math.cos(angle) - dy * Math.sin(angle),
          y: dx * Math.sin(angle) + dy * Math.cos(angle)
        };
      };

      // ── Robot Center Indicator ──
      ctx.beginPath(); ctx.arc(cx, cy, 6, 0, Math.PI * 2);
      ctx.fillStyle = '#3b82f6'; ctx.fill();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
      // Directional triangle
      ctx.beginPath();
      ctx.moveTo(cx + 10, cy);
      ctx.lineTo(cx + 6, cy - 4);
      ctx.lineTo(cx + 6, cy + 4);
      ctx.closePath();
      ctx.fillStyle = '#fff'; ctx.fill();

      // ── Planned Path ──
      if (this.app._currentPath && this.app._currentPath.length > 0) {
        ctx.beginPath();
        ctx.setLineDash([2, 2]);
        ctx.strokeStyle = 'rgba(59, 130, 246, 0.6)';
        ctx.lineWidth = 2;
        this.app._currentPath.forEach((p, i) => {
          const lp = toRobotFrame(p.x, p.y);
          const px = cx + lp.x * scale;
          const py = cy - lp.y * scale;
          if (i === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        });
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // ── Goal Preview ──
      if (this.app._goalPreview) {
        const lp = toRobotFrame(this.app._goalPreview.x, this.app._goalPreview.y);
        const pose = this.app._hasAmcl ? this.app._mapPose : this.app._odomPose;
        const localGoal = {
          x: lp.x,
          y: lp.y,
          yaw: this.app._goalPreview.yaw - (pose.yaw * 180 / Math.PI)
        };
        this._renderGoalMarker(ctx, cx, cy, scale, W, H, localGoal, '#f59e0b', 'Preview');
      }

      // ── Active Goal ──
      if (this.app._activeGoal) {
        const lp = toRobotFrame(this.app._activeGoal.x, this.app._activeGoal.y);
        const pose = this.app._hasAmcl ? this.app._mapPose : this.app._odomPose;
        const localGoal = {
          x: lp.x,
          y: lp.y,
          yaw: this.app._activeGoal.yaw - (pose.yaw * 180 / Math.PI)
        };
        this._renderGoalMarker(ctx, cx, cy, scale, W, H, localGoal, '#10b981', 'Active');
      }

      this._animationFrameId = requestAnimationFrame(render);
    };

    render();
  }

  _renderGoalMarker(ctx, cx, cy, scale, W, H, goal, color, label) {
    const { x, y, yaw } = goal;
    const gx = cx + x * scale;
    const gy = cy - y * scale;

    // Bounds check with safe margin
    if (gx < -50 || gx > W + 50 || gy < -50 || gy > H + 50) return;

    const yawRad = yaw * Math.PI / 180;

    // Dashed line from robot
    ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(gx, gy);
    ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.setLineDash([]);

    // Goal Target
    ctx.beginPath(); ctx.arc(gx, gy, 10, 0, Math.PI * 2);
    ctx.fillStyle = color === '#f59e0b' ? 'rgba(245,158,11,0.2)' : 'rgba(16,185,129,0.2)';
    ctx.fill();
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.stroke();

    // Directional Arrow
    const arrowLen = 22;
    const ax = gx + arrowLen * Math.cos(yawRad);
    const ay = gy - arrowLen * Math.sin(yawRad);
    ctx.beginPath(); ctx.moveTo(gx, gy); ctx.lineTo(ax, ay);
    ctx.strokeStyle = color; ctx.lineWidth = 2.5; ctx.stroke();
    
    const headLen = 8, headAngle = Math.PI / 6;
    const angle = Math.atan2(gy - ay, gx - ax);
    ctx.beginPath();
    ctx.moveTo(ax, ay);
    ctx.lineTo(ax + headLen * Math.cos(angle - headAngle), ay + headLen * Math.sin(angle - headAngle));
    ctx.lineTo(ax + headLen * Math.cos(angle + headAngle), ay + headLen * Math.sin(angle + headAngle));
    ctx.closePath();
    ctx.fillStyle = color; ctx.fill();

    // Label with coordinates
    ctx.fillStyle = color;
    ctx.font = 'bold 11px monospace';
    ctx.fillText(`${label} (${x.toFixed(1)},${y.toFixed(1)})`, gx + 12, gy - 12);
  }
}

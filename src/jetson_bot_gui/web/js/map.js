// autoJetsonBot — Occupancy Map Manager
export class MapManager {
  constructor(app) {
    this.app = app;
    this.mapTopic = null;
    this.mapData = null;
    this.mapInfo = null;
    
    // Canvas elements
    this.canvas = document.getElementById('mapCanvas');
    this.ctx = this.canvas ? this.canvas.getContext('2d') : null;
    this.offscreenCanvas = document.createElement('canvas');
    this.offscreenCtx = this.offscreenCanvas.getContext('2d');
    
    this.isDrawing = false;
    this.scale = 1.0;
    this.offsetX = 0;
    this.offsetY = 0;
    
    // Interactive state
    this.lastMouseX = 0;
    this.lastMouseY = 0;
    this.isPanning = false;
  }

  setupTopics() {
    const ros = this.app.ros;
    
    this.app.log('Subscribing to /map topic...', 'info');
    
    this.mapTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid',
      queue_length: 1,
      // ROS 2 QOS Profile
      qos: {
        reliability: 'reliable',
        durability: 'transient_local',
        history: 'keep_last',
        depth: 1
      }
    });

    this.mapTopic.subscribe((msg) => {
      this._onMap(msg);
    });

    // Service fallback: Fetch map immediately on connection
    this.fetchMapViaService();
  }

  fetchMapViaService() {
    if (!this.app.isConnected) return;
    
    this.app.log('Requesting map via service...', 'info');
    const mapService = new ROSLIB.Service({
      ros: this.app.ros,
      name: '/map_server/map',
      serviceType: 'nav_msgs/srv/GetMap'
    });

    mapService.callService(new ROSLIB.ServiceRequest({}), (result) => {
      if (result && result.map) {
        this.app.log('Map received via service', 'info');
        this._onMap(result.map);
      }
    }, (error) => {
      this.app.log(`Map service call failed: ${error}`, 'warning');
      
      // Fallback for mapping mode (slam_toolbox uses a different service name)
      const slamMapService = new ROSLIB.Service({
        ros: this.app.ros,
        name: '/slam_toolbox/get_map',
        serviceType: 'nav_msgs/srv/GetMap'
      });
      slamMapService.callService(new ROSLIB.ServiceRequest({}), (res) => {
        if (res && res.map) this._onMap(res.map);
      }, () => {});
    });
  }

  teardownTopics() {
    if (this.mapTopic) {
      try { this.mapTopic.unsubscribe(); } catch (_) {}
    }
    this.mapTopic = null;
  }

  _onMap(msg) {
    this.app.log(`Map received: ${msg.info.width}x${msg.info.height} @ ${msg.info.resolution}m/px`, 'info');
    this.mapInfo = msg.info;
    this.mapData = msg.data;
    this._renderMapToCache();
    
    // Auto-center on first load
    if (this.offsetX === 0 && this.offsetY === 0) {
      this.resetView();
    }
  }

  _renderMapToCache() {
    if (!this.mapInfo || !this.mapData) return;
    
    const { width, height } = this.mapInfo;
    this.offscreenCanvas.width = width;
    this.offscreenCanvas.height = height;
    
    const imageData = this.offscreenCtx.createImageData(width, height);
    const data = imageData.data;
    
    for (let i = 0; i < this.mapData.length; i++) {
      const val = this.mapData[i];
      const idx = i * 4;
      
      // Mirror vertically because ROS maps start bottom-left, Canvas starts top-left
      // Actually, we'll handle mirroring in the drawing step for simplicity
      
      if (val === -1) { // Unknown
        data[idx] = 40; data[idx+1] = 45; data[idx+2] = 60; data[idx+3] = 255;
      } else if (val === 0) { // Free
        data[idx] = 240; data[idx+1] = 245; data[idx+2] = 255; data[idx+3] = 255;
      } else { // Occupied (val > 0)
        data[idx] = 15; data[idx+1] = 23; data[idx+2] = 42; data[idx+3] = 255;
      }
    }
    
    this.offscreenCtx.putImageData(imageData, 0, 0);
  }

  start() {
    this.resize();
    setTimeout(() => this.resize(), 500); // Delayed resize to catch layout settlement
    this.setupInteractions();
    this._drawLoop();
    window.addEventListener('resize', () => this.resize());
  }

  resize() {
    if (!this.canvas) return;
    this.canvas.width = this.canvas.clientWidth;
    this.canvas.height = this.canvas.clientHeight;
    if (this.mapInfo) this.resetView();
  }

  setupInteractions() {
    if (!this.canvas) return;
    
    this.canvas.addEventListener('mousedown', (e) => {
      this.isPanning = true;
      this.lastMouseX = e.clientX;
      this.lastMouseY = e.clientY;
    });

    window.addEventListener('mousemove', (e) => {
      if (!this.isPanning) return;
      const dx = e.clientX - this.lastMouseX;
      const dy = e.clientY - this.lastMouseY;
      this.offsetX += dx;
      this.offsetY += dy;
      this.lastMouseX = e.clientX;
      this.lastMouseY = e.clientY;
    });

    window.addEventListener('mouseup', () => {
      this.isPanning = false;
    });

    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      const zoom = e.deltaY > 0 ? 0.9 : 1.1;
      const newScale = this.scale * zoom;
      if (newScale > 0.1 && newScale < 20) {
        // Zoom towards mouse position
        const rect = this.canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        
        this.offsetX = mouseX - (mouseX - this.offsetX) * zoom;
        this.offsetY = mouseY - (mouseY - this.offsetY) * zoom;
        this.scale = newScale;
      }
    }, { passive: false });
  }

  resetView() {
    if (!this.canvas) return;
    
    // Re-request map if we don't have it yet or if user manually centers
    if (!this.mapInfo) {
      this.fetchMapViaService();
    }

    if (!this.mapInfo) return;
    
    const cw = this.canvas.width;
    const ch = this.canvas.height;
    const mw = this.mapInfo.width;
    const mh = this.mapInfo.height;
    
    this.scale = Math.min(cw / mw, ch / mh) * 0.9;
    this.offsetX = (cw - mw * this.scale) / 2;
    this.offsetY = (ch - mh * this.scale) / 2;
  }

  worldToCanvas(wx, wy) {
    if (!this.mapInfo) return { x: 0, y: 0 };
    const { origin, resolution } = this.mapInfo;
    
    // 1. World -> Map pixels
    const px = (wx - origin.position.x) / resolution;
    const py = (wy - origin.position.y) / resolution;
    
    // 2. Map pixels -> Canvas pixels (with zoom/pan and vertical flip)
    // ROS origin is bottom-left. Canvas is top-left.
    return {
      x: this.offsetX + px * this.scale,
      y: this.offsetY + (this.mapInfo.height - py) * this.scale
    };
  }

  _drawLoop() {
    if (!this.ctx) return;
    
    const draw = () => {
      const { width: cw, height: ch } = this.canvas;
      this.ctx.clearRect(0, 0, cw, ch);
      
      // Draw background
      this.ctx.fillStyle = '#0f172a';
      this.ctx.fillRect(0, 0, cw, ch);

      if (this.mapInfo) {
        // Draw the cached map
        this.ctx.save();
        this.ctx.translate(this.offsetX, this.offsetY);
        this.ctx.scale(this.scale, this.scale);
        
        // Mirror the map data vertically (ROS map starts bottom-left)
        // We can do this with scale(1, -1) and translate
        this.ctx.save();
        this.ctx.scale(1, -1);
        this.ctx.drawImage(this.offscreenCanvas, 0, -this.mapInfo.height);
        this.ctx.restore();
        
        this.ctx.restore();

        // Draw Robot Pose
        const pose = this.app._hasAmcl ? this.app._mapPose : this.app._odomPose;
        const cp = this.worldToCanvas(pose.x, pose.y);
        
        this.ctx.save();
        this.ctx.translate(cp.x, cp.y);
        this.ctx.rotate(-pose.yaw); // Negative because canvas Y is flipped

        // Robot shape (arrow)
        this.ctx.beginPath();
        this.ctx.moveTo(10, 0);
        this.ctx.lineTo(-6, -6);
        this.ctx.lineTo(-6, 6);
        this.ctx.closePath();
        this.ctx.fillStyle = '#3b82f6';
        this.ctx.fill();
        this.ctx.strokeStyle = '#fff';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();
        this.ctx.restore();

        // Draw Active Goal
        if (this.app._activeGoal) {
          const gp = this.worldToCanvas(this.app._activeGoal.x, this.app._activeGoal.y);
          this.ctx.beginPath();
          this.ctx.arc(gp.x, gp.y, 5, 0, Math.PI * 2);
          this.ctx.fillStyle = '#10b981';
          this.ctx.fill();
          this.ctx.strokeStyle = '#fff';
          this.ctx.stroke();
        }
      } else {
        this.ctx.fillStyle = '#94a3b8';
        this.ctx.font = '14px Inter';
        this.ctx.textAlign = 'center';
        this.ctx.fillText('Waiting for map data...', cw / 2, ch / 2);
      }

      requestAnimationFrame(draw);
    };
    
    draw();
  }
}

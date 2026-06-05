# UI Upgrade Plan — autoJetsonBot Web Dashboard

**Goal:** Transform the current joystick-only web UI into a full robot operations dashboard.
**Constraint:** No new frontend frameworks. Pure HTML/CSS/JS + roslibjs. All backend in Python (ROS 2 Foxy nodes).

---

## Current State Audit

| Feature | Status | Issue |
|---|---|---|
| D-Pad / WASD movement | ✅ Working | — |
| Emergency Stop | ✅ Working | — |
| Speed sliders | ✅ Working | — |
| Left/Right RPM display | ✅ Working | — |
| Battery % | ✅ Working | Fed by `telemetry_node.py` |
| Obstacle warning (log) | ✅ Working | Throttled to 5s |
| Reset Odometry | ✅ Working | Publishes to `/initialpose` |
| Save Map | ✅ Working | Calls `/slam_toolbox/save_map` |
| CPU / Memory display | ❌ Show only | `telemetry_node.py` collects it but never publishes |
| Temperature display | ❌ Show only | HTML element exists, JS never updates it |
| Camera feed | ❌ Placeholder | No stream backend |
| Topic sidebar inputs | ❌ Broken | Fields not wired to `setupROSTopics()` |
| Dark mode button | ❌ Broken | Button exists, does nothing |

---

## Phase 1 — Fix What's Broken (Est. 2–3 hours)

**Priority: High. These are regressions, not features.**

### 1.1 Fix Broken Topic Inputs
- **File:** `src/jetson_bot_gui/web/js/robot-controller.js`
- **Problem:** `cmdVelTopic`, `jointStatesTopic`, `scanTopic` sidebar inputs are saved to `localStorage` but never read back into `this.config.topics`. `setupROSTopics()` always uses hardcoded values.
- **Fix:**
  - On `change` event for each topic input, update `this.config.topics.<key>`.
  - If already connected, unsubscribe old topics and call `setupROSTopics()` again.
  - On `connect()`, read topic values from the input fields before calling `setupROSTopics()`.

### 1.2 Wire CPU / Memory / Temperature to UI
- **Files:** `src/jetson_bot_gui/jetson_bot_gui/telemetry_node.py`, `src/jetson_bot_gui/web/js/robot-controller.js`
- **Problem:** `telemetry_node.py` already calls `psutil.cpu_percent()` and `psutil.virtual_memory()` but only logs them. The UI shows `N/A`.
- **Fix (backend):**
  - Add a new publisher in `telemetry_node.py` on topic `/telemetry/system` with message type `std_msgs/String` publishing a JSON string:
    ```json
    {"cpu": 34.2, "memory": 61.5, "temperature": 47.0}
    ```
  - For temperature: use `psutil.sensors_temperatures()` if available (Jetson), else publish `null`.
- **Fix (frontend):**
  - Subscribe to `/telemetry/system` in `setupROSTopics()`.
  - Parse JSON and update `#cpuUsage`, `#memoryUsage`, `#temperature` elements.
  - Remove the hardcoded `N/A` from `updateMetricsDisplay()`.

### 1.3 Fix Dark Mode Toggle
- **File:** `src/jetson_bot_gui/web/css/modern-style.css`, `src/jetson_bot_gui/web/index.html`
- **Problem:** The 🌙 button is added dynamically but has no handler.
- **Fix:**
  - Add a `data-theme` attribute to `<body>`.
  - Define a `[data-theme="dark"]` CSS block with overridden CSS variables.
  - Wire the button click to toggle the attribute and persist to `localStorage`.

---

## Phase 2 — Navigation Control (Est. 4–6 hours)

**Priority: High. Makes the UI actually useful in navigation mode.**

### 2.1 Nav2 Goal Sending
- **File:** `src/jetson_bot_gui/web/js/robot-controller.js`, `src/jetson_bot_gui/web/index.html`
- **What:** Add a "Send Nav Goal" panel with X, Y, Yaw inputs and a **Go** button.
- **How:**
  - Publish to `/goal_pose` with message type `geometry_msgs/PoseStamped`.
  - Convert yaw (degrees, user-friendly) to quaternion `(x=0, y=0, z=sin(yaw/2), w=cos(yaw/2))` in JS.
  - Header `frame_id: "map"`.
- **UI placement:** New card in the right status panel, visible only when connected.
- **Example message:**
  ```js
  {
    header: { frame_id: 'map' },
    pose: {
      position: { x: 1.5, y: 0.5, z: 0.0 },
      orientation: { x: 0, y: 0, z: 0.707, w: 0.707 }
    }
  }
  ```

### 2.2 Nav2 Status Panel
- **File:** `src/jetson_bot_gui/web/js/robot-controller.js`, `src/jetson_bot_gui/web/index.html`
- **What:** Show current Nav2 state (Idle / Planning / Executing / Recovering / Goal Reached).
- **How:**
  - Subscribe to `/bt_navigator/transition_event` (`lifecycle_msgs/TransitionEvent`) — shows lifecycle state changes.
  - Subscribe to `/navigate_to_pose/_action/status` (`action_msgs/GoalStatusArray`) — shows active goal status.
  - Map status codes to human-readable labels with color indicators (green=executing, yellow=recovering, grey=idle).
- **UI placement:** Small status badge next to the connection indicator in the header.

### 2.3 Cancel Navigation Goal
- **File:** `src/jetson_bot_gui/web/js/robot-controller.js`
- **What:** A **Cancel Goal** button that appears while Nav2 is executing.
- **How:**
  - Publish an empty `geometry_msgs/PoseStamped` to `/goal_pose` is not sufficient.
  - Use `ROSLIB.ActionClient` to cancel the active goal on `/navigate_to_pose`.
  - Show button only when Nav2 status is `EXECUTING` or `ACCEPTED`.

---

## Phase 3 — Sensor Visualization (Est. 6–8 hours)

**Priority: Medium. Eliminates need to open RViz for basic monitoring.**

### 3.1 Lidar Radar Canvas
- **File:** `src/jetson_bot_gui/web/js/robot-controller.js`, `src/jetson_bot_gui/web/index.html`
- **What:** Real-time polar plot of `/scan` data rendered on an HTML `<canvas>`.
- **How:**
  - Already subscribed to `/scan` in `setupROSTopics()` — just add rendering.
  - On each scan message, clear canvas and draw:
    - Concentric range rings (0.5m, 1m, 2m, 3m).
    - Each valid range as a dot at `(r * cos(angle), r * sin(angle))`.
    - Robot icon at center.
    - Color-code by distance: green > 1m, yellow 0.5–1m, red < 0.5m.
  - Throttle render to 10 Hz to avoid browser jank.
- **UI placement:** Replace the "Camera Feed" placeholder temporarily, or add as a new tab.
- **Canvas size:** 300×300px minimum.

### 3.2 Map + Robot Position Overlay
- **File:** `src/jetson_bot_gui/web/js/robot-controller.js`, `src/jetson_bot_gui/web/index.html`
- **What:** Render the occupancy grid map with the robot's current position overlaid.
- **How (map):**
  - Subscribe to `/map` (`nav_msgs/OccupancyGrid`) — fires once on load.
  - Convert `data` array to a grayscale `ImageData` on a `<canvas>` (0=white/free, 100=black/occupied, -1=grey/unknown).
  - Cache the rendered map; re-render only on map update.
- **How (robot pose):**
  - Subscribe to `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`) in navigation mode.
  - Subscribe to `/odom` (`nav_msgs/Odometry`) in mapping mode.
  - Convert world coordinates to canvas pixel coordinates using map `origin`, `resolution`, and canvas scale.
  - Draw a directional arrow at the robot's position.
- **UI placement:** New full-width panel below the D-pad, collapsible.
- **Note:** `/map` topic can be large (~100KB). Subscribe with `throttle_rate: 5000` (every 5s) via roslibjs.

---

## Phase 4 — Camera Feed (Est. 2–3 hours)

**Priority: Medium. Hardware-dependent.**

### 4.1 Backend: Add `web_video_server`
- **File:** `src/jetson_bot_bringup/launch/main.launch.py`
- **What:** Launch `web_video_server` node to stream `/camera/image_raw` as MJPEG over HTTP.
- **How:**
  - Add to `main.launch.py` (simulation and hardware modes):
    ```python
    Node(
        package='web_video_server',
        executable='web_video_server',
        parameters=[{'port': 8080}],
        output='screen'
    )
    ```
  - Expose port 8080 in `docker-compose.yaml`.

### 4.2 Frontend: Camera Panel
- **File:** `src/jetson_bot_gui/web/index.html`
- **What:** Replace the camera placeholder with a live `<img>` tag.
- **How:**
  ```html
  <img src="http://localhost:8080/stream?topic=/camera/image_raw&type=mjpeg"
       width="100%" style="border-radius: 8px;" />
  ```
  - Add an error handler: if stream fails (sim mode without camera), show the placeholder text.
  - Add a toggle button to pause/resume the stream (set `img.src = ''` to pause).

---

## Phase 5 — Waypoint Queue (Est. 4–5 hours)

**Priority: Low. Advanced feature, requires Phase 2 complete.**

### 5.1 Waypoint List UI
- **File:** `src/jetson_bot_gui/web/js/robot-controller.js`, `src/jetson_bot_gui/web/index.html`
- **What:** A list where users can add multiple X/Y/Yaw waypoints and send them as a sequence.
- **How:**
  - Store waypoints in a JS array.
  - Render as a numbered list with delete buttons.
  - "Run Waypoints" button calls the `/follow_waypoints` action (`nav2_msgs/action/FollowWaypoints`) via `ROSLIB.ActionClient`.
  - `waypoint_follower` is already configured in `nav2_params.yaml` with `stop_on_failure: false`.
- **UI placement:** Expandable panel below the Nav Goal card (Phase 2.1).

---

## Phase 6 — Reliability & Polish (Est. 3–4 hours)

**Priority: Low. Quality of life.**

### 6.1 Auto-Reconnect with Backoff
- **File:** `src/jetson_bot_gui/web/js/robot-controller.js`
- **What:** If rosbridge disconnects, retry automatically with exponential backoff.
- **How:**
  ```js
  // In ros.on('close') handler:
  this.reconnectDelay = Math.min((this.reconnectDelay || 1000) * 2, 30000);
  setTimeout(() => this.connect(), this.reconnectDelay);
  // Reset delay on successful connection.
  ```

### 6.2 Mode Indicator in Header
- **File:** `src/jetson_bot_gui/web/index.html`, `src/jetson_bot_gui/web/js/robot-controller.js`
- **What:** Show current mode (`MAPPING` / `NAVIGATION`) and sim/hardware badge in the header.
- **How:** Read from `config.json` (already fetched in `fetchConfig()`). Display as colored badges next to the connection status.

### 6.3 Keyboard Shortcut Reference Card
- **File:** `src/jetson_bot_gui/web/index.html`
- **What:** A collapsible `<details>` element listing all keyboard shortcuts.
- **Shortcuts to document:** WASD/Arrows (move), Space (e-stop), F11 (fullscreen), G (send goal), C (cancel goal), M (save map).

### 6.4 Log Export Button
- **File:** `src/jetson_bot_gui/web/js/robot-controller.js`
- **What:** A button to download the activity log as a `.txt` file.
- **How:** Collect all `#logContainer` entries, create a `Blob`, trigger a download link.

---

## File Change Summary

| File | Changes |
|---|---|
| `src/jetson_bot_gui/jetson_bot_gui/telemetry_node.py` | Publish CPU/memory/temperature on `/telemetry/system` as JSON string |
| `src/jetson_bot_gui/web/js/robot-controller.js` | Fix topic inputs, subscribe to system telemetry, add Nav2 goal/cancel/status, lidar canvas, map overlay, auto-reconnect |
| `src/jetson_bot_gui/web/index.html` | Add Nav2 goal panel, map canvas, camera img tag, mode badges, shortcut card, log export button |
| `src/jetson_bot_gui/web/css/modern-style.css` | Add dark mode CSS variables, canvas styles, new panel styles |
| `src/jetson_bot_bringup/launch/main.launch.py` | Add `web_video_server` node launch |
| `docker-compose.yaml` | Expose port 8080 for camera stream |

---

## Implementation Order

```
Phase 1 (Fix Broken)     ──► Phase 2 (Nav Control) ──► Phase 3 (Sensor Viz)
     │                              │
     └── No dependencies            └── Requires rosbridge connected
                                        (same as Phase 1)

Phase 4 (Camera)         ──► Independent, can be done anytime
Phase 5 (Waypoints)      ──► Requires Phase 2 complete
Phase 6 (Polish)         ──► Can be done alongside any phase
```

**Start here:** `Phase 1.2` (telemetry publishing) — it's pure Python, no frontend changes, and immediately makes 3 broken UI elements work.

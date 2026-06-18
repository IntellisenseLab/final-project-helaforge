(function () {
  "use strict";

  var els = {
    bridgeUrl: document.getElementById("bridgeUrl"),
    connectBtn: document.getElementById("connectBtn"),
    connectionDot: document.getElementById("connectionDot"),
    connectionText: document.getElementById("connectionText"),
    mapCanvas: document.getElementById("mapCanvas"),
    mapWrap: document.getElementById("mapWrap"),
    mapEmpty: document.getElementById("mapEmpty"),
    resetViewBtn: document.getElementById("resetViewBtn"),
    zoomInBtn: document.getElementById("zoomInBtn"),
    zoomOutBtn: document.getElementById("zoomOutBtn"),
    commandInput: document.getElementById("commandInput"),
    sendCommandBtn: document.getElementById("sendCommandBtn"),
    voiceBtn: document.getElementById("voiceBtn"),
    voiceStatus: document.getElementById("voiceStatus"),
    teleopState: document.getElementById("teleopState"),
    forwardBtn: document.getElementById("forwardBtn"),
    backBtn: document.getElementById("backBtn"),
    leftBtn: document.getElementById("leftBtn"),
    rightBtn: document.getElementById("rightBtn"),
    stopBtn: document.getElementById("stopBtn"),
    linearSpeed: document.getElementById("linearSpeed"),
    angularSpeed: document.getElementById("angularSpeed"),
    objectList: document.getElementById("objectList"),
    statusLog: document.getElementById("statusLog")
  };

  var ctx = els.mapCanvas.getContext("2d");
  var state = {
    ws: null,
    connected: false,
    subscribers: {},
    advertised: {},
    map: null,
    mapBitmap: null,
    robot: null,
    path: [],
    objects: {},
    statusLines: [],
    teleopEnabled: false,
    activeTeleop: null,
    teleopTimer: null,
    smoothStopTimer: null,
    lastTwist: { linear: 0.0, angular: 0.0 },
    zoom: 1.0,
    panX: 0,
    panY: 0,
    dragging: false,
    dragStartX: 0,
    dragStartY: 0,
    dragPanX: 0,
    dragPanY: 0,
    reconnectTimer: null
  };

  function defaultBridgeUrl() {
    var host = window.location.hostname || "localhost";
    return "ws://" + host + ":9090";
  }

  function addStatus(text) {
    var stamp = new Date().toLocaleTimeString();
    state.statusLines.unshift("[" + stamp + "] " + text);
    state.statusLines = state.statusLines.slice(0, 80);
    els.statusLog.textContent = state.statusLines.join("\n");
  }

  function setConnected(connected, text) {
    state.connected = connected;
    els.connectionDot.classList.toggle("connected", connected);
    els.connectionText.textContent = text || (connected ? "Connected" : "Disconnected");
    els.connectBtn.textContent = connected ? "Disconnect" : "Connect";
    // Update the map overlay to reflect connection state
    if (!state.map) {
      var title = document.getElementById("mapEmptyTitle");
      var hint  = document.getElementById("mapEmptyHint");
      if (title && hint) {
        if (connected) {
          title.textContent = "Waiting for /map";
          hint.textContent  = "Connected ✅ — LiDAR scanning. Map appears automatically.";
        } else {
          title.textContent = "Not connected";
          hint.textContent  = "Rosbridge: " + (els.bridgeUrl.value || defaultBridgeUrl()) +
                              " — auto-reconnecting…";
        }
      }
    }
  }

  function sendRos(obj) {
    if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
      return;
    }
    state.ws.send(JSON.stringify(obj));
  }

  function advertise(topic, type) {
    if (state.advertised[topic]) {
      return;
    }
    state.advertised[topic] = true;
    sendRos({ op: "advertise", topic: topic, type: type });
  }

  function publish(topic, type, msg) {
    advertise(topic, type);
    sendRos({ op: "publish", topic: topic, msg: msg });
  }

  function subscribe(topic, type, throttleRate, callback) {
    state.subscribers[topic] = callback;
    sendRos({
      op: "subscribe",
      id: "sub:" + topic,
      topic: topic,
      type: type,
      throttle_rate: throttleRate,
      queue_length: 1
    });
  }

  function connect() {
    if (state.connected && state.ws) {
      state.ws.close();
      return;
    }

    var url = els.bridgeUrl.value.trim() || defaultBridgeUrl();
    state.ws = new WebSocket(url);
    setConnected(false, "Connecting");

    state.ws.onopen = function () {
      state.advertised = {};
      setConnected(true, "Connected");
      addStatus("Connected to " + url);
      subscribeTopics();
    };

    state.ws.onclose = function () {
      setConnected(false, "Disconnected");
      addStatus("ROSBridge disconnected — will retry in 3 s");
      scheduleReconnect();
    };

    state.ws.onerror = function () {
      setConnected(false, "Connection error");
      addStatus("Could not reach ROSBridge at " + url + " — retrying…");
    };

    state.ws.onmessage = function (event) {
      var packet;
      try {
        packet = JSON.parse(event.data);
      } catch (err) {
        return;
      }
      if (packet.op !== "publish" || !packet.topic) {
        return;
      }
      var cb = state.subscribers[packet.topic];
      if (cb) {
        cb(packet.msg);
      }
    };
  }

  function subscribeTopics() {
    subscribe("/map", "nav_msgs/OccupancyGrid", 1000, handleMap);
    subscribe("/slam_pose", "nav_msgs/Odometry", 100, handleRobotPose);
    subscribe("/planned_path", "nav_msgs/Path", 200, handlePath);
    subscribe("/semantic_nav/object_markers", "visualization_msgs/MarkerArray", 300, handleMarkers);
    subscribe("/semantic_nav/status", "std_msgs/String", 200, function (msg) {
      addStatus("semantic: " + msg.data);
    });
    subscribe("/qbot_nav/status", "std_msgs/String", 200, function (msg) {
      addStatus("nav: " + msg.data);
    });
    subscribe("/semantic_nav/teleop_enabled", "std_msgs/Bool", 100, function (msg) {
      state.teleopEnabled = !!msg.data;
      updateTeleopUi();
      if (!state.teleopEnabled) {
        stopTeleop(true);
      }
    });
  }

  function quaternionYaw(q) {
    if (!q) {
      return 0;
    }
    var siny = 2.0 * (q.w * q.z + q.x * q.y);
    var cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return Math.atan2(siny, cosy);
  }

  function handleMap(msg) {
    state.map = msg;
    buildMapBitmap(msg);
    els.mapEmpty.style.display = "none";
    render();
    addStatus("/map received: " + msg.info.width + "×" + msg.info.height +
              " cells @ " + msg.info.resolution.toFixed(3) + " m/cell");
  }

  function buildMapBitmap(msg) {
    var width = msg.info.width;
    var height = msg.info.height;
    var offscreen = document.createElement("canvas");
    offscreen.width = width;
    offscreen.height = height;
    var offCtx = offscreen.getContext("2d");
    var image = offCtx.createImageData(width, height);
    var data = msg.data || [];

    for (var y = 0; y < height; y += 1) {
      for (var x = 0; x < width; x += 1) {
        var src = x + y * width;
        var dstY = height - 1 - y;
        var dst = (x + dstY * width) * 4;
        var value = data[src];
        var r;
        var g;
        var b;
        if (value < 0) {
          r = 211;
          g = 216;
          b = 222;
        } else if (value > 65) {
          r = 36;
          g = 43;
          b = 52;
        } else {
          var shade = Math.max(235, 255 - value);
          r = shade;
          g = shade;
          b = shade;
        }
        image.data[dst] = r;
        image.data[dst + 1] = g;
        image.data[dst + 2] = b;
        image.data[dst + 3] = 255;
      }
    }

    offCtx.putImageData(image, 0, 0);
    state.mapBitmap = offscreen;
  }

  function handleRobotPose(msg) {
    var pose = msg.pose.pose;
    state.robot = {
      x: pose.position.x,
      y: pose.position.y,
      yaw: quaternionYaw(pose.orientation)
    };
    render();
  }

  function handlePath(msg) {
    state.path = (msg.poses || []).map(function (poseStamped) {
      return {
        x: poseStamped.pose.position.x,
        y: poseStamped.pose.position.y
      };
    });
    render();
  }

  function handleMarkers(msg) {
    (msg.markers || []).forEach(function (marker) {
      if (marker.action === 3) {
        state.objects = {};
        return;
      }

      var key = String(Math.floor(marker.id / 2));
      if (marker.action === 2) {
        delete state.objects[key];
        return;
      }

      if (!state.objects[key]) {
        state.objects[key] = { label: "object_" + key, x: 0, y: 0, z: 0 };
      }

      if (marker.ns === "semantic_objects") {
        state.objects[key].x = marker.pose.position.x;
        state.objects[key].y = marker.pose.position.y;
        state.objects[key].z = marker.pose.position.z;
      }
      if (marker.ns === "semantic_object_labels") {
        state.objects[key].label = marker.text || state.objects[key].label;
      }
    });

    renderObjectList();
    render();
  }

  function mapView() {
    var dpr = window.devicePixelRatio || 1;
    var width = els.mapCanvas.width / dpr;
    var height = els.mapCanvas.height / dpr;
    if (!state.map) {
      return null;
    }
    var mapWidth = state.map.info.width;
    var mapHeight = state.map.info.height;
    var baseScale = Math.min(width / mapWidth, height / mapHeight);
    var scale = baseScale * state.zoom;
    return {
      scale: scale,
      x: (width - mapWidth * scale) / 2 + state.panX,
      y: (height - mapHeight * scale) / 2 + state.panY,
      width: width,
      height: height
    };
  }

  function worldToGrid(x, y) {
    if (!state.map) {
      return null;
    }
    var info = state.map.info;
    var origin = info.origin;
    var yaw = quaternionYaw(origin.orientation);
    var dx = x - origin.position.x;
    var dy = y - origin.position.y;
    var c = Math.cos(-yaw);
    var s = Math.sin(-yaw);
    return {
      gx: (c * dx - s * dy) / info.resolution,
      gy: (s * dx + c * dy) / info.resolution
    };
  }

  function worldToScreen(x, y, view) {
    var grid = worldToGrid(x, y);
    if (!grid) {
      return null;
    }
    return {
      x: view.x + grid.gx * view.scale,
      y: view.y + (state.map.info.height - grid.gy) * view.scale
    };
  }

  function resizeCanvas() {
    var rect = els.mapWrap.getBoundingClientRect();
    var dpr = window.devicePixelRatio || 1;
    els.mapCanvas.width = Math.max(1, Math.floor(rect.width * dpr));
    els.mapCanvas.height = Math.max(1, Math.floor(rect.height * dpr));
    els.mapCanvas.style.width = rect.width + "px";
    els.mapCanvas.style.height = rect.height + "px";
    render();
  }

  function render() {
    var dpr = window.devicePixelRatio || 1;
    var width = els.mapCanvas.width / dpr;
    var height = els.mapCanvas.height / dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#d9dee5";
    ctx.fillRect(0, 0, width, height);

    if (!state.map || !state.mapBitmap) {
      return;
    }

    var view = mapView();
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(
      state.mapBitmap,
      view.x,
      view.y,
      state.map.info.width * view.scale,
      state.map.info.height * view.scale
    );

    drawPath(view);
    drawObjects(view);
    drawRobot(view);
  }

  function drawPath(view) {
    if (!state.path.length) {
      return;
    }
    ctx.save();
    ctx.strokeStyle = "#2a67c7";
    ctx.lineWidth = 3;
    ctx.beginPath();
    state.path.forEach(function (point, index) {
      var screen = worldToScreen(point.x, point.y, view);
      if (!screen) {
        return;
      }
      if (index === 0) {
        ctx.moveTo(screen.x, screen.y);
      } else {
        ctx.lineTo(screen.x, screen.y);
      }
    });
    ctx.stroke();
    ctx.restore();
  }

  function drawObjects(view) {
    Object.keys(state.objects).forEach(function (key) {
      var obj = state.objects[key];
      var screen = worldToScreen(obj.x, obj.y, view);
      if (!screen) {
        return;
      }
      ctx.save();
      ctx.fillStyle = "#f2c94c";
      ctx.strokeStyle = "#6f4d00";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(screen.x, screen.y, 7, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = "#1e2732";
      ctx.font = "12px system-ui, sans-serif";
      ctx.fillText(obj.label, screen.x + 10, screen.y - 10);
      ctx.restore();
    });
  }

  function drawRobot(view) {
    if (!state.robot) {
      return;
    }
    var screen = worldToScreen(state.robot.x, state.robot.y, view);
    if (!screen) {
      return;
    }
    ctx.save();
    ctx.translate(screen.x, screen.y);
    ctx.rotate(-state.robot.yaw);
    ctx.fillStyle = "#147a7e";
    ctx.strokeStyle = "#0a3436";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(15, 0);
    ctx.lineTo(-10, 9);
    ctx.lineTo(-7, 0);
    ctx.lineTo(-10, -9);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  function renderObjectList() {
    var objects = Object.keys(state.objects)
      .map(function (key) { return state.objects[key]; })
      .filter(function (obj) { return obj.label; })
      .sort(function (a, b) { return a.label.localeCompare(b.label); });

    els.objectList.innerHTML = "";
    if (!objects.length) {
      els.objectList.className = "objectList empty";
      els.objectList.textContent = "No objects yet";
      return;
    }

    els.objectList.className = "objectList";
    objects.forEach(function (obj) {
      var row = document.createElement("div");
      row.className = "objectItem";

      var text = document.createElement("div");
      var name = document.createElement("div");
      name.className = "objectName";
      name.textContent = obj.label;
      var meta = document.createElement("div");
      meta.className = "objectMeta";
      meta.textContent = "map(" + obj.x.toFixed(2) + ", " + obj.y.toFixed(2) + ")";
      text.appendChild(name);
      text.appendChild(meta);

      var go = document.createElement("button");
      go.type = "button";
      go.textContent = "Go";
      go.addEventListener("click", function () {
        publishCommand("go to " + obj.label);
      });

      row.appendChild(text);
      row.appendChild(go);
      els.objectList.appendChild(row);
    });
  }

  function publishCommand(command) {
    var text = (command || "").trim();
    if (!text) {
      return;
    }
    publish("/semantic_nav/command", "std_msgs/String", { data: text });
    addStatus("web command: " + text);
    els.commandInput.value = "";
  }

  function updateTeleopUi() {
    els.teleopState.textContent = state.teleopEnabled ? "Enabled" : "Disabled";
    els.teleopState.classList.toggle("enabled", state.teleopEnabled);
    [els.forwardBtn, els.backBtn, els.leftBtn, els.rightBtn].forEach(function (button) {
      button.disabled = !state.teleopEnabled;
    });
  }

  function currentLinearSpeed() {
    return parseFloat(els.linearSpeed.value || "0.10");
  }

  function currentAngularSpeed() {
    return parseFloat(els.angularSpeed.value || "0.35");
  }

  function publishTwist(linear, angular) {
    state.lastTwist = { linear: linear, angular: angular };
    publish("/cmd_vel", "geometry_msgs/Twist", {
      linear: { x: linear, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angular }
    });
  }

  function clearSmoothStop() {
    if (state.smoothStopTimer) {
      clearInterval(state.smoothStopTimer);
      state.smoothStopTimer = null;
    }
  }

  function startTeleop(linear, angular) {
    if (!state.teleopEnabled) {
      addStatus("teleop is disabled; start mapping first");
      return;
    }
    clearSmoothStop();
    state.activeTeleop = { linear: linear, angular: angular };
    publishTwist(linear, angular);
    if (state.teleopTimer) {
      clearInterval(state.teleopTimer);
    }
    state.teleopTimer = setInterval(function () {
      if (state.activeTeleop) {
        publishTwist(state.activeTeleop.linear, state.activeTeleop.angular);
      }
    }, 125);
  }

  function smoothStopFrom(linear, angular) {
    clearSmoothStop();
    var startLinear = linear;
    var startAngular = angular;
    var durationMs = 450;
    var startedAt = performance.now();

    function step() {
      var elapsed = performance.now() - startedAt;
      var progress = Math.min(1.0, elapsed / durationMs);
      var factor = 1.0 - progress;
      publishTwist(startLinear * factor, startAngular * factor);
      if (progress >= 1.0) {
        clearSmoothStop();
        publishTwist(0.0, 0.0);
      }
    }

    step();
    state.smoothStopTimer = setInterval(step, 50);
  }

  function stopTeleop(immediate) {
    var last = state.activeTeleop || state.lastTwist;
    state.activeTeleop = null;
    if (state.teleopTimer) {
      clearInterval(state.teleopTimer);
      state.teleopTimer = null;
    }

    if (immediate) {
      clearSmoothStop();
      publishTwist(0.0, 0.0);
      return;
    }

    if (Math.abs(last.linear) < 0.001 && Math.abs(last.angular) < 0.001) {
      clearSmoothStop();
      publishTwist(0.0, 0.0);
      return;
    }
    smoothStopFrom(last.linear, last.angular);
  }

  function bindTeleopButton(button, linear, angular) {
    function down(event) {
      event.preventDefault();
      startTeleop(linear(), angular());
    }
    button.addEventListener("pointerdown", down);
    button.addEventListener("pointerup", function () { stopTeleop(false); });
    button.addEventListener("pointercancel", function () { stopTeleop(false); });
    button.addEventListener("pointerleave", function () { stopTeleop(false); });
  }

  function setupVoice() {
    var SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
    if (!SpeechRecognition) {
      els.voiceBtn.disabled = true;
      els.voiceStatus.textContent = "Browser voice is not supported here. Use command text.";
      return;
    }

    var recognition = new SpeechRecognition();
    recognition.lang = "en-US";
    recognition.continuous = false;
    recognition.interimResults = false;

    recognition.onstart = function () {
      els.voiceBtn.textContent = "Listening";
      els.voiceStatus.textContent = "Speak one robot command.";
    };
    recognition.onend = function () {
      els.voiceBtn.textContent = "Start Browser Voice";
    };
    recognition.onerror = function (event) {
      els.voiceStatus.textContent = "Voice error: " + event.error;
    };
    recognition.onresult = function (event) {
      var transcript = event.results[0][0].transcript.trim();
      els.voiceStatus.textContent = "Heard: " + transcript;
      publishCommand(transcript);
    };

    els.voiceBtn.addEventListener("click", function () {
      try {
        recognition.start();
      } catch (err) {
        els.voiceStatus.textContent = "Voice is already listening.";
      }
    });
  }

  function setupMapInteraction() {
    els.mapCanvas.addEventListener("pointerdown", function (event) {
      state.dragging = true;
      state.dragStartX = event.clientX;
      state.dragStartY = event.clientY;
      state.dragPanX = state.panX;
      state.dragPanY = state.panY;
      els.mapCanvas.setPointerCapture(event.pointerId);
    });

    els.mapCanvas.addEventListener("pointermove", function (event) {
      if (!state.dragging) {
        return;
      }
      state.panX = state.dragPanX + event.clientX - state.dragStartX;
      state.panY = state.dragPanY + event.clientY - state.dragStartY;
      render();
    });

    els.mapCanvas.addEventListener("pointerup", function () {
      state.dragging = false;
    });
    els.mapCanvas.addEventListener("pointercancel", function () {
      state.dragging = false;
    });
  }

  function setupEvents() {
    els.bridgeUrl.value = defaultBridgeUrl();
    els.connectBtn.addEventListener("click", connect);
    els.sendCommandBtn.addEventListener("click", function () {
      publishCommand(els.commandInput.value);
    });
    els.commandInput.addEventListener("keydown", function (event) {
      if (event.key === "Enter") {
        publishCommand(els.commandInput.value);
      }
    });
    document.querySelectorAll("[data-command]").forEach(function (button) {
      button.addEventListener("click", function () {
        publishCommand(button.getAttribute("data-command"));
      });
    });

    els.resetViewBtn.addEventListener("click", function () {
      state.zoom = 1.0;
      state.panX = 0;
      state.panY = 0;
      render();
    });
    els.zoomInBtn.addEventListener("click", function () {
      state.zoom = Math.min(8, state.zoom * 1.25);
      render();
    });
    els.zoomOutBtn.addEventListener("click", function () {
      state.zoom = Math.max(0.25, state.zoom / 1.25);
      render();
    });

    bindTeleopButton(els.forwardBtn, currentLinearSpeed, function () { return 0.0; });
    bindTeleopButton(els.backBtn, function () { return -currentLinearSpeed(); }, function () { return 0.0; });
    bindTeleopButton(els.leftBtn, function () { return 0.0; }, currentAngularSpeed);
    bindTeleopButton(els.rightBtn, function () { return 0.0; }, function () { return -currentAngularSpeed(); });
    els.stopBtn.addEventListener("click", function () { stopTeleop(false); });

    window.addEventListener("keydown", function (event) {
      var tag = document.activeElement ? document.activeElement.tagName : "";
      if (tag === "INPUT" || tag === "TEXTAREA") {
        return;
      }
      if (event.repeat) {
        return;
      }
      if (event.key === "ArrowUp") {
        event.preventDefault();
        startTeleop(currentLinearSpeed(), 0.0);
      } else if (event.key === "ArrowDown") {
        event.preventDefault();
        startTeleop(-currentLinearSpeed(), 0.0);
      } else if (event.key === "ArrowLeft") {
        event.preventDefault();
        startTeleop(0.0, currentAngularSpeed());
      } else if (event.key === "ArrowRight") {
        event.preventDefault();
        startTeleop(0.0, -currentAngularSpeed());
      } else if (event.key === " ") {
        event.preventDefault();
        stopTeleop(false);
      }
    });

    window.addEventListener("keyup", function (event) {
      if (event.key.indexOf("Arrow") === 0) {
        event.preventDefault();
        stopTeleop(false);
      }
    });
    window.addEventListener("blur", function () { stopTeleop(true); });
    window.addEventListener("resize", resizeCanvas);
    setupMapInteraction();
    setupVoice();
  }

  function scheduleReconnect() {
    if (state.reconnectTimer) { return; }
    state.reconnectTimer = setTimeout(function () {
      state.reconnectTimer = null;
      if (!state.connected) {
        addStatus("Auto-reconnecting to ROSBridge…");
        connect();
      }
    }, 3000);
  }

  setupEvents();
  updateTeleopUi();
  resizeCanvas();
  connect();
}());

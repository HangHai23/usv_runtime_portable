(() => {
  "use strict";

  const byId = (id) => document.getElementById(id);
  const elements = {};
  const ids = [
    "connectionChip", "connectionText", "vesselState", "modePill", "modeText",
    "updatedText", "linkTag", "qualityRing", "qualityValue", "rssiValue", "snrValue",
    "safetyHeadline", "safetyOrb", "estopState", "watchdogState", "dryRunState",
    "sourceValue", "leftApplied", "rightApplied", "leftRequested", "rightRequested",
    "leftPwm", "rightPwm", "leftBar", "rightBar", "frameRate", "detailRssi",
    "detailLq", "detailSnr", "rfMode", "validFrames", "crcErrors", "crcNote",
    "downlinkNotice", "footerAddress", "signalChart", "deviceStatus", "memoryRing",
    "memoryValue", "memoryDetail", "cpuTempValue", "tempState", "currentValue",
    "currentRail", "powerValue", "powerDetail", "cpuUsageBar", "cpuUsageValue",
    "uptimeValue", "settingsStatus", "limitInput", "leftTrimInput",
    "rightTrimInput", "applySettingsButton",
    "visionStatus", "visionStream", "visionEmpty", "visionResolution",
    "visionFrameAge", "visionClass", "visionConfidence", "bearingAngle",
    "bearingDirection", "bearingIndicator", "trackingMode", "inferenceTime",
    "yawRate", "desiredYawRate", "detectionAge", "imuVisionState",
  ];
  ids.forEach((id) => { elements[id] = byId(id); });

  const signalHistory = [];
  let lastFrameCount = null;
  let reconnectTimer = null;
  let socket = null;
  let lastTelemetryAt = 0;
  let settingsDirty = false;

  const text = (element, value, suffix = "") => {
    element.textContent = value === null || value === undefined ? "—" : `${value}${suffix}`;
  };

  const setConnection = (state, label) => {
    elements.connectionChip.classList.remove("online", "offline");
    if (state) elements.connectionChip.classList.add(state);
    elements.connectionText.textContent = label;
  };

  const setSettingsStatus = (label, state = "") => {
    elements.settingsStatus.textContent = label;
    elements.settingsStatus.className = `settings-status${state ? ` ${state}` : ""}`;
  };

  const loadPropulsionSettings = async (force = false) => {
    if (settingsDirty && !force) return;
    try {
      const response = await fetch("/api/propulsion-settings", { cache: "no-store" });
      if (!response.ok) throw new Error(await response.text());
      const payload = await response.json();
      const settings = payload.settings;
      elements.limitInput.value = settings.output_limit_percent;
      elements.leftTrimInput.value = settings.channel_1_trim_percent;
      elements.rightTrimInput.value = settings.channel_2_trim_percent;
      settingsDirty = false;
      setSettingsStatus("已同步", "good");
    } catch (error) {
      setSettingsStatus("执行器未连接", "bad");
    }
  };

  const applyPropulsionSettings = async () => {
    const values = {
      output_limit_percent: Number(elements.limitInput.value),
      channel_1_trim_percent: Number(elements.leftTrimInput.value),
      channel_2_trim_percent: Number(elements.rightTrimInput.value),
    };
    const valid = Number.isFinite(values.output_limit_percent)
      && values.output_limit_percent >= 0 && values.output_limit_percent <= 100
      && Number.isFinite(values.channel_1_trim_percent)
      && values.channel_1_trim_percent >= -100 && values.channel_1_trim_percent <= 100
      && Number.isFinite(values.channel_2_trim_percent)
      && values.channel_2_trim_percent >= -100 && values.channel_2_trim_percent <= 100;
    if (!valid) {
      setSettingsStatus("参数超出范围", "bad");
      return;
    }
    if (!window.confirm(
      `应用动力设置？\n\n全局限幅：${values.output_limit_percent}%\n左微调：${values.channel_1_trim_percent}%\n右微调：${values.channel_2_trim_percent}%`
    )) return;

    elements.applySettingsButton.disabled = true;
    setSettingsStatus("正在应用");
    try {
      const response = await fetch("/api/propulsion-settings", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(values),
      });
      if (!response.ok) throw new Error(await response.text());
      const payload = await response.json();
      const settings = payload.settings;
      elements.limitInput.value = settings.output_limit_percent;
      elements.leftTrimInput.value = settings.channel_1_trim_percent;
      elements.rightTrimInput.value = settings.channel_2_trim_percent;
      settingsDirty = false;
      setSettingsStatus("已生效", "good");
    } catch (error) {
      console.warn("Failed to apply propulsion settings", error);
      setSettingsStatus("应用失败", "bad");
    } finally {
      elements.applySettingsButton.disabled = false;
    }
  };

  const classifyLink = (quality, age) => {
    if (age > 1200 || quality === undefined) return ["数据中断", "bad"];
    if (quality >= 90) return ["链路优秀", "good"];
    if (quality >= 70) return ["链路可用", "warn"];
    return ["链路较弱", "bad"];
  };

  const badge = (element, label, state) => {
    element.textContent = label;
    element.className = `state-badge ${state}`;
  };

  const valueAt = (array, index) => Array.isArray(array) && Number.isFinite(array[index])
    ? array[index]
    : null;

  const setThrust = (bar, value) => {
    const safe = Number.isFinite(value) ? Math.max(-100, Math.min(100, value)) : 0;
    const width = Math.abs(safe) / 2;
    bar.style.width = `${width}%`;
    bar.style.left = safe < 0 ? `${50 - width}%` : "50%";
    bar.classList.toggle("reverse", safe < 0);
  };

  const formatPercent = (value) => Number.isFinite(value) ? Math.round(value) : null;
  const formatRequest = (value) => Number.isFinite(value) ? `${value.toFixed(1)}%` : "—";

  const formatUptime = (seconds) => {
    if (!Number.isFinite(seconds)) return "运行时间 —";
    const days = Math.floor(seconds / 86400);
    const hours = Math.floor((seconds % 86400) / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    return `运行时间 ${days ? `${days}天 ` : ""}${hours}小时 ${minutes}分`;
  };

  const drawChart = () => {
    const canvas = elements.signalChart;
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(rect.width * ratio));
    canvas.height = Math.max(1, Math.round(rect.height * ratio));
    const context = canvas.getContext("2d");
    context.scale(ratio, ratio);
    const width = rect.width;
    const height = rect.height;
    context.clearRect(0, 0, width, height);
    if (signalHistory.length < 2) return;

    const min = Math.min(-10, ...signalHistory) - 4;
    const max = Math.max(-90, ...signalHistory) + 4;
    const points = signalHistory.map((value, index) => ({
      x: (index / (signalHistory.length - 1)) * width,
      y: height - ((value - min) / Math.max(1, max - min)) * (height - 14) - 7,
    }));

    const gradient = context.createLinearGradient(0, 0, 0, height);
    gradient.addColorStop(0, "rgba(0, 113, 227, 0.24)");
    gradient.addColorStop(1, "rgba(0, 113, 227, 0)");
    context.beginPath();
    context.moveTo(points[0].x, height);
    points.forEach((point) => context.lineTo(point.x, point.y));
    context.lineTo(points[points.length - 1].x, height);
    context.closePath();
    context.fillStyle = gradient;
    context.fill();

    context.beginPath();
    points.forEach((point, index) => index ? context.lineTo(point.x, point.y) : context.moveTo(point.x, point.y));
    context.strokeStyle = "#0071e3";
    context.lineWidth = 2.25;
    context.lineJoin = "round";
    context.lineCap = "round";
    context.stroke();
  };

  const updateDashboard = (data) => {
    const link = data.link || {};
    const rc = data.rc || {};
    const control = data.control || {};
    const propulsion = data.propulsion || {};
    const safety = data.safety || {};
    const system = data.system || {};
    const vision = data.vision || {};
    const age = data.ageMs || {};
    const linkAge = Number.isFinite(age.link) ? age.link : 999999;
    const connected = rc.connected === true && linkAge < 1200;
    elements.vesselState.textContent = connected ? "遥控在线" : "遥控未连接";
    setConnection("online", data.demo ? "演示模式 · 非实船" : "已连接");
    elements.updatedText.textContent = linkAge < 1000 ? `${linkAge} ms 前更新` : "链路数据未连接";

    if (control.automatic === true) {
      elements.modePill.className = "mode-pill automatic";
      elements.modeText.textContent = "自动航行";
    } else if (control.automatic === false) {
      elements.modePill.className = "mode-pill manual";
      elements.modeText.textContent = "人工遥控";
    } else {
      elements.modePill.className = "mode-pill unknown";
      elements.modeText.textContent = "模式未知";
    }

    const systemFresh = Number.isFinite(age.system) && age.system < 2500;
    elements.deviceStatus.className = `device-status${systemFresh ? " online" : ""}`;
    elements.deviceStatus.innerHTML = `<i></i>${systemFresh ? "运行正常" : "数据中断"}`;

    const memoryPercent = Number.isFinite(system.memoryPercent) ? system.memoryPercent : null;
    elements.memoryRing.style.setProperty("--memory", memoryPercent ?? 0);
    text(elements.memoryValue, memoryPercent === null ? null : Math.round(memoryPercent));
    if (Number.isFinite(system.memoryUsedMb) && Number.isFinite(system.memoryTotalMb)) {
      elements.memoryDetail.textContent = `${(system.memoryUsedMb / 1024).toFixed(1)} / ${(system.memoryTotalMb / 1024).toFixed(1)} GB`;
    } else {
      elements.memoryDetail.textContent = "等待 /proc/meminfo";
    }

    if (Number.isFinite(system.cpuTempC)) {
      elements.cpuTempValue.textContent = `${system.cpuTempC.toFixed(1)}°C`;
      elements.tempState.textContent = system.cpuTempC >= 85 ? "温度过高" : system.cpuTempC >= 70 ? "温度偏高" : "热状态正常";
      elements.tempState.style.color = system.cpuTempC >= 85 ? "#ff8d86" : system.cpuTempC >= 70 ? "#ffc467" : "rgba(255,255,255,.43)";
    } else {
      elements.cpuTempValue.textContent = "—";
      elements.tempState.textContent = "未发现 thermal zone";
    }

    if (Number.isFinite(system.totalCurrentA)) {
      elements.currentValue.textContent = `${system.totalCurrentA.toFixed(2)} A`;
      elements.currentRail.textContent = `${system.powerRail || "VDD_IN"} · 模块电源轨`;
    } else {
      elements.currentValue.textContent = "—";
      elements.currentRail.textContent = "未发现 INA3221 电流";
    }

    if (Number.isFinite(system.totalPowerW)) {
      elements.powerValue.textContent = `${system.totalPowerW.toFixed(2)} W`;
      elements.powerDetail.textContent = Number.isFinite(system.moduleRailVoltageV)
        ? `${system.powerRail || "VDD_IN"} · ${system.moduleRailVoltageV.toFixed(2)} V 模块轨`
        : `${system.powerRail || "VDD_IN"} · 非外部19V`;
    } else {
      elements.powerValue.textContent = "—";
      elements.powerDetail.textContent = "未发现模块功率";
    }

    const cpuUsage = Number.isFinite(system.cpuUsage) ? Math.max(0, Math.min(100, system.cpuUsage)) : null;
    elements.cpuUsageBar.style.width = `${cpuUsage ?? 0}%`;
    elements.cpuUsageValue.textContent = cpuUsage === null ? "—" : `${cpuUsage.toFixed(0)}%`;
    elements.uptimeValue.textContent = formatUptime(system.uptimeSec);

    const frameAge = Number.isFinite(age.visionFrame) ? age.visionFrame : 999999;
    const detectionFresh = Number.isFinite(age.detection) && age.detection < 800;
    const targetFresh = Number.isFinite(age.target) && age.target < 500;
    const frameFresh = frameAge < 1200;
    elements.visionEmpty.classList.toggle("hidden", frameFresh);
    elements.visionFrameAge.textContent = frameFresh ? `${frameAge} ms 前更新` : "画面未连接";
    elements.visionResolution.textContent = Number.isFinite(vision.imageWidth) && Number.isFinite(vision.imageHeight)
      ? `${vision.imageWidth} × ${vision.imageHeight}` : "— × —";

    let visionLabel = "等待推理";
    let visionClassName = "waiting";
    if (targetFresh && vision.trackingState === "TRACKING") {
      visionLabel = "稳定追踪";
      visionClassName = "";
    } else if (targetFresh && vision.trackingState === "PREDICTING") {
      visionLabel = "IMU预测中";
      visionClassName = "predicting";
    } else if (detectionFresh && vision.detected === false) {
      visionLabel = "未发现目标";
    }
    elements.visionStatus.className = `vision-status${visionClassName ? ` ${visionClassName}` : ""}`;
    elements.visionStatus.innerHTML = `<i></i>${visionLabel}`;
    elements.visionClass.textContent = detectionFresh && vision.detected
      ? (vision.className || `类别 ${vision.classId}`) : "未检测";
    elements.visionConfidence.textContent = detectionFresh && vision.detected && Number.isFinite(vision.confidence)
      ? `${(vision.confidence * 100).toFixed(1)}%` : "—";
    elements.inferenceTime.textContent = detectionFresh && Number.isFinite(vision.inferenceMs)
      ? `${vision.inferenceMs.toFixed(1)} ms` : "—";

    const bearing = targetFresh && vision.targetValid && Number.isFinite(vision.bearingDeg)
      ? vision.bearingDeg : null;
    elements.bearingAngle.textContent = bearing === null ? "—" : `${bearing >= 0 ? "+" : ""}${bearing.toFixed(1)}`;
    if (bearing === null) {
      elements.bearingDirection.textContent = "等待目标状态";
    } else if (Math.abs(bearing) < 2.0) {
      elements.bearingDirection.textContent = "目标位于航向中心";
    } else {
      elements.bearingDirection.textContent = `目标位于船体${bearing > 0 ? "右侧" : "左侧"}`;
    }
    const bearingPosition = bearing === null ? 50 : Math.max(0, Math.min(100, 50 + bearing / 35 * 50));
    elements.bearingIndicator.style.setProperty("--bearing-position", `${bearingPosition}%`);
    elements.trackingMode.textContent = targetFresh ? (vision.trackingState || "—") : "OFFLINE";
    elements.yawRate.textContent = targetFresh && Number.isFinite(vision.yawRateDegS)
      ? `${vision.yawRateDegS >= 0 ? "+" : ""}${vision.yawRateDegS.toFixed(1)} °/s` : "—";
    elements.desiredYawRate.textContent = Number.isFinite(age.desiredYawRate) && age.desiredYawRate < 500
      && Number.isFinite(vision.desiredYawRateDegS)
      ? `${vision.desiredYawRateDegS >= 0 ? "+" : ""}${vision.desiredYawRateDegS.toFixed(1)} °/s` : "—";
    elements.detectionAge.textContent = targetFresh && Number.isFinite(vision.detectionAgeMs)
      ? `${Math.round(vision.detectionAgeMs)} ms` : "—";
    elements.imuVisionState.textContent = targetFresh
      ? (vision.imuValid ? "有效" : "超时") : "—";

    const quality = Number.isFinite(link.quality) ? link.quality : null;
    const [linkLabel, linkClass] = classifyLink(quality, linkAge);
    elements.linkTag.textContent = linkLabel;
    elements.linkTag.className = `tag ${linkClass}`;
    elements.qualityRing.style.setProperty("--quality", quality ?? 0);
    text(elements.qualityValue, quality);
    text(elements.rssiValue, Number.isFinite(link.rssi1) ? link.rssi1 : null);
    text(elements.snrValue, Number.isFinite(link.snr) ? link.snr : null);

    if (Number.isFinite(link.rssi1)) {
      signalHistory.push(link.rssi1);
      if (signalHistory.length > 60) signalHistory.shift();
      drawChart();
    }

    const emergency = safety.emergencyStop;
    const watchdog = safety.watchdog;
    const dryRun = safety.dryRun;
    badge(elements.estopState, emergency === true ? "已触发" : emergency === false ? "正常" : "未知", emergency === true ? "danger" : emergency === false ? "safe" : "unknown");
    badge(elements.watchdogState, watchdog === true ? "已接管" : watchdog === false ? "正常" : "未知", watchdog === true ? "warning" : watchdog === false ? "safe" : "unknown");
    badge(elements.dryRunState, dryRun === true ? "模拟" : dryRun === false ? "真实" : "未知", dryRun === true ? "warning" : dryRun === false ? "safe" : "unknown");

    elements.safetyOrb.className = "safety-orb";
    if (emergency === true) {
      elements.safetyHeadline.textContent = "紧急停止中";
      elements.safetyOrb.classList.add("danger");
    } else if (watchdog === true) {
      elements.safetyHeadline.textContent = "看门狗已接管";
      elements.safetyOrb.classList.add("warning");
    } else if (emergency === false && watchdog === false) {
      elements.safetyHeadline.textContent = "所有保护正常";
    } else {
      elements.safetyHeadline.textContent = "等待执行器";
      elements.safetyOrb.classList.add("unknown");
    }

    text(elements.sourceValue, control.source || "—");
    const applied = propulsion.applied || propulsion.command || [];
    const requested = propulsion.requested || propulsion.command || [];
    const pulse = propulsion.pulseUs || [];
    const left = valueAt(applied, 0);
    const right = valueAt(applied, 1);
    text(elements.leftApplied, formatPercent(left));
    text(elements.rightApplied, formatPercent(right));
    text(elements.leftRequested, formatRequest(valueAt(requested, 0)));
    text(elements.rightRequested, formatRequest(valueAt(requested, 1)));
    text(elements.leftPwm, valueAt(pulse, 0), valueAt(pulse, 0) === null ? "" : " μs");
    text(elements.rightPwm, valueAt(pulse, 1), valueAt(pulse, 1) === null ? "" : " μs");
    setThrust(elements.leftBar, left);
    setThrust(elements.rightBar, right);

    text(elements.frameRate, Number.isFinite(rc.frameRate) ? rc.frameRate.toFixed(1) : null, Number.isFinite(rc.frameRate) ? " Hz" : "");
    text(elements.detailRssi, link.rssi1);
    text(elements.detailLq, quality);
    text(elements.detailSnr, link.snr);
    text(elements.rfMode, link.rfMode);
    text(elements.validFrames, rc.validFrames);
    text(elements.crcErrors, rc.crcErrors);
    elements.crcNote.textContent = rc.crcErrors === 0 ? "未发现校验错误" : Number.isFinite(rc.crcErrors) ? "请检查串口链路" : "等待数据";
    elements.downlinkNotice.classList.toggle("hidden", Number.isFinite(link.downlinkQuality) && link.downlinkQuality > 0);

    if (Number.isFinite(rc.validFrames) && lastFrameCount !== rc.validFrames) lastFrameCount = rc.validFrames;
  };

  const connect = () => {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    socket = new WebSocket(`${protocol}//${window.location.host}/ws`);
    setConnection("offline", "未连接");

    socket.addEventListener("open", () => {
      setConnection("online", "已连接");
      loadPropulsionSettings();
    });
    socket.addEventListener("message", (event) => {
      try {
        const payload = JSON.parse(event.data);
        if (payload.type !== "telemetry") return;
        lastTelemetryAt = performance.now();
        updateDashboard(payload);
      } catch (error) {
        console.warn("Ignored malformed telemetry", error);
      }
    });
    socket.addEventListener("close", () => {
      setConnection("offline", "未连接");
      window.clearTimeout(reconnectTimer);
      reconnectTimer = window.setTimeout(connect, 2500);
    });
    socket.addEventListener("error", () => socket.close());
  };

  elements.footerAddress.textContent = window.location.host || "Jetson ROS 2";
  elements.visionStream.addEventListener("load", () => elements.visionEmpty.classList.add("hidden"));
  elements.visionStream.addEventListener("error", () => elements.visionEmpty.classList.remove("hidden"));
  [elements.limitInput, elements.leftTrimInput, elements.rightTrimInput]
    .forEach((input) => input.addEventListener("input", () => {
      settingsDirty = true;
      setSettingsStatus("待应用");
    }));
  elements.applySettingsButton.addEventListener("click", applyPropulsionSettings);
  window.addEventListener("resize", drawChart);
  window.setInterval(() => loadPropulsionSettings(), 5000);
  window.setInterval(() => {
    if (socket?.readyState === WebSocket.OPEN &&
        performance.now() - lastTelemetryAt > 2500) {
      setConnection("offline", "数据中断 · 重连中");
      elements.deviceStatus.innerHTML = "<i></i>数据中断";
      elements.safetyHeadline.textContent = "状态已过期 · 不可据此判断安全";
      socket.close();
    }
  }, 500);
  connect();
})();

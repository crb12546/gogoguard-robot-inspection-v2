const state = {
  active: null,
  job: null,
  mapJobs: [],
  latestMapJob: null,
  mapSelectionExplicit: false,
  editingMapId: null,
  mapArtifact: null,
  glimEditor: null,
  navigationWorkspace: null,
  savedNavigationWorkspace: null,
  workspaceEditMode: null,
  workspaceDisplayMode: 'split',
  workspaceTransform: null,
  workspaceView: {zoom: 1, centerX: null, centerY: null, panMode: false, drag: null},
  workspaceCursorWorld: null,
  workspaceCloudView: {yaw: 0.72, pitch: 0.62, zoom: 18, zoomFactor: 1, centerX: 0, centerY: 0, sliceOnly: false},
  navigationSurface: null,
  surfaceBrushDown: false,
  surfaceUndo: [],
  planPreview: null,
  planStart: null,
  planGoal: null,
  navigation: null,
  navigationPrepare: null,
  live: null,
  camera: null,
  gimbal: null,
  recordingCheckpoints: [],
  checkpointAudit: null,
  checkpointCaptureKey: null,
  checkpointCaptureRetryAt: 0,
  checkpointControlBusy: false,
  checkpointCameraStarted: false,
  platformUpload: null,
  platformUploadSupported: null,
  cameraStarted: false,
  robotReachable: false,
  gimbalTarget: null,
  gimbalTargetEditing: false,
  navigationProfile: null,
  diagnosticProfile: null,
  incidents: [],
  incident: null,
  incidentReplay: null,
  views: {
    live: {yaw: 0.7, pitch: 0.55, zoom: 28},
    result: {yaw: 0.7, pitch: 0.55, zoom: 18},
    incident: {yaw: 0.7, pitch: 0.55, zoom: 28},
  },
};

// Must remain identical to the padded hard envelope in go2_nav2_patrol.yaml
// and gogoguard_route.workspace. It is intentionally not a display-only
// circle: route clearing and review must use the same body polygon as Nav2.
const PADDED_BASE_FOOTPRINT_M = [
  [0.50, 0.30],
  [0.50, -0.30],
  [-0.43, -0.30],
  [-0.43, 0.30],
];

const $ = id => document.getElementById(id);
const escapeHtml = value => String(value ?? '').replace(
  /[&<>"]/g,
  character => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[character]),
);
const fmtDisk = bytes => `${(Number(bytes || 0) / 1024 ** 3).toFixed(1)} GB 可用`;
const fmtLocalTime = value => {
  if (!value) return '时间未知';
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return String(value);
  return new Intl.DateTimeFormat('zh-CN', {
    month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit',
    hour12: false,
  }).format(date);
};
const fmtTransfer = job => {
  const total = Number(job?.bytes_total || 0);
  const done = Number(job?.bytes_transferred || 0);
  const rate = Number(job?.transfer_rate_bps || 0);
  if (!total) return job?.stage ? `阶段：${job.stage}` : '尚未传输';
  const speed = rate > 0 ? ` · ${(rate / 1024 ** 2).toFixed(1)} MiB/s` : '';
  return `${(done / 1024 ** 2).toFixed(1)} / ${(total / 1024 ** 2).toFixed(1)} MiB${speed}`;
};
const friendlyError = error => {
  const message = String(error?.message || error || '未知错误');
  if (message.includes('robot edge agent unavailable')) {
    return '机器狗离线；开机并接入网线后，工作台会自动恢复连接';
  }
  if (message.includes('Failed to fetch')) return '工作台服务暂时不可用，请稍后刷新';
  return message;
};
const setFault = error => { $('error').textContent = friendlyError(error); };

const operationKindLabels = {
  'runtime.start': '启动定位与 Nav2',
  'runtime.stop': '停止并释放遥控权',
  'runtime.recover': '清理并恢复导航',
  'localization.reset': '重新定位',
  'patrol.start': '提交巡检',
  'patrol.start_selected': '启动本地巡检点验收',
  'patrol.stop': '停止并释放遥控权',
};
const operationStateText = operation => {
  if (!operation) return '无';
  if (operation.state === 'accepted') return '请求已接收';
  if (operation.state === 'running') return '请求处理中';
  if (operation.state === 'failed') return '请求失败';
  if (operation.state === 'complete') {
    if (operation.kind === 'patrol.start') return '巡检请求已提交';
    if (operation.kind === 'runtime.start') return '启动请求已完成';
    return '请求已完成';
  }
  return operation.state;
};
const operationMessageText = operation => {
  if (!operation) return '点击后会在这里持续显示结果';
  if (operation.state === 'complete' && operation.kind === 'patrol.start') {
    return 'Nav2 已接收开始请求；整条路线是否成功以下方“巡检状态”为准';
  }
  if (operation.state === 'complete' && operation.kind === 'runtime.start') {
    return '运行进程已启动；是否可以巡检以“定位可用”为准';
  }
  return operation.message || operation.error || '等待状态更新';
};
const patrolStepStateText = runtime => {
  const labels = {
    STARTING: '请求确认中',
    PATROLLING: runtime.reason === 'NAV2_GLOBAL_PATH_USING_MPPI' ? '绕障中' : '巡检中',
    REPLANNING: '规划绕障中',
    DETOURING: 'MPPI 执行绕行中',
    REJOINING: '恢复蓝色路线中',
    RETRYING: 'MPPI 自动重试中',
    RECOVERING: '自动恢复中',
    SEARCHING_PATH: '持续寻路中',
    HOLDING: '定位等待中',
    RESUMING: '恢复路线中',
    COMPLETED: '已完成',
    BLOCKED: '路线受阻',
    FAULT: '运行故障',
    STOPPING: '正在停止',
    CHECKPOINT_PAUSING: '到点停车中',
    CHECKPOINT_SETTLING: '验证停稳中',
    INSPECTING: '调整姿态中',
    PAUSED: '等待人工确认',
  };
  return labels[runtime.state] || '等待';
};

async function api(path, options = {}) {
  const response = await fetch(path, options);
  const value = await response.json();
  if (!response.ok) throw Error(value.error || `HTTP ${response.status}`);
  return value;
}

const post = (path, body) => api(path, {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: body ? JSON.stringify(body) : undefined,
});

function sizeCanvas(canvas) {
  const density = devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (canvas.width !== width * density || canvas.height !== height * density) {
    canvas.width = width * density;
    canvas.height = height * density;
  }
  const context = canvas.getContext('2d');
  context.setTransform(density, 0, 0, density, 0, 0);
  return [context, width, height];
}

function project(point, view, width, height) {
  const cy = Math.cos(view.yaw);
  const sy = Math.sin(view.yaw);
  const cp = Math.cos(view.pitch);
  const sp = Math.sin(view.pitch);
  const a = point[0] * cy - point[1] * sy;
  const b = point[0] * sy + point[1] * cy;
  const z = point[2] || 0;
  return [width / 2 + a * view.zoom, height * 0.58 - (b * sp + z * cp) * view.zoom];
}

function projectCentered(point, view, width, height) {
  const shifted = [
    Number(point[0]) - Number(view.centerX || 0),
    Number(point[1]) - Number(view.centerY || 0),
    Number(point[2] || 0),
  ];
  return project(shifted, view, width, height);
}

function drawCloud(canvas, points, trajectory, view) {
  const [context, width, height] = sizeCanvas(canvas);
  context.fillStyle = '#071421';
  context.fillRect(0, 0, width, height);
  if (canvas.id !== 'resultCloud' || $('showMapPoints')?.checked !== false) {
    for (const point of points || []) {
      const [x, y] = project(point, view, width, height);
      if (x < 0 || y < 0 || x > width || y > height) continue;
      const z = point[2] || 0;
      context.fillStyle = `hsla(${185 + z * 18},85%,${55 + (point[3] || 0.4) * 18}%,.8)`;
      context.fillRect(x, y, 2, 2);
    }
  }
  if (trajectory?.length && (canvas.id !== 'resultCloud' || $('showMapRoute')?.checked !== false)) {
    context.strokeStyle = '#22d3ee';
    context.lineWidth = 2.5;
    context.beginPath();
    trajectory.forEach((point, index) => {
      const [x, y] = project(point, view, width, height);
      index ? context.lineTo(x, y) : context.moveTo(x, y);
    });
    context.stroke();
  }
}

function drawTrajectory() {
  const canvas = $('trajectory');
  const [context, width, height] = sizeCanvas(canvas);
  context.clearRect(0, 0, width, height);
  context.strokeStyle = '#172c42';
  context.lineWidth = 1;
  for (let index = 0; index < width; index += 40) {
    context.beginPath(); context.moveTo(index, 0); context.lineTo(index, height); context.stroke();
  }
  for (let index = 0; index < height; index += 40) {
    context.beginPath(); context.moveTo(0, index); context.lineTo(width, index); context.stroke();
  }
  const trajectory = state.live?.trajectory || [];
  const scale = Math.min(width, height) / 11;
  const originX = width / 2;
  const originY = height / 2;
  if (!trajectory.length) return;
  context.strokeStyle = '#22d3ee';
  context.lineWidth = 3;
  context.beginPath();
  trajectory.forEach((point, index) => {
    const x = originX + point[0] * scale;
    const y = originY - point[1] * scale;
    index ? context.lineTo(x, y) : context.moveTo(x, y);
  });
  context.stroke();
  const point = trajectory[trajectory.length - 1];
  context.fillStyle = '#fb923c';
  context.beginPath();
  context.arc(originX + point[0] * scale, originY - point[1] * scale, 7, 0, Math.PI * 2);
  context.fill();
}

function renderLive() {
  if (!state.live) return;
  const device = state.live.device;
  const pose = state.live.pose;
  $('modeBadge').textContent = device.mode === 'demo' ? '本机演示' : 'Mac 工作站 · 机器狗实时';
  $('robotState').textContent = device.online ? '在线' : '离线';
  $('robotId').textContent = device.message;
  $('lidarRate').textContent = `${device.lidar_hz.toFixed(1)} Hz`;
  $('imuRate').textContent = `${device.imu_hz.toFixed(1)} Hz`;
  $('lidarAge').textContent = device.lidar_age_s === null ? '未收到点云' : `${device.lidar_age_s.toFixed(2)} 秒前`;
  $('diskFree').textContent = fmtDisk(device.disk_free_bytes);
  $('pose').textContent = `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}`;
  $('pointCount').textContent = `${state.live.points.length} 点`;
  drawTrajectory();
  drawCloud($('cloud'), state.live.points, null, state.views.live);
}

function cameraUrl(camera) {
  if (camera.url) return camera.url;
  const scheme = location.protocol === 'https:' ? 'https:' : 'http:';
  return `${scheme}//${location.hostname}:${camera.port}/${encodeURIComponent(camera.stream_path)}/?controls=false&muted=true&autoplay=true&playsInline=true`;
}

function renderCamera() {
  const camera = state.camera;
  if (!camera) return;
  if (!camera.enabled) {
    $('cameraState').textContent = '未启用';
    $('cameraMessage').textContent = camera.message;
    return;
  }
  if (!state.cameraStarted) {
    $('cameraFrame').src = cameraUrl(camera);
    state.cameraStarted = true;
  }
  $('cameraState').textContent = camera.ready ? '实时' : '连接中';
  $('cameraState').className = camera.ready ? 'online' : '';
  $('cameraMessage').textContent = camera.ready && camera.width && camera.height
    ? `${camera.width}×${camera.height} · WebRTC`
    : camera.message;
}

async function prepareLocalCheckpoint(checkpoint) {
  const key = [
    checkpoint.missionId,
    checkpoint.activeCheckpointId,
    checkpoint.attempt,
  ].join(':');
  if (
    state.checkpointControlBusy
    || state.checkpointCaptureKey === key
    || Date.now() < state.checkpointCaptureRetryAt
  ) return;
  state.checkpointCaptureKey = key;
  state.checkpointControlBusy = true;
  try {
    await post('/api/v1/navigation/checkpoint/control', {action: 'capture'});
    $('navigationError').textContent = '';
  } catch (error) {
    state.checkpointCaptureKey = null;
    state.checkpointCaptureRetryAt = Date.now() + 3000;
    $('navigationError').textContent = `巡检点准备失败：${friendlyError(error)}`;
  } finally {
    state.checkpointControlBusy = false;
    await refreshNavigation();
  }
}

function renderLocalCheckpoint() {
  const runtime = state.navigation?.runtime || {};
  const checkpoint = runtime.checkpoint || {};
  const local = checkpoint.decisionMode === 'local_operator';
  const activeId = checkpoint.activeCheckpointId;
  const active = local && Boolean(activeId) && checkpoint.phase !== 'TRAVELING';
  $('checkpointValidation').hidden = !active;
  if (!active) return;

  const audit = (state.checkpointAudit?.checkpoints || []).find(
    item => item.checkpointId === activeId,
  );
  const jobId = /^map-[A-Za-z0-9]{12}$/.test(String(runtime.mapVersion || ''))
    ? runtime.mapVersion
    : state.latestMapJob?.job_id;
  const referenceKey = `${jobId || ''}:${activeId}`;
  if (jobId && $('checkpointReference').dataset.referenceKey !== referenceKey) {
    $('checkpointReference').src = `/api/v1/map-jobs/${encodeURIComponent(jobId)}/checkpoints/${encodeURIComponent(activeId)}/reference`;
    $('checkpointReference').dataset.referenceKey = referenceKey;
  }
  if (state.camera && !state.checkpointCameraStarted) {
    $('checkpointCameraFrame').src = cameraUrl(state.camera);
    state.checkpointCameraStarted = true;
  }

  const phaseLabels = {
    PAUSING: '正在取消路线速度',
    SETTLING: '正在确认真实停稳',
    POSE_REQUESTED: '正在转到录制时朝向',
    POSING: '正在转到录制时朝向',
    WAITING_PLATFORM: '正在调整镜头',
    SPIN_REQUESTED: '正在准备 360° 旋转',
    SPINNING: '正在 360° 旋转',
    WAITING_VERDICT: '已停稳，等待您确认',
    FAILED: '巡检点执行失败',
  };
  const ordinal = Number(checkpoint.completedCheckpointCount || 0) + 1;
  const total = Number(checkpoint.checkpointCount || 0);
  $('checkpointValidationState').textContent = phaseLabels[checkpoint.phase] || checkpoint.phase;
  $('checkpointValidationState').className = checkpoint.phase === 'WAITING_VERDICT' ? 'online' : '';
  $('checkpointValidationTitle').textContent = `第 ${ordinal} / ${total} 个巡检点 · ${activeId}`;
  const alignmentMode = checkpoint.observationAlignment?.mode;
  const alignmentLabel = alignmentMode === 'camera_only'
    ? '镜头对准'
    : alignmentMode === 'camera_fallback'
      ? '镜头补偿对准'
      : alignmentMode === 'body_plus_camera'
        ? '机身最小转向 + 镜头对准'
        : '定点照片';
  $('checkpointValidationMeta').textContent = `${audit?.note || '未填备注'} · pan ${Number(checkpoint.camera?.pan || 0).toFixed(1)}° / tilt ${Number(checkpoint.camera?.tilt || 0).toFixed(1)}° · ${alignmentLabel}`;
  const ready = checkpoint.phase === 'WAITING_VERDICT';
  $('checkpointVerdictActions').hidden = !ready;
  for (const id of ['checkpointContinue', 'checkpointRetake', 'checkpointSkip']) {
    $(id).disabled = state.checkpointControlBusy;
  }
  if (checkpoint.phase === 'WAITING_PLATFORM') prepareLocalCheckpoint(checkpoint);
}

async function controlLocalCheckpoint(action) {
  if (state.checkpointControlBusy) return;
  state.checkpointControlBusy = true;
  for (const id of ['checkpointContinue', 'checkpointRetake', 'checkpointSkip']) {
    $(id).disabled = true;
  }
  try {
    await post('/api/v1/navigation/checkpoint/control', {action});
    $('navigationError').textContent = '';
  } catch (error) {
    $('navigationError').textContent = friendlyError(error);
  } finally {
    state.checkpointControlBusy = false;
    await refreshNavigation();
  }
}

async function refreshCamera() {
  try {
    state.camera = await api('/api/v1/camera');
    renderCamera();
  } catch (error) {
    $('cameraState').textContent = '不可用';
    $('cameraMessage').textContent = friendlyError(error);
  }
}

async function refreshGimbal() {
  try {
    state.gimbal = await api('/api/v1/gimbal');
    const angles = state.gimbal.angles || {};
    const actualPan = Number(angles.pan || 0);
    const actualTilt = Number(angles.tilt || 0);
    const actualRoll = Number(angles.roll || 0);
    $('gimbalAngles').textContent = `左右 ${actualPan.toFixed(1)}° · 上下 ${actualTilt.toFixed(1)}°`;
    if (!gimbalCommandBusy) {
      state.gimbalTarget = {pan: actualPan, tilt: actualTilt, roll: actualRoll};
    }
    if (!state.gimbalTargetEditing && !gimbalCommandBusy) {
      $('gimbalPanTarget').value = Number(state.gimbalTarget.pan).toFixed(1);
      $('gimbalTiltTarget').value = Number(state.gimbalTarget.tilt).toFixed(1);
    }
    for (const id of [
      'gimbalLeft', 'gimbalRight', 'gimbalUp', 'gimbalDown',
      'gimbalCenter', 'gimbalApplyTarget',
    ]) {
      $(id).disabled = !state.gimbal.supported;
    }
  } catch (_) {
    $('gimbalAngles').textContent = '镜头不可用';
  }
}

let gimbalCommandBusy = false;
async function moveGimbal(panDelta, tiltDelta, center = false, explicitTarget = null) {
  if (gimbalCommandBusy) return;
  gimbalCommandBusy = true;
  const actual = state.gimbal?.angles || {};
  const current = state.gimbalTarget || {
    pan: Number(actual.pan || 0),
    tilt: Number(actual.tilt || 0),
    roll: Number(actual.roll || 0),
  };
  const target = center
    ? {pan: 0, tilt: 0, roll: 0}
    : explicitTarget || {
      pan: Number(current.pan || 0) + panDelta,
      tilt: Number(current.tilt || 0) + tiltDelta,
      roll: Number(current.roll || 0),
    };
  $('gimbalControlMessage').textContent = `正在转到：左右 ${Number(target.pan).toFixed(1)}° · 上下 ${Number(target.tilt).toFixed(1)}°`;
  try {
    state.gimbal = await post(
      center ? '/api/v1/gimbal/center' : '/api/v1/gimbal/move',
      center ? undefined : {...target, precision: 'fine'},
    );
    const reached = state.gimbal.angles || target;
    state.gimbalTarget = {
      pan: Number(target.pan),
      tilt: Number(target.tilt),
      roll: Number(target.roll),
    };
    $('gimbalControlMessage').textContent = `已到位：左右 ${Number(reached.pan).toFixed(1)}° · 上下 ${Number(reached.tilt).toFixed(1)}°`;
    $('error').textContent = '';
    await refreshGimbal();
  } catch (error) {
    const message = friendlyError(error);
    $('gimbalControlMessage').textContent = `镜头调整失败：${message}`;
    $('error').textContent = message;
  } finally {
    gimbalCommandBusy = false;
  }
}

function submitGimbalTarget(event) {
  event.preventDefault();
  const pan = $('gimbalPanTarget').valueAsNumber;
  const tilt = $('gimbalTiltTarget').valueAsNumber;
  if (!Number.isFinite(pan) || pan < -140 || pan > 140) {
    $('gimbalControlMessage').textContent = '左右目标必须在 -140° 到 140° 之间';
    return;
  }
  if (!Number.isFinite(tilt) || tilt < -110 || tilt > 120) {
    $('gimbalControlMessage').textContent = '上下目标必须在 -110° 到 120° 之间';
    return;
  }
  const roll = Number(state.gimbalTarget?.roll ?? state.gimbal?.angles?.roll ?? 0);
  moveGimbal(0, 0, false, {pan, tilt, roll});
}

function bindGimbalHold(id, panDelta, tiltDelta) {
  const button = $(id);
  let timer = null;
  const stop = () => {
    if (timer !== null) clearInterval(timer);
    timer = null;
  };
  button.addEventListener('pointerdown', event => {
    event.preventDefault();
    moveGimbal(panDelta, tiltDelta);
    timer = setInterval(() => moveGimbal(panDelta, tiltDelta), 250);
    button.setPointerCapture?.(event.pointerId);
  });
  button.addEventListener('pointerup', stop);
  button.addEventListener('pointercancel', stop);
  button.addEventListener('pointerleave', stop);
}

function renderRecordingCheckpoints() {
  $('checkpointRecorder').hidden = state.active?.state !== 'recording';
  $('recordingCheckpoints').innerHTML = state.recordingCheckpoints.length
    ? state.recordingCheckpoints.map(item => `<div class="checkpoint-item"><span><b>${escapeHtml(item.checkpointId)}</b> · pan ${Number(item.camera?.pan || 0).toFixed(1)}° / tilt ${Number(item.camera?.tilt || 0).toFixed(1)}°<br>${escapeHtml(item.note || '未填备注')}</span><button class="delete-checkpoint" data-checkpoint-id="${escapeHtml(item.checkpointId)}">删除重标</button></div>`).join('')
    : '尚未标记巡检点';
}

async function refreshRecordingCheckpoints() {
  if (!state.active?.session_id) {
    state.recordingCheckpoints = [];
    renderRecordingCheckpoints();
    return;
  }
  const value = await api(`/api/v1/sessions/${encodeURIComponent(state.active.session_id)}/checkpoints`);
  state.recordingCheckpoints = value.checkpoints || [];
  renderRecordingCheckpoints();
}

async function markCheckpoint() {
  if (!state.active) return;
  $('markCheckpoint').disabled = true;
  try {
    await post(`/api/v1/sessions/${encodeURIComponent(state.active.session_id)}/checkpoints`, {
      note: $('checkpointNote').value,
      spin: false,
      camera: state.gimbal?.angles || {pan: 0, tilt: 0, roll: 0},
    });
    $('checkpointNote').value = '';
    await refreshRecordingCheckpoints();
  } catch (error) {
    $('error').textContent = friendlyError(error);
  } finally {
    $('markCheckpoint').disabled = false;
  }
}

function renderRecording() {
  const button = $('recordButton');
  if (state.active?.state === 'recording') {
    button.textContent = '停止录制并同步到 Mac';
    button.className = 'stop';
    $('phase').textContent = '正在录制';
    $('phaseHint').textContent = state.active.session_id;
    $('recordInfo').textContent = `已采集 ${state.active.sample_count} 帧`;
  } else {
    button.textContent = '开始录制地图';
    button.className = '';
    $('phase').textContent = state.job ? '数据与建图处理中' : '观察模式';
    $('phaseHint').textContent = state.job?.message || '尚未录制';
    $('recordInfo').textContent = '等待开始';
  }
  button.disabled = !state.robotReachable || (
    !state.active && !state.live?.device?.online
  );
  renderRecordingCheckpoints();
}

async function toggleRecord() {
  try {
    $('recordButton').disabled = true;
    $('error').textContent = '';
    if (state.active) {
      const outcome = await api(`/api/v1/sessions/${state.active.session_id}/stop`, {method: 'POST'});
      state.active = null;
      state.job = outcome.map_job || null;
    } else {
      state.active = await api('/api/v1/sessions/start', {method: 'POST'});
      await refreshRecordingCheckpoints();
    }
    await refreshMapJobs();
    await refreshSessions();
    renderRecording();
  } catch (error) {
    setFault(error);
    $('recordButton').disabled = false;
  }
}

async function refreshJob() {
  if (!state.job) return;
  state.job = await api(`/api/v1/map-jobs/${state.job.job_id}`);
  $('jobProgress').style.width = `${state.job.progress}%`;
  $('jobMessage').textContent = state.job.error || state.job.message;
  $('transferDetail').textContent = fmtTransfer(state.job);
  if (state.job.state === 'complete') {
    await refreshMapJobs();
    await showResult(state.job);
    state.job = null;
    renderRecording();
  } else if (state.job.state === 'failed') {
    state.job = null;
    renderRecording();
  }
}

function redrawResult() {
  if (!state.mapArtifact) return;
  drawCloud(
    $('resultCloud'),
    state.mapArtifact.points || [],
    state.mapArtifact.trajectory || [],
    state.views.result,
  );
}

async function showResult(job, {explicit = false} = {}) {
  if (!job?.point_cloud_url) return;
  if (explicit) state.mapSelectionExplicit = true;
  if (state.latestMapJob?.job_id !== job.job_id) state.navigationPrepare = null;
  state.latestMapJob = job;
  $('resultEmpty').style.display = 'none';
  document.querySelector('.map-review').style.display = 'grid';
  $('overview').src = job.overview_url;
  state.mapArtifact = await api(job.point_cloud_url);
  state.navigationWorkspace = await api(
    `/api/v1/map-jobs/${encodeURIComponent(job.job_id)}/navigation-workspace`,
  );
  try {
    state.navigationSurface = await api(
      `/api/v1/map-jobs/${encodeURIComponent(job.job_id)}/navigation-surface`,
    );
  } catch (_) {
    state.navigationSurface = null;
  }
  try {
    state.glimEditor = await api(
      `/api/v1/map-jobs/${encodeURIComponent(job.job_id)}/glim-editor`,
    );
  } catch (_) {
    state.glimEditor = {state: 'unavailable'};
  }
  state.savedNavigationWorkspace = structuredClone(state.navigationWorkspace);
  state.checkpointAudit = null;
  state.platformUpload = null;
  state.workspaceEditMode = null;
  state.workspaceDisplayMode = 'split';
  state.workspaceView = {zoom: 1, centerX: null, centerY: null, panMode: false, drag: null};
  state.workspaceCursorWorld = null;
  state.workspaceCloudView = {yaw: 0.72, pitch: 0.62, zoom: 18, zoomFactor: 1, centerX: null, centerY: null, sliceOnly: false};
  $('workspacePan').classList.remove('active');
  $('workspaceMap').classList.remove('pan-mode');
  $('workspace3dSlice').classList.remove('active');
  $('workspace3dSlice').textContent = '只看高度层';
  setWorkspaceDisplayMode('split');
  state.surfaceUndo = [];
  state.planPreview = null;
  state.planStart = state.navigationWorkspace?.route?.[0] || null;
  state.planGoal = state.navigationWorkspace?.route?.at(-1) || null;
  syncSurfaceControls();
  const mapName = job.label ? `${job.label} · ${job.job_id}` : job.job_id;
  $('resultMeta').textContent = `${mapName} · ${job.metrics?.point_count || state.mapArtifact.points.length} 点 · ${job.metrics?.worker || 'cloud-glim'}`;
  redrawResult();
  drawWorkspace();
  renderWorkspaceControls();
  if (state.navigationWorkspace.ready) await refreshCheckpointAudit();
  await refreshPlatformUpload();
  renderMapHistory();
  drawNavigation();
}

function workspaceBounds() {
  const points = (state.mapArtifact?.points || []).map(point => [point[0], point[1]]);
  const route = state.navigationWorkspace?.route || [];
  const area = state.navigationWorkspace?.allowedArea || [];
  const all = points.concat(route, area);
  if (!all.length) return null;
  const minX = Math.min(...all.map(point => point[0]));
  const maxX = Math.max(...all.map(point => point[0]));
  const minY = Math.min(...all.map(point => point[1]));
  const maxY = Math.max(...all.map(point => point[1]));
  return {minX, maxX, minY, maxY};
}

function workspaceViewBounds() {
  const source = workspaceBounds();
  if (!source) return null;
  const view = state.workspaceView;
  const spanX = Math.max(source.maxX - source.minX, 1.0);
  const spanY = Math.max(source.maxY - source.minY, 1.0);
  if (!Number.isFinite(view.centerX)) view.centerX = (source.minX + source.maxX) / 2;
  if (!Number.isFinite(view.centerY)) view.centerY = (source.minY + source.maxY) / 2;
  const zoom = Math.max(1, Math.min(16, Number(view.zoom || 1)));
  const halfX = spanX / (2 * zoom);
  const halfY = spanY / (2 * zoom);
  view.centerX = Math.max(source.minX + halfX, Math.min(source.maxX - halfX, view.centerX));
  view.centerY = Math.max(source.minY + halfY, Math.min(source.maxY - halfY, view.centerY));
  return {
    minX: view.centerX - halfX,
    maxX: view.centerX + halfX,
    minY: view.centerY - halfY,
    maxY: view.centerY + halfY,
    source,
    zoom,
  };
}

function workspaceWorldToCanvas(point) {
  const transform = state.workspaceTransform;
  if (!transform) return null;
  return [
    transform.offsetX + (point[0] - transform.bounds.minX) * transform.scale,
    transform.height - transform.offsetY - (point[1] - transform.bounds.minY) * transform.scale,
  ];
}

function updateWorkspaceReadout() {
  const transform = state.workspaceTransform;
  const zoom = transform?.bounds?.zoom || state.workspaceView.zoom || 1;
  $('workspaceZoomLabel').textContent = `${Math.round(zoom * 100)}%`;
  const cursor = state.workspaceCursorWorld;
  const brush = Number($('surfaceBrushSize')?.value || 0.35);
  if (!cursor) {
    $('workspaceCursor').textContent = `移动鼠标查看坐标 · 刷子 ${brush.toFixed(2)} m`;
    return;
  }
  const nearby = (state.mapArtifact?.points || []).filter(point => Math.hypot(
    Number(point[0]) - cursor[0], Number(point[1]) - cursor[1],
  ) <= Math.max(0.18, brush / 2));
  const heights = nearby.map(point => Number(point[2] || 0)).filter(Number.isFinite);
  const heightText = heights.length
    ? `附近 ${heights.length} 点 · 高度 ${Math.min(...heights).toFixed(2)}~${Math.max(...heights).toFixed(2)} m`
    : '附近没有点云';
  $('workspaceCursor').textContent = `x ${cursor[0].toFixed(2)} m · y ${cursor[1].toFixed(2)} m · ${heightText} · 刷子 ${brush.toFixed(2)} m`;
}

function drawWorkspaceScale(context, width, height, scale) {
  if (!Number.isFinite(scale) || scale <= 0) return;
  const candidates = [0.25, 0.5, 1, 2, 5, 10, 20];
  const worldLength = candidates.find(value => value * scale >= 70) || 20;
  const pixels = worldLength * scale;
  const startX = 24;
  const y = height - 58;
  context.strokeStyle = '#e8f0fa';
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(startX, y); context.lineTo(startX + pixels, y);
  context.moveTo(startX, y - 5); context.lineTo(startX, y + 5);
  context.moveTo(startX + pixels, y - 5); context.lineTo(startX + pixels, y + 5);
  context.stroke();
  context.fillStyle = '#e8f0fa';
  context.font = '11px sans-serif';
  context.fillText(`${worldLength} m`, startX, y - 9);
}

function workspaceGroundModel() {
  return state.navigationSurface?.ground || null;
}

function addProjectedGroundCells(context, rows, resolution, view, width, height, color) {
  if (!rows?.length) return;
  context.beginPath();
  for (const row of rows) {
    const cellX = Number(row[0]);
    const cellY = Number(row[1]);
    const elevation = Number(row[2] || 0);
    const corners = [
      [cellX * resolution, cellY * resolution, elevation],
      [(cellX + 1) * resolution, cellY * resolution, elevation],
      [(cellX + 1) * resolution, (cellY + 1) * resolution, elevation],
      [cellX * resolution, (cellY + 1) * resolution, elevation],
    ].map(point => projectCentered(point, view, width, height));
    context.moveTo(...corners[0]);
    for (const corner of corners.slice(1)) context.lineTo(...corner);
    context.closePath();
  }
  context.fillStyle = color;
  context.fill();
}

function drawWorkspaceGround3d(context, view, width, height) {
  const ground = workspaceGroundModel();
  if (!ground || !$('showWorkspaceGround').checked) return;
  const resolution = Number(ground.resolutionM || 0.20);
  addProjectedGroundCells(context, ground.unknownCells, resolution, view, width, height, '#64748b2e');
  addProjectedGroundCells(
    context, ground.inferredCells,
    resolution, view, width, height, '#22d3ee4f',
  );
  addProjectedGroundCells(
    context, ground.measuredCells,
    resolution, view, width, height, '#10b98187',
  );
}

function drawWorkspaceCloud() {
  const canvas = $('workspaceCloud');
  if (state.workspaceDisplayMode === '2d' || canvas.clientWidth < 2 || canvas.clientHeight < 2) return;
  const [context, width, height] = sizeCanvas(canvas);
  context.fillStyle = '#071421';
  context.fillRect(0, 0, width, height);
  const points = state.mapArtifact?.points || [];
  const view = state.workspaceCloudView;
  const source = workspaceBounds();
  if (!source) return;
  const visible = workspaceViewBounds();
  view.centerX = Number(state.workspaceView.centerX ?? (source.minX + source.maxX) / 2);
  view.centerY = Number(state.workspaceView.centerY ?? (source.minY + source.maxY) / 2);
  const spanX = Math.max((visible?.maxX || source.maxX) - (visible?.minX || source.minX), 0.5);
  const spanY = Math.max((visible?.maxY || source.maxY) - (visible?.minY || source.minY), 0.5);
  view.zoom = Math.max(3, Math.min(220, Math.min(width / spanX, height / spanY) * 0.72 * Number(view.zoomFactor || 1)));
  const surface = state.navigationWorkspace?.navigationSurface || {};
  const minimumZ = Number(surface.obstacleMinZ ?? 0.05);
  const maximumZ = Number(surface.obstacleMaxZ ?? 1.80);
  const ground = workspaceGroundModel();
  const groundZ = Number(ground?.referenceElevationM || 0);

  drawWorkspaceGround3d(context, view, width, height);

  const area = state.navigationWorkspace?.allowedArea || [];
  if ($('showWorkspaceArea').checked && area.length >= 2) {
    context.strokeStyle = '#4ade80';
    context.lineWidth = 2.5;
    context.beginPath();
    area.forEach((point, index) => {
      const projected = projectCentered([point[0], point[1], groundZ + 0.04], view, width, height);
      index ? context.lineTo(...projected) : context.moveTo(...projected);
    });
    if (area.length >= 3) context.closePath();
    context.stroke();
  }

  if ($('showWorkspaceSurface').checked) {
    const resolution = Number(surface.resolutionM || 0.10);
    const rows = [...visibleNavigationSurfaceCells(surface)].map(key => {
      const [cellX, cellY] = key.split(',').map(Number);
      return [cellX, cellY, groundZ + 0.06];
    });
    addProjectedGroundCells(context, rows, resolution, view, width, height, '#e11d48a8');
  }

  if ($('showWorkspacePoints').checked) {
    for (const point of points) {
      if (view.sliceOnly && (Number(point[2]) < minimumZ || Number(point[2]) > maximumZ)) continue;
      const [x, y] = projectCentered(point, view, width, height);
      if (x < 0 || y < 0 || x > width || y > height) continue;
      const z = Number(point[2] || 0);
      context.fillStyle = `hsla(${190 + z * 20},88%,64%,.78)`;
      context.fillRect(x, y, 1.8, 1.8);
    }
  }

  const route = state.navigationWorkspace?.route || [];
  if ($('showWorkspaceRoute').checked && route.length) {
    context.strokeStyle = '#22d3ee';
    context.lineWidth = 3;
    context.beginPath();
    route.forEach((point, index) => {
      const projected = projectCentered([point[0], point[1], groundZ + 0.10], view, width, height);
      index ? context.lineTo(...projected) : context.moveTo(...projected);
    });
    context.stroke();
  }

  if (state.workspaceCursorWorld) {
    let samples = points.filter(point => Math.hypot(
      Number(point[0]) - state.workspaceCursorWorld[0],
      Number(point[1]) - state.workspaceCursorWorld[1],
    ) <= 0.28);
    if (!samples.length) {
      let nearest = null;
      let nearestDistance = Infinity;
      for (const point of points) {
        const distance = Math.hypot(
          Number(point[0]) - state.workspaceCursorWorld[0],
          Number(point[1]) - state.workspaceCursorWorld[1],
        );
        if (distance < nearestDistance) { nearest = point; nearestDistance = distance; }
      }
      if (nearest) samples = [nearest];
    }
    const heights = samples.map(point => Number(point[2] || 0)).filter(Number.isFinite);
    const lowZ = heights.length ? Math.min(...heights) : groundZ;
    const highZ = heights.length ? Math.max(...heights) : groundZ + 1.8;
    const centerZ = (lowZ + highZ) / 2;
    const low = projectCentered([state.workspaceCursorWorld[0], state.workspaceCursorWorld[1], lowZ], view, width, height);
    const high = projectCentered([state.workspaceCursorWorld[0], state.workspaceCursorWorld[1], highZ], view, width, height);
    const [x, y] = projectCentered([state.workspaceCursorWorld[0], state.workspaceCursorWorld[1], centerZ], view, width, height);
    context.strokeStyle = '#facc15'; context.lineWidth = 3;
    context.beginPath(); context.moveTo(...low); context.lineTo(...high); context.stroke();
    context.beginPath(); context.arc(x, y, 10, 0, Math.PI * 2); context.stroke();
  }
}

function setWorkspaceDisplayMode(mode) {
  if (!['2d', '3d', 'split'].includes(mode)) return;
  state.workspaceDisplayMode = mode;
  const viewport = $('workspaceViewport');
  const buttonIds = {
    '2d': 'workspaceView2d',
    '3d': 'workspaceView3d',
    split: 'workspaceViewSplit',
  };
  for (const value of ['2d', '3d', 'split']) {
    viewport.classList.toggle(`view-${value}`, value === mode);
    $(buttonIds[value]).classList.toggle('active', value === mode);
  }
  requestAnimationFrame(() => {
    drawWorkspace();
    drawWorkspaceCloud();
  });
}

function setWorkspaceView(bounds, zoom = 1) {
  if (!bounds) return;
  state.workspaceView.centerX = (bounds.minX + bounds.maxX) / 2;
  state.workspaceView.centerY = (bounds.minY + bounds.maxY) / 2;
  const source = workspaceBounds();
  const sourceSpanX = Math.max((source?.maxX || 1) - (source?.minX || 0), 1);
  const sourceSpanY = Math.max((source?.maxY || 1) - (source?.minY || 0), 1);
  const wantedSpanX = Math.max(bounds.maxX - bounds.minX, 0.5);
  const wantedSpanY = Math.max(bounds.maxY - bounds.minY, 0.5);
  const minimumZoom = ['blocked', 'clear'].includes(state.workspaceEditMode) ? 2 : 1;
  state.workspaceView.zoom = Math.max(minimumZoom, Math.min(16, zoom * Math.min(
    sourceSpanX / wantedSpanX,
    sourceSpanY / wantedSpanY,
  )));
  drawWorkspace();
}

function zoomWorkspace(factor, anchor = state.workspaceCursorWorld) {
  const previousTransform = state.workspaceTransform;
  const anchorCanvas = anchor && previousTransform ? workspaceWorldToCanvas(anchor) : null;
  const previous = Number(state.workspaceView.zoom || 1);
  const minimumZoom = ['blocked', 'clear'].includes(state.workspaceEditMode) ? 2 : 1;
  state.workspaceView.zoom = Math.max(minimumZoom, Math.min(16, previous * factor));
  if (anchor && anchorCanvas && previousTransform) {
    const nextBounds = workspaceViewBounds();
    const canvas = $('workspaceMap');
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    const padding = 42;
    const spanX = Math.max(nextBounds.maxX - nextBounds.minX, 0.1);
    const spanY = Math.max(nextBounds.maxY - nextBounds.minY, 0.1);
    const nextScale = Math.min((width - 2 * padding) / spanX, (height - 2 * padding) / spanY);
    const nextOffsetX = (width - spanX * nextScale) / 2;
    const nextOffsetY = (height - spanY * nextScale) / 2;
    const worldAtAnchorCanvas = [
      nextBounds.minX + (anchorCanvas[0] - nextOffsetX) / nextScale,
      nextBounds.minY + (height - nextOffsetY - anchorCanvas[1]) / nextScale,
    ];
    state.workspaceView.centerX += anchor[0] - worldAtAnchorCanvas[0];
    state.workspaceView.centerY += anchor[1] - worldAtAnchorCanvas[1];
  }
  drawWorkspace();
}

function fillWorkspaceGround2d(context, rows, resolution, scale, worldToCanvas, color) {
  if (!rows?.length) return;
  context.fillStyle = color;
  const size = Math.max(1.2, resolution * scale + 0.35);
  for (const row of rows) {
    const topLeft = worldToCanvas([
      Number(row[0]) * resolution,
      (Number(row[1]) + 1) * resolution,
    ]);
    context.fillRect(topLeft[0], topLeft[1], size, size);
  }
}

function drawWorkspaceGround2d(context, scale, worldToCanvas) {
  const ground = workspaceGroundModel();
  if (!ground || !$('showWorkspaceGround').checked) return;
  const resolution = Number(ground.resolutionM || 0.20);
  fillWorkspaceGround2d(context, ground.unknownCells, resolution, scale, worldToCanvas, '#64748b32');
  fillWorkspaceGround2d(
    context, ground.inferredCells,
    resolution, scale, worldToCanvas, '#22d3ee4a',
  );
  fillWorkspaceGround2d(
    context, ground.measuredCells,
    resolution, scale, worldToCanvas, '#10b98180',
  );
}

function drawWorkspace() {
  if (state.workspaceDisplayMode === '3d') {
    drawWorkspaceCloud();
    return;
  }
  const canvas = $('workspaceMap');
  const [context, width, height] = sizeCanvas(canvas);
  context.fillStyle = '#07111f';
  context.fillRect(0, 0, width, height);
  const bounds = workspaceViewBounds();
  if (!bounds) {
    state.workspaceTransform = null;
    return;
  }
  const padding = 42;
  const spanX = Math.max(bounds.maxX - bounds.minX, 0.1);
  const spanY = Math.max(bounds.maxY - bounds.minY, 0.1);
  const scale = Math.min((width - 2 * padding) / spanX, (height - 2 * padding) / spanY);
  const offsetX = (width - spanX * scale) / 2;
  const offsetY = (height - spanY * scale) / 2;
  const worldToCanvas = point => [
    offsetX + (point[0] - bounds.minX) * scale,
    height - offsetY - (point[1] - bounds.minY) * scale,
  ];
  state.workspaceTransform = {bounds, scale, offsetX, offsetY, height};
  drawWorkspaceGround2d(context, scale, worldToCanvas);
  const area = state.navigationWorkspace?.allowedArea || [];
  if ($('showWorkspaceArea').checked && area.length >= 2) {
    context.beginPath();
    area.forEach((point, index) => {
      const value = worldToCanvas(point);
      index ? context.lineTo(...value) : context.moveTo(...value);
    });
    if (area.length >= 3) context.closePath();
    context.fillStyle = '#4ade8026';
    context.fill();
    context.strokeStyle = '#4ade80';
    context.lineWidth = 3;
    context.stroke();
  }
  const surface = state.navigationWorkspace?.navigationSurface || {};
  const resolution = Number(surface.resolutionM || 0.10);
  const maximumZ = Number(surface.obstacleMaxZ ?? 1.80);
  const occupied = visibleNavigationSurfaceCells(surface);
  if ($('showWorkspaceSurface').checked) {
    context.fillStyle = '#e11d4899';
    for (const key of occupied) {
      const cell = key.split(',').map(Number);
      const topLeft = worldToCanvas([cell[0] * resolution, (cell[1] + 1) * resolution]);
      context.fillRect(
        topLeft[0],
        topLeft[1],
        Math.max(1.5, resolution * scale + 0.4),
        Math.max(1.5, resolution * scale + 0.4),
      );
    }
    if (state.workspaceEditMode === 'clear') {
      context.fillStyle = '#67e8f929';
      context.strokeStyle = '#67e8f9aa';
      context.lineWidth = 1;
      for (const cell of surface.manualClearCells || []) {
        const topLeft = worldToCanvas([cell[0] * resolution, (cell[1] + 1) * resolution]);
        const size = Math.max(1.5, resolution * scale + 0.4);
        context.fillRect(topLeft[0], topLeft[1], size, size);
        context.strokeRect(topLeft[0], topLeft[1], size, size);
      }
    }
  }
  context.fillStyle = '#cbd5e171';
  const points = state.mapArtifact?.points || [];
  if ($('showWorkspacePoints').checked) {
    for (let index = 0; index < points.length; index += 1) {
      if (Number(points[index][2]) > maximumZ + 0.4) continue;
      const point = worldToCanvas(points[index]);
      context.fillRect(point[0], point[1], Math.max(1.2, Math.min(2.4, scale * 0.015)), Math.max(1.2, Math.min(2.4, scale * 0.015)));
    }
  }
  const route = state.navigationWorkspace?.route || [];
  if ($('showWorkspaceRoute').checked && route.length) {
    context.strokeStyle = '#22d3ee';
    context.lineWidth = 3;
    context.beginPath();
    route.forEach((point, index) => {
      const value = worldToCanvas(point);
      index ? context.lineTo(...value) : context.moveTo(...value);
    });
    context.stroke();
    context.fillStyle = '#a5f3fc';
    route.forEach(point => {
      const value = worldToCanvas(point);
      context.beginPath(); context.arc(value[0], value[1], 3.5, 0, Math.PI * 2); context.fill();
    });
  }
  const previewPaths = state.planPreview?.segments?.length
    ? state.planPreview.segments.map(segment => segment.path || [])
    : state.planPreview?.path?.length ? [state.planPreview.path] : [];
  if (previewPaths.some(path => path.length)) {
    context.strokeStyle = '#c084fc';
    context.lineWidth = 4;
    context.setLineDash([9, 6]);
    for (const path of previewPaths) {
      if (!path.length) continue;
      context.beginPath();
      path.forEach((point, index) => {
        const value = worldToCanvas(point);
        index ? context.lineTo(...value) : context.moveTo(...value);
      });
      context.stroke();
    }
    context.setLineDash([]);
  }
  for (const [point, color, label] of [
    [state.planStart, '#facc15', '起'],
    [state.planGoal, '#c084fc', '终'],
  ]) {
    if (!point) continue;
    const value = worldToCanvas(point);
    context.fillStyle = color;
    context.beginPath(); context.arc(value[0], value[1], 7, 0, Math.PI * 2); context.fill();
    context.fillStyle = '#07111f'; context.font = 'bold 10px sans-serif';
    context.fillText(label, value[0] - 5, value[1] + 3.5);
  }
  for (const checkpoint of $('showWorkspaceRoute').checked ? (state.checkpointAudit?.checkpoints || []) : []) {
    const point = checkpoint.position ? [checkpoint.position.x, checkpoint.position.y] : null;
    if (!point) continue;
    const value = worldToCanvas(point);
    context.fillStyle = checkpoint.binding?.needsReview ? '#fb923c' : '#f43f5e';
    context.beginPath(); context.arc(value[0], value[1], 7, 0, Math.PI * 2); context.fill();
    context.fillStyle = '#ffffff'; context.font = '10px sans-serif';
    context.fillText(checkpoint.checkpointId, value[0] + 9, value[1] - 7);
  }
  if (state.workspaceCursorWorld) {
    const value = worldToCanvas(state.workspaceCursorWorld);
    context.strokeStyle = '#facc15'; context.lineWidth = 1.5;
    context.setLineDash([5, 5]);
    context.beginPath(); context.moveTo(value[0], 0); context.lineTo(value[0], height);
    context.moveTo(0, value[1]); context.lineTo(width, value[1]); context.stroke();
    context.setLineDash([]);
    const brushRadius = Number($('surfaceBrushSize').value || 0.35) / 2;
    if (['blocked', 'clear'].includes(state.workspaceEditMode)) {
      context.strokeStyle = state.workspaceEditMode === 'blocked' ? '#fb7185' : '#e8f0fa';
      context.lineWidth = 2;
      context.beginPath(); context.arc(value[0], value[1], brushRadius * scale, 0, Math.PI * 2); context.stroke();
    }
  }
  drawWorkspaceScale(context, width, height, scale);
  updateWorkspaceReadout();
  drawWorkspaceCloud();
}

function recordedWorkspaceRoute() {
  return (state.navigationWorkspace?.recordedRoute || []).map(point => [
    Number(point[0]), Number(point[1]),
  ]);
}

function renderWorkspaceControls() {
  const workspace = state.navigationWorkspace;
  const editing = Boolean(state.workspaceEditMode);
  const surfaceEditing = ['blocked', 'clear'].includes(state.workspaceEditMode);
  $('workspaceBadge').textContent = !workspace
    ? '请先选地图'
    : workspace.ready ? `可发布 · 版本 ${workspace.revision}`
      : workspace.allowedArea?.length >= 3 ? '等待确认导航地图' : '等待绿色可走外圈';
  $('workspaceBadge').className = workspace?.ready ? 'online' : '';
  const mode = state.workspaceEditMode;
  const modeTitle = mode === 'clear' ? '正在擦除红色'
    : mode === 'blocked' ? '正在补画红色'
      : mode === 'area' ? '正在画绿色外圈'
        : mode === 'route' ? '正在画蓝色路线'
          : mode === 'plan-start' ? '正在选择起点'
            : mode === 'plan-goal' ? '正在选择目标'
              : '查看地图';
  const modeHint = mode === 'clear' ? '在地图上按住鼠标拖动；浅蓝格表示擦除标记。'
    : mode === 'blocked' ? '在地图上按住鼠标拖动，把确认不可走的位置补成红色。'
      : mode === 'area' ? '沿可走区域外边界依次点击，保存时自动闭合。'
        : mode === 'route' ? '按巡检顺序依次点击路线点。'
          : mode === 'plan-start' ? '在绿色区域内点击预演起点。'
            : mode === 'plan-goal' ? '在绿色区域内点击预演目标。'
              : workspace ? '选择右侧工具后，在这里直接操作。' : '先从上方历史中选择地图。';
  $('workspaceModeTitle').textContent = modeTitle;
  $('workspaceModeHint').textContent = modeHint;
  $('workspaceModeBanner').className = `workspace-mode-banner${editing ? ' editing' : ''}${mode === 'clear' ? ' clear-mode' : ''}${mode === 'blocked' ? ' blocked-mode' : ''}`;
  $('finishWorkspaceEdit').hidden = !editing;
  $('workspaceHelp').textContent = state.workspaceEditMode === 'route'
      ? '请在地图上依次点击推荐路线点；蓝线保留巡检顺序，狗端由 Nav2 规划实际可走路径。'
    : state.workspaceEditMode === 'area'
      ? '请沿允许行走区域的外边界依次点击；保存时会自动闭合。系统会按与 Nav2 一致的前 0.50 m、后 0.43 m、左右各 0.30 m 加垫矩形校验蓝线。'
      : state.workspaceEditMode === 'blocked'
        ? '已进入补画模式：在地图上按住鼠标拖动，表示这里固定不可走。'
        : state.workspaceEditMode === 'clear'
          ? '已进入擦除模式：在地图上按住鼠标拖动，只擦确认能走的地方。'
          : state.workspaceEditMode === 'plan-start'
            ? '在绿色区域里点一下作为预演起点。'
            : state.workspaceEditMode === 'plan-goal'
              ? '在绿色区域里点一下作为预演目标。'
      : workspace?.ready
        ? '已准备好：静态导航地图、推荐路线和绿色区域会一起发布。'
        : '先确认红色障碍，再画绿色可走外圈，最后预演和保存。';
  for (const id of ['editRoute', 'restoreRecordedRoute', 'editAllowedArea', 'indoorSurfacePreset', 'outdoorSurfacePreset', 'selectPlanStart', 'selectPlanGoal', 'previewNavigationPlan']) {
    $(id).disabled = !workspace || editing;
  }
  for (const id of ['paintBlocked', 'paintClear', 'clearSurfaceEdits']) {
    $(id).disabled = !workspace || (editing && !surfaceEditing);
  }
  const pointEditing = ['route', 'area'].includes(state.workspaceEditMode);
  const pointKey = state.workspaceEditMode === 'route' ? 'route' : 'allowedArea';
  const canUndo = surfaceEditing
    ? state.surfaceUndo.length > 0
    : pointEditing && Boolean(workspace?.[pointKey]?.length);
  $('undoWorkspacePoint').textContent = surfaceEditing ? '撤销上一笔' : '撤销上一个点';
  $('undoWorkspacePoint').disabled = !canUndo;
  $('cancelWorkspaceEdit').disabled = !editing;
  $('finishWorkspaceEdit').disabled = !editing;
  $('saveWorkspace').disabled = !workspace || editing;
  for (const id of ['paintBlocked', 'paintClear', 'selectPlanStart', 'selectPlanGoal']) $(id).classList.remove('active');
  if (state.workspaceEditMode === 'blocked') $('paintBlocked').classList.add('active');
  if (state.workspaceEditMode === 'clear') $('paintClear').classList.add('active');
  if (state.workspaceEditMode === 'plan-start') $('selectPlanStart').classList.add('active');
  if (state.workspaceEditMode === 'plan-goal') $('selectPlanGoal').classList.add('active');
  for (const id of ['workspaceView2d', 'workspaceView3d', 'workspaceViewSplit', 'workspaceZoomOut', 'workspaceZoomIn', 'workspaceFit', 'workspaceFocusRoute', 'workspacePan', 'workspaceFullscreen', 'workspace3dSlice', 'workspace3dTop', 'workspace3dAngle']) {
    $(id).disabled = !workspace;
  }
  const editor = state.glimEditor || {state: 'not_started'};
  const editorActive = editor.state === 'active';
  $('startGlimEditor').hidden = editorActive;
  $('openGlimEditor').hidden = !editorActive || !editor.url;
  $('publishGlimEditor').hidden = !['active', 'tunnel_closed'].includes(editor.state);
  $('stopGlimEditor').hidden = !['active', 'tunnel_closed'].includes(editor.state);
  $('startGlimEditor').disabled = !workspace;
  $('glimEditorState').textContent = editorActive
    ? '官方 GLIM 清图工作台已打开。删除临时人车后，务必点 File -> Save map 并选择 SAVED_MAP，再回来生成新版本。'
    : editor.state === 'published'
      ? `清理结果已生成新地图 ${editor.publishedJobId || ''}，原地图未修改。`
      : editor.state === 'tunnel_closed'
        ? '云端编辑会话还在，但 Mac 浏览器通道已断开；可直接导出已保存的结果。'
        : '需要清掉建图时的人、车等临时点云时，才打开官方 GLIM 3D 工具。';
}

function renderCheckpointAudit() {
  const audit = state.checkpointAudit;
  if (!audit) {
    $('checkpointAudit').textContent = '保存路线和绿色区域后，这里会显示录制时标记的点位。';
    $('checkpointAuditList').innerHTML = '';
    return;
  }
  $('checkpointAudit').textContent = `${audit.audit?.message || ''} · 共 ${audit.checkpoints?.length || 0} 个点`;
  $('checkpointAudit').className = audit.audit?.ready ? 'ok' : 'warn';
  $('checkpointAuditList').innerHTML = (audit.checkpoints || []).map(item => `<div class="checkpoint-item"><span><b>${escapeHtml(item.checkpointId)}</b> · 路线索引 ${item.routeProgressIndex}<br>距路线 ${Number(item.binding?.distanceToExecutionRouteM || 0).toFixed(2)} m · pan ${Number(item.camera?.pan || 0).toFixed(1)}° / tilt ${Number(item.camera?.tilt || 0).toFixed(1)}°</span><b>${item.binding?.needsReview ? '待复核' : '已对齐'}</b></div>`).join('') || '<small>本次录制没有标记巡检点，仍可发布纯路线。</small>';
  drawWorkspace();
  drawNavigation();
}

async function refreshCheckpointAudit() {
  if (!state.latestMapJob || !state.navigationWorkspace?.ready) return;
  try {
    state.checkpointAudit = await api(`/api/v1/map-jobs/${encodeURIComponent(state.latestMapJob.job_id)}/checkpoints`);
  } catch (error) {
    state.checkpointAudit = {checkpoints: [], audit: {ready: false, message: friendlyError(error)}};
  }
  renderCheckpointAudit();
  renderPlatformUpload();
}

function renderPlatformUpload() {
  const upload = state.platformUpload;
  const visible = Boolean(
    state.latestMapJob
    && state.navigationWorkspace?.ready
    && state.platformUploadSupported !== false
  );
  $('uploadPlatformBundle').hidden = !visible;
  $('platformUpload').hidden = !visible;
  $('platformUploadProgress').style.width = `${Number(upload?.progress || 0)}%`;
  $('platformUploadMessage').textContent = upload?.siteName
    ? `${upload.message || upload.state} · 项目「${upload.siteName}」`
    : upload?.message || '尚未上传';
  const transferBusy = ['queued', 'uploading', 'verifying'].includes(upload?.state);
  const acceptedCurrent = ['verified', 'activated'].includes(upload?.state) && !upload?.stale;
  $('uploadPlatformBundle').disabled = !state.checkpointAudit?.audit?.ready || transferBusy || acceptedCurrent;
  $('uploadPlatformBundle').textContent = upload?.state === 'activated' && !upload?.stale
    ? '平台已激活这一版'
    : upload?.state === 'verified' && !upload?.stale
    ? '已上传并校验通过（等待运营激活）'
    : '导出并上传到 GoGoGuard 平台';
}

async function refreshPlatformUpload() {
  if (!state.latestMapJob) return;
  try {
    state.platformUpload = await api(`/api/v1/map-jobs/${encodeURIComponent(state.latestMapJob.job_id)}/platform-upload`);
    state.platformUploadSupported = true;
    renderPlatformUpload();
  } catch (_) {
    state.platformUploadSupported = false;
    renderPlatformUpload();
  }
}

async function uploadPlatformBundle() {
  if (!state.latestMapJob) return;
  $('navigationError').textContent = '';
  state.platformUpload = {
    ...(state.platformUpload || {}),
    state: 'queued',
    progress: 0,
    message: '正在生成并上传当前地图、路线、绿色区域和巡检点，请稍候…',
  };
  renderPlatformUpload();
  try {
    state.platformUpload = await post(`/api/v1/map-jobs/${encodeURIComponent(state.latestMapJob.job_id)}/platform-upload`);
    renderPlatformUpload();
  } catch (error) {
    const message = friendlyError(error);
    state.platformUpload = {...(state.platformUpload || {}), state: 'failed', message};
    renderPlatformUpload();
    $('navigationError').textContent = message;
  }
}

async function startGlimEditor() {
  if (!state.latestMapJob) return;
  $('workspaceError').textContent = '';
  const editorWindow = window.open('about:blank', 'gogoguard-glim-editor');
  try {
    state.glimEditor = await post(
      `/api/v1/map-jobs/${encodeURIComponent(state.latestMapJob.job_id)}/glim-editor/start`,
    );
    renderWorkspaceControls();
    if (editorWindow) editorWindow.location = state.glimEditor.url;
    else window.open(state.glimEditor.url, '_blank', 'noopener');
  } catch (error) {
    if (editorWindow) editorWindow.close();
    $('workspaceError').textContent = friendlyError(error);
  }
}

function openGlimEditor() {
  if (state.glimEditor?.url) window.open(state.glimEditor.url, '_blank', 'noopener');
}

async function publishGlimEditor() {
  if (!state.latestMapJob) return;
  $('workspaceError').textContent = '';
  $('publishGlimEditor').disabled = true;
  $('glimEditorState').textContent = '正在云端导出清理后的点云并生成新地图版本，请等待……';
  try {
    const result = await post(
      `/api/v1/map-jobs/${encodeURIComponent(state.latestMapJob.job_id)}/glim-editor/publish`,
    );
    await refreshMapJobs();
    const created = state.mapJobs.find(item => item.job_id === result.mapJob.job_id) || result.mapJob;
    await showResult(created, {explicit: true});
  } catch (error) {
    $('workspaceError').textContent = friendlyError(error);
    $('publishGlimEditor').disabled = false;
  }
}

async function stopGlimEditor() {
  if (!state.latestMapJob) return;
  try {
    state.glimEditor = await post(
      `/api/v1/map-jobs/${encodeURIComponent(state.latestMapJob.job_id)}/glim-editor/stop`,
    );
    renderWorkspaceControls();
  } catch (error) {
    $('workspaceError').textContent = friendlyError(error);
  }
}

function workspaceCanvasPoint(event) {
  const transform = state.workspaceTransform;
  if (!transform) return null;
  const bounds = $('workspaceMap').getBoundingClientRect();
  const x = event.clientX - bounds.left;
  const y = event.clientY - bounds.top;
  return [
    transform.bounds.minX + (x - transform.offsetX) / transform.scale,
    transform.bounds.minY + (transform.height - transform.offsetY - y) / transform.scale,
  ];
}

function requireWorkspaceEditZoom() {
  if (Number(state.workspaceView.zoom || 1) >= 2) return true;
  state.workspaceView.zoom = 2.2;
  $('workspaceError').textContent = '';
  drawWorkspace();
  return true;
}

function startSurfaceEdit(mode) {
  if (!state.navigationWorkspace || !['blocked', 'clear'].includes(mode)) return;
  if (!['blocked', 'clear'].includes(state.workspaceEditMode)) {
    state.savedNavigationWorkspace = structuredClone(state.navigationWorkspace);
    state.surfaceUndo = [];
  }
  state.workspaceEditMode = mode;
  setWorkspaceDisplayMode('2d');
  state.workspaceView.panMode = false;
  state.workspaceView.drag = null;
  $('workspacePan').classList.remove('active');
  $('workspaceMap').classList.remove('pan-mode');
  requireWorkspaceEditZoom();
  $('workspaceError').textContent = '';
  drawWorkspace();
  renderWorkspaceControls();
}

function finishWorkspaceEditing() {
  if (!state.workspaceEditMode) return;
  state.surfaceBrushDown = false;
  state.workspaceEditMode = null;
  $('workspaceError').textContent = '';
  drawWorkspace();
  renderWorkspaceControls();
  $('workspaceHelp').textContent = '当前修改已保留在页面中，尚未保存。可以继续编辑；全部确认后再点“确认并保存这张导航地图”。';
}

function ensureNavigationSurface() {
  if (!state.navigationWorkspace) return null;
  if (!state.navigationWorkspace.navigationSurface) {
    state.navigationWorkspace.navigationSurface = {
      schema: 'gogoguard.navigation_surface_edit.v1',
      resolutionM: 0.10,
      obstacleMinZ: 0.05,
      obstacleMaxZ: 1.80,
      manualBlockedCells: [],
      manualClearCells: [],
      reviewed: false,
    };
  }
  return state.navigationWorkspace.navigationSurface;
}

function syncSurfaceControls() {
  const surface = ensureNavigationSurface();
  if (!surface) return;
  $('obstacleMinZ').value = Number(surface.obstacleMinZ ?? 0.05).toFixed(2);
  $('obstacleMaxZ').value = Number(surface.obstacleMaxZ ?? 1.80).toFixed(2);
  const automatic = visibleNavigationSurfaceCells(surface);
  const clear = new Set((surface.manualClearCells || []).map(surfaceCellKey));
  const blocked = new Set((surface.manualBlockedCells || []).map(surfaceCellKey));
  const ground = workspaceGroundModel();
  const groundStats = ground?.stats;
  const groundText = groundStats
    ? `地面：实测 ${groundStats.measuredCellCount} 格 · 插值 ${groundStats.inferredCellCount} 格 · 未知 ${groundStats.unknownCellCount} 格。`
    : '地面层尚未生成。';
  $('surfaceStats').textContent = `${groundText} 红色占用 ${automatic.size} 格 · 手工补画 ${blocked.size} 格 · 擦除 ${clear.size} 格。地面是编辑参考，现阶段不直接改变 Nav2 通行权限。`;
}

function surfaceCellKey(cell) { return `${cell[0]},${cell[1]}`; }

function pointInPolygon(point, polygon) {
  let inside = false;
  let previous = polygon.at(-1);
  for (const current of polygon) {
    const crosses = (current[1] > point[1]) !== (previous[1] > point[1]);
    if (crosses) {
      const intersectionX = (
        (previous[0] - current[0]) * (point[1] - current[1])
        / (previous[1] - current[1]) + current[0]
      );
      if (point[0] <= intersectionX) inside = !inside;
    }
    previous = current;
  }
  return inside;
}

function recordedRouteFootprintCells(route, resolution) {
  const cells = new Set();
  for (let segment = 0; segment + 1 < route.length; segment += 1) {
    const start = route[segment];
    const end = route[segment + 1];
    const distance = Math.hypot(end[0] - start[0], end[1] - start[1]);
    const steps = Math.max(1, Math.ceil(distance / (resolution / 2)));
    const yaw = Math.atan2(end[1] - start[1], end[0] - start[0]);
    const cosine = Math.cos(yaw);
    const sine = Math.sin(yaw);
    for (let step = 0; step <= steps; step += 1) {
      const ratio = step / steps;
      const x = start[0] + (end[0] - start[0]) * ratio;
      const y = start[1] + (end[1] - start[1]) * ratio;
      const footprint = PADDED_BASE_FOOTPRINT_M.map(([localX, localY]) => [
        x + cosine * localX - sine * localY,
        y + sine * localX + cosine * localY,
      ]);
      const minimumX = Math.floor(Math.min(...footprint.map(point => point[0])) / resolution);
      const maximumX = Math.floor(Math.max(...footprint.map(point => point[0])) / resolution);
      const minimumY = Math.floor(Math.min(...footprint.map(point => point[1])) / resolution);
      const maximumY = Math.floor(Math.max(...footprint.map(point => point[1])) / resolution);
      for (let cellX = minimumX; cellX <= maximumX; cellX += 1) {
        for (let cellY = minimumY; cellY <= maximumY; cellY += 1) {
          const center = [(cellX + 0.5) * resolution, (cellY + 0.5) * resolution];
          if (pointInPolygon(center, footprint)) cells.add(`${cellX},${cellY}`);
        }
      }
    }
  }
  return cells;
}

function visibleNavigationSurfaceCells(surface) {
  const resolution = Number(surface.resolutionM || 0.10);
  const counts = new Map();
  for (const point of state.mapArtifact?.points || []) {
    if (Number(point[2]) < Number(surface.obstacleMinZ) || Number(point[2]) > Number(surface.obstacleMaxZ)) continue;
    const key = `${Math.floor(Number(point[0]) / resolution)},${Math.floor(Number(point[1]) / resolution)}`;
    counts.set(key, (counts.get(key) || 0) + 1);
  }
  const rawKeys = new Set(counts.keys());
  const occupied = new Set();
  for (const [key, count] of counts) {
    const [cellX, cellY] = key.split(',').map(Number);
    let supported = count >= 2;
    for (let dx = -1; dx <= 1 && !supported; dx += 1) {
      for (let dy = -1; dy <= 1; dy += 1) {
        if ((dx || dy) && rawKeys.has(`${cellX + dx},${cellY + dy}`)) { supported = true; break; }
      }
    }
    if (supported) occupied.add(key);
  }
  if (state.navigationWorkspace?.routeSource === 'recorded') {
    const route = state.navigationWorkspace.route || [];
    for (const key of recordedRouteFootprintCells(route, resolution)) occupied.delete(key);
  }
  for (const cell of surface.manualClearCells || []) occupied.delete(surfaceCellKey(cell));
  for (const cell of surface.manualBlockedCells || []) occupied.add(surfaceCellKey(cell));
  return occupied;
}

function paintSurfaceAt(point) {
  const surface = ensureNavigationSurface();
  if (!surface || !['blocked', 'clear'].includes(state.workspaceEditMode)) return;
  if (!requireWorkspaceEditZoom()) return;
  const resolution = Number(surface.resolutionM || 0.10);
  const brushRadius = Number($('surfaceBrushSize').value || 0.35) / 2;
  const center = [Math.floor(point[0] / resolution), Math.floor(point[1] / resolution)];
  const radiusCells = Math.max(0, Math.ceil(brushRadius / resolution));
  const blocked = new Map((surface.manualBlockedCells || []).map(cell => [surfaceCellKey(cell), cell]));
  const clear = new Map((surface.manualClearCells || []).map(cell => [surfaceCellKey(cell), cell]));
  let changed = false;
  for (let dx = -radiusCells; dx <= radiusCells; dx += 1) {
    for (let dy = -radiusCells; dy <= radiusCells; dy += 1) {
      if (Math.hypot(dx, dy) * resolution > brushRadius + resolution / 2) continue;
      const cell = [center[0] + dx, center[1] + dy];
      const key = surfaceCellKey(cell);
      if (state.workspaceEditMode === 'blocked') {
        changed = clear.delete(key) || changed;
        if (!blocked.has(key)) { blocked.set(key, cell); changed = true; }
      } else {
        changed = blocked.delete(key) || changed;
        if (!clear.has(key)) { clear.set(key, cell); changed = true; }
      }
    }
  }
  if (!changed) return;
  surface.manualBlockedCells = [...blocked.values()];
  surface.manualClearCells = [...clear.values()];
  surface.reviewed = false;
  state.planPreview = null;
  syncSurfaceControls();
  drawWorkspace();
}

function snapshotSurfaceUndo() {
  const surface = ensureNavigationSurface();
  if (!surface) return;
  state.surfaceUndo.push({
    manualBlockedCells: structuredClone(surface.manualBlockedCells || []),
    manualClearCells: structuredClone(surface.manualClearCells || []),
  });
  if (state.surfaceUndo.length > 30) state.surfaceUndo.shift();
}

function applySurfacePreset(minimumZ, maximumZ) {
  const surface = ensureNavigationSurface();
  if (!surface) return;
  surface.obstacleMinZ = minimumZ;
  surface.obstacleMaxZ = maximumZ;
  surface.reviewed = false;
  state.planPreview = null;
  syncSurfaceControls();
  drawWorkspace();
  renderWorkspaceControls();
}

async function previewNavigationPlan() {
  if (!state.latestMapJob || !state.navigationWorkspace) return;
  const route = state.navigationWorkspace.route || [];
  state.planStart ||= route[0] || null;
  state.planGoal ||= route.at(-1) || null;
  if (!state.planStart || !state.planGoal) {
    $('planPreviewResult').textContent = '请先确认起点和目标。';
    return;
  }
  if (state.checkpointAudit && !state.checkpointAudit.audit?.ready) {
    $('planPreviewResult').textContent = `暂时不能预演整趟巡检：${state.checkpointAudit.audit?.message || '巡检点尚未完成路线绑定'}。`;
    return;
  }
  const rawCheckpoints = state.checkpointAudit?.checkpoints || [];
  const checkpoints = rawCheckpoints
    .filter(checkpoint => Number.isFinite(Number(checkpoint.position?.x)) && Number.isFinite(Number(checkpoint.position?.y)))
    .sort((left, right) => (
      Number(left.routeProgressIndex ?? Number.MAX_SAFE_INTEGER)
      - Number(right.routeProgressIndex ?? Number.MAX_SAFE_INTEGER)
    ) || String(left.checkpointId).localeCompare(String(right.checkpointId)));
  if (checkpoints.length !== rawCheckpoints.length) {
    $('planPreviewResult').textContent = '暂时不能预演整趟巡检：有巡检点缺少可用的地图坐标。';
    return;
  }
  const stops = [
    {label: '起点', point: state.planStart},
    ...checkpoints.map(checkpoint => ({
      label: String(checkpoint.checkpointId),
      point: [Number(checkpoint.position.x), Number(checkpoint.position.y)],
    })),
    {label: '终点', point: state.planGoal},
  ];
  const legs = stops.slice(0, -1).map((stop, index) => ({from: stop, to: stops[index + 1]}));
  $('planPreviewResult').textContent = `正在按巡检顺序规划 ${legs.length} 段路线……`;
  $('previewNavigationPlan').disabled = true;
  try {
    const surface = ensureNavigationSurface();
    const workspace = {
      route: state.navigationWorkspace.route,
      allowedArea: state.navigationWorkspace.allowedArea,
      routeSource: state.navigationWorkspace.routeSource,
      robotRadiusM: state.navigationWorkspace.robotRadiusM || 0.48,
      navigationSurface: {...surface, reviewed: false},
    };
    const segments = await Promise.all(legs.map(async leg => {
      const distance = Math.hypot(
        leg.to.point[0] - leg.from.point[0],
        leg.to.point[1] - leg.from.point[1],
      );
      if (distance < 0.02) {
        return {
          fromLabel: leg.from.label,
          toLabel: leg.to.label,
          reachable: true,
          reason: 'SAME_POSITION',
          path: [leg.from.point],
          lengthM: 0,
        };
      }
      try {
        const result = await post(
          `/api/v1/map-jobs/${encodeURIComponent(state.latestMapJob.job_id)}/navigation-plan-preview`,
          {start: leg.from.point, goal: leg.to.point, workspace},
        );
        return {...result, fromLabel: leg.from.label, toLabel: leg.to.label};
      } catch (error) {
        return {
          fromLabel: leg.from.label,
          toLabel: leg.to.label,
          reachable: false,
          reason: 'PREVIEW_FAILED',
          path: [],
          message: friendlyError(error),
        };
      }
    }));
    const reachable = segments.every(segment => segment.reachable);
    const lengthM = segments.reduce((total, segment) => total + Number(segment.lengthM || 0), 0);
    state.planPreview = {
      schema: 'gogoguard.mission_navigation_plan_preview.v1',
      reachable,
      checkpointCount: checkpoints.length,
      segments,
      lengthM,
    };
    const segmentSummary = segments.map(segment => (
      segment.reachable
        ? `${segment.fromLabel}→${segment.toLabel} ${Number(segment.lengthM || 0).toFixed(1)}m`
        : `${segment.fromLabel}→${segment.toLabel} 不通${segment.message ? `（${segment.message}）` : ''}`
    )).join('；');
    $('planPreviewResult').textContent = reachable
      ? `整趟可到达 · 经过 ${checkpoints.length} 个巡检点 · 共 ${lengthM.toFixed(1)} m。${segmentSummary}。紫色虚线是分段连通性预演，狗端每段由 Nav2 Smac 重新规划。`
      : `整趟不可达：${segmentSummary}。请检查不通的那一段绿色边界和红色障碍。`;
  } catch (error) {
    state.planPreview = null;
    $('planPreviewResult').textContent = friendlyError(error);
  } finally {
    $('previewNavigationPlan').disabled = false;
    drawWorkspace();
    renderWorkspaceControls();
  }
}

async function saveNavigationWorkspace() {
  if (!state.latestMapJob || !state.navigationWorkspace) return;
  $('workspaceError').textContent = '';
  $('saveWorkspace').disabled = true;
  try {
    const payload = {
      baseRevision: state.navigationWorkspace.revision,
      baseWorkspaceHash: state.navigationWorkspace.workspaceHash,
      route: state.navigationWorkspace.route,
      allowedArea: state.navigationWorkspace.allowedArea,
      routeSource: state.navigationWorkspace.routeSource,
      routePoseFrame: state.navigationWorkspace.routePoseFrame,
      robotRadiusM: state.navigationWorkspace.robotRadiusM || 0.48,
      navigationSurface: {...ensureNavigationSurface(), reviewed: true},
    };
    state.navigationWorkspace = await post(
      `/api/v1/map-jobs/${encodeURIComponent(state.latestMapJob.job_id)}/navigation-workspace`,
      payload,
    );
    // The newly saved workspace is a new unpublished revision, so a success
    // notice from the previous revision must not remain visible.
    state.navigationPrepare = null;
    state.savedNavigationWorkspace = structuredClone(state.navigationWorkspace);
    state.workspaceEditMode = null;
    $('workspaceHelp').textContent = '已保存；这一版静态导航地图、推荐路线和绿色范围已绑定。';
    state.navigationSurface = await api(
      `/api/v1/map-jobs/${encodeURIComponent(state.latestMapJob.job_id)}/navigation-surface`,
    );
    syncSurfaceControls();
    drawWorkspace();
    renderWorkspaceControls();
    await refreshCheckpointAudit();
    await refreshPlatformUpload();
  } catch (error) {
    $('workspaceError').textContent = friendlyError(error);
    renderWorkspaceControls();
  }
}

function renderMapHistory() {
  const completed = state.mapJobs.filter(job => job.state === 'complete');
  const tasks = state.mapJobs.filter(job => job.state !== 'complete');
  $('maps').innerHTML = completed.map(job => {
    const selected = state.latestMapJob?.job_id === job.job_id ? ' selected' : '';
    const editing = state.editingMapId === job.job_id;
    const title = job.label || job.job_id;
    const identity = job.label ? `${job.job_id} · ${fmtLocalTime(job.created_at)}` : fmtLocalTime(job.created_at);
    const editor = editing
      ? `<div class="map-name-editor"><input class="map-name-input" maxlength="80" value="${escapeHtml(job.label || '')}" placeholder="例如：一楼大厅巡检"><button class="mini-button save-map-name" type="button">保存</button><button class="mini-button cancel-map-name" type="button">取消</button></div>`
      : '<button class="mini-button rename-map" type="button">重命名</button>';
    return `<div class="session selectable${selected}" data-map-job="${escapeHtml(job.job_id)}"><div><strong>${escapeHtml(title)}</strong><small>${escapeHtml(identity)} · ${escapeHtml(job.message || job.stage)}</small>${editor}</div><b>${job === completed[0] ? '最新' : '可用'}</b></div>`;
  }).join('') || '<div class="asset-empty">暂无可用地图</div>';
  $('mapTasks').innerHTML = tasks.map(job => {
    const failed = job.state === 'failed';
    const retry = failed ? '<button class="mini-button retry-map" type="button">重试此任务</button>' : '';
    const detail = failed ? (job.error || job.message || '未知错误') : (job.message || job.stage);
    const status = failed ? '失败' : `${escapeHtml(job.state)} ${Number(job.progress || 0)}%`;
    return `<div class="session" data-map-job="${escapeHtml(job.job_id)}"><div><strong>${escapeHtml(job.job_id)}</strong><small>${escapeHtml(detail)} · ${escapeHtml(job.session_id)}</small>${retry}</div><b class="${failed ? 'failed' : 'pending'}">${status}</b></div>`;
  }).join('') || '<div class="asset-empty">当前没有处理任务</div>';
}

async function refreshMapJobs() {
  const data = await api('/api/v1/map-jobs');
  state.mapJobs = data.items || [];
  renderMapHistory();
  if (!state.latestMapJob) {
    const latest = state.mapJobs.find(job => job.state === 'complete');
    if (latest) await showResult(latest);
  }
}

async function refreshSessions() {
  const data = await api('/api/v1/sessions');
  const sessions = data.items || [];
  $('sessions').innerHTML = sessions.slice(0, 20).map(session => {
    const knownJob = state.mapJobs.find(job => job.job_id === session.map_job_id);
    const selectable = knownJob ? ' selectable' : '';
    const submit = session.state === 'sealed' && !knownJob
      ? '<button class="mini-button submit-session-map" type="button">同步到 Mac 并建图</button>'
      : '';
    return `<div class="session${selectable}" data-session-id="${escapeHtml(session.session_id)}" data-session-job="${escapeHtml(knownJob?.job_id || '')}"><div><strong>${escapeHtml(session.session_id)}</strong><small>${Number(session.sample_count || 0)} 帧 · ${escapeHtml(session.started_at || '')}</small>${submit}</div><b>${escapeHtml(session.state)}</b></div>`;
  }).join('') || '<div class="empty">暂无录制</div>';
  const active = sessions.find(session => session.state === 'recording');
  if (active) state.active = active;
  if (!state.job) {
    const pending = state.mapJobs.find(job => !['complete', 'failed'].includes(job.state));
    if (pending) state.job = pending;
  }
}

function drawNavigation() {
  const canvas = $('navigationMap');
  const [context, width, height] = sizeCanvas(canvas);
  context.fillStyle = '#07111f';
  context.fillRect(0, 0, width, height);
  const points = state.mapArtifact?.points || [];
  const route = state.navigation?.route?.length
    ? state.navigation.route
    : state.navigationWorkspace?.route || [];
  const allowedArea = state.navigationWorkspace?.allowedArea || [];
  const pose = state.navigation?.localization_pose;
  const checkpoints = (state.checkpointAudit?.checkpoints || [])
    .filter(item => Number.isFinite(Number(item.position?.x)) && Number.isFinite(Number(item.position?.y)))
    .sort((left, right) => (
      Number(left.routeProgressIndex ?? Number.MAX_SAFE_INTEGER)
      - Number(right.routeProgressIndex ?? Number.MAX_SAFE_INTEGER)
    ) || String(left.checkpointId).localeCompare(String(right.checkpointId)));
  if (!points.length && !route.length && !checkpoints.length) return;
  const all = points.map(point => [point[0], point[1]])
    .concat(route, allowedArea, checkpoints.map(item => [Number(item.position.x), Number(item.position.y)]));
  if (pose) all.push([pose.x, pose.y]);
  const minX = Math.min(...all.map(point => point[0]));
  const maxX = Math.max(...all.map(point => point[0]));
  const minY = Math.min(...all.map(point => point[1]));
  const maxY = Math.max(...all.map(point => point[1]));
  const padding = 38;
  const scale = Math.min(
    (width - padding * 2) / Math.max(maxX - minX, 0.1),
    (height - padding * 2) / Math.max(maxY - minY, 0.1),
  );
  const xy = point => [padding + (point[0] - minX) * scale, height - padding - (point[1] - minY) * scale];
  if (allowedArea.length >= 3) {
    context.beginPath();
    allowedArea.forEach((point, index) => {
      const value = xy(point);
      index ? context.lineTo(...value) : context.moveTo(...value);
    });
    context.closePath();
    context.fillStyle = '#4ade801f'; context.fill();
    context.strokeStyle = '#4ade80'; context.lineWidth = 2.5; context.stroke();
  }
  context.fillStyle = '#64748b99';
  for (let index = 0; index < points.length; index += 2) {
    const point = xy(points[index]); context.fillRect(point[0], point[1], 1.5, 1.5);
  }
  if (route.length) {
    context.strokeStyle = '#22d3ee'; context.lineWidth = 3; context.beginPath();
    route.forEach((point, index) => {
      const value = xy(point); index ? context.lineTo(value[0], value[1]) : context.moveTo(value[0], value[1]);
    });
    context.stroke();
  }
  const checkpointRuntime = state.navigation?.runtime?.checkpoint || {};
  const completedCount = Math.max(0, Number(checkpointRuntime.completedCheckpointCount || 0));
  const activeCheckpointId = String(checkpointRuntime.activeCheckpointId || '');
  const lastCompletedCheckpointId = String(checkpointRuntime.lastCompletedCheckpointId || '');
  checkpoints.forEach((checkpoint, index) => {
    const checkpointId = String(checkpoint.checkpointId || `P${index + 1}`);
    const active = checkpointId === activeCheckpointId;
    const completed = index < completedCount || checkpointId === lastCompletedCheckpointId;
    const point = xy([Number(checkpoint.position.x), Number(checkpoint.position.y)]);
    context.beginPath();
    context.arc(point[0], point[1], active ? 9 : 7, 0, Math.PI * 2);
    context.fillStyle = active ? '#fbbf24' : completed ? '#4ade80' : '#ef4444';
    context.fill();
    context.strokeStyle = '#f8fafc';
    context.lineWidth = 2;
    context.stroke();
    context.font = '600 11px system-ui, sans-serif';
    const labelWidth = context.measureText(checkpointId).width + 10;
    const labelX = point[0] + 10;
    const labelY = point[1] - 17;
    context.fillStyle = '#07111fe6';
    context.fillRect(labelX - 4, labelY - 11, labelWidth, 16);
    context.fillStyle = '#f8fafc';
    context.fillText(checkpointId, labelX, labelY + 1);
  });
  if (pose) {
    const point = xy([pose.x, pose.y]); const yaw = pose.yaw || 0;
    context.fillStyle = '#fb923c'; context.beginPath(); context.arc(point[0], point[1], 8, 0, Math.PI * 2); context.fill();
    context.strokeStyle = '#fb923c'; context.lineWidth = 4; context.beginPath(); context.moveTo(point[0], point[1]);
    context.lineTo(point[0] + Math.cos(yaw) * 24, point[1] - Math.sin(yaw) * 24); context.stroke();
  }
}

function renderNavigation() {
  const navigation = state.navigation || {};
  const candidate = navigation.candidate;
  const runtime = navigation.runtime || {};
  const localization = navigation.localization || {};
  const command = navigation.commands?.final || {};
  const runtimeRunning = Boolean(navigation.runtime_process?.running);
  const motionBridgeRunning = Boolean(navigation.motion_bridge?.running);
  const selectedJobId = state.latestMapJob?.job_id;
  const workspaceReady = Boolean(state.navigationWorkspace?.ready && !state.workspaceEditMode);
  const candidateGeneration = Number(candidate?.candidate_generation || 0);
  const assetReady = Boolean(
    candidate
    && selectedJobId
    && candidate.map_job_id === selectedJobId
    && candidateGeneration >= 10
    && state.navigationWorkspace?.workspaceHash
    && candidate.workspace_hash === state.navigationWorkspace.workspaceHash
  );
  const runtimeMapId = runtime.mapVersion || localization.mapVersion;
  const runtimeMatchesCandidate = Boolean(
    runtimeRunning && candidate && runtimeMapId && runtimeMapId === candidate.map_version
  );
  const selectedRuntimeReady = assetReady && runtimeMatchesCandidate;
  const operation = (navigation.operations || [])[0];
  const operationBusy = ['accepted', 'running'].includes(operation?.state);
  const preparePending = state.navigationPrepare?.state === 'running';
  const stopOperationBusy = operationBusy && ['runtime.stop', 'patrol.stop'].includes(operation?.kind);
  const patrolRunning = [
    'STARTING', 'PATROLLING', 'HOLDING', 'RESUMING', 'REPLANNING',
    'DETOURING', 'REJOINING', 'RETRYING', 'RECOVERING', 'SEARCHING_PATH',
    'CHECKPOINT_PAUSING', 'CHECKPOINT_SETTLING', 'INSPECTING',
    'PAUSED',
  ].includes(runtime.state);
  $('navigationCandidate').textContent = candidate ? `${candidate.map_version} · ${candidate.route_id}` : '等待地图与路线';
  $('runtimeBadge').textContent = runtimeRunning ? '运行中' : '未启动';
  $('runtimeBadge').className = runtimeRunning ? 'online' : '';
  $('localizationState').textContent = localization.state || '—';
  $('localizationDetail').textContent = localization.reason || localization.lastReason || '等待 VGICP';
  $('localizationBadge').textContent = localization.usable ? '定位可用' : (localization.state || '尚未启动');
  $('localizationBadge').className = localization.usable ? 'online' : '';
  $('routeLength').textContent = candidate ? `${candidate.route_length_m.toFixed(1)} m` : '—';
  $('routePoints').textContent = candidate ? `${candidate.route_waypoint_count} 个控制点` : '尚未生成';
  $('patrolState').textContent = runtime.state || '—';
  $('patrolReason').textContent = runtime.operatorMessage || runtime.reason || '尚未启动';
  $('finalVelocity').textContent = `${Number(command.vx || 0).toFixed(2)} m/s`;
  const controller = runtime.activeController || runtime.selectedController;
  const cruise = state.navigationProfile?.motion?.targetCruiseMps;
  $('motionAuthority').textContent = `${runtime.motionAuthorized ? '运动权已打开' : '运动权关闭'}${controller ? ` · ${controller}` : ''}${cruise == null ? '' : ` · MPPI上限 ${Number(cruise).toFixed(2)}`}`;
  $('routeProgress').textContent = `${Number(runtime.routeProgressPercent || 0).toFixed(0)}%`;
  $('remainingRoute').textContent = runtime.remainingRoutePointCount == null
    ? '尚未开始' : `剩余 ${runtime.remainingRoutePointCount} 个路径点`;
  $('operationState').textContent = operationStateText(operation);
  $('operationMessage').textContent = operationMessageText(operation);

  $('assetStepState').textContent = preparePending
    ? '发布中'
    : assetReady
    ? (runtimeRunning && !runtimeMatchesCandidate ? '已发布，待切换' : '已发布')
    : (selectedJobId ? (workspaceReady ? '待发布' : '待保存绿色范围') : '未选择');
  $('assetStep').classList.toggle('done', assetReady && !preparePending);
  $('runtimeStepState').textContent = runtimeRunning
    ? (!runtimeMatchesCandidate ? '运行旧地图' : (localization.usable ? '定位可用' : '定位中'))
    : '未启动';
  $('runtimeStep').classList.toggle('done', selectedRuntimeReady && localization.usable);
  $('patrolStepState').textContent = patrolStepStateText(runtime);
  $('patrolStep').classList.toggle('done', runtime.state === 'COMPLETED');

  let guidance = '请从历史中选择一张可用地图。';
  if (selectedJobId && !workspaceReady) guidance = '下一步：先在上方工作台画出并保存绿色允许范围。';
  else if (selectedJobId && !assetReady) guidance = '下一步：将所选地图、蓝色路线和绿色允许范围重新发布到机器狗。';
  else if (assetReady && !runtimeRunning) guidance = '地图已在机器狗上。下一步：启动定位与 Nav2。';
  else if (assetReady && runtimeRunning && !runtimeMatchesCandidate) guidance = `新地图已经发布，但 Nav2 仍运行 ${runtimeMapId || '上一张地图'}。下一步：切换到所选地图并重启 Nav2。`;
  else if (runtimeRunning && !localization.usable) guidance = '正在地图中定位，请让机器狗站稳等待；无需重复点击。';
  else if (selectedRuntimeReady && localization.usable && !patrolRunning) guidance = '定位已可用。下一步：确认现场后点击“开始巡检”。';
  if (runtime.state === 'STARTING') guidance = '开始请求正在由 Nav2 确认；这还不代表整条路线已经完成。';
  if (runtime.state === 'PATROLLING') guidance = '巡检正在进行；地图上会显示位置和路线进度。需要中断或改用遥控器时，点击“停止巡检并释放遥控权”。';
  if (runtime.state === 'REPLANNING') guidance = '原路线局部受阻，正在根据当前代价地图规划绕行并接回前方路线。';
  if (runtime.state === 'DETOURING') guidance = 'Nav2 已找到绕行路径，仍由同一个 MPPI 控制机器狗接回蓝色路线。';
  if (runtime.state === 'REJOINING') guidance = '已接回蓝色路线，MPPI 继续完成剩余巡检。';
  if (runtime.state === 'RETRYING') guidance = '原路线前方没有确认障碍，正从当前进度自动重试 MPPI，不会误入绕行。';
  if (runtime.state === 'RECOVERING') guidance = '控制或代价地图短暂中断，任务和当前进度已保留，系统会持续自动恢复。';
  if (runtime.state === 'SEARCHING_PATH') guidance = '原路线已确认受阻，系统会按固定频率不断重新寻路，只有操作员停止才会结束。';
  if (runtime.state === 'PATROLLING' && runtime.reason === 'NAV2_GLOBAL_PATH_USING_MPPI') guidance = '已找到绕行路径：同一个 MPPI 会沿 Nav2 规划结果接回蓝色路线。';
  if (runtime.state === 'HOLDING') guidance = '定位暂时不可用，路线任务仍保留；定位恢复稳定后会从未完成位置继续。';
  if (runtime.state === 'RESUMING') guidance = '定位已经恢复，正在从未完成的路线位置继续巡检。';
  if (runtime.state === 'CHECKPOINT_PAUSING') guidance = '已到平台标记点，正在取消路线速度并关闭运动权。';
  if (runtime.state === 'CHECKPOINT_SETTLING') guidance = '已到平台标记点，正在验证机器狗连续停稳，不是只记录“收到停止”。';
  if (runtime.state === 'INSPECTING') guidance = '机器狗已停稳，正在优先用摄像头恢复录制时的观察方向；只有摄像头角度不够时才让机身补转。';
  if (runtime.state === 'PAUSED' && runtime.checkpoint?.decisionMode === 'local_operator') guidance = '机器狗已在巡检点保持停车。请在下方对比参考照片和实时画面，由您决定是否继续。';
  if (['FAULT', 'BLOCKED'].includes(runtime.state)) guidance = `${runtime.operatorMessage || runtime.reason}；需要遥控机器狗时先点击“停止巡检并释放遥控权”，需要继续测试时再启动定位与 Nav2。`;
  if (operation?.state === 'failed') {
    guidance = `操作失败：${operation.message || operation.error || '未知错误'}。请不要重复点击，可查看下方诊断。`;
    $('navigationError').textContent = operation.message || operation.error || '导航操作失败';
  }
  if (operationBusy) guidance = `正在${operationKindLabels[operation.kind] || operation.kind || '执行操作'}，请等待请求结果，不要重复点击。`;
  if (preparePending) guidance = '正在把当前地图、路线和绿色允许范围发布到机器狗；请等待明确的成功或失败结果，不要重复点击。';
  $('workflowMessage').textContent = guidance;

  $('prepareNavigation').hidden = (assetReady && !preparePending) || !selectedJobId;
  $('prepareNavigationHelp').hidden = $('prepareNavigation').hidden;
  $('prepareNavigation').disabled = !selectedJobId || !workspaceReady || operationBusy || preparePending;
  $('prepareNavigation').textContent = preparePending
    ? '正在发布地图与路线…'
    : '发布所选地图与路线到机器狗';
  $('prepareNavigationStatus').hidden = !state.navigationPrepare;
  $('prepareNavigationMessage').textContent = state.navigationPrepare?.message || '';
  $('downloadPlatformBundle').hidden = !selectedJobId || !workspaceReady;
  $('downloadPlatformBundleHelp').hidden = $('downloadPlatformBundle').hidden;
  $('downloadPlatformBundle').href = selectedJobId
    ? `/api/v1/map-jobs/${encodeURIComponent(selectedJobId)}/platform-bundle/file`
    : '#';
  $('startRuntime').hidden = !assetReady || runtimeRunning;
  $('startRuntimeHelp').hidden = $('startRuntime').hidden;
  $('startRuntime').disabled = !candidate || operationBusy;
  $('resetLocalization').hidden = !runtimeRunning || localization.usable || patrolRunning;
  $('resetLocalization').disabled = operationBusy;
  $('recoverRuntime').hidden = !runtimeRunning || (runtimeMatchesCandidate && !['FAULT', 'BLOCKED'].includes(runtime.state));
  $('recoverRuntime').textContent = runtimeRunning && !runtimeMatchesCandidate
    ? '切换到所选地图并重启 Nav2'
    : '运行异常：自动清理并恢复';
  $('recoverRuntime').disabled = operationBusy;
  $('startPatrol').hidden = !selectedRuntimeReady || patrolRunning;
  $('startPatrol').disabled = !localization.usable || operationBusy || ['FAULT', 'BLOCKED'].includes(runtime.state);
  const localCheckpointCount = Number(state.checkpointAudit?.checkpoints?.length || 0);
  $('startPatrol').textContent = localCheckpointCount
    ? `开始本地验收（${localCheckpointCount} 个巡检点）`
    : '开始巡检';
  $('stopPatrol').hidden = !runtimeRunning && !motionBridgeRunning;
  $('stopPatrolHelp').hidden = !runtimeRunning && !motionBridgeRunning;
  // Stop uses its own supervisor lane and must remain available even when a
  // start/recovery operation is stuck in the control lane.
  $('stopPatrol').disabled = stopOperationBusy;
  renderPlatformUpload();
  renderLocalCheckpoint();
  drawNavigation();
}

async function navigationAction(action) {
  $('navigationError').textContent = '';
  try {
    if (action === 'prepare') {
      if (!state.latestMapJob) throw Error('请先选择一张已完成的 GLIM 地图');
      state.navigationPrepare = {
        state: 'running',
        message: '请求已发出，正在生成并校验机器狗使用的地图与路线…',
      };
      renderNavigation();
      await post('/api/v1/navigation/prepare', {job_id: state.latestMapJob.job_id});
      state.navigationPrepare = {
        state: 'complete',
        message: '发布成功：机器狗已收到当前地图与路线。',
      };
    } else if (action === 'runtime') {
      await post('/api/v1/navigation/runtime/start', {candidate_id: state.navigation.candidate.candidate_id});
    } else if (action === 'reset') {
      await post('/api/v1/navigation/localization/reset');
    } else if (action === 'start') {
      const checkpointCount = Number(state.checkpointAudit?.checkpoints?.length || 0);
      if (checkpointCount) {
        if (!state.latestMapJob) throw Error('请先选择要验收的地图');
        await post(`/api/v1/map-jobs/${encodeURIComponent(state.latestMapJob.job_id)}/local-inspection/start`);
      } else {
        await post('/api/v1/navigation/patrol/start');
      }
    } else if (action === 'stop') {
      await post('/api/v1/navigation/runtime/stop');
    } else if (action === 'recover') {
      await post('/api/v1/navigation/runtime/recover');
    }
    await refreshNavigation();
  } catch (error) {
    const message = friendlyError(error);
    if (action === 'prepare') {
      state.navigationPrepare = {state: 'failed', message: `发布失败：${message}`};
      renderNavigation();
    }
    $('navigationError').textContent = message;
  }
}

function fillProfile(profile) {
  state.navigationProfile = profile;
  $('straightSpeed').value = profile.motion.targetCruiseMps;
  $('maxForwardSpeed').value = profile.motion.maxForwardMps;
  $('acceleration').value = profile.motion.accelerationMps2;
  $('deceleration').value = profile.motion.decelerationMps2;
  $('turnSpeed').value = profile.motion.turnSpeedRadps;
  $('lateralSpeed').value = profile.motion.lateralSpeedMps;
  $('progressTimeout').value = profile.recovery.progressTimeoutS;
  $('dropoutGrace').value = profile.localization.dropoutGraceS;
  $('rejoinLookahead').value = profile.avoidance.rejoinLookaheadM;
  $('localizationTimeout').value = profile.localization.statusTimeoutS;
  $('recoveryStable').value = profile.localization.recoveryStableS;
  $('controllerFrequency').value = profile.controller.frequencyHz;
  $('mppiTimeSteps').value = profile.controller.timeSteps;
  $('mppiBatchSize').value = profile.controller.batchSize;
  $('replanInterval').value = profile.recovery.replanIntervalS;
  $('obstructionCost').value = profile.avoidance.obstructionCostThreshold;
  $('obstructionSamples').value = profile.avoidance.obstructionMinSamples;
  $('obstructionConfirmation').value = profile.avoidance.obstructionConfirmationS;
  $('profileMessage').textContent = `当前第 ${profile.revision} 版`;
}

async function refreshProfile() {
  try { fillProfile(await api('/api/v1/navigation/profile')); }
  catch (error) { $('profileMessage').textContent = friendlyError(error); }
}

async function saveProfile() {
  if (!state.navigationProfile) return;
  const profile = JSON.parse(JSON.stringify(state.navigationProfile));
  profile.motion.targetCruiseMps = Number($('straightSpeed').value);
  profile.motion.maxForwardMps = Number($('maxForwardSpeed').value);
  profile.motion.accelerationMps2 = Number($('acceleration').value);
  profile.motion.decelerationMps2 = Number($('deceleration').value);
  profile.motion.turnSpeedRadps = Number($('turnSpeed').value);
  profile.motion.lateralSpeedMps = Number($('lateralSpeed').value);
  profile.recovery.progressTimeoutS = Number($('progressTimeout').value);
  profile.localization.dropoutGraceS = Number($('dropoutGrace').value);
  profile.avoidance.rejoinLookaheadM = Number($('rejoinLookahead').value);
  profile.localization.statusTimeoutS = Number($('localizationTimeout').value);
  profile.localization.recoveryStableS = Number($('recoveryStable').value);
  profile.controller.frequencyHz = Number($('controllerFrequency').value);
  profile.controller.timeSteps = Number($('mppiTimeSteps').value);
  profile.controller.batchSize = Number($('mppiBatchSize').value);
  profile.recovery.replanIntervalS = Number($('replanInterval').value);
  profile.avoidance.obstructionCostThreshold = Number($('obstructionCost').value);
  profile.avoidance.obstructionMinSamples = Number($('obstructionSamples').value);
  profile.avoidance.obstructionConfirmationS = Number($('obstructionConfirmation').value);
  try {
    const result = await post('/api/v1/navigation/profile', {profile});
    fillProfile(result.profile);
    $('profileMessage').textContent = result.message;
  } catch (error) { $('profileMessage').textContent = friendlyError(error); }
}

async function rollbackProfile() {
  try {
    const result = await post('/api/v1/navigation/profile/rollback');
    fillProfile(result.profile);
    $('profileMessage').textContent = result.message;
  } catch (error) { $('profileMessage').textContent = friendlyError(error); }
}

async function refreshDiagnostics() {
  $('diagnosticSummary').textContent = '正在分析…';
  try {
    const value = await api('/api/v1/navigation/diagnostics');
    $('diagnosticSummary').textContent = value.summary;
    $('diagnosticChecks').innerHTML = (value.checks || []).map(check =>
      `<div class="diagnostic-check ${check.ok ? 'ok' : 'warn'}"><b>${check.ok ? '✓' : '!'} ${escapeHtml(check.name)}</b><small>${escapeHtml(check.detail)}</small></div>`
    ).join('');
    $('diagnosticLogs').textContent = (value.logs || []).flatMap(log =>
      [`[${log.name}]`, ...(log.highlights || [])]
    ).join('\n') || '暂无错误摘要';
  } catch (error) { $('diagnosticSummary').textContent = friendlyError(error); }
}

const diagnosticModeLabels = {
  development: '研发完整采集',
  acceptance: '验收低负载采集',
  production: '生产轻量记录',
};

function fillDiagnosticProfile(profile) {
  state.diagnosticProfile = profile;
  $('diagnosticMode').value = profile.mode;
  if (profile.mode === 'production') {
    $('diagnosticLifetime').value = 'manual';
  } else if (profile.remaining_patrols != null) {
    $('diagnosticLifetime').value = profile.remaining_patrols <= 1 ? 'next1' : 'next3';
  } else if (profile.expires_at) {
    $('diagnosticLifetime').value = '60m';
  } else {
    $('diagnosticLifetime').value = 'manual';
  }
  $('diagnosticLifetime').disabled = profile.mode === 'production';
  $('diagnosticModeBadge').textContent = diagnosticModeLabels[profile.mode] || profile.mode;
  $('diagnosticModeBadge').className = profile.mode === 'production' ? '' : 'online';
  const expiry = profile.expires_at ? ` · ${profile.expires_at} 到期` : '';
  const patrols = profile.remaining_patrols == null ? '' : ` · 剩余 ${profile.remaining_patrols} 次`;
  $('diagnosticProfileMessage').textContent = `第 ${profile.revision} 版${patrols}${expiry}`;
  const descriptions = {
    development: '故障前 15 秒+后 5 秒；10 Hz 原始点云、H.264 视频、代价地图和规划过程。',
    acceptance: '故障前 10 秒+后 5 秒；2 Hz 点云，不保存摄像头，用于验收时低负载取证。',
    production: '故障前 15 秒+后 5 秒；自动保存里程计、定位、命令链和决策，不保存原始点云和视频。',
  };
  $('diagnosticProfileHelp').textContent = descriptions[profile.mode];
}

async function refreshDiagnosticProfile() {
  try { fillDiagnosticProfile(await api('/api/v1/diagnostics/profile')); }
  catch (error) { $('diagnosticProfileMessage').textContent = friendlyError(error); }
}

async function saveDiagnosticProfile() {
  const mode = $('diagnosticMode').value;
  const lifetime = $('diagnosticLifetime').value;
  const presets = {
    development: {pre_trigger_s: 15, post_trigger_s: 5, point_cloud_hz: 10, record_camera: true, record_costmap: true, record_planner_detail: true},
    acceptance: {pre_trigger_s: 10, post_trigger_s: 5, point_cloud_hz: 2, record_camera: false, record_costmap: true, record_planner_detail: true},
    production: {pre_trigger_s: 15, post_trigger_s: 5, point_cloud_hz: 0, record_camera: false, record_costmap: false, record_planner_detail: false},
  };
  const profile = {...presets[mode], mode, max_incidents: 20, max_storage_bytes: 2 * 1024 ** 3};
  if (mode !== 'production' && lifetime === 'next1') profile.remaining_patrols = 1;
  if (mode !== 'production' && lifetime === 'next3') profile.remaining_patrols = 3;
  if (mode !== 'production' && lifetime === '60m') profile.expires_at = new Date(Date.now() + 60 * 60 * 1000).toISOString();
  try {
    const result = await post('/api/v1/diagnostics/profile', {profile});
    fillDiagnosticProfile(result.profile);
    $('diagnosticProfileMessage').textContent = result.message;
  } catch (error) { $('diagnosticProfileMessage').textContent = friendlyError(error); }
}

function renderIncidents() {
  $('incidents').innerHTML = state.incidents.map(incident => {
    const selected = incident.incident_id === state.incident?.incident_id ? ' selected' : '';
    const missing = (incident.evidence_missing || []).length;
    const stateLabel = incident.state === 'sealed' ? '证据完整' : '证据不完整';
    return `<button class="incident-row${selected}" data-incident-id="${escapeHtml(incident.incident_id)}"><span><strong>${escapeHtml(incident.trigger || '未知事件')}</strong><small>${escapeHtml(incident.triggered_at || '')} · ${escapeHtml(incident.map_version || '未绑定地图')}</small></span><b class="${missing ? 'warn' : 'ok'}">${stateLabel}</b></button>`;
  }).join('') || '<div class="asset-empty">暂无事故记录</div>';
}

async function refreshIncidents() {
  try {
    const value = await api('/api/v1/incidents');
    state.incidents = value.items || [];
    renderIncidents();
  } catch (_) {
    // Robot offline does not invalidate incidents already synchronized to Mac.
  }
}

function incidentEventAt(kind, epoch, predicate = () => true) {
  const events = state.incidentReplay?.events || [];
  let selected = null;
  for (const event of events) {
    // Slider values are decimal strings; tolerate sub-millisecond floating
    // point rounding at the exact final event.
    if (Number(event.timestamp) > epoch + 0.001) break;
    if (event.kind === kind && predicate(event)) selected = event;
  }
  return selected;
}

function drawIncidentPlan(epoch) {
  const canvas = $('incidentPlan');
  const [context, width, height] = sizeCanvas(canvas);
  context.fillStyle = '#07111f'; context.fillRect(0, 0, width, height);
  const costmap = incidentEventAt('costmap', epoch);
  const routeEvent = incidentEventAt('route', epoch);
  const poseEvent = incidentEventAt('pose', epoch);
  const planner = incidentEventAt('decision', epoch, event => event.topic === '/go2/runtime/planner_diagnostics');
  const route = routeEvent?.points || state.incidentReplay?.route || [];
  if (costmap) {
    const gridWidth = costmap.width || 1, gridHeight = costmap.height || 1;
    const scale = Math.min(width / gridWidth, height / gridHeight);
    const ox = (width - gridWidth * scale) / 2, oy = (height - gridHeight * scale) / 2;
    for (let y = 0; y < gridHeight; y += 1) for (let x = 0; x < gridWidth; x += 1) {
      const value = Number(costmap.costs[y * gridWidth + x]);
      if (value < 0) context.fillStyle = '#16263a';
      else if (value >= 65) context.fillStyle = '#ef4444cc';
      else if (value > 0) context.fillStyle = `rgba(251,146,60,${Math.max(.12, value / 100)})`;
      else context.fillStyle = '#0b1d2e';
      context.fillRect(ox + x * scale, oy + (gridHeight - 1 - y) * scale, Math.ceil(scale), Math.ceil(scale));
    }
    const world = point => [ox + (point[0] - costmap.origin[0]) / costmap.resolution * scale,
      oy + (gridHeight - (point[1] - costmap.origin[1]) / costmap.resolution) * scale];
    if (route.length) {
      context.strokeStyle = '#22d3ee'; context.lineWidth = 2; context.beginPath();
      route.forEach((point, index) => { const p = world(point); index ? context.lineTo(...p) : context.moveTo(...p); }); context.stroke();
    }
    for (const cell of planner?.payload?.expandedCells || []) {
      context.fillStyle = '#a78bfa55'; context.fillRect(ox + cell[0] * scale, oy + (gridHeight - 1 - cell[1]) * scale, Math.max(1, scale), Math.max(1, scale));
    }
    const path = planner?.payload?.pathPoints || [];
    if (path.length) {
      context.strokeStyle = '#4ade80'; context.lineWidth = 3; context.beginPath();
      path.forEach((point, index) => { const p = world(point); index ? context.lineTo(...p) : context.moveTo(...p); }); context.stroke();
    }
    if (poseEvent?.pose) {
      const p = world([poseEvent.pose.x, poseEvent.pose.y]);
      context.fillStyle = '#fb923c'; context.beginPath(); context.arc(p[0], p[1], 7, 0, Math.PI * 2); context.fill();
      context.strokeStyle = '#fb923c'; context.lineWidth = 3; context.beginPath(); context.moveTo(...p);
      context.lineTo(p[0] + Math.cos(poseEvent.pose.yaw || 0) * 22, p[1] - Math.sin(poseEvent.pose.yaw || 0) * 22); context.stroke();
    }
  } else {
    context.fillStyle = '#8fa4bb'; context.textAlign = 'center'; context.fillText('这次事故没有保存局部代价地图', width / 2, height / 2);
  }
  const runtime = incidentEventAt('runtime', epoch);
  $('incidentDecision').textContent = JSON.stringify({runtime: runtime?.payload || null, planner: planner?.payload || null, pose: poseEvent?.pose || null}, null, 2);
}

function renderIncidentFrame() {
  if (!state.incidentReplay) return;
  const events = state.incidentReplay.events || [];
  const start = events.length ? Number(events[0].timestamp) : Number(state.incidentReplay.triggered_at_epoch || 0);
  const offset = Number($('incidentTimeline').value || 0);
  const epoch = start + offset;
  $('incidentTimeLabel').textContent = `${offset.toFixed(1)} s`;
  const cloud = incidentEventAt('cloud', epoch);
  const route = incidentEventAt('route', epoch)?.points || state.incidentReplay.route || [];
  drawCloud($('incidentCloud'), cloud?.points || [], route.map(point => [point[0], point[1], 0]), state.views.incident);
  drawIncidentPlan(epoch);
}

async function openIncident(incidentId) {
  state.incident = await api(`/api/v1/incidents/${encodeURIComponent(incidentId)}`);
  renderIncidents();
  $('incidentReplayState').textContent = state.incident.state === 'sealed' ? '已封存' : '证据不完整';
  $('incidentReplayState').className = state.incident.state === 'sealed' ? 'online' : '';
  $('incidentReplayMeta').textContent = `${state.incident.incident_id} · ${state.incident.map_version || '未绑定地图'} · ${state.incident.route_id || '未绑定路线'}`;
  const present = state.incident.evidence_present || [], missing = state.incident.evidence_missing || [];
  $('incidentTruth').innerHTML = `<b>已有证据：</b>${escapeHtml(present.join('、') || '无')}<br><b>缺失证据：</b>${escapeHtml(missing.join('、') || '无')}`;
  const replayFile = (state.incident.files || []).find(item => item.path === 'replay.json');
  if (!replayFile) {
    state.incidentReplay = null; $('incidentDecision').textContent = '该事故没有可视化回放数据'; return;
  }
  state.incidentReplay = await api(`/api/v1/incidents/${encodeURIComponent(incidentId)}/files/replay.json`);
  const events = state.incidentReplay.events || [];
  const duration = events.length > 1 ? Number(events[events.length - 1].timestamp) - Number(events[0].timestamp) : 0;
  $('incidentTimeline').max = Math.max(0, duration).toFixed(1);
  $('incidentTimeline').value = Math.min(Math.max(0, Number(state.incidentReplay.triggered_at_epoch || 0) - Number(events[0]?.timestamp || 0)), duration).toFixed(1);
  const video = (state.incident.files || []).find(item => item.path.startsWith('camera/') && item.path.endsWith('.mp4'));
  $('incidentVideoMissing').hidden = Boolean(video);
  $('incidentVideo').hidden = !video;
  if (video) $('incidentVideo').src = `/api/v1/incidents/${encodeURIComponent(incidentId)}/files/${video.path.split('/').map(encodeURIComponent).join('/')}`;
  renderIncidentFrame();
}

async function refreshNavigation() {
  try {
    state.navigation = await api('/api/v1/navigation');
    const candidateJobId = state.navigation?.candidate?.map_job_id;
    if (candidateJobId && !state.mapSelectionExplicit && state.latestMapJob?.job_id !== candidateJobId) {
      const candidateJob = state.mapJobs.find(job => job.job_id === candidateJobId);
      if (candidateJob?.state === 'complete') await showResult(candidateJob);
    }
    // A polling failure is transient unless the current durable operation or
    // runtime state also reports failure. Clear stale timeout text before the
    // successful snapshot is rendered; renderNavigation restores real faults.
    $('navigationError').textContent = '';
    renderNavigation();
  } catch (error) {
    $('navigationError').textContent = friendlyError(error);
  }
}

async function tick() {
  try {
    state.live = await api('/api/v1/live');
    state.robotReachable = true;
    renderLive();
    renderRecording();
    if (state.active) {
      state.active = await api(`/api/v1/sessions/${state.active.session_id}`);
      renderRecording();
    }
    await refreshJob();
  } catch (error) {
    state.robotReachable = false;
    $('modeBadge').textContent = 'Mac 工作站 · 机器狗离线';
    $('modeBadge').className = '';
    $('robotState').textContent = '离线';
    $('robotId').textContent = friendlyError(error);
    setFault(error);
    renderRecording();
  }
}

function attachCloudControls(canvas, view, redraw) {
  let drag = null;
  canvas.addEventListener('pointerdown', event => {
    drag = [event.clientX, event.clientY];
    canvas.setPointerCapture?.(event.pointerId);
  });
  canvas.addEventListener('pointermove', event => {
    if (!drag) return;
    view.yaw += (event.clientX - drag[0]) * 0.008;
    view.pitch = Math.max(0.05, Math.min(1.5, view.pitch + (event.clientY - drag[1]) * 0.006));
    drag = [event.clientX, event.clientY];
    redraw();
  });
  canvas.addEventListener('pointerup', () => { drag = null; });
  canvas.addEventListener('pointercancel', () => { drag = null; });
  canvas.addEventListener('wheel', event => {
    event.preventDefault();
    view.zoom = Math.max(3, Math.min(120, view.zoom * (event.deltaY > 0 ? 0.9 : 1.1)));
    redraw();
  }, {passive: false});
}

function attachWorkspaceCloudControls() {
  const canvas = $('workspaceCloud');
  let drag = null;
  canvas.addEventListener('pointerdown', event => {
    drag = [event.clientX, event.clientY];
    canvas.setPointerCapture?.(event.pointerId);
  });
  canvas.addEventListener('pointermove', event => {
    if (!drag) return;
    state.workspaceCloudView.yaw += (event.clientX - drag[0]) * 0.008;
    state.workspaceCloudView.pitch = Math.max(0.05, Math.min(1.55,
      state.workspaceCloudView.pitch + (event.clientY - drag[1]) * 0.006));
    drag = [event.clientX, event.clientY];
    drawWorkspaceCloud();
  });
  for (const eventName of ['pointerup', 'pointercancel', 'pointerleave']) {
    canvas.addEventListener(eventName, () => { drag = null; });
  }
  canvas.addEventListener('wheel', event => {
    event.preventDefault();
    state.workspaceCloudView.zoomFactor = Math.max(0.5, Math.min(6,
      Number(state.workspaceCloudView.zoomFactor || 1) * (event.deltaY > 0 ? 0.88 : 1.14)));
    drawWorkspaceCloud();
  }, {passive: false});
}

attachCloudControls(
  $('cloud'), state.views.live,
  () => drawCloud($('cloud'), state.live?.points, null, state.views.live),
);
attachCloudControls($('resultCloud'), state.views.result, redrawResult);
$('showMapPoints').addEventListener('change', redrawResult);
$('showMapRoute').addEventListener('change', redrawResult);
$('resetMapView').addEventListener('click', () => {
  Object.assign(state.views.result, {yaw: 0.7, pitch: 0.55, zoom: 18});
  redrawResult();
});
for (const id of ['showWorkspaceGround', 'showWorkspacePoints', 'showWorkspaceSurface', 'showWorkspaceRoute', 'showWorkspaceArea']) {
  $(id).addEventListener('change', drawWorkspace);
}
$('workspaceView2d').addEventListener('click', () => setWorkspaceDisplayMode('2d'));
$('workspaceView3d').addEventListener('click', () => setWorkspaceDisplayMode('3d'));
$('workspaceViewSplit').addEventListener('click', () => setWorkspaceDisplayMode('split'));
$('workspaceZoomIn').addEventListener('click', () => zoomWorkspace(1.35));
$('workspaceZoomOut').addEventListener('click', () => zoomWorkspace(1 / 1.35));
$('workspaceFit').addEventListener('click', () => {
  const bounds = workspaceBounds();
  if (!bounds) return;
  const zoom = ['blocked', 'clear'].includes(state.workspaceEditMode) ? 2.2 : 1;
  state.workspaceView = {...state.workspaceView, zoom, centerX: (bounds.minX + bounds.maxX) / 2, centerY: (bounds.minY + bounds.maxY) / 2, drag: null};
  drawWorkspace();
});
$('workspaceFocusRoute').addEventListener('click', () => {
  const route = state.navigationWorkspace?.route || [];
  if (!route.length) return;
  const margin = 1.5;
  setWorkspaceView({
    minX: Math.min(...route.map(point => point[0])) - margin,
    maxX: Math.max(...route.map(point => point[0])) + margin,
    minY: Math.min(...route.map(point => point[1])) - margin,
    maxY: Math.max(...route.map(point => point[1])) + margin,
  });
});
$('workspacePan').addEventListener('click', () => {
  state.workspaceView.panMode = !state.workspaceView.panMode;
  $('workspacePan').classList.toggle('active', state.workspaceView.panMode);
  $('workspaceMap').classList.toggle('pan-mode', state.workspaceView.panMode);
  $('workspaceHelp').textContent = state.workspaceView.panMode
    ? '拖动地图查看局部；再次点击“拖动地图”退出。滚轮始终可以缩放。'
    : '已退出拖动模式；现在可以继续标记。';
});
$('workspaceFullscreen').addEventListener('click', async () => {
  const panel = $('mapWorkbenchSection');
  try {
    if (document.fullscreenElement === panel) await document.exitFullscreen();
    else await panel.requestFullscreen();
  } catch (error) {
    $('workspaceError').textContent = `无法进入全屏：${friendlyError(error)}`;
  }
});
document.addEventListener('fullscreenchange', () => {
  $('workspaceFullscreen').textContent = document.fullscreenElement === $('mapWorkbenchSection') ? '退出全屏' : '全屏编辑（含工具）';
  requestAnimationFrame(drawWorkspace);
});
$('workspace3dTop').addEventListener('click', () => {
  Object.assign(state.workspaceCloudView, {yaw: 0, pitch: Math.PI / 2});
  drawWorkspaceCloud();
});
$('workspace3dAngle').addEventListener('click', () => {
  Object.assign(state.workspaceCloudView, {yaw: 0.72, pitch: 0.62});
  drawWorkspaceCloud();
});
$('workspace3dSlice').addEventListener('click', () => {
  state.workspaceCloudView.sliceOnly = !state.workspaceCloudView.sliceOnly;
  $('workspace3dSlice').classList.toggle('active', state.workspaceCloudView.sliceOnly);
  $('workspace3dSlice').textContent = state.workspaceCloudView.sliceOnly ? '显示全部高度' : '只看高度层';
  drawWorkspaceCloud();
});
attachWorkspaceCloudControls();
$('editRoute').addEventListener('click', () => {
  if (!state.navigationWorkspace) return;
  state.savedNavigationWorkspace = structuredClone(state.navigationWorkspace);
  state.navigationWorkspace.route = [];
  state.navigationWorkspace.routeSource = 'edited';
  state.planPreview = null;
  state.workspaceEditMode = 'route';
  setWorkspaceDisplayMode('2d');
  $('workspaceError').textContent = '';
  drawWorkspace(); renderWorkspaceControls();
});
$('restoreRecordedRoute').addEventListener('click', () => {
  if (!state.navigationWorkspace) return;
  const route = recordedWorkspaceRoute();
  if (route.length < 2) {
    $('workspaceError').textContent = '当前地图没有可用的录制路线';
    return;
  }
  state.navigationWorkspace.route = route;
  state.navigationWorkspace.routeSource = 'recorded';
  state.planStart = route[0];
  state.planGoal = route.at(-1);
  state.planPreview = null;
  $('workspaceError').textContent = '';
  drawWorkspace(); renderWorkspaceControls();
});
$('editAllowedArea').addEventListener('click', () => {
  if (!state.navigationWorkspace) return;
  state.savedNavigationWorkspace = structuredClone(state.navigationWorkspace);
  state.navigationWorkspace.allowedArea = [];
  state.planPreview = null;
  state.workspaceEditMode = 'area';
  setWorkspaceDisplayMode('2d');
  $('workspaceError').textContent = '';
  drawWorkspace(); renderWorkspaceControls();
});
$('undoWorkspacePoint').addEventListener('click', () => {
  if (!state.workspaceEditMode || !state.navigationWorkspace) return;
  if (['blocked', 'clear'].includes(state.workspaceEditMode)) {
    const previous = state.surfaceUndo.pop();
    const surface = ensureNavigationSurface();
    if (previous && surface) {
      surface.manualBlockedCells = previous.manualBlockedCells;
      surface.manualClearCells = previous.manualClearCells;
      surface.reviewed = false;
      syncSurfaceControls();
      drawWorkspace();
      renderWorkspaceControls();
    }
    return;
  }
  if (['plan-start', 'plan-goal'].includes(state.workspaceEditMode)) return;
  const key = state.workspaceEditMode === 'route' ? 'route' : 'allowedArea';
  state.navigationWorkspace[key].pop();
  drawWorkspace();
  renderWorkspaceControls();
});
$('cancelWorkspaceEdit').addEventListener('click', () => {
  if (!state.workspaceEditMode) return;
  if (!['plan-start', 'plan-goal'].includes(state.workspaceEditMode) && state.savedNavigationWorkspace) {
    state.navigationWorkspace = structuredClone(state.savedNavigationWorkspace);
  }
  state.surfaceBrushDown = false;
  state.surfaceUndo = [];
  state.workspaceEditMode = null;
  $('workspaceError').textContent = '';
  syncSurfaceControls();
  drawWorkspace(); renderWorkspaceControls();
  $('workspaceHelp').textContent = '已放弃本次绘制，地图恢复到进入该工具之前的状态。';
});
$('finishWorkspaceEdit').addEventListener('click', finishWorkspaceEditing);
$('saveWorkspace').addEventListener('click', saveNavigationWorkspace);
$('indoorSurfacePreset').addEventListener('click', () => applySurfacePreset(0.05, 1.80));
$('outdoorSurfacePreset').addEventListener('click', () => applySurfacePreset(0.05, 2.50));
$('paintBlocked').addEventListener('click', () => {
  startSurfaceEdit('blocked');
});
$('paintClear').addEventListener('click', () => {
  startSurfaceEdit('clear');
});
$('clearSurfaceEdits').addEventListener('click', () => {
  const surface = ensureNavigationSurface();
  if (!surface) return;
  if (!['blocked', 'clear'].includes(state.workspaceEditMode)) startSurfaceEdit('clear');
  snapshotSurfaceUndo();
  surface.manualBlockedCells = [];
  surface.manualClearCells = [];
  surface.reviewed = false;
  state.planPreview = null;
  syncSurfaceControls();
  drawWorkspace();
  renderWorkspaceControls();
  $('workspaceHelp').textContent = '已暂时清空所有手工补画和擦除标记；可点“撤销上一笔”恢复，未保存前不会改动已发布地图。';
});
$('surfaceBrushSize').addEventListener('input', () => {
  $('surfaceBrushSizeValue').textContent = `${Number($('surfaceBrushSize').value).toFixed(2)} m`;
  updateWorkspaceReadout();
  drawWorkspace();
});
for (const id of ['obstacleMinZ', 'obstacleMaxZ']) $(id).addEventListener('change', () => {
  const surface = ensureNavigationSurface();
  if (!surface) return;
  const minimum = Number($('obstacleMinZ').value);
  const maximum = Number($('obstacleMaxZ').value);
  if (!Number.isFinite(minimum) || !Number.isFinite(maximum) || minimum >= maximum) {
    $('workspaceError').textContent = '障碍高度填写不对：起始高度必须小于结束高度。';
    syncSurfaceControls();
    return;
  }
  surface.obstacleMinZ = minimum;
  surface.obstacleMaxZ = maximum;
  surface.reviewed = false;
  state.planPreview = null;
  $('workspaceError').textContent = '';
  syncSurfaceControls();
  drawWorkspace();
});
$('selectPlanStart').addEventListener('click', () => {
  state.workspaceEditMode = 'plan-start';
  renderWorkspaceControls();
});
$('selectPlanGoal').addEventListener('click', () => {
  state.workspaceEditMode = 'plan-goal';
  renderWorkspaceControls();
});
$('previewNavigationPlan').addEventListener('click', previewNavigationPlan);
$('startGlimEditor').addEventListener('click', startGlimEditor);
$('openGlimEditor').addEventListener('click', openGlimEditor);
$('publishGlimEditor').addEventListener('click', publishGlimEditor);
$('stopGlimEditor').addEventListener('click', stopGlimEditor);
$('workspaceMap').addEventListener('click', event => {
  if (state.workspaceView.panMode || !state.workspaceEditMode || !state.navigationWorkspace) return;
  const point = workspaceCanvasPoint(event);
  if (!point) return;
  if (state.workspaceEditMode === 'plan-start') {
    state.planStart = point.map(value => Number(value.toFixed(3)));
    state.planPreview = null;
    state.workspaceEditMode = null;
    drawWorkspace(); renderWorkspaceControls();
    return;
  }
  if (state.workspaceEditMode === 'plan-goal') {
    state.planGoal = point.map(value => Number(value.toFixed(3)));
    state.planPreview = null;
    state.workspaceEditMode = null;
    drawWorkspace(); renderWorkspaceControls();
    return;
  }
  if (['blocked', 'clear'].includes(state.workspaceEditMode)) return;
  const key = state.workspaceEditMode === 'route' ? 'route' : 'allowedArea';
  const values = state.navigationWorkspace[key];
  const last = values[values.length - 1];
  if (last && Math.hypot(point[0] - last[0], point[1] - last[1]) < 0.02) return;
  values.push(point.map(value => Number(value.toFixed(3))));
  state.planPreview = null;
  drawWorkspace();
});
$('workspaceMap').addEventListener('wheel', event => {
  event.preventDefault();
  const point = workspaceCanvasPoint(event);
  if (point) state.workspaceCursorWorld = point;
  zoomWorkspace(event.deltaY > 0 ? 0.86 : 1.16, point);
}, {passive: false});
$('workspaceMap').addEventListener('pointermove', event => {
  const point = workspaceCanvasPoint(event);
  if (point) state.workspaceCursorWorld = point;
  if (state.workspaceView.panMode && state.workspaceView.drag && state.workspaceTransform) {
    const dx = event.clientX - state.workspaceView.drag[0];
    const dy = event.clientY - state.workspaceView.drag[1];
    state.workspaceView.centerX -= dx / state.workspaceTransform.scale;
    state.workspaceView.centerY += dy / state.workspaceTransform.scale;
    state.workspaceView.drag = [event.clientX, event.clientY];
  }
  drawWorkspace();
});
$('workspaceMap').addEventListener('pointerleave', () => {
  if (!state.workspaceView.drag) state.workspaceCursorWorld = null;
  drawWorkspace();
});
for (const eventName of ['pointerdown', 'pointermove', 'pointerup', 'pointercancel', 'pointerleave']) {
  $('workspaceMap').addEventListener(eventName, event => {
    if (state.workspaceView.panMode) {
      if (eventName === 'pointerdown') {
        state.workspaceView.drag = [event.clientX, event.clientY];
        $('workspaceMap').setPointerCapture?.(event.pointerId);
      } else if (eventName === 'pointerup' || eventName === 'pointercancel' || eventName === 'pointerleave') {
        state.workspaceView.drag = null;
      }
      return;
    }
    if (!['blocked', 'clear'].includes(state.workspaceEditMode)) return;
    if (eventName === 'pointerdown') {
      if (!requireWorkspaceEditZoom()) return;
      state.surfaceBrushDown = true;
      snapshotSurfaceUndo();
      renderWorkspaceControls();
      $('workspaceMap').setPointerCapture?.(event.pointerId);
    } else if (eventName === 'pointerup' || eventName === 'pointercancel' || eventName === 'pointerleave') {
      state.surfaceBrushDown = false;
      return;
    }
    if (!state.surfaceBrushDown) return;
    event.preventDefault();
    const point = workspaceCanvasPoint(event);
    if (point) paintSurfaceAt(point);
  });
}
async function handleMapClick(event) {
  const row = event.target.closest('[data-map-job]');
  if (!row) return;
  const job = state.mapJobs.find(item => item.job_id === row.dataset.mapJob);
  const renameButton = event.target.closest('.rename-map');
  const cancelButton = event.target.closest('.cancel-map-name');
  const saveButton = event.target.closest('.save-map-name');
  if (renameButton) {
    state.editingMapId = job?.job_id || null;
    renderMapHistory();
    const input = $('maps').querySelector(`[data-map-job="${job.job_id}"] .map-name-input`);
    input?.focus();
    return;
  }
  if (cancelButton) {
    state.editingMapId = null;
    renderMapHistory();
    return;
  }
  if (saveButton && job) {
    const input = row.querySelector('.map-name-input');
    saveButton.disabled = true;
    try {
      const updated = await post(`/api/v1/map-jobs/${encodeURIComponent(job.job_id)}/label`, {label: input?.value || ''});
      state.mapJobs = state.mapJobs.map(item => item.job_id === updated.job_id ? updated : item);
      if (state.latestMapJob?.job_id === updated.job_id) state.latestMapJob = updated;
      state.editingMapId = null;
      renderMapHistory();
      if (state.latestMapJob?.job_id === updated.job_id) {
        const mapName = updated.label ? `${updated.label} · ${updated.job_id}` : updated.job_id;
        $('resultMeta').textContent = `${mapName} · ${updated.metrics?.point_count || state.mapArtifact?.points?.length || 0} 点 · ${updated.metrics?.worker || 'cloud-glim'}`;
      }
    } catch (error) {
      setFault(error);
      saveButton.disabled = false;
    }
    return;
  }
  const retryButton = event.target.closest('.retry-map');
  if (retryButton && job?.state === 'failed') {
    retryButton.disabled = true;
    try {
      await api(`/api/v1/map-jobs/${encodeURIComponent(job.job_id)}/retry`, {method: 'POST'});
      await refreshMapJobs();
    } catch (error) {
      setFault(error);
      retryButton.disabled = false;
    }
    return;
  }
  if (job?.state === 'complete') await showResult(job, {explicit: true});
}
$('maps').addEventListener('click', handleMapClick);
$('mapTasks').addEventListener('click', handleMapClick);
$('sessions').addEventListener('click', async event => {
  const row = event.target.closest('[data-session-id]');
  if (!row) return;
  const submitButton = event.target.closest('.submit-session-map');
  if (submitButton) {
    submitButton.disabled = true;
    try {
      const outcome = await api(`/api/v1/sessions/${encodeURIComponent(row.dataset.sessionId)}/map`, {method: 'POST'});
      state.job = outcome.map_job;
      await refreshMapJobs();
      await refreshSessions();
      renderRecording();
    } catch (error) {
      setFault(error);
      submitButton.disabled = false;
    }
    return;
  }
  if (!row.dataset.sessionJob) return;
  const job = state.mapJobs.find(item => item.job_id === row.dataset.sessionJob);
  if (job?.state === 'complete') await showResult(job, {explicit: true});
});
$('recordButton').addEventListener('click', toggleRecord);
$('markCheckpoint').addEventListener('click', markCheckpoint);
$('recordingCheckpoints').addEventListener('click', async event => {
  const button = event.target.closest('.delete-checkpoint');
  if (!button || !state.active) return;
  try {
    await api(`/api/v1/sessions/${encodeURIComponent(state.active.session_id)}/checkpoints/${encodeURIComponent(button.dataset.checkpointId)}`, {method: 'DELETE'});
    await refreshRecordingCheckpoints();
  } catch (error) { $('error').textContent = friendlyError(error); }
});
bindGimbalHold('gimbalLeft', -1, 0);
bindGimbalHold('gimbalRight', 1, 0);
bindGimbalHold('gimbalUp', 0, 1);
bindGimbalHold('gimbalDown', 0, -1);
$('gimbalCenter').addEventListener('click', () => moveGimbal(0, 0, true));
$('gimbalTargetForm').addEventListener('submit', event => {
  state.gimbalTargetEditing = false;
  submitGimbalTarget(event);
});
$('gimbalTargetForm').addEventListener('focusin', () => {
  state.gimbalTargetEditing = true;
});
$('gimbalTargetForm').addEventListener('focusout', () => {
  setTimeout(() => {
    if (!$('gimbalTargetForm').contains(document.activeElement)) {
      state.gimbalTargetEditing = false;
    }
  }, 0);
});
$('uploadPlatformBundle').addEventListener('click', uploadPlatformBundle);
$('prepareNavigation').addEventListener('click', () => navigationAction('prepare'));
$('startRuntime').addEventListener('click', () => navigationAction('runtime'));
$('resetLocalization').addEventListener('click', () => navigationAction('reset'));
$('recoverRuntime').addEventListener('click', () => navigationAction('recover'));
$('startPatrol').addEventListener('click', () => navigationAction('start'));
$('stopPatrol').addEventListener('click', () => navigationAction('stop'));
$('checkpointContinue').addEventListener('click', () => controlLocalCheckpoint('continue'));
$('checkpointRetake').addEventListener('click', () => controlLocalCheckpoint('retake'));
$('checkpointSkip').addEventListener('click', () => controlLocalCheckpoint('skip'));
$('saveProfile').addEventListener('click', saveProfile);
$('rollbackProfile').addEventListener('click', rollbackProfile);
$('refreshDiagnostics').addEventListener('click', refreshDiagnostics);
$('diagnosticMode').addEventListener('change', () => {
  const mode = $('diagnosticMode').value;
  const descriptions = {
    development: '故障前 15 秒+后 5 秒；10 Hz 原始点云、H.264 视频、代价地图和规划过程。',
    acceptance: '故障前 10 秒+后 5 秒；2 Hz 点云，不保存摄像头。',
    production: '故障前 15 秒+后 5 秒；自动保存里程计、定位、命令链和决策，不保存原始点云和视频。',
  };
  $('diagnosticProfileHelp').textContent = descriptions[mode];
  $('diagnosticLifetime').disabled = mode === 'production';
});
$('saveDiagnosticMode').addEventListener('click', saveDiagnosticProfile);
$('captureIncident').addEventListener('click', async () => {
  try {
    await post('/api/v1/incidents/capture');
    $('diagnosticProfileMessage').textContent = '已请求保存当前现场；将在后置时间窗结束后封存';
  } catch (error) { $('diagnosticProfileMessage').textContent = friendlyError(error); }
});
$('incidents').addEventListener('click', event => {
  const row = event.target.closest('[data-incident-id]');
  if (row) openIncident(row.dataset.incidentId).catch(error => {
    $('incidentDecision').textContent = friendlyError(error);
  });
});
$('incidentTimeline').addEventListener('input', renderIncidentFrame);
attachCloudControls($('incidentCloud'), state.views.incident, renderIncidentFrame);
addEventListener('resize', () => {
  renderLive();
  redrawResult();
  drawWorkspace();
  drawNavigation();
  renderIncidentFrame();
});

async function initialize() {
  try {
    await refreshMapJobs();
    await refreshSessions();
  } catch (error) {
    setFault(error);
  }
  renderRecording();
  refreshCamera();
  refreshGimbal();
  refreshNavigation();
  refreshProfile();
  refreshDiagnosticProfile();
  refreshIncidents();
  scheduledTick();
  setInterval(refreshCamera, 2000);
  setInterval(refreshGimbal, 3000);
  setInterval(refreshPlatformUpload, 2000);
  setInterval(refreshMapJobs, 3000);
  setInterval(refreshIncidents, 5000);
}

let tickBusy = false;
async function scheduledTick() {
  if (tickBusy) return;
  tickBusy = true;
  try {
    await tick();
    await refreshNavigation();
  } finally {
    tickBusy = false;
    setTimeout(scheduledTick, 1000);
  }
}

initialize();

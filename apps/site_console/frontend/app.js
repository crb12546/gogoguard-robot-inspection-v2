const state = {
  active: null,
  job: null,
  mapJobs: [],
  latestMapJob: null,
  mapArtifact: null,
  navigation: null,
  live: null,
  camera: null,
  cameraStarted: false,
  robotReachable: false,
  navigationProfile: null,
  views: {
    live: {yaw: 0.7, pitch: 0.55, zoom: 28},
    result: {yaw: 0.7, pitch: 0.55, zoom: 18},
  },
};

const $ = id => document.getElementById(id);
const escapeHtml = value => String(value ?? '').replace(
  /[&<>"]/g,
  character => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[character]),
);
const fmtDisk = bytes => `${(Number(bytes || 0) / 1024 ** 3).toFixed(1)} GB 可用`;
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

function drawCloud(canvas, points, trajectory, view) {
  const [context, width, height] = sizeCanvas(canvas);
  context.fillStyle = '#071421';
  context.fillRect(0, 0, width, height);
  if ($('showMapPoints')?.checked !== false || canvas.id === 'cloud') {
    for (const point of points || []) {
      const [x, y] = project(point, view, width, height);
      if (x < 0 || y < 0 || x > width || y > height) continue;
      const z = point[2] || 0;
      context.fillStyle = `hsla(${185 + z * 18},85%,${55 + (point[3] || 0.4) * 18}%,.8)`;
      context.fillRect(x, y, 2, 2);
    }
  }
  if (trajectory?.length && $('showMapRoute')?.checked !== false) {
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

async function refreshCamera() {
  try {
    state.camera = await api('/api/v1/camera');
    renderCamera();
  } catch (error) {
    $('cameraState').textContent = '不可用';
    $('cameraMessage').textContent = friendlyError(error);
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

async function showResult(job) {
  if (!job?.point_cloud_url) return;
  state.latestMapJob = job;
  $('resultEmpty').style.display = 'none';
  document.querySelector('.map-review').style.display = 'grid';
  $('overview').src = job.overview_url;
  state.mapArtifact = await api(job.point_cloud_url);
  $('resultMeta').textContent = `${job.job_id} · ${job.metrics?.point_count || state.mapArtifact.points.length} 点 · ${job.metrics?.worker || 'cloud-glim'}`;
  redrawResult();
  renderMapHistory();
  drawNavigation();
}

function renderMapHistory() {
  const completed = state.mapJobs.filter(job => job.state === 'complete');
  const tasks = state.mapJobs.filter(job => job.state !== 'complete');
  $('maps').innerHTML = completed.map(job => {
    const selected = state.latestMapJob?.job_id === job.job_id ? ' selected' : '';
    return `<div class="session selectable${selected}" data-map-job="${escapeHtml(job.job_id)}"><div><strong>${escapeHtml(job.job_id)}</strong><small>${escapeHtml(job.message || job.stage)} · ${escapeHtml(job.session_id)}</small></div><b>可用</b></div>`;
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
  const route = state.navigation?.route || [];
  const pose = state.navigation?.localization_pose;
  if (!points.length && !route.length) return;
  const all = points.map(point => [point[0], point[1]]).concat(route);
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
  const selectedJobId = state.latestMapJob?.job_id;
  const assetReady = Boolean(candidate && selectedJobId && candidate.map_job_id === selectedJobId);
  const operation = (navigation.operations || [])[0];
  const operationBusy = ['accepted', 'running'].includes(operation?.state);
  const patrolRunning = ['STARTING', 'PATROLLING', 'HOLDING', 'RESUMING', 'REPLANNING'].includes(runtime.state);
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
  $('motionAuthority').textContent = runtime.motionAuthorized ? '运动权已打开' : '运动权关闭';
  $('routeProgress').textContent = `${Number(runtime.routeProgressPercent || 0).toFixed(0)}%`;
  $('remainingRoute').textContent = runtime.remainingRoutePointCount == null
    ? '尚未开始' : `剩余 ${runtime.remainingRoutePointCount} 个路径点`;
  $('operationState').textContent = operation ? ({accepted: '已接收', running: '执行中', complete: '已完成', failed: '失败'}[operation.state] || operation.state) : '无';
  $('operationMessage').textContent = operation?.message || '点击后会在这里持续显示结果';

  $('assetStepState').textContent = assetReady ? '已发布' : (selectedJobId ? '待发布' : '未选择');
  $('assetStep').classList.toggle('done', assetReady);
  $('runtimeStepState').textContent = runtimeRunning ? (localization.usable ? '定位可用' : '定位中') : '未启动';
  $('runtimeStep').classList.toggle('done', runtimeRunning && localization.usable);
  $('patrolStepState').textContent = patrolRunning ? '巡检中' : (runtime.state === 'COMPLETED' ? '已完成' : '等待');
  $('patrolStep').classList.toggle('done', runtime.state === 'COMPLETED');

  let guidance = '请从历史中选择一张可用地图。';
  if (selectedJobId && !assetReady) guidance = '下一步：将所选地图和当时录制的路线发布到机器狗。';
  else if (assetReady && !runtimeRunning) guidance = '地图已在机器狗上。下一步：启动定位与 Nav2。';
  else if (runtimeRunning && !localization.usable) guidance = '正在地图中定位，请让机器狗站稳等待；无需重复点击。';
  else if (runtimeRunning && localization.usable && !patrolRunning) guidance = '定位已可用。下一步：确认现场后点击“开始巡检”。';
  if (patrolRunning) guidance = '巡检正在进行；地图上会显示位置和路线进度。需要中断时点击“暂停巡检”。';
  if (['FAULT', 'BLOCKED'].includes(runtime.state)) guidance = `${runtime.operatorMessage || runtime.reason}；可先查看下方诊断，或点击“自动清理并恢复”。`;
  if (operationBusy) guidance = `正在${operation.kind || '执行操作'}，请等待本页面显示完成，不要重复点击。`;
  $('workflowMessage').textContent = guidance;

  $('prepareNavigation').hidden = assetReady || !selectedJobId;
  $('prepareNavigationHelp').hidden = $('prepareNavigation').hidden;
  $('prepareNavigation').disabled = !selectedJobId || operationBusy;
  $('startRuntime').hidden = !assetReady || runtimeRunning;
  $('startRuntimeHelp').hidden = $('startRuntime').hidden;
  $('startRuntime').disabled = !candidate || operationBusy;
  $('resetLocalization').hidden = !runtimeRunning || localization.usable || patrolRunning;
  $('resetLocalization').disabled = operationBusy;
  $('recoverRuntime').hidden = !runtimeRunning || !['FAULT', 'BLOCKED'].includes(runtime.state);
  $('recoverRuntime').disabled = operationBusy;
  $('startPatrol').hidden = !runtimeRunning || patrolRunning;
  $('startPatrol').disabled = !localization.usable || operationBusy || ['FAULT', 'BLOCKED'].includes(runtime.state);
  $('stopPatrol').hidden = !patrolRunning;
  $('stopPatrol').disabled = operationBusy;
  drawNavigation();
}

async function navigationAction(action) {
  $('navigationError').textContent = '';
  try {
    if (action === 'prepare') {
      if (!state.latestMapJob) throw Error('请先选择一张已完成的 GLIM 地图');
      await post('/api/v1/navigation/prepare', {job_id: state.latestMapJob.job_id});
    } else if (action === 'runtime') {
      await post('/api/v1/navigation/runtime/start', {candidate_id: state.navigation.candidate.candidate_id});
    } else if (action === 'reset') {
      await post('/api/v1/navigation/localization/reset');
    } else if (action === 'start') {
      await post('/api/v1/navigation/patrol/start');
    } else if (action === 'stop') {
      await post('/api/v1/navigation/patrol/stop');
    } else if (action === 'recover') {
      await post('/api/v1/navigation/runtime/recover');
    }
    await refreshNavigation();
  } catch (error) {
    $('navigationError').textContent = friendlyError(error);
  }
}

function fillProfile(profile) {
  state.navigationProfile = profile;
  $('straightSpeed').value = profile.motion.straightSpeedMps;
  $('turnSpeed').value = profile.motion.turnSpeedRadps;
  $('lateralSpeed').value = profile.motion.lateralSpeedMps;
  $('slowHalfWidth').value = profile.avoidance.slowZoneHalfWidthM;
  $('slowdownRatio').value = profile.avoidance.slowdownRatio;
  $('blockedDecision').value = profile.avoidance.blockedDecisionS;
  $('dropoutGrace').value = profile.localization.dropoutGraceS;
  $('rejoinLookahead').value = profile.avoidance.rejoinLookaheadM;
  $('stopFront').value = profile.avoidance.stopZoneFrontM;
  $('stopHalfWidth').value = profile.avoidance.stopZoneHalfWidthM;
  $('slowFront').value = profile.avoidance.slowZoneFrontM;
  $('localizationTimeout').value = profile.localization.statusTimeoutS;
  $('recoveryStable').value = profile.localization.recoveryStableS;
  $('controllerFrequency').value = profile.controller.frequencyHz;
  $('mppiTimeSteps').value = profile.controller.timeSteps;
  $('mppiBatchSize').value = profile.controller.batchSize;
  $('profileMessage').textContent = `当前第 ${profile.revision} 版`;
}

async function refreshProfile() {
  try { fillProfile(await api('/api/v1/navigation/profile')); }
  catch (error) { $('profileMessage').textContent = friendlyError(error); }
}

async function saveProfile() {
  if (!state.navigationProfile) return;
  const profile = JSON.parse(JSON.stringify(state.navigationProfile));
  profile.motion.straightSpeedMps = Number($('straightSpeed').value);
  profile.motion.turnSpeedRadps = Number($('turnSpeed').value);
  profile.motion.lateralSpeedMps = Number($('lateralSpeed').value);
  profile.avoidance.slowZoneHalfWidthM = Number($('slowHalfWidth').value);
  profile.avoidance.slowdownRatio = Number($('slowdownRatio').value);
  profile.avoidance.blockedDecisionS = Number($('blockedDecision').value);
  profile.localization.dropoutGraceS = Number($('dropoutGrace').value);
  profile.avoidance.rejoinLookaheadM = Number($('rejoinLookahead').value);
  profile.avoidance.stopZoneFrontM = Number($('stopFront').value);
  profile.avoidance.stopZoneHalfWidthM = Number($('stopHalfWidth').value);
  profile.avoidance.slowZoneFrontM = Number($('slowFront').value);
  profile.localization.statusTimeoutS = Number($('localizationTimeout').value);
  profile.localization.recoveryStableS = Number($('recoveryStable').value);
  profile.controller.frequencyHz = Number($('controllerFrequency').value);
  profile.controller.timeSteps = Number($('mppiTimeSteps').value);
  profile.controller.batchSize = Number($('mppiBatchSize').value);
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

async function refreshNavigation() {
  try {
    state.navigation = await api('/api/v1/navigation');
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
async function handleMapClick(event) {
  const row = event.target.closest('[data-map-job]');
  if (!row) return;
  const job = state.mapJobs.find(item => item.job_id === row.dataset.mapJob);
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
  if (job?.state === 'complete') await showResult(job);
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
  if (job?.state === 'complete') await showResult(job);
});
$('recordButton').addEventListener('click', toggleRecord);
$('prepareNavigation').addEventListener('click', () => navigationAction('prepare'));
$('startRuntime').addEventListener('click', () => navigationAction('runtime'));
$('resetLocalization').addEventListener('click', () => navigationAction('reset'));
$('recoverRuntime').addEventListener('click', () => navigationAction('recover'));
$('startPatrol').addEventListener('click', () => navigationAction('start'));
$('stopPatrol').addEventListener('click', () => navigationAction('stop'));
$('saveProfile').addEventListener('click', saveProfile);
$('rollbackProfile').addEventListener('click', rollbackProfile);
$('refreshDiagnostics').addEventListener('click', refreshDiagnostics);
addEventListener('resize', () => {
  renderLive();
  redrawResult();
  drawNavigation();
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
  refreshNavigation();
  refreshProfile();
  scheduledTick();
  setInterval(refreshCamera, 2000);
  setInterval(refreshMapJobs, 3000);
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

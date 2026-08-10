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
  workspaceTransform: null,
  navigation: null,
  live: null,
  camera: null,
  cameraStarted: false,
  robotReachable: false,
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

async function showResult(job, {explicit = false} = {}) {
  if (!job?.point_cloud_url) return;
  if (explicit) state.mapSelectionExplicit = true;
  state.latestMapJob = job;
  $('resultEmpty').style.display = 'none';
  document.querySelector('.map-review').style.display = 'grid';
  $('overview').src = job.overview_url;
  state.mapArtifact = await api(job.point_cloud_url);
  state.navigationWorkspace = await api(
    `/api/v1/map-jobs/${encodeURIComponent(job.job_id)}/navigation-workspace`,
  );
  try {
    state.glimEditor = await api(
      `/api/v1/map-jobs/${encodeURIComponent(job.job_id)}/glim-editor`,
    );
  } catch (_) {
    state.glimEditor = {state: 'unavailable'};
  }
  state.savedNavigationWorkspace = structuredClone(state.navigationWorkspace);
  state.workspaceEditMode = null;
  const mapName = job.label ? `${job.label} · ${job.job_id}` : job.job_id;
  $('resultMeta').textContent = `${mapName} · ${job.metrics?.point_count || state.mapArtifact.points.length} 点 · ${job.metrics?.worker || 'cloud-glim'}`;
  redrawResult();
  drawWorkspace();
  renderWorkspaceControls();
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

function drawWorkspace() {
  const canvas = $('workspaceMap');
  const [context, width, height] = sizeCanvas(canvas);
  context.fillStyle = '#07111f';
  context.fillRect(0, 0, width, height);
  const bounds = workspaceBounds();
  if (!bounds) {
    state.workspaceTransform = null;
    return;
  }
  const padding = 42;
  const spanX = Math.max(bounds.maxX - bounds.minX, 1.0);
  const spanY = Math.max(bounds.maxY - bounds.minY, 1.0);
  const scale = Math.min((width - 2 * padding) / spanX, (height - 2 * padding) / spanY);
  const offsetX = (width - spanX * scale) / 2;
  const offsetY = (height - spanY * scale) / 2;
  const worldToCanvas = point => [
    offsetX + (point[0] - bounds.minX) * scale,
    height - offsetY - (point[1] - bounds.minY) * scale,
  ];
  state.workspaceTransform = {bounds, scale, offsetX, offsetY, height};
  const area = state.navigationWorkspace?.allowedArea || [];
  if (area.length >= 2) {
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
  context.fillStyle = '#94a3b87d';
  const points = state.mapArtifact?.points || [];
  for (let index = 0; index < points.length; index += 2) {
    const point = worldToCanvas(points[index]);
    context.fillRect(point[0], point[1], 1.5, 1.5);
  }
  const route = state.navigationWorkspace?.route || [];
  if (route.length) {
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
}

function recordedWorkspaceRoute() {
  const raw = (state.mapArtifact?.trajectory || []).map(point => [Number(point[0]), Number(point[1])]);
  if (raw.length < 2) return [];
  const sampled = [raw[0]];
  for (const point of raw.slice(1)) {
    const last = sampled[sampled.length - 1];
    if (Math.hypot(point[0] - last[0], point[1] - last[1]) >= 0.4) sampled.push(point);
  }
  const last = raw[raw.length - 1];
  const sampledLast = sampled[sampled.length - 1];
  if (Math.hypot(last[0] - sampledLast[0], last[1] - sampledLast[1]) >= 0.1) sampled.push(last);
  return sampled;
}

function renderWorkspaceControls() {
  const workspace = state.navigationWorkspace;
  const editing = Boolean(state.workspaceEditMode);
  $('workspaceBadge').textContent = !workspace
    ? '请先选地图'
    : workspace.ready ? `可发布 · 版本 ${workspace.revision}` : '等待绿色允许范围';
  $('workspaceBadge').className = workspace?.ready ? 'online' : '';
  $('workspaceHelp').textContent = state.workspaceEditMode === 'route'
    ? '请在地图上依次点击路线点；机器狗会按蓝线前进，需要时再由 Nav2 绕行。'
    : state.workspaceEditMode === 'area'
      ? '请沿允许行走区域的外边界依次点击；保存时会自动闭合。蓝线与边界至少留出 0.48 m。'
      : workspace?.ready
        ? '已准备好：路线和允许范围会随地图一起发布，任何修改都会生成新版本。'
        : '蓝色录制路线已就绪；再画一块绿色允许范围即可发布。';
  for (const id of ['editRoute', 'restoreRecordedRoute', 'editAllowedArea']) $(id).disabled = !workspace || editing;
  $('undoWorkspacePoint').disabled = !editing;
  $('cancelWorkspaceEdit').disabled = !editing;
  $('saveWorkspace').disabled = !workspace;
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

async function saveNavigationWorkspace() {
  if (!state.latestMapJob || !state.navigationWorkspace) return;
  $('workspaceError').textContent = '';
  $('saveWorkspace').disabled = true;
  try {
    const payload = {
      route: state.navigationWorkspace.route,
      allowedArea: state.navigationWorkspace.allowedArea,
      routeSource: state.navigationWorkspace.routeSource,
      robotRadiusM: state.navigationWorkspace.robotRadiusM || 0.48,
    };
    state.navigationWorkspace = await post(
      `/api/v1/map-jobs/${encodeURIComponent(state.latestMapJob.job_id)}/navigation-workspace`,
      payload,
    );
    state.savedNavigationWorkspace = structuredClone(state.navigationWorkspace);
    state.workspaceEditMode = null;
    $('workspaceHelp').textContent = '已保存；这一版路线和允许范围会与地图绑定。';
    drawWorkspace();
    renderWorkspaceControls();
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
  if (!points.length && !route.length) return;
  const all = points.map(point => [point[0], point[1]]).concat(route, allowedArea);
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
    && candidateGeneration >= 5
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
  const stopOperationBusy = operationBusy && ['runtime.stop', 'patrol.stop'].includes(operation?.kind);
  const patrolRunning = [
    'STARTING', 'PATROLLING', 'HOLDING', 'RESUMING', 'REPLANNING',
    'DETOURING', 'REJOINING', 'RETRYING', 'RECOVERING', 'SEARCHING_PATH',
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

  $('assetStepState').textContent = assetReady
    ? (runtimeRunning && !runtimeMatchesCandidate ? '已发布，待切换' : '已发布')
    : (selectedJobId ? (workspaceReady ? '待发布' : '待保存绿色范围') : '未选择');
  $('assetStep').classList.toggle('done', assetReady);
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
  if (['FAULT', 'BLOCKED'].includes(runtime.state)) guidance = `${runtime.operatorMessage || runtime.reason}；需要遥控机器狗时先点击“停止巡检并释放遥控权”，需要继续测试时再启动定位与 Nav2。`;
  if (operation?.state === 'failed') {
    guidance = `操作失败：${operation.message || operation.error || '未知错误'}。请不要重复点击，可查看下方诊断。`;
    $('navigationError').textContent = operation.message || operation.error || '导航操作失败';
  }
  if (operationBusy) guidance = `正在${operationKindLabels[operation.kind] || operation.kind || '执行操作'}，请等待请求结果，不要重复点击。`;
  $('workflowMessage').textContent = guidance;

  $('prepareNavigation').hidden = assetReady || !selectedJobId;
  $('prepareNavigationHelp').hidden = $('prepareNavigation').hidden;
  $('prepareNavigation').disabled = !selectedJobId || !workspaceReady || operationBusy;
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
  $('stopPatrol').hidden = !runtimeRunning && !motionBridgeRunning;
  $('stopPatrolHelp').hidden = !runtimeRunning && !motionBridgeRunning;
  // Stop uses its own supervisor lane and must remain available even when a
  // start/recovery operation is stuck in the control lane.
  $('stopPatrol').disabled = stopOperationBusy;
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
      await post('/api/v1/navigation/runtime/stop');
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
$('editRoute').addEventListener('click', () => {
  if (!state.navigationWorkspace) return;
  state.savedNavigationWorkspace = structuredClone(state.navigationWorkspace);
  state.navigationWorkspace.route = [];
  state.navigationWorkspace.routeSource = 'edited';
  state.workspaceEditMode = 'route';
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
  $('workspaceError').textContent = '';
  drawWorkspace(); renderWorkspaceControls();
});
$('editAllowedArea').addEventListener('click', () => {
  if (!state.navigationWorkspace) return;
  state.savedNavigationWorkspace = structuredClone(state.navigationWorkspace);
  state.navigationWorkspace.allowedArea = [];
  state.workspaceEditMode = 'area';
  $('workspaceError').textContent = '';
  drawWorkspace(); renderWorkspaceControls();
});
$('undoWorkspacePoint').addEventListener('click', () => {
  if (!state.workspaceEditMode || !state.navigationWorkspace) return;
  const key = state.workspaceEditMode === 'route' ? 'route' : 'allowedArea';
  state.navigationWorkspace[key].pop();
  drawWorkspace();
});
$('cancelWorkspaceEdit').addEventListener('click', () => {
  if (!state.savedNavigationWorkspace) return;
  state.navigationWorkspace = structuredClone(state.savedNavigationWorkspace);
  state.workspaceEditMode = null;
  $('workspaceError').textContent = '';
  drawWorkspace(); renderWorkspaceControls();
});
$('saveWorkspace').addEventListener('click', saveNavigationWorkspace);
$('startGlimEditor').addEventListener('click', startGlimEditor);
$('openGlimEditor').addEventListener('click', openGlimEditor);
$('publishGlimEditor').addEventListener('click', publishGlimEditor);
$('stopGlimEditor').addEventListener('click', stopGlimEditor);
$('workspaceMap').addEventListener('click', event => {
  if (!state.workspaceEditMode || !state.navigationWorkspace) return;
  const point = workspaceCanvasPoint(event);
  if (!point) return;
  const key = state.workspaceEditMode === 'route' ? 'route' : 'allowedArea';
  const values = state.navigationWorkspace[key];
  const last = values[values.length - 1];
  if (last && Math.hypot(point[0] - last[0], point[1] - last[1]) < 0.02) return;
  values.push(point.map(value => Number(value.toFixed(3))));
  drawWorkspace();
});
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
$('prepareNavigation').addEventListener('click', () => navigationAction('prepare'));
$('startRuntime').addEventListener('click', () => navigationAction('runtime'));
$('resetLocalization').addEventListener('click', () => navigationAction('reset'));
$('recoverRuntime').addEventListener('click', () => navigationAction('recover'));
$('startPatrol').addEventListener('click', () => navigationAction('start'));
$('stopPatrol').addEventListener('click', () => navigationAction('stop'));
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
  refreshNavigation();
  refreshProfile();
  refreshDiagnosticProfile();
  refreshIncidents();
  scheduledTick();
  setInterval(refreshCamera, 2000);
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

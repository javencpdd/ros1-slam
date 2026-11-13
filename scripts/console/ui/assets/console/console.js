/* Lite3 3D console: map mission + monitor + teleop over rosbridge.
 * P2/P3 interaction layer only; stack/launch lifecycle stays in the CLI work orders. */
(function () {
  'use strict';

  const $ = (id) => document.getElementById(id);
  const el = {
    bridgeUrl: $('bridge-url'), connect: $('connect'), bridgeStatus: $('bridge-status'),
    clientRole: $('client-role'), localRoleHint: $('local-role-hint'), localNote: $('local-note'),
    mapSelect: $('map-select'), mode: $('mode'), surface: $('surface'), goalZ: $('goal-z'),
    cloudWrap: $('cloud-wrap'), cloudSummary: $('cloud-summary'), cloudCanvas: $('cloud-canvas'),
    cloudTopic: $('cloud-topic'), cloudLive: $('cloud-live'), cloudLiveState: $('cloud-live-state'),
    cloudLiveRate: $('cloud-live-rate'), worldScene: $('world-scene'), cloudLayer: $('cloud-layer'),
    cloudSource: $('cloud-source'), cloudReset: $('cloud-reset'), cloudHint: $('cloud-hint'),
    view3dJump: $('view-3d-jump'),
    overlayToggle: $('overlay-toggle'), mMbf: $('m-mbf'),
    mPath: $('m-path'), mGoal: $('m-goal'),
    mBtStage: $('m-bt-stage'), btStage: $('bt-stage'),
    mapGuard: $('map-guard'), mapGuardState: $('map-guard-state'),
    goalTimeout: $('goal-timeout'), missionContinue: $('mission-continue'),
    missionExportJson: $('mission-export-json'), missionExportCsv: $('mission-export-csv'),
    mapRefresh: $('map-refresh'), mapRegrid: $('map-regrid'), mapHide: $('map-hide'),
    mapRestore: $('map-restore'), regridRes: $('regrid-res'), mapOpsState: $('map-ops-state'),
    mapEdit: $('map-edit'), mapEditSave: $('map-edit-save'), mapEditClear: $('map-edit-clear'),
    editValue: $('edit-value'), editState: $('edit-state'),
    goalYaw: $('goal-yaw'), canvas: $('map-canvas'), mapMeta: $('map-meta'),
    modeGoal: $('mode-goal'), modeStart: $('mode-start'), send: $('send'),
    sendAll: $('send-all'), btSend: $('bt-send'), cancel: $('cancel'), clear: $('clear'),
    goals: $('goals').querySelector('tbody'), log: $('log'),
    rates: $('rates').querySelector('tbody'), mBridge: $('m-bridge'), mMission: $('m-mission'),
    mMaster: $('m-master'), mTf: $('m-tf'), tfTree: $('tf-tree'),
    refresh: $('refresh-ros'), rosapiState: $('rosapi-state'),
    rosNodes: $('ros-nodes'), rosTopics: $('ros-topics'),
    vMax: $('v-max'), wMax: $('w-max'), vVal: $('v-val'), wVal: $('w-val'),
    deadman: $('deadman'), estop: $('estop'), unlock: $('unlock'), estopState: $('estop-state'),
    agentUrl: $('agent-url'), agentRefresh: $('agent-refresh'), agentState: $('agent-state'),
    stackList: $('stack-list'), stackLog: $('stack-log'),
    vizBackend: $('viz-backend'), foxgloveGuide: $('foxglove-guide'),
    foxgloveWs: $('foxglove-ws'),
  };

  const BT_STAGES = [
    ['wait_goal', '等目标'], ['request_path', '请求路径'], ['wait_path', '等路径'],
    ['follow_path', '跟随路径'], ['wait_arrival', '等到点'], ['dwell_at_goal', '到点停留'],
    ['stop_after_success', '停止'], ['recovery_on_failure', '失败恢复'],
  ];

  const TOPIC_SPECS = [
    { topic: '/cmd_vel', type: 'geometry_msgs/Twist', label: '/cmd_vel', expect: 20 },
    { topic: '/v8/3d/global/path', type: 'nav_msgs/Path', label: '/v8/3d/global/path', expect: 0 },
    { topic: '/v8/local/trajectory', type: 'nav_msgs/Path', label: '/v8/local/trajectory', expect: 10 },
    { topic: '/v8/local/pose', type: 'geometry_msgs/PoseStamped', label: '/v8/local/pose', expect: 20 },
    { topic: '/tf', type: 'tf2_msgs/TFMessage', label: '/tf', expect: 20 },
    { topic: '/tf_static', type: 'tf2_msgs/TFMessage', label: '/tf_static', expect: 0 },
    { topic: '/move_base/status', type: 'actionlib_msgs/GoalStatusArray', label: '/move_base/status', expect: 5 },
    { topic: '/move_base/global_costmap/costmap', type: 'nav_msgs/OccupancyGrid', label: '/move_base/global_costmap/costmap', expect: 1 },
    { topic: '/move_base/local_costmap/costmap', type: 'nav_msgs/OccupancyGrid', label: '/move_base/local_costmap/costmap', expect: 1 },
    { topic: '/v8/bt/status', type: 'std_msgs/String', label: '/v8/bt/status', expect: 2 },
    { topic: '/map', type: 'nav_msgs/OccupancyGrid', label: '/map', expect: 1 },
  ];
  const AD_TOPICS = [
    ['/move_base_simple/goal', 'geometry_msgs/PoseStamped'],
    ['/initialpose', 'geometry_msgs/PoseWithCovarianceStamped'],
    ['/v8/3d/global/start', 'geometry_msgs/PoseStamped'],
    ['/v8/bt/goal', 'geometry_msgs/PoseStamped'],
    ['/lite3/teleop/app_cmd_vel', 'geometry_msgs/Twist'],
    ['/lite3/safety/estop_cmd_vel', 'geometry_msgs/Twist'],
  ];
  const TELEOP_TOPIC = '/lite3/teleop/app_cmd_vel';
  const ESTOP_TOPIC = '/lite3/safety/estop_cmd_vel';

  let bridge = null;
  let online = false;
  let maps = [];
  let stackSnapshot = [];
  const HIDDEN_KEY = 'lite3-console-hidden-maps';
  let hiddenMaps = new Set();
  try { hiddenMaps = new Set(JSON.parse(localStorage.getItem(HIDDEN_KEY) || '[]')); } catch (e) {}
  let current = null;
  let mapImage = null;
  let view = null;
  let goals = [];
  let start = null;
  let selected = -1;
  let clickMode = 'goal';
  let estopLatched = false;
  let teleop = { active: false, v: 0, w: 0, last: 0, holding: false };
  const rates = Object.create(null);
  const tfPairs = new Map();
  let tfCount = 0;
  let cloud = { points: [], colors: [], zmin: 0, zmax: 1, yaw: -0.9, pitch: 0.55, dist: 10,
                center: [0, 0, 0], fitPending: true, mode: 'map', note: '' };
  let live = { enabled: false, topic: null, count: 0, points: 0, last: 0, window: 0 };
  let scene = { id: '', boxes: [], meshes: [], bounds: null, triCount: 0, layer: 'both',
                cache: Object.create(null) };
  let worldIndex = [];
  let serverVox = { enabled: false, timer: null, meta: null, stamp: null, fetching: false,
                    points: 0, error: '', stale: false };
  let clientIdentity = { local: null, checked: false, reason: '' };
  let clientRoleTimer = null;
  let robotCloudLast = 0;
  let overlay = { enabled: true, global: null, local: null, tf: Object.create(null), robot: null,
                  mapMsg: null, costmapGlobal: null, costmapLocal: null };
  let edit = { active: false, changes: new Map() };
  let mission = { active: false, queue: [], index: -1, goal: null, sentAt: 0, sentSecs: 0,
                  pathM: 0, lastPose: null, arrivalSince: 0, error: null, mode: null,
                  timeoutS: 60, continueOnFail: false,
                  btSentSeq: 0, btHandledSeq: null };
  let btStatus = { text: '', parsed: null };

  // ---------------------------------------------------------------- utilities
  function log(message, cls) {
    const stamp = new Date().toLocaleTimeString('zh-CN', { hour12: false });
    const line = document.createElement('div');
    if (cls) line.className = cls;
    line.textContent = '[' + stamp + '] ' + message;
    el.log.appendChild(line);
    while (el.log.childNodes.length > 200) el.log.removeChild(el.log.firstChild);
    el.log.scrollTop = el.log.scrollHeight;
  }
  function fmt(value, digits) {
    return value == null || !isFinite(value) ? '—' : Number(value).toFixed(digits == null ? 3 : digits);
  }
  function stamp() {
    const t = Date.now() / 1000;
    return { secs: Math.floor(t), nsecs: Math.round((t - Math.floor(t)) * 1e9) };
  }
  function poseStamped(x, y, z, yaw) {
    const h = yaw * Math.PI / 180.0;
    return {
      header: { frame_id: 'map', stamp: stamp() },
      pose: {
        position: { x: x, y: y, z: z },
        orientation: { x: 0, y: 0, z: Math.sin(h / 2), w: Math.cos(h / 2) },
      },
    };
  }
  function twist(v, w) {
    return { linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: w } };
  }
  function isFormTarget(target) {
    return ['INPUT', 'SELECT', 'TEXTAREA', 'BUTTON'].indexOf(target && target.tagName) >= 0;
  }

  // ---------------------------------------------------------------- rosbridge
  function connect() {
    const url = el.bridgeUrl.value.trim();
    if (!url) { log('先填 rosbridge 地址', 'err'); return; }
    if (bridge) bridge.close();
    bridge = new RosBridge(url, {
      open: onOpen, close: onClose, status: onStatus,
    });
    bridge.connect();
    setBridge('连接中…', '');
    log('连接 ' + url);
  }
  function onOpen() {
    online = true;
    setBridge('已连接', 'on');
    el.mBridge.textContent = '已连接';
    log('rosbridge 已连接', 'ok');
    AD_TOPICS.forEach(([topic, type]) => {
      try { bridge.advertise(topic, type); } catch (e) { log('advertise 失败 ' + topic + ': ' + e.message, 'err'); }
    });
    setupSubscriptions();
    refreshRosapi();
  }
  function onClose() {
    online = false;
    setBridge('已断开', 'err');
    el.mBridge.textContent = '已断开';
    stopTeleop(false);
    if (live.enabled) stopLiveCloud(false);
    log('rosbridge 断开', 'err');
    const url = el.bridgeUrl.value.trim();
    if (url) setTimeout(() => {
      if (!online && el.bridgeUrl.value.trim() === url) {
        log('自动重连 ' + url);
        connect();
      }
    }, 5000);
  }
  function onStatus(state) {
    if (state === 'error') setBridge('连接错误', 'err');
    if (state === 'offline') setBridge('未连接', '');
  }
  function setBridge(text, cls) {
    el.bridgeStatus.textContent = text;
    el.bridgeStatus.className = 'pill ' + cls;
  }
  function pub(topic, type, msg) {
    if (!bridge || !online) { log('未连接，丢弃 ' + topic, 'err'); return false; }
    try { bridge.publish(topic, type, msg); return true; }
    catch (e) { log('publish 失败 ' + topic + ': ' + e.message, 'err'); return false; }
  }

  // ---------------------------------------------------------------- map / mission
  async function refreshMaps(preserve) {
    const response = await fetch('assets/data/maps.json', { cache: 'no-store' });
    if (!response.ok) throw new Error('HTTP ' + response.status);
    const payload = await response.json();
    const previous = preserve || (current ? current.id : null);
    maps = payload.maps || [];
    const visible = maps.filter((m) => !hiddenMaps.has(m.id));
    if (!visible.length) {
      el.mapSelect.innerHTML = '';
      el.mapMeta.textContent = '没有可用地图（可能全部隐藏）';
      return;
    }
    el.mapSelect.innerHTML = visible.map((m) =>
      '<option value="' + m.id + '">' + escapeHtml(m.label) + '</option>').join('');
    let target = visible.some((m) => m.id === previous) ? previous : null;
    if (!target) target = visible.some((m) => m.id === 'two_floor_nav') ? 'two_floor_nav' : visible[0].id;
    el.mapSelect.value = target;
    selectMap(target);
    log('地图库 ' + maps.length + ' 张（可见 ' + visible.length + '，隐藏 ' + hiddenMaps.size +
        '）｜ HEAD ' + (payload.git_head || '—'));
  }
  async function loadMaps() {
    try {
      await refreshMaps(null);
    } catch (e) {
      el.mapMeta.textContent = '地图库加载失败：' + e.message;
      log('地图库加载失败：' + e.message, 'err');
    }
  }
  function saveHidden() {
    localStorage.setItem(HIDDEN_KEY, JSON.stringify(Array.from(hiddenMaps)));
  }
  function mapOps(text, cls) {
    el.mapOpsState.textContent = text;
    el.mapOpsState.className = 'pill ' + (cls || '');
  }
  async function refreshMapLibrary() {
    mapOps('刷新中…');
    try {
      const response = await fetch(agentBase() + '/api/maps/refresh', { method: 'POST' });
      const payload = await response.json();
      if (!payload.ok) throw new Error(payload.error || payload.output || 'agent 失败');
      await refreshMaps(current ? current.id : null);
      mapOps('地图库已刷新', 'on');
    } catch (error) {
      mapOps('刷新失败', 'err');
      const hint = /Failed to fetch|NetworkError/i.test(error.message)
        ? '（agent 不可达，检查 ' + agentBase() + ' 与 LITE3_AGENT_BIND）' : '';
      log('地图库刷新失败：' + error.message + hint, 'err');
    }
  }
  async function regridCurrentMap() {
    if (!current) { log('先选地图', 'err'); return; }
    const source = (current.files || []).find((file) => file.endsWith('.pcd'));
    if (!source) { log('当前地图没有源 PCD，不能重栅格化', 'err'); return; }
    const resolution = Math.max(0.01, parseFloat(el.regridRes.value) || 0.05);
    const name = current.id.replace(/[^A-Za-z0-9_.-]/g, '_') +
      '_r' + Math.round(resolution * 1000);
    mapOps('重栅格化中…');
    try {
      const response = await fetch(agentBase() + '/api/maps/regrid', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ source: source, name: name, resolution: resolution,
                               min_z: -0.5, max_z: 2.0 }),
      });
      const payload = await response.json();
      if (!payload.ok) throw new Error(payload.error || 'regrid failed');
      await refreshMaps(payload.map_id);
      if (maps.some((m) => m.id === payload.map_id)) {
        el.mapSelect.value = payload.map_id;
        selectMap(payload.map_id);
      }
      mapOps('已生成 ' + payload.map_id, 'on');
      log('重栅格化 ' + payload.map_id + '（源 ' + source + '，' + resolution + ' m/px）', 'ok');
    } catch (e) {
      mapOps('重栅格化失败', 'err');
      log('重栅格化失败：' + e.message, 'err');
    }
  }
  function selectMap(id) {
    if (mission.active) stopMission('切换地图');
    current = maps.find((m) => m.id === id) || null;
    goals = []; start = null; selected = -1;
    edit.active = false; edit.changes.clear(); updateEditState();
    el.mapGuard.checked = false;
    overlay.mapMsg = null;
    checkMapGuard();
    if (!current) return;
    selectWorldForMap(current);
    paintGoals();
    const img = new Image();
    img.onload = () => { mapImage = img; draw(); updateMeta(); updateSurfaces(); updateMissionState(); };
    loadCloud(current);
    img.onerror = () => { mapImage = null; el.mapMeta.textContent = '预览图加载失败：' + current.preview; };
    img.src = current.preview;
  }
  function updateMeta() {
    if (!current) return;
    const parts = [
      '<span>类型 <b>' + escapeHtml(current.kind) + '</b></span>',
      '<span>尺寸 <b>' + current.width + '×' + current.height + '</b></span>',
      '<span>分辨率 <b>' + current.resolution + ' m/px</b></span>',
      '<span>原点 <b>' + current.origin.slice(0, 2).map((v) => Number(v).toFixed(2)).join(', ') + '</b></span>',
    ];
    if (current.world) parts.push('<span>world <b>' + escapeHtml(current.world) + '</b></span>');
    if (current.z_range) parts.push('<span>z <b>' + current.z_range[0] + '…' + current.z_range[1] + ' m</b></span>');
    if (current.surfaces) parts.push('<span>支撑面 <b>' + current.surfaces.length + '</b></span>');
    const files = (current.files || []).map((f) =>
      '<a href="../' + escapeHtml(f) + '" target="_blank" rel="noopener">' +
      escapeHtml(f.split('/').pop()) + '</a>').join(' · ');
    if (files) parts.push('<span>证据 ' + files + '</span>');
    el.mapMeta.innerHTML = parts.join('');
    updateMissionState();
  }
  function updateSurfaces() {
    const surfaces = (current && current.surfaces) || [];
    el.surface.innerHTML = '';
    if (!surfaces.length) {
      el.surface.innerHTML = '<option value="">固定 z</option>';
      el.surface.disabled = true;
      return;
    }
    el.surface.disabled = false;
    el.surface.innerHTML = '<option value="custom">自定义 z</option>' + surfaces.map((s) =>
      '<option value="' + escapeHtml(s.id) + '">' + escapeHtml(s.id) + ' · z=' + s.z + '</option>').join('');
  }
  function surfaceZ() {
    if (el.mode.value !== '3d' || !current || !current.surfaces) return 0;
    const value = el.surface.value;
    if (!value || value === 'custom') return parseFloat(el.goalZ.value) || 0;
    const surface = current.surfaces.find((s) => s.id === value);
    return surface ? Number(surface.z) : parseFloat(el.goalZ.value) || 0;
  }
  function draw() {
    const canvas = el.canvas;
    const ctx = canvas.getContext('2d');
    ctx.imageSmoothingEnabled = false;
    ctx.fillStyle = '#04080d';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (!mapImage) return;
    const scale = Math.min(canvas.width / mapImage.width, canvas.height / mapImage.height);
    const ox = (canvas.width - mapImage.width * scale) / 2;
    const oy = (canvas.height - mapImage.height * scale) / 2;
    view = { scale: scale, ox: ox, oy: oy };
    ctx.drawImage(mapImage, ox, oy, mapImage.width * scale, mapImage.height * scale);
    // 1 m scale bar
    if (current) {
      const bar = Math.max(12, scale * (1 / current.resolution));
      ctx.strokeStyle = 'rgba(220,235,245,.75)';
      ctx.fillStyle = 'rgba(220,235,245,.75)';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(ox + 12, oy + 16); ctx.lineTo(ox + 12 + bar, oy + 16); ctx.stroke();
      ctx.font = '12px ui-monospace,monospace';
      ctx.fillText('1 m', ox + 12, oy + 31);
    }
    paintOverlay(ctx);
    paintGoals(ctx);
    paintEdits(ctx);
  }
  function mapToPixel(x, y) {
    if (!current || !mapImage || !view) return null;
    const px = (x - current.origin[0]) / current.resolution;
    const py = current.height - (y - current.origin[1]) / current.resolution;
    return { x: view.ox + px * view.scale, y: view.oy + py * view.scale };
  }
  function pixelToMap(cx, cy) {
    if (!current || !mapImage || !view) return null;
    const px = (cx - view.ox) / view.scale;
    const py = (cy - view.oy) / view.scale;
    if (px < 0 || py < 0 || px > current.width || py > current.height) return null;
    return {
      x: current.origin[0] + px * current.resolution,
      y: current.origin[1] + (current.height - py) * current.resolution,
    };
  }
  function paintOverlay(ctx) {
    if (!overlay.enabled || !view || !current) return;
    paintCostmap(ctx, overlay.costmapGlobal, '#ff5d73');
    paintCostmap(ctx, overlay.costmapLocal, '#ffb703');
    paintPath(ctx, overlay.global, '#4cc9f0', 1.8);
    paintPath(ctx, overlay.local, '#ffb703', 1.5);
    if (overlay.robot) {
      const p = mapToPixel(overlay.robot.x, overlay.robot.y);
      if (p) {
        ctx.beginPath(); ctx.arc(p.x, p.y, 6, 0, Math.PI * 2);
        ctx.fillStyle = '#7bf1a8'; ctx.fill();
        ctx.strokeStyle = '#04121c'; ctx.lineWidth = 1.5; ctx.stroke();
        ctx.beginPath(); ctx.moveTo(p.x, p.y);
        ctx.lineTo(p.x + Math.cos(overlay.robot.yaw) * 14,
                   p.y - Math.sin(overlay.robot.yaw) * 14);
        ctx.strokeStyle = '#7bf1a8'; ctx.lineWidth = 2; ctx.stroke();
      }
    }
  }
  function updateEditState() {
    el.editState.textContent = edit.active
      ? '编辑中 · 未保存 ' + edit.changes.size : (edit.changes.size ? '未保存 ' + edit.changes.size : '未编辑');
    el.editState.className = 'pill ' + (edit.active ? 'on' : (edit.changes.size ? 'err' : ''));
  }
  function paintEdits(ctx) {
    if (!edit.changes.size || !view || !current) return;
    edit.changes.forEach((value, key) => {
      const parts = key.split(',');
      const x = current.origin[0] + (Number(parts[0]) + 0.5) * current.resolution;
      const y = current.origin[1] + (Number(parts[1]) + 0.5) * current.resolution;
      const p = mapToPixel(x, y);
      if (!p) return;
      ctx.fillStyle = value === 0 ? 'rgba(255,93,115,.9)' : 'rgba(123,241,168,.9)';
      ctx.fillRect(p.x - 2, p.y - 2, 4, 4);
    });
  }
  function toggleEdit() {
    if (!current || current.kind !== '2d_grid' || !String(current.id).startsWith('console_')) {
      log('只有控制台重栅格化生成的 2D 栅格可编辑（先点“重栅格化”）', 'err');
      return;
    }
    edit.active = !edit.active;
    log(edit.active ? '进入栅格编辑：点地图把格子设为 障碍/自由' : '退出栅格编辑');
    updateEditState();
  }
  function editCell(point) {
    const col = Math.floor((point.x - current.origin[0]) / current.resolution);
    const row = Math.floor((point.y - current.origin[1]) / current.resolution);
    if (col < 0 || row < 0 || col >= current.width || row >= current.height) return;
    const value = parseInt(el.editValue.value, 10);
    edit.changes.set(col + ',' + row, value === 0 ? 0 : 254);
    updateEditState();
    draw();
  }
  async function saveEdits() {
    if (!current || !String(current.id).startsWith('console_')) { log('只保存控制台栅格', 'err'); return; }
    if (!edit.changes.size) { log('没有未保存的栅格修改', 'err'); return; }
    const changes = [];
    edit.changes.forEach((value, key) => {
      const parts = key.split(',');
      changes.push([Number(parts[0]), Number(parts[1]), value]);
    });
    try {
      const response = await fetch(agentBase() + '/api/maps/edit', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ id: current.id, changes: changes }),
      });
      const payload = await response.json();
      if (!payload.ok) throw new Error(payload.error || 'edit failed');
      log('栅格已保存：' + payload.applied + ' 格', 'ok');
      edit.changes.clear(); edit.active = false; updateEditState();
      const keep = current.id;
      await refreshMaps(keep);
      if (maps.some((m) => m.id === keep)) { el.mapSelect.value = keep; selectMap(keep); }
    } catch (error) {
      log('栅格保存失败：' + error.message, 'err');
    }
  }
  function paintCostmap(ctx, message, color) {
    if (!message || !message.info || !message.data) return;
    const info = message.info, data = message.data;
    const res = info.resolution, w = info.width, h = info.height;
    const ox = info.origin.position.x, oy = info.origin.position.y;
    const stride = Math.max(1, Math.floor((w * h) / 6000));
    ctx.globalAlpha = 0.22;
    ctx.fillStyle = color;
    for (let i = 0; i < data.length; i += stride) {
      const value = data[i];
      if (value < 20) continue;
      const col = i % w, row = Math.floor(i / w);
      const p = mapToPixel(ox + (col + 0.5) * res, oy + (row + 0.5) * res);
      if (!p) continue;
      ctx.fillRect(p.x - 1, p.y - 1, 2, 2);
    }
    ctx.globalAlpha = 1;
  }
  function paintPath(ctx, message, color, width) {
    const poses = message && message.poses;
    if (!poses || !poses.length) return;
    let started = false;
    ctx.beginPath();
    poses.forEach((pose) => {
      const position = (pose.pose && pose.pose.position) || pose.position;
      if (!position) return;
      const p = mapToPixel(position.x, position.y);
      if (!p) return;
      if (!started) { ctx.moveTo(p.x, p.y); started = true; } else { ctx.lineTo(p.x, p.y); }
    });
    if (!started) return;
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.globalAlpha = 0.85;
    ctx.stroke(); ctx.globalAlpha = 1;
  }
  function rotateQ(q, v) {
    const x = q.x, y = q.y, z = q.z, w = q.w == null ? 1 : q.w;
    const t = [2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])];
    return [v[0] + w * t[0] + y * t[2] - z * t[1],
            v[1] + w * t[1] + z * t[0] - x * t[2],
            v[2] + w * t[2] + x * t[1] - y * t[0]];
  }
  function applyTF(tf, p) {
    const t = tf.translation || { x: 0, y: 0, z: 0 };
    const q = tf.rotation || { x: 0, y: 0, z: 0, w: 1 };
    const r = rotateQ(q, p);
    return [r[0] + (t.x || 0), r[1] + (t.y || 0), r[2] + (t.z || 0)];
  }
  function quatMul(a, b) {
    const ax = a.x || 0, ay = a.y || 0, az = a.z || 0, aw = a.w == null ? 1 : a.w;
    const bx = b.x || 0, by = b.y || 0, bz = b.z || 0, bw = b.w == null ? 1 : b.w;
    return {
      x: aw * bx + ax * bw + ay * bz - az * by,
      y: aw * by - ax * bz + ay * bw + az * bx,
      z: aw * bz + ax * by - ay * bx + az * bw,
      w: aw * bw - ax * bx - ay * by - az * bz,
    };
  }
  function quatYaw(q) {
    return Math.atan2(2 * ((q.w || 0) * (q.z || 0) + (q.x || 0) * (q.y || 0)),
                      1 - 2 * ((q.y || 0) * (q.y || 0) + (q.z || 0) * (q.z || 0)));
  }
  function computeRobot() {
    const start = 'map', goal = 'base_link';
    const edges = Object.keys(overlay.tf).map((key) => key.split('|'));
    const queue = [[start, []]];
    const seen = new Set([start]);
    let chain = null;
    while (queue.length) {
      const item = queue.shift();
      if (item[0] === goal) { chain = item[1]; break; }
      edges.forEach((edge) => {
        if (edge[0] === item[0] && !seen.has(edge[1])) {
          seen.add(edge[1]);
          queue.push([edge[1], item[1].concat([overlay.tf[edge[0] + '|' + edge[1]]])]);
        }
      });
    }
    if (!chain) { overlay.robot = null; requestRobotCloudDraw(); return; }
    let p = [0, 0, 0];
    let q = { x: 0, y: 0, z: 0, w: 1 };
    for (let i = chain.length - 1; i >= 0; i--) {
      p = applyTF(chain[i], p);
      q = quatMul(chain[i].rotation || { x: 0, y: 0, z: 0, w: 1 }, q);
    }
    overlay.robot = { x: p[0], y: p[1], z: p[2], yaw: quatYaw(q) };
    requestRobotCloudDraw();
  }
  function paintGoals(ctx) {
    if (!ctx) { draw(); return; }
    if (!view || !current) return;
    const circle = (x, y, color, label) => {
      ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2);
      ctx.fillStyle = color; ctx.fill();
      ctx.strokeStyle = '#04121c'; ctx.lineWidth = 1.5; ctx.stroke();
      if (label != null) {
        ctx.fillStyle = '#04121c';
        ctx.font = 'bold 10px ui-monospace,monospace';
        ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText(String(label), x, y + 0.5);
        ctx.textAlign = 'start'; ctx.textBaseline = 'alphabetic';
      }
    };
    if (start) {
      const p = mapToPixel(start.x, start.y);
      if (p) { circle(p.x, p.y, '#7bf1a8', 'S'); }
    }
    goals.forEach((goal, index) => {
      const p = mapToPixel(goal.x, goal.y);
      if (!p) return;
      if (index === selected) {
        ctx.beginPath(); ctx.arc(p.x, p.y, 11, 0, Math.PI * 2);
        ctx.strokeStyle = '#ffb703'; ctx.lineWidth = 2; ctx.stroke();
      }
      circle(p.x, p.y, '#4cc9f0', index + 1);
    });
  }
  function updateMissionState() {
    const name = current ? current.id : '—';
    el.mMission.textContent = name + ' ｜ ' + goals.length + ' 目标' + (start ? ' ｜ 有起点' : '');
    if (!mission.active) {
      el.mGoal.textContent = '空闲';
    } else {
      const goal = mission.goal;
      el.mGoal.textContent = (mission.index + 1) + '/' + goals.length +
        (goal ? ' · ' + goal.state + ' · ' + fmt(mission.error, 2) + ' m' : ' · 等待');
    }
  }

  // ---------------------------------------------------------------- 3D point preview
  function scenePrimitiveCount() { return scene.boxes.length + scene.meshes.length; }
  function solidVisible() { return scenePrimitiveCount() > 0 && scene.layer !== 'cloud'; }
  function cloudVisible() { return scene.layer !== 'solid'; }
  function updateCloudSummary() {
    const parts = [];
    if (solidVisible()) {
      parts.push('实体 ' + scene.boxes.length + ' box' +
        (scene.triCount ? ' / ' + scene.triCount + ' 面' : ''));
    }
    if (cloudVisible() && cloud.points.length) parts.push('点云 ' + cloud.points.length + ' 点');
    if (cloudVisible() && cloud.note) parts.push(cloud.note);
    parts.push('拖拽旋转 / 滚轮缩放');
    el.cloudSummary.textContent = '3D 场景 · ' + parts.join(' · ');
  }
  function pointBounds(points) {
    if (!points.length) return null;
    let minx = Infinity, miny = Infinity, minz = Infinity;
    let maxx = -Infinity, maxy = -Infinity, maxz = -Infinity;
    points.forEach((p) => {
      minx = Math.min(minx, p[0]); maxx = Math.max(maxx, p[0]);
      miny = Math.min(miny, p[1]); maxy = Math.max(maxy, p[1]);
      minz = Math.min(minz, p[2]); maxz = Math.max(maxz, p[2]);
    });
    return [minx, miny, minz, maxx, maxy, maxz];
  }
  function unionBounds(a, b) {
    if (!a) return b ? b.slice() : null;
    if (!b) return a.slice();
    return [Math.min(a[0], b[0]), Math.min(a[1], b[1]), Math.min(a[2], b[2]),
            Math.max(a[3], b[3]), Math.max(a[4], b[4]), Math.max(a[5], b[5])];
  }
  function fitView(force) {
    if (!force && !cloud.fitPending) return;
    let bounds = null;
    if (solidVisible()) {
      bounds = scene.bounds;
    } else if (cloudVisible()) {
      bounds = pointBounds(cloud.points);
    }
    if (overlay.robot && !bounds) {
      const robot = overlay.robot;
      const z = Number.isFinite(robot.z) ? robot.z : 0.15;
      bounds = [robot.x, robot.y, z - 0.1, robot.x, robot.y, z + 0.6];
    }
    if (!bounds) return;
    cloud.center = [(bounds[0] + bounds[3]) / 2, (bounds[1] + bounds[4]) / 2,
                    (bounds[2] + bounds[5]) / 2];
    const span = Math.max(bounds[3] - bounds[0], bounds[4] - bounds[1],
                          bounds[5] - bounds[2], 1.0);
    cloud.dist = span * 1.5;
    cloud.fitPending = false;
  }
  function loadCloud(map) {
    if (serverVox.enabled) { drawCloud(); return; }
    if (live.enabled) { drawCloud(); return; }
    cloud.points = [];
    cloud.colors = [];
    cloud.mode = 'map';
    cloud.note = map && map.points3d ? '' : '该地图无点云样本';
    cloud.fitPending = true;
    updateCloudSummary();
    drawCloud();
    if (!map || !map.points3d) return;
    fetch(map.points3d, { cache: 'no-store' }).then((response) => {
      if (!response.ok) throw new Error('HTTP ' + response.status);
      return response.json();
    }).then((payload) => {
      if (serverVox.enabled || live.enabled) return;
      const points = payload.points || [];
      if (!points.length) return;
      let zmin = Infinity, zmax = -Infinity;
      points.forEach((p) => {
        if (p[2] < zmin) zmin = p[2];
        if (p[2] > zmax) zmax = p[2];
      });
      cloud.points = points;
      cloud.colors = [];
      cloud.zmin = payload.z_range ? payload.z_range[0] : zmin;
      cloud.zmax = payload.z_range ? payload.z_range[1] : zmax;
      cloud.mode = 'map';
      cloud.note = '';
      cloud.fitPending = true;
      updateCloudSummary();
      drawCloud();
    }).catch((error) => {
      cloud.note = '点云加载失败：' + error.message;
      updateCloudSummary();
      drawCloud();
    });
  }
  async function loadWorldIndex() {
    try {
      const response = await fetch('assets/data/worlds/worlds.json', { cache: 'no-store' });
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const payload = await response.json();
      worldIndex = payload.worlds || [];
      const keep = el.worldScene.value;
      el.worldScene.innerHTML = '<option value="">不加载实体</option>' + worldIndex.map((world) =>
        '<option value="' + escapeHtml(world.id) + '">' + escapeHtml(world.label) +
        '（' + world.boxes + ' box' + (world.triangles ? ' / ' + world.triangles + ' 面' : '') +
        '）</option>').join('');
      if (keep && worldIndex.some((world) => world.id === keep)) el.worldScene.value = keep;
    } catch (error) {
      log('实体世界索引加载失败：' + error.message, 'err');
    }
  }
  function applyWorldScene(id, payload) {
    scene.id = id;
    scene.boxes = payload.boxes || [];
    scene.meshes = payload.meshes || [];
    scene.bounds = payload.bounds || null;
    scene.triCount = scene.meshes.reduce((total, mesh) => total + (mesh.faces || []).length, 0);
    cloud.fitPending = true;
    updateCloudSummary();
    drawCloud();
    const skipped = payload.stats && payload.stats.skipped ? payload.stats.skipped.length : 0;
    log('实体世界 ' + id + '：' + scene.boxes.length + ' box / ' + scene.triCount + ' 面' +
        (skipped ? '（跳过 ' + skipped + ' 项）' : ''), 'ok');
  }
  async function loadWorldScene(id) {
    if (!id) {
      scene.id = ''; scene.boxes = []; scene.meshes = []; scene.bounds = null;
      scene.triCount = 0; cloud.fitPending = true;
      updateCloudSummary(); drawCloud();
      return;
    }
    if (scene.cache[id]) { applyWorldScene(id, scene.cache[id]); return; }
    try {
      const entry = worldIndex.find((world) => world.id === id);
      const url = entry ? entry.file : 'assets/data/worlds/' + id + '.json';
      const response = await fetch(url, { cache: 'no-store' });
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const payload = await response.json();
      scene.cache[id] = payload;
      applyWorldScene(id, payload);
    } catch (error) {
      log('实体世界加载失败：' + error.message, 'err');
    }
  }
  function worldForMap(map) {
    if (!map || !worldIndex.length) return '';
    if (map.world_id && worldIndex.some((world) => world.id === map.world_id)) {
      return map.world_id;
    }
    const text = String(map.world || '').toLowerCase();
    if (!text || text.includes('真机') || text.includes('2d')) return '';
    const exact = worldIndex.slice()
      .sort((a, b) => b.id.length - a.id.length)
      .find((world) => text.includes(world.id.toLowerCase()));
    if (exact) return exact.id;
    if (text.includes('m1')) return 'm1_world';
    if (text.includes('nist')) return 'nist_stairs_120';
    if (text.includes('stair_lab')) return 'stair_lab_stairs_easy';
    return '';
  }
  function selectWorldForMap(map) {
    const id = worldForMap(map);
    el.worldScene.value = id;
    loadWorldScene(id);
  }
  function cloudColour(t) {
    t = Math.max(0, Math.min(1, t));
    const stops = [[0, [30, 60, 140]], [.3, [30, 170, 190]], [.55, [80, 200, 90]],
                   [.78, [230, 200, 60]], [1, [230, 80, 60]]];
    for (let i = 0; i < stops.length - 1; i++) {
      if (t <= stops[i + 1][0]) {
        const f = (t - stops[i][0]) / (stops[i + 1][0] - stops[i][0]);
        return stops[i][1].map((v, j) => Math.round(v + (stops[i + 1][1][j] - v) * f));
      }
    }
    return stops[stops.length - 1][1];
  }
  function applyBoxMatrix(matrix, point) {
    const [x, y, z] = point;
    return [
      matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
      matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
      matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
    ];
  }
  const LIGHT = (() => {
    const v = [0.35, -0.55, 0.76];
    const n = Math.hypot(v[0], v[1], v[2]) || 1;
    return [v[0] / n, v[1] / n, v[2] / n];
  })();
  const BOX_FACES = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
                     [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]];
  function shadeColour(colour, intensity) {
    const rgb = Array.isArray(colour) && colour.length >= 3 ? colour : [0.6, 0.62, 0.68];
    const scale = Math.max(0, Math.min(1.25, intensity));
    return 'rgb(' + rgb.slice(0, 3).map((v) =>
      Math.max(0, Math.min(255, Math.round(v * 255 * scale)))).join(',') + ')';
  }
  function drawCloud() {
    const canvas = el.cloudCanvas;
    const ctx = canvas.getContext('2d');
    const width = canvas.width, height = canvas.height;
    ctx.fillStyle = '#04080d';
    ctx.fillRect(0, 0, width, height);
    const showSolid = solidVisible();
    const showCloud = cloudVisible() && cloud.points.length > 0;
    const showRobot = !!overlay.robot;
    if (!showSolid && !showCloud && !showRobot) {
      ctx.fillStyle = '#8899ab';
      ctx.font = '12px ui-monospace,monospace';
      const message = scene.layer === 'solid'
        ? '未加载实体世界；在“世界”下拉框选择 m1 / 楼梯 / two_floor。'
        : (scenePrimitiveCount() ? '该图层没有点云；切到“点云 + 实体”或“仅实体”。'
                                 : '该地图没有 3D 点云样本；可加载实体世界或订阅实时点云。');
      ctx.fillText(message, 16, 24);
      return;
    }
    fitView(false);
    const [cx, cy, cz] = cloud.center;
    const dist = cloud.dist;
    const cp = [cx + dist * Math.cos(cloud.yaw) * Math.cos(cloud.pitch),
                cy + dist * Math.sin(cloud.yaw) * Math.cos(cloud.pitch),
                cz + dist * Math.sin(cloud.pitch)];
    const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                             a[0] * b[1] - a[1] * b[0]];
    const norm = (a) => { const n = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };
    const forward = norm([cx - cp[0], cy - cp[1], cz - cp[2]]);
    const right = norm(cross(forward, [0, 0, 1]));
    const up = cross(right, forward);
    const focal = Math.min(width, height) * 0.9;
    const project = (point) => {
      const v = [point[0] - cp[0], point[1] - cp[1], point[2] - cp[2]];
      const zc = dot(v, forward);
      if (zc <= 0.05) return null;
      return { x: width / 2 + focal * dot(v, right) / zc,
               y: height / 2 - focal * dot(v, up) / zc, z: zc };
    };

    if (showSolid) {
      const bounds = scene.bounds || [-5, -5, 0, 5, 5, 2];
      const x0 = Math.floor(bounds[0] / 5) * 5 - 2;
      const x1 = Math.ceil(bounds[3] / 5) * 5 + 2;
      const y0 = Math.floor(bounds[1] / 5) * 5 - 2;
      const y1 = Math.ceil(bounds[4] / 5) * 5 + 2;
      const step = Math.max(x1 - x0, y1 - y0) > 40 ? 2 : 1;
      for (let x = x0; x <= x1; x += step) {
        const a = project([x, y0, 0]), b = project([x, y1, 0]);
        if (!a || !b) continue;
        ctx.strokeStyle = Math.abs(x % 5) < 1e-6 ? '#1b2c3b' : '#101c26';
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      }
      for (let y = y0; y <= y1; y += step) {
        const a = project([x0, y, 0]), b = project([x1, y, 0]);
        if (!a || !b) continue;
        ctx.strokeStyle = Math.abs(y % 5) < 1e-6 ? '#1b2c3b' : '#101c26';
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      }
    }

    const items = [];
    let robotScreen = null;
    const addFace = (worldPoints, colour) => {
      const projected = worldPoints.map(project);
      if (projected.some((point) => !point)) return;
      const centroid = [0, 0, 0];
      worldPoints.forEach((point) => {
        centroid[0] += point[0] / worldPoints.length;
        centroid[1] += point[1] / worldPoints.length;
        centroid[2] += point[2] / worldPoints.length;
      });
      const normal = norm(cross(sub(worldPoints[1], worldPoints[0]),
                                sub(worldPoints[2], worldPoints[0])));
      const intensity = 0.35 + 0.65 * Math.abs(dot(normal, LIGHT));
      items.push({
        kind: 'face',
        depth: dot(sub(centroid, cp), forward),
        pts: projected,
        colour: shadeColour(colour, intensity),
      });
    };
    if (showSolid) {
      scene.boxes.forEach((box) => {
        const half = [box.size[0] / 2, box.size[1] / 2, box.size[2] / 2];
        const corners = [
          [-half[0], -half[1], -half[2]], [half[0], -half[1], -half[2]],
          [half[0], half[1], -half[2]], [-half[0], half[1], -half[2]],
          [-half[0], -half[1], half[2]], [half[0], -half[1], half[2]],
          [half[0], half[1], half[2]], [-half[0], half[1], half[2]],
        ].map((point) => applyBoxMatrix(box.matrix, point));
        BOX_FACES.forEach((face) => addFace(face.map((index) => corners[index]), box.color));
      });
      scene.meshes.forEach((mesh) => {
        mesh.faces.forEach((face) => {
          const points = face.map((index) => mesh.vertices[index]);
          if (points.length === 3 && points.every(Boolean)) addFace(points, mesh.color);
        });
      });
    }
    if (showCloud) {
      const zspan = Math.max(cloud.zmax - cloud.zmin, 1e-3);
      cloud.points.forEach((point, index) => {
        const screen = project(point);
        if (!screen) return;
        let colour;
        if (cloud.colors[index]) {
          const rgb = cloud.colors[index];
          colour = 'rgb(' + rgb[0] + ',' + rgb[1] + ',' + rgb[2] + ')';
        } else {
          colour = 'rgb(' + cloudColour((point[2] - cloud.zmin) / zspan).join(',') + ')';
        }
        items.push({ kind: 'point', depth: screen.z, x: screen.x, y: screen.y, colour: colour });
      });
    }
    if (showRobot) {
      const robot = overlay.robot;
      const z = Number.isFinite(robot.z) ? robot.z : 0.15;
      const yaw = Number.isFinite(robot.yaw) ? robot.yaw : 0;
      const cosYaw = Math.cos(yaw), sinYaw = Math.sin(yaw);
      const local = (lx, ly, lz) => [
        robot.x + cosYaw * lx - sinYaw * ly,
        robot.y + sinYaw * lx + cosYaw * ly,
        z + lz,
      ];
      const robotBox = (cx, cy, cz, sx, sy, sz, colour) => {
        const half = [sx / 2, sy / 2, sz / 2];
        const corners = [
          [-half[0], -half[1], -half[2]], [half[0], -half[1], -half[2]],
          [half[0], half[1], -half[2]], [-half[0], half[1], -half[2]],
          [-half[0], -half[1], half[2]], [half[0], -half[1], half[2]],
          [half[0], half[1], half[2]], [-half[0], half[1], half[2]],
        ].map((p) => local(cx + p[0], cy + p[1], cz + p[2]));
        BOX_FACES.forEach((face) => addFace(face.map((index) => corners[index]), colour));
      };
      robotBox(0, 0, 0, 0.42, 0.31, 0.22, [0.98, 0.65, 0.16]);
      robotBox(0.23, 0, 0.02, 0.08, 0.20, 0.10, [0.98, 0.35, 0.18]);
      robotBox(0, 0, 0.225, 0.05, 0.05, 0.23, [0.65, 0.85, 0.95]);
      const arrowZ = Math.max(0.02, z - 0.13);
      const arrowStart = project([robot.x, robot.y, arrowZ]);
      const arrowTip = project(local(0.55, 0, arrowZ - z));
      if (arrowStart && arrowTip) {
        ctx.strokeStyle = '#7bf1a8';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(arrowStart.x, arrowStart.y);
        ctx.lineTo(arrowTip.x, arrowTip.y);
        ctx.stroke();
      }
      const label = project(local(0, 0, 0.55));
      robotScreen = {
        center: project(local(0, 0, 0)),
        heading: project(local(0.8, 0, 0)),
        top: project(local(0, 0, 0.8)),
        label: label,
      };
    }
    items.sort((a, b) => b.depth - a.depth);
    items.forEach((item) => {
      if (item.kind === 'point') {
        ctx.fillStyle = item.colour;
        ctx.fillRect(item.x - 0.9, item.y - 0.9, 1.8, 1.8);
        return;
      }
      ctx.beginPath();
      ctx.moveTo(item.pts[0].x, item.pts[0].y);
      for (let i = 1; i < item.pts.length; i++) ctx.lineTo(item.pts[i].x, item.pts[i].y);
      ctx.closePath();
      ctx.fillStyle = item.colour;
      ctx.strokeStyle = item.colour;
      ctx.lineWidth = 0.6;
      ctx.fill();
      ctx.stroke();
    });

    if (robotScreen) {
      if (robotScreen.center && robotScreen.heading) {
        ctx.strokeStyle = '#7bf1a8';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(robotScreen.center.x, robotScreen.center.y);
        ctx.lineTo(robotScreen.heading.x, robotScreen.heading.y);
        ctx.stroke();
      }
      if (robotScreen.center && robotScreen.top) {
        ctx.strokeStyle = 'rgba(255,209,102,.7)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(robotScreen.center.x, robotScreen.center.y);
        ctx.lineTo(robotScreen.top.x, robotScreen.top.y);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      if (robotScreen.center) {
        ctx.beginPath();
        ctx.arc(robotScreen.center.x, robotScreen.center.y, 10, 0, Math.PI * 2);
        ctx.strokeStyle = '#ffd166';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.fillStyle = 'rgba(255,209,102,.18)';
        ctx.fill();
        ctx.fillStyle = '#ffd166';
        ctx.font = 'bold 11px ui-monospace,monospace';
        ctx.fillText('载体', robotScreen.center.x + 13, robotScreen.center.y - 8);
      }
    }

    ctx.fillStyle = 'rgba(220,235,245,.78)';
    ctx.font = '11px ui-monospace,monospace';
    const z0 = showCloud && cloud.points.length ? cloud.zmin : (scene.bounds ? scene.bounds[2] : 0);
    const z1 = showCloud && cloud.points.length ? cloud.zmax : (scene.bounds ? scene.bounds[5] : 1);
    const layers = [];
    if (showSolid) layers.push(scene.boxes.length + ' box / ' + scene.triCount + ' 面');
    if (showCloud) layers.push(cloud.points.length + ' pts');
    const layerName = { both: '点云+实体', solid: '仅实体', cloud: '仅点云' }[scene.layer] || scene.layer;
    ctx.fillText('z ' + z0.toFixed(2) + '…' + z1.toFixed(2) + ' m · ' + layers.join(' + ') +
                 ' · ' + layerName, 10, 18);
    ctx.fillStyle = 'rgba(255,209,102,.9)';
    ctx.fillText(overlay.robot
      ? '载体 x=' + overlay.robot.x.toFixed(2) + ' y=' + overlay.robot.y.toFixed(2) +
        ' z=' + (Number.isFinite(overlay.robot.z) ? overlay.robot.z.toFixed(2) : '—') +
        '（Gazebo 方盒载体，无腿）'
      : '载体位姿：未收到 TF map→base_link', 10, 34);
    if (cloud.mode === 'server' && cloud.note) {
      ctx.fillStyle = 'rgba(255,179,191,.85)';
      ctx.fillText(cloud.note, 10, 50);
    }
  }
  function bindCloud() {
    const canvas = el.cloudCanvas;
    let drag = null;
    canvas.addEventListener('pointerdown', (event) => {
      drag = { x: event.clientX, y: event.clientY };
      canvas.setPointerCapture(event.pointerId);
      canvas.style.cursor = 'grabbing';
    });
    canvas.addEventListener('pointermove', (event) => {
      if (!drag) return;
      const dx = event.clientX - drag.x, dy = event.clientY - drag.y;
      drag = { x: event.clientX, y: event.clientY };
      cloud.yaw -= dx * 0.008;
      cloud.pitch = Math.max(0.08, Math.min(1.45, cloud.pitch + dy * 0.006));
      drawCloud();
    });
    const end = () => { drag = null; canvas.style.cursor = 'grab'; };
    canvas.addEventListener('pointerup', end);
    canvas.addEventListener('pointercancel', end);
    canvas.addEventListener('wheel', (event) => {
      event.preventDefault();
      cloud.dist *= Math.exp(event.deltaY * 0.0015);
      drawCloud();
    }, { passive: false });
    el.cloudWrap.addEventListener('toggle', () => { if (el.cloudWrap.open) drawCloud(); });
    el.view3dJump.addEventListener('click', () => {
      el.cloudWrap.open = true;
      el.cloudWrap.scrollIntoView({ behavior: 'smooth', block: 'center' });
      drawCloud();
    });
    el.cloudReset.addEventListener('click', () => {
      fitView(true);
      drawCloud();
    });
    el.cloudLayer.addEventListener('change', () => {
      scene.layer = el.cloudLayer.value;
      cloud.fitPending = true;
      updateCloudSummary();
      drawCloud();
    });
    el.worldScene.addEventListener('change', () => loadWorldScene(el.worldScene.value));
    el.cloudSource.addEventListener('change', () => {
      if (el.cloudSource.value === 'server') startServerVoxels();
      else stopServerVoxels();
    });
    el.cloudLive.addEventListener('click', () => {
      if (serverVox.enabled) {
        log('当前是服务端体素源；先切回“浏览器抽稀”再订阅 rosbridge', 'err');
        return;
      }
      if (live.enabled) stopLiveCloud(); else startLiveCloud();
    });
    el.cloudTopic.addEventListener('change', () => {
      if (live.enabled) { stopLiveCloud(false); startLiveCloud(); }
    });
    el.vizBackend.addEventListener('change', () => applyVizMode(el.vizBackend.value, true));
  }
  function setLiveState(text, cls) {
    el.cloudLiveState.textContent = text;
    el.cloudLiveState.className = 'pill ' + (cls || '');
  }
  function startLiveCloud() {
    if (serverVox.enabled) { log('当前是服务端体素源；先切回“浏览器抽稀”', 'err'); return; }
    if (!bridge || !online) { log('rosbridge 未连接，不能订阅实时点云', 'err'); return; }
    const topic = el.cloudTopic.value;
    live.enabled = true;
    live.topic = topic;
    live.count = 0; live.points = 0; live.last = 0; live.window = performance.now();
    try {
      bridge.subscribe(topic, 'sensor_msgs/PointCloud2', onLiveCloud, 500);
    } catch (error) {
      log('订阅 ' + topic + ' 失败：' + error.message, 'err');
      live.enabled = false;
      return;
    }
    el.cloudLive.textContent = '停止实时点云';
    setLiveState('等待点云', '');
    el.cloudLiveRate.textContent = '';
    el.cloudWrap.open = true;
    cloud.points = [];
    cloud.colors = [];
    cloud.mode = 'live';
    cloud.note = topic === '/rslidar_points' ? '原始点云为传感器系，与实体叠加仅示意' : '';
    cloud.fitPending = true;
    updateCloudSummary();
    drawCloud();
    setTimeout(() => {
      if (live.enabled && !live.points) {
        const hint = live.topic === '/cloud_registered'
          ? '无消息：先启 mapping_m1（依赖 world 栈），或改用 /rslidar_points'
          : '无消息：先启一个 Gazebo world 栈（sim_m1）';
        setLiveState(live.topic === '/cloud_registered' ? '无消息 · 需 mapping_m1' : '无消息 · 需 world 栈', 'err');
        log(hint, 'err');
      }
    }, 4000);
  }
  function stopLiveCloud(restore) {
    if (live.topic && bridge) {
      try { bridge.unsubscribe(live.topic); } catch (error) {}
    }
    live.enabled = false;
    live.topic = null;
    el.cloudLive.textContent = '订阅实时点云';
    setLiveState('离线', '');
    el.cloudLiveRate.textContent = '';
    cloud.mode = 'map';
    cloud.note = '';
    if (restore !== false) loadCloud(current); else drawCloud();
  }
  function onLiveCloud(msg) {
    live.count += 1;
    live.last = performance.now();
    const fields = msg.fields || [];
    const find = (name) => fields.findIndex((field) => field.name === name);
    const ix = find('x'), iy = find('y'), iz = find('z');
    const n = (msg.width || 0) * (msg.height || 1);
    if (ix < 0 || iy < 0 || iz < 0 || !n || !msg.data) return;
    const step = msg.point_step;
    if (!step) return;
    let bytes;
    if (typeof msg.data === 'string') {
      const binary = atob(msg.data);
      bytes = new Uint8Array(binary.length);
      for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
    } else {
      bytes = Uint8Array.from(msg.data);
    }
    const view = new DataView(bytes.buffer);
    const stride = Math.max(1, Math.floor(n / 8000));
    const points = [];
    let zmin = Infinity, zmax = -Infinity;
    for (let i = 0; i < n; i += stride) {
      const offset = i * step;
      const x = view.getFloat32(offset + fields[ix].offset, true);
      const y = view.getFloat32(offset + fields[iy].offset, true);
      const z = view.getFloat32(offset + fields[iz].offset, true);
      if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
      points.push([x, y, z]);
      if (z < zmin) zmin = z;
      if (z > zmax) zmax = z;
    }
    if (!points.length) return;
    const first = !cloud.points.length || cloud.mode !== 'live';
    const frame = msg.header && msg.header.frame_id;
    cloud.points = points;
    cloud.colors = [];
    cloud.zmin = zmin; cloud.zmax = zmax;
    cloud.mode = 'live';
    cloud.note = live.topic === '/rslidar_points'
      ? '原始点云为传感器系，与实体叠加仅示意'
      : (frame && frame !== 'map' && frame !== 'world'
        ? '帧 ' + frame + '：与实体叠加仅示意' : '');
    if (first) cloud.fitPending = true;
    live.points = points.length;
    setLiveState('实时', 'on');
    el.cloudLiveRate.textContent = points.length + ' pts · ' + live.count + ' msg';
    updateCloudSummary();
    drawCloud();
  }
  function setServerControls(enabled) {
    el.cloudTopic.disabled = enabled;
    el.cloudLive.disabled = enabled;
  }
  function startServerVoxels() {
    if (live.enabled) stopLiveCloud(false);
    serverVox.enabled = true;
    serverVox.stamp = null;
    serverVox.points = 0;
    serverVox.error = '';
    serverVox.stale = false;
    setServerControls(true);
    setLiveState('服务端等待', '');
    cloud.note = '';
    updateCloudSummary();
    el.cloudWrap.open = true;
    if (serverVox.timer) clearInterval(serverVox.timer);
    serverVox.timer = setInterval(pollServerVoxels, 600);
    pollServerVoxels();
    log('服务端体素源已启用；先启动 voxel_stream 栈，或等待已有快照', 'ok');
  }
  function stopServerVoxels() {
    if (serverVox.timer) clearInterval(serverVox.timer);
    serverVox.timer = null;
    serverVox.enabled = false;
    setServerControls(false);
    setLiveState('离线', '');
    el.cloudLiveRate.textContent = '';
    cloud.colors = [];
    cloud.note = '';
    loadCloud(current);
  }
  async function pollServerVoxels() {
    if (!serverVox.enabled || serverVox.fetching) return;
    serverVox.fetching = true;
    try {
      const metaResponse = await fetch(agentBase() + '/api/voxels/meta', { cache: 'no-store' });
      if (!metaResponse.ok) {
        throw new Error(metaResponse.status === 404
          ? '无快照（先启动 voxel_stream 栈）' : 'HTTP ' + metaResponse.status);
      }
      const meta = await metaResponse.json();
      serverVox.meta = meta;
      if (serverVox.stamp !== meta.stamp || serverVox.points !== meta.count) {
        const binaryResponse = await fetch(agentBase() + '/api/voxels/latest', { cache: 'no-store' });
        if (!binaryResponse.ok) throw new Error('二进制 HTTP ' + binaryResponse.status);
        const parsed = parseVoxelBinary(await binaryResponse.arrayBuffer());
        const first = serverVox.stamp == null || !cloud.points.length;
        serverVox.stamp = meta.stamp;
        serverVox.points = parsed.points.length;
        cloud.points = parsed.points;
        cloud.colors = parsed.colors;
        cloud.zmin = parsed.zmin;
        cloud.zmax = parsed.zmax;
        cloud.mode = 'server';
        cloud.note = parsed.frame && parsed.frame !== 'map' && parsed.frame !== 'world'
          ? '帧 ' + parsed.frame + '：与实体叠加仅示意'
          : '';
        if (first) cloud.fitPending = true;
        updateCloudSummary();
        drawCloud();
      }
      const age = meta.written ? Math.max(0, Date.now() / 1000 - meta.written) : null;
      if (age != null && age > 3) {
        setLiveState('快照 ' + age.toFixed(1) + ' s', 'err');
        if (!serverVox.stale) {
          serverVox.stale = true;
          log('服务端体素快照已 ' + age.toFixed(1) + ' s 未更新（voxel_stream 已停？）', 'err');
        }
      } else {
        serverVox.stale = false;
        setLiveState('服务端 ' + meta.count + ' 点', 'on');
      }
      el.cloudLiveRate.textContent = 'leaf ' + meta.leaf + ' m · ' +
        (age == null ? '—' : age.toFixed(1) + ' s') + (meta.source ? ' · ' + meta.source : '');
      serverVox.error = '';
    } catch (error) {
      setLiveState('服务端离线', 'err');
      if (serverVox.error !== error.message) {
        serverVox.error = error.message;
        log('服务端体素：' + error.message, 'err');
      }
    } finally {
      serverVox.fetching = false;
    }
  }
  function parseVoxelBinary(buffer) {
    const view = new DataView(buffer);
    const magic = String.fromCharCode(view.getUint8(0), view.getUint8(1),
                                      view.getUint8(2), view.getUint8(3));
    if (magic !== 'V8VX') throw new Error('体素快照 magic 不符');
    const count = view.getUint32(8, true);
    const frameLength = view.getUint16(48, true);
    const frameEnd = 50 + frameLength;
    if (buffer.byteLength < frameEnd + count * 16) throw new Error('体素快照长度不足');
    const frame = new TextDecoder('utf-8').decode(new Uint8Array(buffer, 50, frameLength));
    const points = new Array(count);
    const colors = new Array(count);
    let zmin = Infinity, zmax = -Infinity;
    for (let i = 0; i < count; i++) {
      const offset = frameEnd + i * 16;
      const x = view.getFloat32(offset, true);
      const y = view.getFloat32(offset + 4, true);
      const z = view.getFloat32(offset + 8, true);
      points[i] = [x, y, z];
      colors[i] = [view.getUint8(offset + 12), view.getUint8(offset + 13),
                   view.getUint8(offset + 14)];
      if (z < zmin) zmin = z;
      if (z > zmax) zmax = z;
    }
    return {
      points: points,
      colors: colors,
      frame: frame,
      zmin: isFinite(zmin) ? zmin : 0,
      zmax: isFinite(zmax) ? zmax : 1,
    };
  }

  // ---------------------------------------------------------------- goal editing
  function canvasPoint(event) {
    const rect = el.canvas.getBoundingClientRect();
    const cx = (event.clientX - rect.left) * el.canvas.width / rect.width;
    const cy = (event.clientY - rect.top) * el.canvas.height / rect.height;
    return pixelToMap(cx, cy);
  }
  function addGoal(point) {
    if (mission.active) { log('任务运行中不能加目标；先取消任务', 'err'); return; }
    const goal = {
      x: point.x, y: point.y, z: el.mode.value === '2d' ? 0 : surfaceZ(),
      yaw: parseFloat(el.goalYaw.value) || 0, state: '待发',
    };
    goals.push(goal);
    selected = goals.length - 1;
    renderGoals();
    paintGoals();
    updateMissionState();
    log('加目标 #' + goals.length + ' (' + fmt(goal.x, 2) + ', ' + fmt(goal.y, 2) + ', z=' + fmt(goal.z, 2) + ')');
  }
  function setStart(point) {
    if (mission.active) { log('任务运行中不能改起点；先取消任务', 'err'); return; }
    start = {
      x: point.x, y: point.y, z: el.mode.value === '2d' ? 0 : surfaceZ(),
      yaw: parseFloat(el.goalYaw.value) || 0,
    };
    renderGoals();
    paintGoals();
    updateMissionState();
    publishStart(true);
  }
  function renderGoals() {
    const rows = [];
    if (start) {
      rows.push('<tr data-start="1"><td>S</td><td>起点</td><td>' + fmt(start.x, 3) + '</td><td>' +
        fmt(start.y, 3) + '</td><td>' + fmt(start.z, 2) + '</td><td>' + fmt(start.yaw, 1) +
        '</td><td>已设</td><td>—</td><td>—</td><td>—</td></tr>');
    }
    goals.forEach((goal, index) => {
      rows.push('<tr data-index="' + index + '"' + (index === selected ? ' class="sel"' : '') +
        '><td>' + (index + 1) + '</td><td>目标</td><td>' + fmt(goal.x, 3) + '</td><td>' +
        fmt(goal.y, 3) + '</td><td>' + fmt(goal.z, 2) + '</td><td>' + fmt(goal.yaw, 1) +
        '</td><td>' + escapeHtml(goal.state) + '</td>' +
        '<td>' + (goal.metrics && goal.metrics.duration != null ? goal.metrics.duration.toFixed(1) + ' s' : '—') + '</td>' +
        '<td>' + (goal.metrics && goal.metrics.path != null ? goal.metrics.path.toFixed(2) + ' m' : '—') + '</td>' +
        '<td>' + (goal.metrics && goal.metrics.error != null ? goal.metrics.error.toFixed(2) + ' m' : '—') + '</td></tr>');
    });
    el.goals.innerHTML = rows.join('') || '<tr><td colspan="10" style="color:#8899ab">还没有目标；在地图上点击添加</td></tr>';
  }
  function publishStart(fromClick) {
    if (!start) return;
    if (el.mode.value === '2d') {
      const msg = {
        header: { frame_id: 'map', stamp: stamp() },
        pose: {
          pose: poseStamped(start.x, start.y, 0, start.yaw).pose,
          covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.06,0,0,0,
                       0,0,0,0.0685,0,0, 0,0,0,0,0.0685,0, 0,0,0,0,0,0.0685],
        },
      };
      if (pub('/initialpose', 'geometry_msgs/PoseWithCovarianceStamped', msg)) {
        log((fromClick ? '设起点并' : '') + '发 /initialpose', 'ok');
      }
    } else {
      const msg = poseStamped(start.x, start.y, start.z, start.yaw);
      if (pub('/v8/3d/global/start', 'geometry_msgs/PoseStamped', msg)) {
        log((fromClick ? '设起点并' : '') + '发 /v8/3d/global/start', 'ok');
      }
    }
  }
  function goalOwner() {
    const owner = stackSnapshot.find((stack) =>
      stack.goal_authority === '3d' && stack.state === 'running');
    return owner ? owner.name : null;
  }
  function publishGoal(goal) {
    if (el.mode.value === '2d') {
      const msg = poseStamped(goal.x, goal.y, 0, goal.yaw);
      if (pub('/move_base_simple/goal', 'geometry_msgs/PoseStamped', msg)) {
        goal.state = '已发 2D'; log('发 /move_base_simple/goal (' + fmt(goal.x, 2) + ', ' + fmt(goal.y, 2) + ')', 'ok');
        return true;
      }
    } else {
      if (!goalOwner()) {
        goal.state = '失败';
        log('3D 目标写入权在 bt_console；先启动作业栈“行为树控制台”', 'err');
        return false;
      }
      if (start) publishStart(false);
      else log('提示：3D 未设起点，规划器可能用当前位姿；可先“设起点”', 'err');
      const msg = poseStamped(goal.x, goal.y, goal.z, goal.yaw);
      if (pub('/v8/bt/goal', 'geometry_msgs/PoseStamped', msg)) {
        goal.state = '已交 BT'; log('发 /v8/bt/goal z=' + fmt(goal.z, 2) + '（BT 拥有 3D goal 写入权）', 'ok');
        return true;
      }
    }
    goal.state = '失败';
    return false;
  }
  function guardHint(action) {
    const mapWorld = current && current.world ? current.world : '—';
    return '先确认“地图与运行 world 一致”再' + action +
      '（地图 world：' + mapWorld + '；作业栈 world 见右栏，无运行 world 时不要勾选）';
  }
  function sendSelected() {
    if (mission.active) { log('任务运行中不能单发；先取消任务', 'err'); return; }
    if (selected < 0 || !goals[selected]) { log('先在地图上加一个目标', 'err'); return; }
    if (!el.mapGuard.checked) { log(guardHint('发送'), 'err'); return; }
    publishGoal(goals[selected]);
    renderGoals();
  }
  function sendAll() {
    if (mission.active) stopMission('重新启动');
    if (!goals.length) { log('没有目标', 'err'); return; }
    if (!el.mapGuard.checked) { log(guardHint('发任务'), 'err'); return; }
    mission.active = true;
    mission.queue = goals;
    mission.index = 0;
    mission.goal = null;
    mission.mode = el.mode.value;
    mission.timeoutS = Math.max(10, parseFloat(el.goalTimeout.value) || 60);
    mission.continueOnFail = !!el.missionContinue.checked;
    goals.forEach((goal) => { goal.state = '待发'; goal.metrics = {}; });
    log('任务开始：' + goals.length + ' 目标 ｜ 模式 ' + mission.mode +
        ' ｜ 逐目标超时 ' + mission.timeoutS + ' s ｜ 失败' +
        (mission.continueOnFail ? '继续' : '停止'));
    if (mission.mode === '3d') {
      log('3D 到点判据＝BT /v8/bt/status 回执（可视化载体位姿只用于统计，不再判到点）');
    }
    sendMissionGoal();
  }
  function sendMissionGoal() {
    if (!mission.active) return;
    if (mission.index >= goals.length) { finishMission('全部结束'); return; }
    const goal = goals[mission.index];
    mission.goal = goal;
    mission.sentAt = performance.now();
    mission.sentSecs = Date.now() / 1000;
    mission.btSentSeq = btStatus.parsed ? btStatus.parsed.current_seq : 0;
    mission.btHandledSeq = null;
    mission.pathM = 0;
    mission.lastPose = null;
    mission.arrivalSince = 0;
    mission.error = null;
    goal.state = '发送中';
    goal.metrics = {};
    renderGoals();
    updateMissionState();
    if (!publishGoal(goal)) { missionGoalDone(false, '发送失败'); return; }
    goal.state = '发送中';
    renderGoals();
    log('任务目标 ' + (mission.index + 1) + '/' + goals.length + ' 发送 (' +
        fmt(goal.x, 2) + ', ' + fmt(goal.y, 2) + ')');
  }
  function missionTick() {
    if (!mission.active || !mission.goal) return;
    const now = performance.now();
    if (now - mission.sentAt > mission.timeoutS * 1000) {
      missionGoalDone(false, '超时'); return;
    }
    const robot = overlay.robot;
    if (robot) {
      if (mission.lastPose) {
        mission.pathM += Math.hypot(robot.x - mission.lastPose.x, robot.y - mission.lastPose.y);
      }
      mission.lastPose = { x: robot.x, y: robot.y };
      const goal = mission.goal;
      mission.error = Math.hypot(goal.x - robot.x, goal.y - robot.y);
      // 3D mode: arrival is owned by the BT /v8/bt/status events.  The
      // visualised carrier pose is used for metrics only, never as a verdict.
      if (mission.mode !== '3d') {
        if (mission.error <= 0.25) {
          if (!mission.arrivalSince) mission.arrivalSince = now;
          else if (now - mission.arrivalSince >= 500) missionGoalDone(true, '到点');
        } else {
          mission.arrivalSince = 0;
        }
      }
    }
    updateMissionState();
  }
  function missionGoalDone(ok, reason) {
    if (!mission.goal) return;
    const goal = mission.goal;
    const duration = (performance.now() - mission.sentAt) / 1000;
    goal.state = ok ? '成功' : (reason === '超时' ? '超时' : '失败');
    goal.metrics = { duration: duration, path: mission.pathM, error: mission.error };
    mission.goal = null;
    mission.arrivalSince = 0;
    renderGoals();
    updateMissionState();
    log('任务目标 ' + (mission.index + 1) + ' ' + goal.state + '：' + reason +
        ' ｜ 耗时 ' + duration.toFixed(1) + ' s ｜ 路径 ' + mission.pathM.toFixed(2) +
        ' m ｜ 终点误差 ' + fmt(mission.error, 2) + ' m', ok ? 'ok' : 'err');
    if (ok || mission.continueOnFail) {
      mission.index += 1;
      if (mission.index < goals.length) {
        setTimeout(() => { if (mission.active) sendMissionGoal(); }, 500);
      } else {
        finishMission('全部结束');
      }
    } else {
      finishMission('失败停止');
    }
  }
  function finishMission(reason) {
    const wasActive = mission.active;
    mission.active = false;
    mission.queue = [];
    mission.goal = null;
    mission.index = -1;
    mission.lastPose = null;
    mission.btSentSeq = 0;
    mission.btHandledSeq = null;
    renderGoals();
    updateMissionState();
    if (wasActive) log('任务结束：' + reason);
  }
  function stopMission(reason) {
    if (!mission.active) return;
    if (mission.goal) {
      mission.goal.state = '取消';
      mission.goal.metrics = {
        duration: (performance.now() - mission.sentAt) / 1000,
        path: mission.pathM, error: mission.error,
      };
    }
    mission.active = false;
    mission.goal = null;
    mission.index = -1;
    mission.queue = [];
    mission.lastPose = null;
    mission.btSentSeq = 0;
    mission.btHandledSeq = null;
    renderGoals();
    updateMissionState();
    log('任务停止：' + reason, 'err');
    if (el.mode.value === '2d') cancel2d();
  }
  function requestRobotCloudDraw() {
    if (!el.cloudWrap || !el.cloudWrap.open) return;
    const now = performance.now();
    if (now - robotCloudLast < 200) return;
    robotCloudLast = now;
    drawCloud();
  }
  function setRobotPose(x, y, yaw, z) {
    overlay.robot = { x: x, y: y, z: z == null ? 0 : z, yaw: yaw == null ? 0 : yaw };
    requestRobotCloudDraw();
  }
  function checkMapGuard() {
    if (!current) {
      el.mapGuardState.textContent = '未选地图';
      el.mapGuardState.className = 'pill';
      return;
    }
    if (current.kind === '2d_grid' && overlay.mapMsg && overlay.mapMsg.info) {
      const info = overlay.mapMsg.info;
      const same = info.width === current.width && info.height === current.height &&
        Math.abs(info.resolution - current.resolution) < 1e-6 &&
        Math.abs(info.origin.position.x - current.origin[0]) < 1e-3 &&
        Math.abs(info.origin.position.y - current.origin[1]) < 1e-3;
      el.mapGuard.checked = same;
      el.mapGuardState.textContent = same ? '2D /map 一致' : '2D /map 与所选地图不一致';
      el.mapGuardState.className = 'pill ' + (same ? 'on' : 'err');
      return;
    }
    el.mapGuardState.textContent = current.kind === '2d_grid'
      ? '未收到 /map；请人工确认' : '3D/点云：请人工确认地图与 world 一致';
    el.mapGuardState.className = 'pill';
  }
  function cancel2d() {
    if (!bridge || !online) { log('未连接，无法取消', 'err'); return; }
    bridge.callService('/move_base/cancel', {}, (response) => {
      log(response.result ? '已发 /move_base/cancel' : '取消服务不可用/超时（需 2D MBF 在跑）',
          response.result ? 'ok' : 'err');
    });
  }

  // ---------------------------------------------------------------- stack lifecycle (host agent)
  function agentBase() {
    const value = (el.agentUrl.value || '').trim().replace(/\/$/, '');
    return value || ('http://' + (location.hostname || '127.0.0.1') + ':8780');
  }
  function setClientRole(local, reason) {
    const known = local === true || local === false;
    clientIdentity.local = known ? local : null;
    clientIdentity.checked = known;
    clientIdentity.reason = reason || '';
    document.body.classList.toggle('local', local === true);
    document.body.classList.toggle('remote', local === false);
    document.body.classList.toggle('unknown', !known);
    el.clientRole.textContent = local === true ? '客户端：本机' :
      (local === false ? '客户端：远程 · 本机功能锁定' : '客户端：未知 · 本机功能锁定');
    el.clientRole.className = 'pill ' + (local === true ? 'on' : 'err');
    el.localRoleHint.textContent = local === true ? '本机可用' : '远程锁定';
    el.localNote.textContent = local === true
      ? '本机可用：以下命令在宿主桌面/终端执行。'
      : '远程锁定：以下命令只能在宿主桌面/终端执行；远程请用任务台。';
  }
  async function loadClientRole() {
    if (clientRoleTimer) { clearTimeout(clientRoleTimer); clientRoleTimer = null; }
    try {
      const response = await fetch(agentBase() + '/api/whoami', { cache: 'no-store' });
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const payload = await response.json();
      const local = payload.local === true ? true : payload.local === false ? false : null;
      const changed = clientIdentity.local !== local || clientIdentity.reason !== (payload.reason || '');
      setClientRole(local, payload.reason || '');
      if (changed) {
        log('客户端身份：' + (local === true ? '本机' : local === false ? '远程' : '未知') +
            '（' + (payload.reason || '?') + (payload.client ? ' · ' + payload.client : '') + '）',
            local === true ? 'ok' : '');
      }
    } catch (error) {
      setClientRole(null, 'agent 不可达');
      clientRoleTimer = setTimeout(loadClientRole, 5000);
    }
  }
  function setAgentState(text, cls) {
    el.agentState.textContent = text;
    el.agentState.className = 'pill ' + (cls || '');
  }
  async function refreshStacks() {
    try {
      const response = await fetch(agentBase() + '/api/stacks', { cache: 'no-store' });
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const payload = await response.json();
      stackSnapshot = payload.stacks || [];
      setAgentState('agent 在线', 'on');
      renderStacks(stackSnapshot);
    } catch (error) {
      setAgentState('agent 未连接', 'err');
      el.stackList.innerHTML = '<div class="stack-empty">agent 未连接（' +
        escapeHtml(agentBase()) + '）：先运行 <code>bash scripts/console/start_agent.sh</code></div>';
    }
  }
  function renderStacks(stacks) {
    if (!stacks.length) {
      el.stackList.innerHTML = '<div class="stack-empty">白名单为空</div>';
      return;
    }
    el.stackList.innerHTML = stacks.map((stack) => {
      const stateText = stack.state === 'running' ? '运行中' :
        stack.state === 'stopped' ? '已停止' : stack.state;
      const stateClass = 'pill stack-' + escapeHtml(stack.state);
      const locked = stack.local_only && clientIdentity.local !== true;
      const choices = stack.world_choices || [];
      const selectedWorld = stack.selected_world || stack.world || '';
      const worldSelect = choices.length
        ? '<select data-world="' + escapeHtml(stack.name) + '" title="Gazebo world"' +
          (locked || stack.state === 'running' ? ' disabled' : '') + '>' +
          choices.map((choice) => '<option value="' + escapeHtml(choice.file) + '"' +
            (choice.file === selectedWorld ? ' selected' : '') + '>' +
            escapeHtml(choice.label) + '</option>').join('') + '</select>'
        : '';
      return '<div class="stack-row">' +
        '<div class="stack-meta"><b>' + escapeHtml(stack.label) + '</b>' +
        '<span>' + escapeHtml(stack.description || '') + '</span>' +
        '<span>' + escapeHtml(stack.notes || '') +
        (stack.world ? ' ｜ world：' + escapeHtml(stack.selected_world || stack.world) : '') +
        (stack.requires && stack.requires.length ? ' ｜ 依赖：' + escapeHtml(stack.requires.join(', ')) : '') +
        '</span></div>' +
        '<div class="stack-actions"><span class="' + stateClass + '">' + escapeHtml(stateText) + '</span>' +
        (locked ? '<span class="pill err">仅本机</span>' : '') + worldSelect +
        '<button data-stack="' + escapeHtml(stack.name) + '" data-action="start"' +
        (locked || stack.state === 'running' ? ' disabled' : '') + '>启动</button>' +
        '<button data-stack="' + escapeHtml(stack.name) + '" data-action="stop"' +
        (locked || stack.state !== 'running' ? ' disabled' : '') + '>停止</button>' +
        '<button data-stack="' + escapeHtml(stack.name) + '" data-action="log">日志</button>' +
        '</div></div>';
    }).join('');
  }
  async function stackAction(name, action, world) {
    if (action === 'log') {
      try {
        const response = await fetch(agentBase() + '/api/stacks/' + encodeURIComponent(name) + '/log?tail=80');
        const payload = await response.json();
        el.stackLog.textContent = payload.log || '（空日志）';
      } catch (error) {
        el.stackLog.textContent = '日志读取失败：' + error.message;
      }
      return;
    }
    el.stackLog.textContent = '请求 ' + action + ' ' + name +
      (world ? ' world=' + world : '') + ' …';
    try {
      const options = { method: 'POST' };
      if (action === 'start' && world) {
        options.headers = { 'Content-Type': 'application/json' };
        options.body = JSON.stringify({ world: world });
      }
      const response = await fetch(agentBase() + '/api/stacks/' + encodeURIComponent(name) + '/' + action,
        options);
      const payload = await response.json();
      el.stackLog.textContent = (payload.message || payload.error || '') + '\n';
      await refreshStacks();
      if (action === 'start') {
        const logResponse = await fetch(agentBase() + '/api/stacks/' + encodeURIComponent(name) + '/log?tail=40');
        const logPayload = await logResponse.json();
        el.stackLog.textContent += (logPayload.log || '').slice(-2000);
      }
    } catch (error) {
      el.stackLog.textContent = action + ' 失败：' + error.message;
    }
  }
  function bindStacks() {
    el.agentUrl.value = 'http://' + (location.hostname || '127.0.0.1') + ':8780';
    el.agentRefresh.addEventListener('click', refreshStacks);
    el.agentUrl.addEventListener('change', () => {
      refreshStacks();
      loadClientRole();
    });
    el.stackList.addEventListener('click', (event) => {
      const button = event.target.closest('button[data-stack]');
      if (!button) return;
      const row = button.closest('.stack-row');
      const worldSelect = row ? row.querySelector('select[data-world]') : null;
      stackAction(button.dataset.stack, button.dataset.action,
                  worldSelect ? worldSelect.value : '');
    });
    setInterval(refreshStacks, 4000);
    refreshStacks();
  }

  // ---------------------------------------------------------------- viz backend
  // 网页内建 / Foxglove 二选一。Foxglove 模式停网页侧重载，只留 /v8/bt/status。
  const VIZ_HEAVY_TOPICS = TOPIC_SPECS.map((spec) => spec.topic)
    .filter((topic) => topic !== '/v8/bt/status');
  let vizMode = 'web';

  function applyVizMode(mode, userAction) {
    vizMode = mode === 'foxglove' ? 'foxglove' : 'web';
    try { localStorage.setItem('lite3-viz-mode', vizMode); } catch (e) {}
    const fox = vizMode === 'foxglove';
    el.vizBackend.value = vizMode;
    if (el.cloudWrap) el.cloudWrap.hidden = fox;
    el.foxgloveGuide.hidden = !fox;
    const url = (el.bridgeUrl.value ||
                 ('ws://' + (location.hostname || '127.0.0.1') + ':9090')).trim();
    el.foxgloveWs.textContent = url;
    if (!userAction) return;
    if (fox) {
      if (serverVox.enabled) stopServerVoxels();
      if (live.enabled) stopLiveCloud(false);
      if (bridge && online) {
        VIZ_HEAVY_TOPICS.forEach((topic) => {
          try { bridge.unsubscribe(topic); } catch (e) {}
        });
      }
      fetch(agentBase() + '/api/stacks/voxel_stream/stop', { method: 'POST' })
        .catch(() => {});
      log('已切到 Foxglove：停服务端体素流/实时点云/3D 与重载订阅（保留 /v8/bt/status）；'
          + 'rosbridge 保持给 Foxglove 用', 'ok');
    } else {
      if (bridge && online) setupSubscriptions();
      log('已切回网页内建可视化；体素流如需请到作业栈重启 voxel_stream', 'ok');
    }
  }

  function setupSubscriptions() {
    TOPIC_SPECS.forEach((spec) => {
      if (vizMode === 'foxglove' && VIZ_HEAVY_TOPICS.indexOf(spec.topic) >= 0) return;
      rates[spec.topic] = { count: 0, last: 0, window: performance.now() };
      try {
        bridge.subscribe(spec.topic, spec.type, (msg) => onTopic(spec.topic, msg));
      } catch (e) { log('subscribe 失败 ' + spec.topic + ': ' + e.message, 'err'); }
    });
    renderRates();
  }
  function onTopic(topic, msg) {
    const rate = rates[topic];
    if (!rate) return;
    rate.count += 1;
    rate.last = performance.now();
    if (topic === '/v8/local/pose') {
      if (msg.pose && msg.pose.position) {
        setRobotPose(msg.pose.position.x, msg.pose.position.y,
                     quatYaw(msg.pose.orientation), msg.pose.position.z);
        draw();
      }
    } else if (topic === '/map') {
      overlay.mapMsg = msg;
      checkMapGuard();
    } else if (topic === '/move_base/global_costmap/costmap') {
      overlay.costmapGlobal = msg;
      draw();
    } else if (topic === '/move_base/local_costmap/costmap') {
      overlay.costmapLocal = msg;
      draw();
    } else if (topic === '/tf' || topic === '/tf_static') {
      tfCount += 1;
      (msg.transforms || []).forEach((transform) => {
        const parent = String(transform.header && transform.header.frame_id || '').replace(/^\//, '');
        const child = String(transform.child_frame_id || '').replace(/^\//, '');
        if (parent && child) {
          tfPairs.set(parent + '→' + child, true);
          overlay.tf[parent + '|' + child] = transform.transform || {};
        }
      });
      el.mTf.textContent = tfPairs.size + ' 对（' + tfCount + ' 条消息）';
      el.tfTree.textContent = Array.from(tfPairs.keys()).sort().slice(0, 120).join('\n') || '（尚无 TF）';
      computeRobot();
      draw();
    } else if (topic === '/v8/3d/global/path') {
      overlay.global = msg;
      updatePathState();
      draw();
    } else if (topic === '/v8/local/trajectory') {
      overlay.local = msg;
      updatePathState();
      draw();
    } else if (topic === '/move_base/status') {
      updateMbf(msg);
    } else if (topic === '/v8/bt/status') {
      onBtStatus(msg);
    }
  }
  function onBtStatus(msg) {
    const text = msg && typeof msg.data === 'string' ? msg.data : '';
    btStatus.text = text;
    const parsed = window.MissionEvents.parseStatus(text);
    if (!parsed) return;
    btStatus.parsed = parsed;
    renderBtStage(parsed);
    if (!mission.active || !mission.goal || mission.mode !== '3d') return;
    const verdict = window.MissionEvents.classify(parsed, mission.btSentSeq,
                                                  mission.btHandledSeq);
    if (verdict.terminal) {
      mission.btHandledSeq = verdict.seq;
      log('BT 编排回执 seq=' + verdict.seq + '：' + verdict.reason,
          verdict.ok ? 'ok' : 'err');
      missionGoalDone(verdict.ok, verdict.reason);
    }
  }
  function renderBtStage(status) {
    if (!el.btStage) return;
    const nodes = status && Array.isArray(status.nodes) ? status.nodes : [];
    if (!nodes.length) {
      el.btStage.innerHTML = '';
      el.mBtStage.textContent = '—';
      return;
    }
    const byName = {};
    nodes.forEach((node) => { if (node && node.name) byName[node.name] = node.status; });
    let running = '';
    el.btStage.innerHTML = '';
    BT_STAGES.forEach(([name, label]) => {
      const state = byName[name] || 'IDLE';
      if (state === 'RUNNING' && !running) running = label;
      const chip = document.createElement('span');
      chip.className = 'bt-node bt-' + String(state).toLowerCase();
      chip.textContent = label;
      chip.title = name + ' · ' + state;
      el.btStage.appendChild(chip);
    });
    const parts = [running || '空闲',
                   'seq ' + (status.current_seq || 0) + '/' + (status.goal_seq || 0)];
    if (status.queue_depth) parts.push('队列 ' + status.queue_depth);
    if (status.planner_mode) parts.push(status.planner_mode);
    if (status.arrival_signal) parts.push('arrived');
    if (status.event) parts.push(status.event);
    el.mBtStage.textContent = parts.join(' ｜ ');
  }

  function updatePathState() {
    const count = (message) => message && message.poses ? message.poses.length : 0;
    el.mPath.textContent = 'global ' + count(overlay.global) + ' ｜ local ' + count(overlay.local);
  }
  function updateMbf(msg) {
    const list = (msg && msg.status_list) || [];
    if (!list.length) { el.mMbf.textContent = '—'; return; }
    const names = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
                   'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST'];
    const latest = list[list.length - 1];
    el.mMbf.textContent = names[latest.status] || ('#' + latest.status);
    if (mission.active && mission.mode === '2d' && mission.goal) {
      const terminal = list.find((status) =>
        [2, 3, 4, 5, 9].indexOf(status.status) >= 0 &&
        status.goal_id && status.goal_id.stamp &&
        status.goal_id.stamp.secs >= mission.sentSecs - 1);
      if (terminal) {
        if (terminal.status === 3) missionGoalDone(true, 'MBF SUCCEEDED');
        else missionGoalDone(false, 'MBF ' + (names[terminal.status] || terminal.status));
      }
    }
  }
  function renderRates() {
    el.rates.innerHTML = TOPIC_SPECS.map((spec) => {
      return '<tr><td>' + escapeHtml(spec.label) + '</td><td id="rate-' + cssId(spec.topic) +
        '">—</td><td>' + (spec.expect ? spec.expect + ' Hz' : '事件') + '</td></tr>';
    }).join('');
  }
  function updateRates() {
    const now = performance.now();
    TOPIC_SPECS.forEach((spec) => {
      const rate = rates[spec.topic];
      const cell = document.getElementById('rate-' + cssId(spec.topic));
      if (!cell) return;
      if (!rate) { cell.textContent = '—'; return; }
      const seconds = (now - rate.window) / 1000;
      const hz = seconds > 0 ? rate.count / seconds : 0;
      const age = rate.last ? (now - rate.last) / 1000 : null;
      cell.textContent = hz.toFixed(1) + ' Hz' + (age == null ? '' : ' · ' + age.toFixed(1) + ' s 前');
      cell.classList.toggle('rate-low', !!(spec.expect && hz < spec.expect * 0.5));
      rate.count = 0;
      rate.window = now;
    });
    if (live.enabled) {
      const seconds = (now - live.window) / 1000;
      const hz = seconds > 0 ? live.count / seconds : 0;
      el.cloudLiveRate.textContent = hz.toFixed(1) + ' Hz';
      live.count = 0;
      live.window = now;
    }
  }
  function cssId(topic) { return topic.replace(/[^a-zA-Z0-9]/g, '_'); }
  function refreshRosapi() {
    if (!bridge || !online) { el.rosapiState.textContent = 'rosapi 未连接'; return; }
    el.rosapiState.textContent = 'rosapi 请求中…';
    bridge.callService('/rosapi/nodes', {}, (response) => {
      if (response.result && response.values && response.values.nodes) {
        const nodes = response.values.nodes;
        el.mMaster.textContent = '在线';
        el.rosapiState.textContent = 'rosapi 在线';
        el.rosNodes.textContent = nodes.join('\n') || '（无节点）';
      } else {
        el.mMaster.textContent = 'rosapi 未响应（无 master？）';
        el.rosapiState.textContent = 'rosapi 不可用';
        el.rosNodes.textContent = '（rosapi /rosapi/nodes 未返回；确认 roscore 与 rosbridge 的 master 一致）';
      }
    });
    bridge.callService('/rosapi/topics', {}, (response) => {
      if (response.result && response.values && response.values.topics) {
        const topics = response.values.topics;
        const types = response.values.types || [];
        el.rosTopics.textContent = topics.map((name, index) =>
          name + (types[index] ? '  [' + types[index] + ']' : '')).join('\n') || '（无话题）';
      } else {
        el.rosTopics.textContent = '（rosapi /rosapi/topics 未返回）';
      }
    });
  }

  // ---------------------------------------------------------------- teleop
  function publishApp(v, w) { pub(TELEOP_TOPIC, 'geometry_msgs/Twist', twist(v, w)); }
  function publishEstop(v, w) { pub(ESTOP_TOPIC, 'geometry_msgs/Twist', twist(v, w)); }
  function setCommand(v, w, holding) {
    if (estopLatched) return;
    const wasActive = teleop.active;
    teleop.v = v; teleop.w = w; teleop.holding = !!holding;
    teleop.active = !!(v || w); teleop.last = performance.now();
    el.deadman.textContent = teleop.active ? (teleop.holding ? '按住发送' : '发送中') : '空闲';
    if (!teleop.active && wasActive) publishApp(0, 0);
  }
  function stopTeleop(sendZero) {
    teleop.active = false; teleop.v = 0; teleop.w = 0; teleop.holding = false;
    el.deadman.textContent = '空闲';
    if (sendZero) publishApp(0, 0);
  }
  function teleopTick() {
    if (!online) return;
    if (estopLatched) {
      publishEstop(0, 0);
      return;
    }
    if (teleop.active) {
      if (teleop.holding) teleop.last = performance.now();
      if (performance.now() - teleop.last > 500) {
        stopTeleop(true);
        el.deadman.textContent = 'deadman 停';
        log('deadman 0.5 s：已停', 'err');
      } else {
        publishApp(teleop.v, teleop.w);
      }
    }
  }
  function setEstop(on) {
    estopLatched = on;
    el.estop.disabled = on;
    el.unlock.disabled = !on;
    el.estopState.textContent = on ? '急停锁存' : '未急停';
    el.estopState.className = 'pill ' + (on ? 'err' : '');
    if (on) {
      stopMission('急停');
      stopTeleop(true);
      publishEstop(0, 0);
      log('急停：持续发 /lite3/safety/estop_cmd_vel 零速；解锁前遥控锁定', 'err');
      cancel2d();
    } else {
      publishEstop(0, 0);
      log('急停解除', 'ok');
    }
  }
  function bindTeleop() {
    document.querySelectorAll('.pad button').forEach((button) => {
      const press = (event) => {
        event.preventDefault();
        const v = (parseFloat(el.vMax.value) || 0.3) * (parseFloat(button.dataset.v) || 0);
        const w = (parseFloat(el.wMax.value) || 0.4) * (parseFloat(button.dataset.w) || 0);
        setCommand(v, w, true);
      };
      const release = () => setCommand(0, 0, false);
      button.addEventListener('pointerdown', press);
      button.addEventListener('pointerup', release);
      button.addEventListener('pointerleave', release);
      button.addEventListener('pointercancel', release);
    });
    const keyMap = { w: [1, 0], s: [-1, 0], a: [0, 1], d: [0, -1], ' ': [0, 0],
                     arrowup: [1, 0], arrowdown: [-1, 0], arrowleft: [0, 1], arrowright: [0, -1] };
    const onKey = (down) => (event) => {
      if (isFormTarget(event.target)) return;
      const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
      const map = keyMap[key];
      if (!map) return;
      event.preventDefault();
      if (!down) { setCommand(0, 0, false); return; }
      setCommand((parseFloat(el.vMax.value) || 0.3) * map[0],
                 (parseFloat(el.wMax.value) || 0.4) * map[1], true);
    };
    document.addEventListener('keydown', onKey(true));
    document.addEventListener('keyup', onKey(false));
    el.vMax.addEventListener('input', () => { el.vVal.textContent = Number(el.vMax.value).toFixed(2); });
    el.wMax.addEventListener('input', () => { el.wVal.textContent = Number(el.wMax.value).toFixed(2); });
    el.estop.addEventListener('click', () => setEstop(true));
    el.unlock.addEventListener('click', () => setEstop(false));
    document.addEventListener('visibilitychange', () => { if (document.hidden) stopTeleop(true); });
    window.addEventListener('blur', () => stopTeleop(true));
    setInterval(teleopTick, 100);
  }

  // ---------------------------------------------------------------- mission run export
  function missionRecord() {
    return {
      schema: 'lite3_console_mission/v1',
      generated_at: new Date().toISOString(),
      map: current ? current.id : null,
      world: current ? (current.world || null) : null,
      mode: el.mode.value,
      start: start,
      goals: goals.map((goal, index) => ({
        index: index + 1,
        x: goal.x, y: goal.y, z: goal.z, yaw: goal.yaw,
        state: goal.state,
        metrics: goal.metrics || {},
      })),
      mission_active: mission.active,
      note: 'console mission export; per-goal metrics from pose/MBF status; not a formal verdict',
    };
  }
  function downloadText(name, text, type) {
    const blob = new Blob([text], { type: type || 'text/plain;charset=utf-8' });
    const anchor = document.createElement('a');
    anchor.href = URL.createObjectURL(blob);
    anchor.download = name;
    document.body.appendChild(anchor);
    anchor.click();
    document.body.removeChild(anchor);
    URL.revokeObjectURL(anchor.href);
  }
  function exportMissionJson() {
    downloadText('mission_' + Date.now() + '.json',
                 JSON.stringify(missionRecord(), null, 1), 'application/json;charset=utf-8');
    log('已导出任务 JSON');
  }
  function exportMissionCsv() {
    const record = missionRecord();
    const lines = ['index,type,x,y,z,yaw,state,duration_s,path_m,error_m'];
    (record.start ? [['S', record.start.x, record.start.y, record.start.z, record.start.yaw,
                      '起点', '', '', '']] : []).forEach((row) => lines.push(row.join(',')));
    record.goals.forEach((goal) => {
      const m = goal.metrics || {};
      lines.push([goal.index, '目标', goal.x, goal.y, goal.z, goal.yaw, goal.state,
                  m.duration != null ? m.duration.toFixed(3) : '',
                  m.path != null ? m.path.toFixed(3) : '',
                  m.error != null ? m.error.toFixed(3) : ''].join(','));
    });
    downloadText('mission_' + Date.now() + '.csv', '\ufeff' + lines.join('\n'), 'text/csv;charset=utf-8');
    log('已导出任务 CSV');
  }

  // ---------------------------------------------------------------- wiring
  function escapeHtml(value) {
    return String(value == null ? '' : value).replace(/[&<>"']/g, (char) => (
      { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[char]));
  }
  function bind() {
    el.bridgeUrl.value = 'ws://' + (location.hostname || '127.0.0.1') + ':9090';
    el.connect.addEventListener('click', connect);
    el.mapSelect.addEventListener('change', () => selectMap(el.mapSelect.value));
    el.mode.addEventListener('change', () => {
      if (mission.active) {
        el.mode.value = mission.mode;
        log('任务运行中不能切换 2D/3D', 'err');
        return;
      }
      updateSurfaces();
      el.goalZ.disabled = el.mode.value === '2d';
      updateMissionState();
    });
    el.surface.addEventListener('change', () => {
      if (el.surface.value !== 'custom') {
        const surface = (current && current.surfaces || []).find((s) => s.id === el.surface.value);
        if (surface) el.goalZ.value = Number(surface.z).toFixed(2);
      }
    });
    el.modeGoal.addEventListener('click', () => {
      clickMode = 'goal'; el.modeGoal.classList.add('on'); el.modeStart.classList.remove('on');
    });
    el.modeStart.addEventListener('click', () => {
      clickMode = 'start'; el.modeStart.classList.add('on'); el.modeGoal.classList.remove('on');
    });
    el.canvas.addEventListener('click', (event) => {
      const point = canvasPoint(event);
      if (!point) return;
      if (edit.active) { editCell(point); return; }
      if (clickMode === 'start') setStart(point); else addGoal(point);
    });
    el.send.addEventListener('click', sendSelected);
    el.sendAll.addEventListener('click', sendAll);
    el.btSend.addEventListener('click', () => {
      if (el.mode.value !== '3d') { log('行为树只接 3D 目标；先切到 3D 全链', 'err'); return; }
      if (selected < 0 || !goals[selected]) { log('先在地图上选一个目标', 'err'); return; }
      if (!el.mapGuard.checked) { log(guardHint('交给行为树'), 'err'); return; }
      const goal = goals[selected];
      if (publishGoal(goal)) {
        renderGoals();
        log('已交给行为树（等待到点/超时）', 'ok');
      }
    });
    el.cancel.addEventListener('click', () => {
      if (mission.active) { stopMission('手工取消'); return; }
      if (el.mode.value === '2d') cancel2d();
      else log('3D 任务没有独立 cancel 服务：请用急停或作业单停止', 'err');
    });
    el.clear.addEventListener('click', () => {
      stopMission('清空目标');
      goals = []; start = null; selected = -1; renderGoals(); paintGoals(); updateMissionState();
    });
    el.goals.addEventListener('click', (event) => {
      const row = event.target.closest('tr[data-index]');
      if (!row) return;
      selected = parseInt(row.dataset.index, 10);
      renderGoals(); paintGoals();
    });
    el.refresh.addEventListener('click', refreshRosapi);
    el.overlayToggle.addEventListener('click', () => {
      overlay.enabled = !overlay.enabled;
      el.overlayToggle.classList.toggle('on', overlay.enabled);
      draw();
    });
    el.missionExportJson.addEventListener('click', exportMissionJson);
    el.missionExportCsv.addEventListener('click', exportMissionCsv);
    el.mapEdit.addEventListener('click', toggleEdit);
    el.mapEditSave.addEventListener('click', saveEdits);
    el.mapEditClear.addEventListener('click', () => {
      edit.changes.clear(); updateEditState(); draw(); log('已清空未保存编辑');
    });
    el.mapRefresh.addEventListener('click', refreshMapLibrary);
    el.mapRegrid.addEventListener('click', regridCurrentMap);
    el.mapHide.addEventListener('click', () => {
      if (!current) return;
      hiddenMaps.add(current.id);
      saveHidden();
      log('已隐藏 ' + current.id + '（仅控制台列表）');
      refreshMaps(null).catch((error) => log('刷新地图库失败：' + error.message, 'err'));
    });
    el.mapRestore.addEventListener('click', () => {
      hiddenMaps.clear();
      saveHidden();
      log('已恢复全部隐藏地图');
      refreshMaps(current ? current.id : null).catch((error) => log('刷新地图库失败：' + error.message, 'err'));
    });
    el.mapGuard.addEventListener('change', () => {
      if (!el.mapGuard.checked) { el.mapGuardState.textContent = '未确认'; el.mapGuardState.className = 'pill err'; }
      else if (current && current.kind === '2d_grid' && overlay.mapMsg) { checkMapGuard(); }
      else { el.mapGuardState.textContent = '已人工确认'; el.mapGuardState.className = 'pill on'; }
    });
    setInterval(missionTick, 200);
    el.estop.disabled = false;
    el.mode.dispatchEvent(new Event('change'));
    bindTeleop();
    bindCloud();
    bindStacks();
    setInterval(updateRates, 1000);
    loadConfig().finally(() => {
      loadClientRole();
      window.setTimeout(() => connect(), 250);
    });
  }
  async function loadConfig() {
    try {
      const response = await fetch('assets/console/config.json', { cache: 'no-store' });
      if (!response.ok) return;
      const config = await response.json();
      if (config.rosbridge) el.bridgeUrl.value = config.rosbridge;
      if (config.agent) el.agentUrl.value = config.agent;
      applyVizMode(vizMode, false);
      if (config.expected_rates) {
        TOPIC_SPECS.forEach((spec) => {
          if (config.expected_rates[spec.topic] != null) {
            spec.expect = Number(config.expected_rates[spec.topic]);
          }
        });
        renderRates();
      }
    } catch (error) { /* config is optional */ }
  }

  bind();
  applyVizMode(localStorage.getItem('lite3-viz-mode') || 'web', false);
  renderRates();
  renderGoals();
  updateMissionState();
  loadWorldIndex().finally(() => loadMaps());
})();

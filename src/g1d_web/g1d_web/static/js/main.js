import { Net, KIND } from './net.js';
import { View } from './scene.js';

const $ = (id) => document.getElementById(id);
const DEG = Math.PI / 180;
const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v) ? '-' : Number(v).toFixed(d));
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

const ui = { cfg: null, state: null, you: false, owner: null, tests: [], wps: [], picking: null,
  params: { guard: {}, nav: {} }, lastExportsAt: 0 };

// ------------------------------------------------------------------------------------ helpers
function toast(text, level = 'error') {
  const d = document.createElement('div');
  d.textContent = text; d.className = level;
  $('toast').appendChild(d);
  setTimeout(() => d.remove(), level === 'error' ? 5000 : 2500);
}

function kv(el, pairs) {
  el.innerHTML = pairs.map(([k, v, cls]) => `<div class="k">${esc(k)}</div><div class="v ${cls || ''}">${esc(v)}</div>`).join('');
}

function pill(el, text, cls) { el.textContent = text; el.className = `pill ${cls || ''}`; }

function send(obj, needControl = true) {
  if (needControl && !ui.you) { toast('需要控制权：先点击右上角「获取控制权」'); return false; }
  if (!net.send(obj)) { toast('未连接到 g1d_web'); return false; }
  return true;
}

const view = new View($('view'));

// ------------------------------------------------------------------------------------ network
const net = new Net({
  onOpen: () => { pill($('conn'), '已连接', 'ok'); net.send({ type: 'hello', name: navigator.platform || 'browser' });
    net.send({ type: 'map', action: 'list' }); net.send({ type: 'mission', action: 'list' }); net.send({ type: 'tests', action: 'list' }); },
  onClose: () => { pill($('conn'), '未连接', 'bad'); ui.you = false; },
  onJson: (m) => {
    switch (m.type) {
      case 'hello': onHello(m); break;
      case 'state': onState(m); break;
      case 'params': onParams(m); break;
      case 'event': addEvent(m); break;
      case 'error': toast(m.text); break;
      case 'exports': renderExports(m.items, m.new); break;
      case 'missions': renderSavedMissions(m.items); break;
      case 'test_results': renderTestResults(m.items); break;
      case 'goal_sent': toast(`目标已发送 ${m.goal.id}`, 'info'); break;
      case 'param_saved': toast(`已写入配置：${m.changed.length} 项`, 'info'); break;
      default: break;
    }
  },
  onBinary: (kind, h, buf, off) => {
    switch (kind) {
      case KIND.SCAN: view.scan.set(new Float32Array(buf.slice(off, off + h.n * 12))); break;
      case KIND.MAP: view.map.append(new Float32Array(buf, off, h.n * 3)); break;
      case KIND.MAP_RESET: view.map.reset(); break;
      case KIND.OBST: view.obst.set(new Float32Array(buf.slice(off, off + h.n * 12))); break;
      case KIND.CHASSIS: view.chassis.set(new Float32Array(buf.slice(off, off + h.n * 12))); break;
      case KIND.LOCAL: view.local.set(h, new Uint8Array(buf, off, h.width * h.height)); break;
      case KIND.GLOBAL: view.global.set(h, new Uint8Array(buf, off, h.width * h.height)); break;
      default: break;
    }
  },
});

function onHello(m) {
  ui.cfg = m.config; ui.tests = m.tests || [];
  view.setFootprint(m.config.footprint, m.config.rotate_radius);
  $('mapVoxel').value = m.config.voxel;
  const g = m.config.guard || {};
  if (g.teleop_vx_max) { $('tVmax').max = g.teleop_vx_max; }
  if (g.teleop_wz_max) { $('tWmax').max = g.teleop_wz_max; }
  updateTeleopLabels();
  $('testSel').innerHTML = ui.tests.map((t) => `<option value="${t.id}">${esc(t.title)}</option>`).join('');
  showTestInfo();
  $('events').innerHTML = '';
  (m.events || []).forEach(addEvent);
}

// ------------------------------------------------------------------------------------ state
function onState(s) {
  ui.state = s;
  const st = s.status || {}, g = s.guard || {}, lio = s.lio || {}, fresh = s.fresh || {};
  ui.you = !!(s.control && s.control.you); ui.owner = s.control && s.control.owner;

  // top bar
  if (!fresh.status) pill($('navState'), '导航 离线', 'warn');
  else {
    const cls = { NAVIGATING: 'info', ROTATING: 'info', RECOVERING: 'warn', SUCCEEDED: 'ok', FAILED: 'bad' }[st.state] || '';
    pill($('navState'), `导航 ${st.state || '-'}${st.reason ? ' · ' + st.reason : ''}`, cls);
  }
  if (!fresh.lio) pill($('lioState'), 'LIO 离线', 'bad');
  else pill($('lioState'), lio.ok ? `LIO 正常 ${lio.effective_points ?? ''}` : `LIO 异常 ${(lio.errors || []).join(',')}`, lio.ok ? 'ok' : 'bad');
  if (!fresh.guard) pill($('guardState'), 'guard 离线', 'bad');
  else if (g.estop) pill($('guardState'), `急停中 ${g.estop_reason || ''}`, 'bad');
  else if (g.manual) pill($('guardState'), `人工接管 ${g.manual_source || ''}`, 'warn');
  else if (g.blocked) pill($('guardState'), `guard 拦截 ${g.blocked}`, 'warn');
  else pill($('guardState'), 'guard 正常', 'ok');
  $('btnEstop').classList.toggle('active', !!g.estop);
  const p = s.pose;
  $('poseText').textContent = p ? `x ${fmt(p.x)}  y ${fmt(p.y)}  θ ${fmt(p.yaw / DEG, 0)}°  v ${fmt(p.v)}  (${p.source})` : '无位姿';
  const c = s.control || {};
  if (ui.you) { $('ctrlText').textContent = '你拥有控制权'; $('btnControl').textContent = '释放控制权'; }
  else if (c.owner) { $('ctrlText').textContent = `控制者：${c.owner_name || c.owner}`; $('btnControl').textContent = '强制接管'; }
  else { $('ctrlText').textContent = '只读'; $('btnControl').textContent = '获取控制权'; }

  // view
  if (p) view.setPose(p);
  if (s.debug) view.setDebug(fresh.debug ? s.debug : null);
  view.setSectors(p, fresh.status ? st.free_sectors : null);
  const ms = s.mission || {};
  if (ms.state && ['running', 'waiting', 'paused'].includes(ms.state)) view.setMission(ms.waypoints, ms.index);
  else view.setMission(ui.wps, -1);

  // hud
  const cn = s.cmd && s.cmd.nav, co = s.cmd && s.cmd.out;
  $('hud').textContent = `nav cmd   v ${cn ? fmt(cn[0]) : '-'}  w ${cn ? fmt(cn[1]) : '-'}\n`
    + `底盘指令  v ${co ? fmt(co[0]) : '-'}  w ${co ? fmt(co[1]) : '-'}  (${g.source || '-'})\n`
    + `前方净距  ${fmt(g.front_clear)} m   地图 ${s.map ? s.map.points : 0} 点`;

  renderNavStatus(st, fresh.status);
  renderMission(ms);
  renderMap(s.map);
  renderTest(s.test);
  if (s.rates) renderRates(s.rates);
  renderRaw();
}

function renderNavStatus(st, fresh) {
  if (!fresh) { kv($('navStatus'), [['状态', 'g1d_nav 无数据', 'bad']]); $('sectorBar').innerHTML = ''; return; }
  const cls = st.state === 'SUCCEEDED' ? 'ok' : st.state === 'FAILED' ? 'bad' : '';
  kv($('navStatus'), [
    ['状态', `${st.state}${st.reason ? ' (' + st.reason + ')' : ''}`, cls],
    ['目标 id', st.goal_id || '-'],
    ['剩余距离', `${fmt(st.distance_remaining)} m`], ['朝向误差', `${fmt((st.yaw_error || 0) / DEG, 1)}°`],
    ['目标已调整 / 部分', `${st.adjusted ? '是' : '否'} / ${st.partial ? '是' : '否'}`],
    ['恢复次数', st.recoveries ?? 0],
    ['LIO / 姿态锁定', `${st.lio_ok ? '正常' : '异常'} / ${st.body_locked ? '锁定' : '未锁定 ' + (st.lock_reasons || []).join(',')}`,
      st.lio_ok && st.body_locked ? 'ok' : 'warn'],
    ['位姿来源', `${st.state_source || '-'}${st.odom_consistent === false ? '（LIO/轮速不一致）' : ''}`],
  ]);
  const fs = st.free_sectors || {};
  $('sectorBar').innerHTML = Object.keys(fs).map(Number).sort((a, b) => b - a)
    .map((d) => `<span class="${fs[d] ? 'free' : 'blocked'}">${d > 0 ? '左' : d < 0 ? '右' : ''}${Math.abs(d)}°</span>`).join('');
}

// ------------------------------------------------------------------------------------ control
$('btnControl').onclick = () => {
  if (ui.you) net.send({ type: 'control', action: 'release' });
  else if (ui.owner) { if (confirm('强制接管控制权？当前控制者将变为只读。')) net.send({ type: 'control', action: 'acquire', force: true }); }
  else net.send({ type: 'control', action: 'acquire' });
};
$('btnEstop').onclick = () => net.send({ type: 'estop', on: true });
$('btnRelease').onclick = () => { if (confirm('确认现场安全后解除急停？')) send({ type: 'estop', on: false }); };
$('btnCancel').onclick = () => send({ type: 'stop' });

// ------------------------------------------------------------------------------------ nav tab
function goalFromForm() {
  const mode = $('gMode').value;
  const g = { mode, frame: $('gFrame').value, x: Number($('gX').value), y: Number($('gY').value) };
  if ($('gUseYaw').checked || mode === 'rotate') g.yaw = Number($('gYaw').value) * DEG;
  if ($('gSpeed').value !== '') g.max_speed = Number($('gSpeed').value);
  if ($('gTol').value !== '') g.xy_tolerance = Number($('gTol').value);
  return g;
}
$('btnSendGoal').onclick = () => send({ type: 'goal', goal: goalFromForm() });
$('btnPickGoal').onclick = () => startPick('goal', (x, y, yaw) => {
  $('gFrame').value = 'odom'; $('gX').value = x.toFixed(2); $('gY').value = y.toFixed(2);
  if (yaw !== null) { $('gYaw').value = (yaw / DEG).toFixed(0); $('gUseYaw').checked = true; }
  else $('gUseYaw').checked = false;
  if ($('gMode').value === 'rotate') $('gMode').value = 'navigate';
  stopPick();
  if ($('gAutoSend').checked) send({ type: 'goal', goal: goalFromForm() });
});
document.querySelectorAll('[data-quick]').forEach((b) => {
  b.onclick = () => {
    const q = b.dataset.quick;
    const goal = q === 'fwd' ? { mode: 'navigate', frame: 'base_at_stamp', x: 1.0, y: 0 }
      : { mode: 'rotate', frame: 'base_at_stamp', x: 0, y: 0, yaw: { left: 90, right: -90, back: 180 }[q] * DEG };
    send({ type: 'goal', goal });
  };
});

function startPick(kind, cb) {
  stopPick();
  ui.picking = kind;
  $('pickHint').classList.remove('hidden');
  view.startPick(cb);
}
function stopPick() { ui.picking = null; $('pickHint').classList.add('hidden'); view.cancelPick(); }

// ------------------------------------------------------------------------------------ mission
function renderWpTable() {
  const tb = $('wpTable').tBodies[0];
  tb.innerHTML = ui.wps.map((w, i) => `<tr data-i="${i}">
    <td>${i + 1}</td>
    <td><input data-f="x" type="number" step="0.1" value="${fmt(w.x)}"></td>
    <td><input data-f="y" type="number" step="0.1" value="${fmt(w.y)}"></td>
    <td><input data-f="yaw" type="number" step="5" value="${w.yaw === null || w.yaw === undefined ? '' : fmt(w.yaw / DEG, 0)}" placeholder="-"></td>
    <td><select data-f="mode">${['navigate', 'simple'].map((m) => `<option ${w.mode === m ? 'selected' : ''}>${m}</option>`).join('')}</select></td>
    <td><input data-f="wait_s" type="number" step="1" min="0" value="${w.wait_s || 0}" style="width:44px"></td>
    <td><button class="small" data-a="up">↑</button><button class="small" data-a="del">✕</button></td></tr>`).join('');
  tb.querySelectorAll('input,select').forEach((el) => {
    el.onchange = () => {
      const w = ui.wps[Number(el.closest('tr').dataset.i)];
      const f = el.dataset.f;
      if (f === 'mode') w.mode = el.value;
      else if (f === 'yaw') w.yaw = el.value === '' ? null : Number(el.value) * DEG;
      else w[f] = Number(el.value);
      view.setMission(ui.wps, -1);
    };
  });
  tb.querySelectorAll('button').forEach((b) => {
    b.onclick = () => {
      const i = Number(b.closest('tr').dataset.i);
      if (b.dataset.a === 'del') ui.wps.splice(i, 1);
      else if (i > 0) [ui.wps[i - 1], ui.wps[i]] = [ui.wps[i], ui.wps[i - 1]];
      renderWpTable();
    };
  });
  view.setMission(ui.wps, -1);
}
function missionFromForm() {
  return { name: $('mName').value.trim() || 'mission', loops: Number($('mLoops').value), on_fail: $('mOnFail').value,
    retries: Number($('mRetries').value),
    waypoints: ui.wps.map((w) => ({ x: w.x, y: w.y, yaw: w.yaw ?? null, mode: w.mode || 'navigate', wait_s: w.wait_s || 0 })) };
}
$('btnPickWp').onclick = () => {
  if (ui.picking === 'mission') { stopPick(); $('btnPickWp').textContent = '地图添加航点'; return; }
  $('btnPickWp').textContent = '结束添加';
  startPick('mission', (x, y, yaw) => { ui.wps.push({ x, y, yaw, mode: 'navigate', wait_s: 0 }); renderWpTable(); });
};
$('btnAddPoseWp').onclick = () => {
  const p = ui.state && ui.state.pose;
  if (!p) return toast('没有位姿');
  ui.wps.push({ x: p.x, y: p.y, yaw: p.yaw, mode: 'navigate', wait_s: 0 }); renderWpTable();
};
$('btnClearWp').onclick = () => { ui.wps = []; renderWpTable(); };
$('btnMStart').onclick = () => { if (!ui.wps.length) return toast('任务没有航点'); send({ type: 'mission', action: 'start', mission: missionFromForm() }); };
$('btnMPause').onclick = () => send({ type: 'mission', action: 'pause' });
$('btnMResume').onclick = () => send({ type: 'mission', action: 'resume' });
$('btnMCancel').onclick = () => send({ type: 'mission', action: 'cancel' });
$('btnMSave').onclick = () => send({ type: 'mission', action: 'save', mission: missionFromForm() });
$('btnMDelete').onclick = () => { const n = $('mSaved').value; if (n && confirm(`删除任务 ${n}？`)) send({ type: 'mission', action: 'delete', name: n }); };
$('btnMLoad').onclick = () => {
  const item = (ui.savedMissions || []).find((m) => m.name === $('mSaved').value);
  if (!item) return;
  const m = item.mission;
  $('mName').value = m.name; $('mLoops').value = m.loops; $('mOnFail').value = m.on_fail; $('mRetries').value = m.retries;
  ui.wps = m.waypoints.map((w) => ({ x: w.x, y: w.y, yaw: w.yaw ?? null, mode: w.mode, wait_s: w.wait_s }));
  renderWpTable();
};
function renderSavedMissions(items) {
  ui.savedMissions = items || [];
  $('mSaved').innerHTML = ui.savedMissions.map((m) => `<option value="${esc(m.name)}">${esc(m.name)}（${m.waypoints} 点）</option>`).join('');
}
function renderMission(ms) {
  if (!ms || ms.state === 'idle') { kv($('missionStatus'), [['任务', '空闲']]); return; }
  const cls = { done: 'ok', failed: 'bad', paused: 'warn', canceled: 'warn' }[ms.state] || '';
  kv($('missionStatus'), [
    ['任务', `${ms.name}：${ms.state}${ms.reason ? '（' + ms.reason + '）' : ''}`, cls],
    ['进度', `航点 ${Math.min(ms.index + 1, ms.total)}/${ms.total}，第 ${ms.loop + 1} 圈${ms.loops ? '/' + ms.loops : '（无限）'}${ms.attempt ? '，重试 ' + ms.attempt : ''}`],
  ]);
  $('mResults').tBodies[0].innerHTML = (ms.results || []).slice(-12).reverse().map((r) =>
    `<tr><td>${r.loop + 1}</td><td>${r.index + 1}</td><td class="${r.state === 'SUCCEEDED' ? 'ok' : 'bad'}">${r.state}</td><td>${esc(r.reason)}</td><td>${r.duration_s}s</td></tr>`).join('');
}

// ------------------------------------------------------------------------------------ teleop
const joy = { active: false, jx: 0, jy: 0, keys: new Set(), timer: null, zeroSent: 0 };
const jc = $('joystick'), jg = jc.getContext('2d');
function drawJoy() {
  const r = jc.width / 2;
  jg.clearRect(0, 0, jc.width, jc.height);
  jg.strokeStyle = '#2f3a46'; jg.beginPath(); jg.arc(r, r, r - 4, 0, Math.PI * 2); jg.stroke();
  jg.beginPath(); jg.moveTo(r, 10); jg.lineTo(r, jc.height - 10); jg.moveTo(10, r); jg.lineTo(jc.width - 10, r); jg.stroke();
  jg.fillStyle = joy.active ? '#3d8bfd' : '#55606c';
  jg.beginPath(); jg.arc(r + joy.jx * (r - 30), r + joy.jy * (r - 30), 26, 0, Math.PI * 2); jg.fill();
}
function joyFromEvent(e) {
  const rect = jc.getBoundingClientRect(), r = rect.width / 2;
  let x = (e.clientX - rect.left - r) / (r - 30), y = (e.clientY - rect.top - r) / (r - 30);
  const n = Math.hypot(x, y); if (n > 1) { x /= n; y /= n; }
  joy.jx = x; joy.jy = y; drawJoy();
}
jc.addEventListener('pointerdown', (e) => {
  if (!ui.you) return toast('需要控制权');
  try { jc.setPointerCapture(e.pointerId); } catch (err) { /* synthetic / unsupported pointer */ }
  joy.active = true; joyFromEvent(e); ensureTeleopTimer();
});
jc.addEventListener('pointermove', (e) => { if (joy.active) joyFromEvent(e); });
const joyUp = () => { joy.active = false; joy.jx = 0; joy.jy = 0; drawJoy(); };
jc.addEventListener('pointerup', joyUp); jc.addEventListener('pointercancel', joyUp);
function teleopCmd() {
  const vmax = Number($('tVmax').value), wmax = Number($('tWmax').value);
  let v = -joy.jy * vmax, w = -joy.jx * wmax;
  if (!joy.active && joy.keys.size) {
    v = (joy.keys.has('w') ? vmax : 0) - (joy.keys.has('s') ? Math.min(vmax, 0.1) : 0);
    w = (joy.keys.has('a') ? wmax : 0) - (joy.keys.has('d') ? wmax : 0);
  }
  if (Math.abs(v) < 0.02) v = 0;
  if (Math.abs(w) < 0.03) w = 0;
  return [v, w];
}
function ensureTeleopTimer() {
  if (joy.timer) return;
  joy.zeroSent = 0;
  joy.timer = setInterval(() => {
    const engaged = joy.active || joy.keys.size > 0;
    const [v, w] = engaged ? teleopCmd() : [0, 0];
    net.send({ type: 'teleop', v, w });
    $('teleopText').textContent = engaged ? `发送 v ${fmt(v)} m/s  w ${fmt(w)} rad/s` : '已松开（停止发送）';
    if (!engaged && ++joy.zeroSent >= 2) { clearInterval(joy.timer); joy.timer = null; }
  }, 100);
}
function updateTeleopLabels() { $('tVmaxText').textContent = ` ${$('tVmax').value} m/s`; $('tWmaxText').textContent = ` ${$('tWmax').value} rad/s`; }
$('tVmax').oninput = updateTeleopLabels; $('tWmax').oninput = updateTeleopLabels;
const inInput = () => ['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName);
document.addEventListener('keydown', (e) => {
  if (e.key === 'Escape') { stopPick(); $('btnPickWp').textContent = '地图添加航点'; }
  if (inInput()) return;
  if (e.code === 'Space') { e.preventDefault(); net.send({ type: 'estop', on: true }); return; }
  const k = e.key.toLowerCase();
  if ($('tKeys').checked && 'wasd'.includes(k) && k.length === 1) {
    if (!ui.you) return;
    joy.keys.add(k); ensureTeleopTimer();
  }
});
document.addEventListener('keyup', (e) => joy.keys.delete(e.key.toLowerCase()));
window.addEventListener('blur', () => { joy.keys.clear(); joyUp(); });
drawJoy();

// ------------------------------------------------------------------------------------ map tab
function renderMap(m) {
  if (!m) return;
  kv($('mapStats'), [
    ['点数', `${m.points}${m.full ? '（已达上限）' : ''}`], ['帧数', m.frames], ['体素', `${m.voxel} m`],
    ['轨迹点', m.trajectory], ['坐标对齐', m.T_valid ? 'T_odom_lio 正常' : '等待 T_odom_lio', m.T_valid ? 'ok' : 'warn'],
    ['累积', m.paused ? `暂停：${m.paused}` : '进行中', m.paused ? 'warn' : 'ok'],
  ]);
}
$('btnMapReset').onclick = () => { if (confirm('清空本次累积的地图？')) send({ type: 'map', action: 'reset', voxel: Number($('mapVoxel').value) }); };
$('mapPause').onchange = () => send({ type: 'map', action: 'pause', paused: $('mapPause').checked });
$('mapColor').onchange = () => view.map.recolor($('mapColor').value);
$('btnMapExport').onclick = () => net.send({ type: 'map', action: 'export' });
function renderExports(items, newName) {
  if (newName) toast(`已导出 maps/${newName}`, 'info');
  $('exports').innerHTML = (items || []).map((e) => `<div class="exp"><b>${esc(e.name)}</b> · ${e.points} 点<br>`
    + e.files.map((f) => `<a href="/files/maps/${encodeURIComponent(e.name)}/${encodeURIComponent(f)}" download>${esc(f)}</a>`).join('') + '</div>').join('')
    || '<div class="note">暂无</div>';
}

// ------------------------------------------------------------------------------------ tests
function showTestInfo() {
  const t = ui.tests.find((x) => x.id === $('testSel').value);
  $('testInfo').textContent = t ? t.instructions : '';
  $('testArgs').classList.toggle('hidden', !(t && ['T1', 'T2'].includes(t.id)));
}
$('testSel').onchange = showTestInfo;
$('btnTestSelect').onclick = () => send({ type: 'test', action: 'select', test: $('testSel').value, args: { speed: Number($('testSpeed').value) } });
$('btnTestStart').onclick = () => send({ type: 'test', action: 'start' });
$('btnTestMark').onclick = () => send({ type: 'test', action: 'mark' });
$('btnTestStop').onclick = () => { send({ type: 'test', action: 'stop' }); setTimeout(() => net.send({ type: 'tests', action: 'list' }), 300); };
$('btnTestAbort').onclick = () => send({ type: 'test', action: 'abort' });
$('btnTestList').onclick = () => net.send({ type: 'tests', action: 'list' });
function renderTest(t) {
  if (!t) { kv($('testStatus'), [['测试', '未选择']]); return; }
  const passTxt = t.passed === null || t.passed === undefined ? `进行中（${t.trials.length}/${t.target_trials}）` : t.passed ? '通过' : '未通过';
  kv($('testStatus'), [['测试', `${t.title}：${t.state}${t.phase ? ' · ' + t.phase : ''}`],
    ['信息', t.message || '-'], ['结论', passTxt, t.passed === true ? 'ok' : t.passed === false ? 'bad' : '']]);
  const trials = t.trials || [];
  const keys = trials.length ? Object.keys(trials[0]).filter((k) => k !== 'goal_id') : [];
  $('testTrials').tHead.innerHTML = keys.length ? `<tr><th>#</th>${keys.map((k) => `<th>${esc(k)}</th>`).join('')}</tr>` : '';
  $('testTrials').tBodies[0].innerHTML = trials.map((r, i) => `<tr><td>${i + 1}</td>${keys.map((k) => {
    const v = r[k];
    const cls = k === 'ok' ? (v ? 'ok' : 'bad') : '';
    return `<td class="${cls}">${esc(typeof v === 'number' ? fmt(v, 3) : v === true ? '✓' : v === false ? '✗' : v ?? '-')}</td>`;
  }).join('')}</tr>`).join('');
}
function renderTestResults(items) {
  $('testResults').innerHTML = (items || []).map((r) => `<div class="exp"><a href="/files/tests/${encodeURIComponent(r.file)}" download>${esc(r.file)}</a>`
    + `${esc(r.id)} · ${r.trials} 次 · ${r.passed === true ? '通过' : r.passed === false ? '未通过' : r.state}</div>`).join('') || '<div class="note">暂无</div>';
}

// ------------------------------------------------------------------------------------ params
function renderParamTable(el, target, values) {
  const names = Object.keys(values || {}).sort();
  el.tBodies[0].innerHTML = names.map((n) => {
    const v = values[n];
    const ro = target === 'nav' && n.startsWith('mppi_');
    const shown = typeof v === 'boolean' ? (v ? 1 : 0) : v;
    return `<tr><td>${esc(n)}</td><td class="mono">${esc(typeof v === 'number' ? fmt(v, 3) : String(v))}</td>`
      + (ro ? '<td colspan="2" class="dim">只读</td>' : `<td><input type="number" step="0.01" value="${shown}" data-n="${esc(n)}"></td>`
      + `<td><button class="small" data-n="${esc(n)}">应用</button></td>`) + '</tr>';
  }).join('') || '<tr><td class="dim">尚未收到参数（进程未运行？）</td></tr>';
  el.querySelectorAll('button[data-n]').forEach((b) => {
    b.onclick = () => {
      const inp = el.querySelector(`input[data-n="${b.dataset.n}"]`);
      send({ type: 'param', target, name: b.dataset.n, value: Number(inp.value) });
    };
  });
}
function onParams(m) {
  const focused = document.activeElement && document.activeElement.closest && document.activeElement.closest('#tab-params');
  ui.params = { guard: m.guard, nav: m.nav };
  if (!focused) { renderParamTable($('pGuard'), 'guard', m.guard); renderParamTable($('pNav'), 'nav', m.nav); }
  if (m.acks && m.acks.length) {
    $('paramAcks').innerHTML = m.acks.slice(-6).reverse().map((a) => `<div class="${a.ok ? '' : 'error'}">${esc(a.target)}.${esc(a.name)}: ${a.ok ? 'OK' : esc(a.error)}</div>`).join('');
  }
}
$('btnSaveGuard').onclick = () => { if (confirm('把当前 guard 参数写入 config/g1d_nav.yaml？')) send({ type: 'param_save', target: 'guard' }); };
$('btnSaveNav').onclick = () => { if (confirm('把当前 nav 参数写入 config/g1d_nav.yaml？')) send({ type: 'param_save', target: 'nav' }); };

// ------------------------------------------------------------------------------------ diag
function renderRates(r) {
  $('rates').tBodies[0].innerHTML = Object.values(r).map((x) =>
    `<tr><td class="mono">${esc(x.topic)}</td><td class="${x.hz > 0 ? 'ok' : 'bad'}">${fmt(x.hz, 1)}</td><td>${x.age === null ? '-' : fmt(x.age, 1)}</td></tr>`).join('');
}
function addEvent(ev) {
  const d = document.createElement('div');
  d.className = ev.level;
  d.textContent = `${new Date(ev.t * 1000).toLocaleTimeString()}  ${ev.text}`;
  const log = $('events');
  log.prepend(d);
  while (log.childElementCount > 200) log.lastChild.remove();
  if (ev.level === 'warn') toast(ev.text, 'info');
}
function renderRaw() {
  if ($('tab-diag').classList.contains('hidden') || !ui.state) return;
  const k = $('rawSel').value;
  $('raw').textContent = JSON.stringify(ui.state[k] ?? null, null, 1);
}

// ------------------------------------------------------------------------------------ layout
document.querySelectorAll('#tabs button').forEach((b) => {
  b.onclick = () => {
    document.querySelectorAll('#tabs button').forEach((x) => x.classList.toggle('active', x === b));
    document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('hidden', t.id !== `tab-${b.dataset.tab}`));
    if (b.dataset.tab === 'map') net.send({ type: 'map', action: 'list' });
    if (b.dataset.tab === 'test') net.send({ type: 'tests', action: 'list' });
  };
});
document.querySelectorAll('[data-layer]').forEach((cb) => { cb.onchange = () => view.setLayer(cb.dataset.layer, cb.checked); });
$('btnView2d').onclick = () => { const to = view.mode === '3d' ? '2d' : '3d'; view.setMode(to); $('btnView2d').textContent = to === '2d' ? '3D 视图' : '俯视 2D'; };
$('btnFollow').onclick = () => { view.follow = !view.follow; $('btnFollow').classList.toggle('on', view.follow); };
$('btnResetView').onclick = () => view.resetView();
$('btnClearTrail').onclick = () => view.clearTrail();
$('pointSize').oninput = () => view.setPointSize(Number($('pointSize').value));
renderWpTable();
net.connect();
window.__g1d = { ui, view, net };

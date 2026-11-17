// three.js view in the odom frame (x forward, y left, z up).
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';

const Z_COLOR_MIN = -0.1, Z_COLOR_MAX = 2.4;

function heightColor(z, out, i) {
  // Turbo-like ramp: blue (floor) -> cyan -> green -> yellow -> red (ceiling)
  const t = Math.min(1, Math.max(0, (z - Z_COLOR_MIN) / (Z_COLOR_MAX - Z_COLOR_MIN)));
  const r = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 3)));
  const g = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 2)));
  const b = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 1)));
  out[i] = 0.15 + 0.85 * r; out[i + 1] = 0.15 + 0.85 * g; out[i + 2] = 0.2 + 0.8 * b;
}

class GrowingPoints {
  constructor(scene, size, colorMode) {
    this.capacity = 0; this.count = 0; this.colorMode = colorMode;
    this.geom = new THREE.BufferGeometry();
    this.mat = new THREE.PointsMaterial({ size, vertexColors: true, sizeAttenuation: true });
    this.points = new THREE.Points(this.geom, this.mat);
    this.points.frustumCulled = false;
    scene.add(this.points);
    this.ensure(65536);
  }
  ensure(n) {
    if (n <= this.capacity) return;
    let cap = Math.max(65536, this.capacity);
    while (cap < n) cap *= 2;
    const pos = new Float32Array(cap * 3), col = new Float32Array(cap * 3);
    if (this.capacity) {
      pos.set(this.geom.attributes.position.array.subarray(0, this.count * 3));
      col.set(this.geom.attributes.color.array.subarray(0, this.count * 3));
    }
    this.geom.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    this.geom.setAttribute('color', new THREE.BufferAttribute(col, 3));
    this.capacity = cap;
  }
  append(xyz) {
    const n = xyz.length / 3;
    this.ensure(this.count + n);
    const pos = this.geom.attributes.position, col = this.geom.attributes.color;
    pos.array.set(xyz, this.count * 3);
    for (let i = 0; i < n; i++) this.colorize(col.array, (this.count + i) * 3, xyz[i * 3 + 2]);
    this.count += n;
    this.geom.setDrawRange(0, this.count);
    pos.needsUpdate = true; col.needsUpdate = true;
  }
  colorize(arr, i, z) {
    if (this.colorMode === 'height') heightColor(z, arr, i);
    else { arr[i] = 0.75; arr[i + 1] = 0.8; arr[i + 2] = 0.85; }
  }
  recolor(mode) {
    this.colorMode = mode;
    const pos = this.geom.attributes.position.array, col = this.geom.attributes.color.array;
    for (let i = 0; i < this.count; i++) this.colorize(col, i * 3, pos[i * 3 + 2]);
    this.geom.attributes.color.needsUpdate = true;
  }
  reset() { this.count = 0; this.geom.setDrawRange(0, 0); }
}

class FramePoints {
  constructor(scene, color, size) {
    this.geom = new THREE.BufferGeometry();
    this.geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3), 3));
    this.mat = new THREE.PointsMaterial({ color, size, sizeAttenuation: true });
    this.points = new THREE.Points(this.geom, this.mat);
    this.points.frustumCulled = false;
    scene.add(this.points);
  }
  set(xyz) {
    this.geom.setAttribute('position', new THREE.BufferAttribute(xyz, 3));
    this.geom.setDrawRange(0, xyz.length / 3);
  }
}

function costmapRGBA(data) {
  // ROS OccupancyGrid values: 0 free, 1..98 inflation, 99 inscribed, 100 lethal, 255 unknown
  const rgba = new Uint8Array(data.length * 4);
  for (let i = 0; i < data.length; i++) {
    const v = data[i], o = i * 4;
    if (v === 0) { rgba[o + 3] = 0; continue; }
    if (v === 255) { rgba[o] = 90; rgba[o + 1] = 100; rgba[o + 2] = 115; rgba[o + 3] = 40; continue; }
    if (v >= 100) { rgba[o] = 255; rgba[o + 1] = 40; rgba[o + 2] = 60; rgba[o + 3] = 215; continue; }
    if (v === 99) { rgba[o] = 190; rgba[o + 1] = 60; rgba[o + 2] = 230; rgba[o + 3] = 170; continue; }
    const t = v / 98;
    rgba[o] = 40 + 200 * t; rgba[o + 1] = 120 + 100 * t; rgba[o + 2] = 230 - 200 * t; rgba[o + 3] = 50 + 100 * t;
  }
  return rgba;
}

class CostmapLayer {
  constructor(scene, z) {
    this.mesh = null; this.scene = scene; this.z = z;
    this.mat = new THREE.MeshBasicMaterial({ transparent: true, depthWrite: false, side: THREE.DoubleSide });
  }
  set(h, data) {
    const tex = new THREE.DataTexture(costmapRGBA(data), h.width, h.height, THREE.RGBAFormat);
    tex.magFilter = THREE.NearestFilter; tex.minFilter = THREE.NearestFilter;
    tex.needsUpdate = true;
    if (this.mat.map) this.mat.map.dispose();
    this.mat.map = tex; this.mat.needsUpdate = true;
    const w = h.width * h.resolution, hh = h.height * h.resolution;
    if (!this.mesh || this.mesh.userData.w !== w || this.mesh.userData.h !== hh) {
      if (this.mesh) { this.scene.remove(this.mesh); this.mesh.geometry.dispose(); }
      this.mesh = new THREE.Mesh(new THREE.PlaneGeometry(w, hh), this.mat);
      this.mesh.userData = { w, h: hh };
      this.mesh.renderOrder = 1;
      this.mesh.visible = this.visible !== false;
      this.scene.add(this.mesh);
    }
    this.mesh.position.set(h.origin[0] + w / 2, h.origin[1] + hh / 2, this.z);
  }
  setVisible(v) { this.visible = v; if (this.mesh) this.mesh.visible = v; }
}

class PolyLine {
  constructor(scene, color, z, width = 1) {
    this.z = z;
    this.geom = new THREE.BufferGeometry();
    this.line = new THREE.Line(this.geom, new THREE.LineBasicMaterial({ color, linewidth: width }));
    this.line.frustumCulled = false;
    scene.add(this.line);
  }
  set(pts) {
    const a = new Float32Array(Math.max(pts.length, 1) * 3);
    pts.forEach((p, i) => { a[i * 3] = p[0]; a[i * 3 + 1] = p[1]; a[i * 3 + 2] = this.z; });
    this.geom.setAttribute('position', new THREE.BufferAttribute(a, 3));
    this.geom.setDrawRange(0, pts.length);
  }
}

function textSprite(text, color = '#ffffff') {
  const c = document.createElement('canvas'); c.width = 64; c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = 'rgba(15,20,26,0.85)'; g.beginPath(); g.arc(32, 32, 26, 0, Math.PI * 2); g.fill();
  g.strokeStyle = color; g.lineWidth = 4; g.stroke();
  g.fillStyle = color; g.font = 'bold 30px sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text, 32, 34);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false }));
  s.scale.set(0.35, 0.35, 1);
  s.renderOrder = 5;
  return s;
}

function arrow(color, length = 0.6) {
  const a = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), length, color, 0.2, 0.12);
  a.line.material.depthTest = false; a.cone.material.depthTest = false;
  a.renderOrder = 4;
  return a;
}

export class View {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0x0e1217);
    container.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(55, 1, 0.05, 500);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-6, -6, 7);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 0, 0);
    this.mode = '3d'; this.follow = true; this.pointScale = 1.0;
    this.layers = {};
    this.build();
    this.resize();
    window.addEventListener('resize', () => this.resize());
    this.pick = null;
    this.raycaster = new THREE.Raycaster();
    this.plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    const el = this.renderer.domElement;
    el.addEventListener('pointerdown', (e) => this.onPointer('down', e));
    el.addEventListener('pointermove', (e) => this.onPointer('move', e));
    el.addEventListener('pointerup', (e) => this.onPointer('up', e));
    this.lastPose = null;
    const loop = () => { requestAnimationFrame(loop); this.controls.update(); this.renderer.render(this.scene, this.camera); };
    loop();
  }

  build() {
    const s = this.scene;
    s.add(new THREE.AmbientLight(0xffffff, 0.9));
    const grid = new THREE.GridHelper(40, 40, 0x3a4652, 0x232b34);
    grid.rotation.x = Math.PI / 2;
    s.add(grid);
    const axes = new THREE.AxesHelper(0.8);
    s.add(axes);
    this.layers.grid = [grid, axes];
    this.map = new GrowingPoints(s, 0.04, 'height');
    this.scan = new FramePoints(s, 0x6ef2c8, 0.05);
    this.obst = new FramePoints(s, 0xff4d6d, 0.07);
    this.chassis = new FramePoints(s, 0xffd23f, 0.07);
    this.local = new CostmapLayer(s, 0.012);
    this.global = new CostmapLayer(s, 0.006);
    this.global.setVisible(false);
    this.globalPath = new PolyLine(s, 0x4da3ff, 0.03);
    this.localPath = new PolyLine(s, 0x00e5ff, 0.035);
    this.mppi = new PolyLine(s, 0xff5cf0, 0.04);
    this.trail = new PolyLine(s, 0xbfc7d1, 0.02);
    this.trailPts = [];
    this.layers.map = [this.map.points]; this.layers.scan = [this.scan.points];
    this.layers.obstacles = [this.obst.points]; this.layers.chassis = [this.chassis.points];
    this.layers.globalPath = [this.globalPath.line]; this.layers.localPath = [this.localPath.line];
    this.layers.mppi = [this.mppi.line]; this.layers.trail = [this.trail.line];
    this.localPath.line.visible = false;

    // robot
    this.robot = new THREE.Group();
    this.footprint = new THREE.LineLoop(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x3df5a5 }));
    this.robot.add(this.footprint);
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.4, 1.3),
      new THREE.MeshBasicMaterial({ color: 0x3df5a5, transparent: true, opacity: 0.12, depthWrite: false }));
    body.position.z = 0.65 + 0.15; this.robot.add(body);
    const bodyEdges = new THREE.LineSegments(new THREE.EdgesGeometry(body.geometry),
      new THREE.LineBasicMaterial({ color: 0x3df5a5, transparent: true, opacity: 0.45 }));
    bodyEdges.position.copy(body.position); this.robot.add(bodyEdges);
    const head = arrow(0x3df5a5, 0.7); head.position.z = 0.05; this.robot.add(head);
    this.rotCircle = new THREE.LineLoop(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x2c7a5a }));
    this.robot.add(this.rotCircle);
    s.add(this.robot);
    this.setFootprint([[0.3, 0.28], [0.3, -0.28], [-0.3, -0.28], [-0.3, 0.28]], 0.38);

    // sectors
    this.sectors = new THREE.Group(); s.add(this.sectors);
    this.layers.sectors = [this.sectors];

    // goal
    this.goal = new THREE.Group();
    this.goalArrow = arrow(0xffb020, 0.7); this.goal.add(this.goalArrow);
    this.goalRing = new THREE.LineLoop(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffb020 }));
    this.goal.add(this.goalRing);
    this.goal.visible = false; s.add(this.goal);

    // mission waypoints
    this.mission = new THREE.Group(); s.add(this.mission);
    this.missionLine = new PolyLine(this.mission, 0xffb020, 0.05);
    this.layers.mission = [this.mission];

    // pick ghost
    this.ghost = arrow(0xffffff, 0.7); this.ghost.visible = false; s.add(this.ghost);
  }

  circlePts(r, n = 48, z = 0.02) {
    const a = [];
    for (let i = 0; i < n; i++) { const t = (i / n) * Math.PI * 2; a.push(r * Math.cos(t), r * Math.sin(t), z); }
    return new Float32Array(a);
  }

  setFootprint(fp, rotR) {
    if (fp && fp.length) {
      const a = new Float32Array(fp.length * 3);
      fp.forEach((p, i) => { a[i * 3] = p[0]; a[i * 3 + 1] = p[1]; a[i * 3 + 2] = 0.03; });
      this.footprint.geometry.setAttribute('position', new THREE.BufferAttribute(a, 3));
    }
    this.rotCircle.geometry.setAttribute('position', new THREE.BufferAttribute(this.circlePts(rotR || 0.38), 3));
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / Math.max(1, h);
    this.camera.updateProjectionMatrix();
  }

  setLayer(name, visible) {
    if (name === 'local') return this.local.setVisible(visible);
    if (name === 'global') return this.global.setVisible(visible);
    (this.layers[name] || []).forEach((o) => { o.visible = visible; });
  }

  setPointSize(v) {
    this.pointScale = v / 4;
    this.map.mat.size = 0.04 * this.pointScale; this.scan.mat.size = 0.05 * this.pointScale;
    this.obst.mat.size = 0.07 * this.pointScale; this.chassis.mat.size = 0.07 * this.pointScale;
  }

  // ---- camera ----------------------------------------------------------------------------
  setMode(mode) {
    this.mode = mode;
    const t = this.controls.target;
    if (mode === '2d') {
      this.controls.enableRotate = false;
      this.camera.position.set(t.x, t.y - 0.001, t.z + 14);
    } else {
      this.controls.enableRotate = true;
      this.camera.position.set(t.x - 5, t.y - 5, t.z + 6);
    }
    this.camera.lookAt(t);
  }

  resetView() {
    const p = this.lastPose || { x: 0, y: 0 };
    this.controls.target.set(p.x, p.y, 0);
    this.setMode(this.mode);
  }

  // ---- data ------------------------------------------------------------------------------
  setPose(p) {
    if (!p) return;
    if (this.follow && this.lastPose) {
      const dx = p.x - this.lastPose.x, dy = p.y - this.lastPose.y;
      this.camera.position.x += dx; this.camera.position.y += dy;
      this.controls.target.x += dx; this.controls.target.y += dy;
    }
    this.lastPose = p;
    this.robot.position.set(p.x, p.y, 0);
    this.robot.rotation.set(0, 0, p.yaw);
    const last = this.trailPts[this.trailPts.length - 1];
    if (!last || Math.hypot(p.x - last[0], p.y - last[1]) > 0.05) {
      this.trailPts.push([p.x, p.y]);
      if (this.trailPts.length > 20000) this.trailPts.splice(0, 5000);
      this.trail.set(this.trailPts);
    }
  }

  clearTrail() { this.trailPts = []; this.trail.set([]); }

  setDebug(d) {
    if (!d) return;
    this.globalPath.set(d.global_path || []);
    this.localPath.set(d.local_path || []);
    this.mppi.set(d.mppi_traj || []);
    const g = d.goal;
    if (g && g.mode !== 'rotate') {
      this.goal.visible = true;
      this.goal.position.set(g.x, g.y, 0.04);
      this.goalArrow.visible = !!g.use_yaw;
      this.goalArrow.setDirection(new THREE.Vector3(Math.cos(g.yaw), Math.sin(g.yaw), 0));
      this.goalRing.geometry.setAttribute('position',
        new THREE.BufferAttribute(this.circlePts(g.xy_tolerance || 0.2, 32, 0), 3));
    } else {
      this.goal.visible = false;
    }
  }

  setSectors(pose, sectors) {
    this.sectors.clear();
    if (!pose || !sectors) return;
    for (const [deg, free] of Object.entries(sectors)) {
      const a = pose.yaw + (Number(deg) * Math.PI) / 180;
      const geom = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(pose.x + 0.45 * Math.cos(a), pose.y + 0.45 * Math.sin(a), 0.06),
        new THREE.Vector3(pose.x + 1.5 * Math.cos(a), pose.y + 1.5 * Math.sin(a), 0.06)]);
      this.sectors.add(new THREE.Line(geom, new THREE.LineBasicMaterial({
        color: free ? 0x2fb36b : 0xe5484d, transparent: true, opacity: 0.8 })));
    }
  }

  setMission(waypoints, current) {
    this.mission.children.filter((o) => o.isSprite).forEach((o) => { this.mission.remove(o); o.material.map.dispose(); });
    const pts = [];
    (waypoints || []).forEach((w, i) => {
      const s = textSprite(String(i + 1), i === current ? '#ffb020' : '#9fb3c8');
      s.position.set(w.x, w.y, 0.35);
      this.mission.add(s);
      pts.push([w.x, w.y]);
    });
    this.missionLine.set(pts);
  }

  // ---- picking ---------------------------------------------------------------------------
  startPick(cb) { this.pick = { cb, down: null }; this.controls.enabled = false; }

  cancelPick() { this.pick = null; this.ghost.visible = false; this.controls.enabled = true; }

  groundPoint(e) {
    const r = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    this.raycaster.setFromCamera(ndc, this.camera);
    const p = new THREE.Vector3();
    return this.raycaster.ray.intersectPlane(this.plane, p) ? p : null;
  }

  onPointer(kind, e) {
    if (!this.pick || e.button > 0) return;
    const p = this.groundPoint(e);
    if (!p) return;
    if (kind === 'down') {
      this.pick.down = p.clone();
      this.ghost.position.set(p.x, p.y, 0.06); this.ghost.visible = true;
      this.ghost.setDirection(new THREE.Vector3(1, 0, 0));
    } else if (kind === 'move' && this.pick.down) {
      const d = this.pick.down, dx = p.x - d.x, dy = p.y - d.y;
      if (Math.hypot(dx, dy) > 0.1) this.ghost.setDirection(new THREE.Vector3(dx, dy, 0).normalize());
    } else if (kind === 'up' && this.pick.down) {
      const d = this.pick.down, dx = p.x - d.x, dy = p.y - d.y;
      const yaw = Math.hypot(dx, dy) > 0.15 ? Math.atan2(dy, dx) : null;
      const cb = this.pick.cb;
      this.pick.down = null;
      this.ghost.visible = false;
      cb(d.x, d.y, yaw);
    }
  }
}

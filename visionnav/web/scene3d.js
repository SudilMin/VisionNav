// scene3d.js: the dashboard's 3D view. What RViz showed (rviz/visionnav.rviz indoors, visionnav_outdoor.rviz
// outdoors), drawn in the page from the same ROS topics, plus the live LiDAR scan. web_dashboard.py sends each layer
// as a server-sent event on /api/scene when it changes, already in the view's fixed frame (RViz's "Fixed Frame":
// map indoors, base_footprint outdoors):
//   frame    {fixed, mode}: what follows is in this frame; the view starts over
//   map      the SLAM occupancy grid: a PNG in RViz's map colours, its size and its origin
//   scan     the LiDAR's points (outdoors only ahead and beside the wearer: nothing behind them is drawn)
//   markers  one MarkerArray topic: every marker shown now, with RViz's types, sizes and colours
//   route    the navigation route (/object_path), as visionnav.rviz's Path display
//   tf       every TF frame, and the wearer (base_footprint)
// Drag to turn, right-drag (or Shift+drag) to move along the floor, scroll to zoom; on a phone one finger turns, two
// fingers zoom and move. Drawn only when something changed: an idle view costs nothing.
import * as THREE from "three";
import { OrbitControls } from "/static/vendor/OrbitControls.js";

THREE.ColorManagement.enabled = false;  // the topics' colours as they are, as RViz draws them

// The switches above the view: [key, label, modes it is offered in, modes it starts on in (as the .rviz files)]
export const LAYERS = [
  ["map", "Map", ["indoor"], ["indoor"]],
  ["scan", "LiDAR", ["indoor", "outdoor"], ["indoor"]],
  ["objects", "Objects", ["indoor", "outdoor"], ["indoor", "outdoor"]],
  ["walls", "Walls", ["indoor"], ["indoor"]],
  ["trail", "Your path", ["indoor"], ["indoor"]],
  ["route", "Route", ["indoor"], ["indoor"]],
  ["occupancy", "Occupancy", ["outdoor"], ["outdoor"]],
  ["you", "You", ["indoor"], ["indoor"]],
  ["tf", "TF frames", ["indoor", "outdoor"], ["indoor"]],
  ["grid", "Grid", ["indoor", "outdoor"], ["outdoor"]],
];
const TOPIC_LAYER = {"/semantic_markers": "objects", "/outdoor_markers": "objects", "/structure_markers": "walls",
                     "/trajectory_node_list": "trail", "/outdoor_occupancy": "occupancy"};
const M = {ARROW: 0, CUBE: 1, SPHERE: 2, CYLINDER: 3, LINE_STRIP: 4, LINE_LIST: 5, CUBE_LIST: 6, SPHERE_LIST: 7,
           POINTS: 8, TEXT: 9, MESH: 10, TRIANGLES: 11};  // visualization_msgs/Marker types

const GEOM = {
  box: new THREE.BoxGeometry(1, 1, 1),
  sphere: new THREE.SphereGeometry(0.5, 20, 14),
  cylinder: new THREE.CylinderGeometry(0.5, 0.5, 1, 24).rotateX(Math.PI / 2),  // along z, as RViz's
  shaft: new THREE.CylinderGeometry(0.5, 0.5, 1, 16).rotateZ(-Math.PI / 2).translate(0.5, 0, 0),  // x 0 … 1
  head: new THREE.ConeGeometry(0.5, 1, 16).rotateZ(-Math.PI / 2).translate(0.5, 0, 0),  // its tip at x 1
  segment: new THREE.BoxGeometry(1, 1, 1).translate(0.5, 0, 0),  // a piece of a line, x 0 … 1
};
const SHARED = new Set(Object.values(GEOM));
const X_AXIS = new THREE.Vector3(1, 0, 0);
const _m = new THREE.Matrix4(), _q = new THREE.Quaternion(), _s = new THREE.Vector3(), _a = new THREE.Vector3(),
      _b = new THREE.Vector3(), _c = new THREE.Color();

const size = (v) => Math.max(Math.abs(v || 0), 1e-4);  // a zero scale draws nothing (RViz warns), never breaks

function setPose(o, p) {
  o.position.set(p[0], p[1], p[2]);
  o.quaternion.set(p[3], p[4], p[5], p[6]).normalize();
}

function paint(mat, c) {
  // RViz's colour and alpha; a see-through material does not hide what is behind it
  mat.color.setRGB(c[0], c[1], c[2]);
  const see = c[3] < 0.999;
  mat.opacity = c[3];
  if (mat.transparent !== see) {
    mat.transparent = see;
    mat.needsUpdate = true;
  }
  mat.depthWrite = !see;
}

function dispose(o) {
  o.traverse((c) => {
    if (c.geometry && !SHARED.has(c.geometry) && !c.isSprite) c.geometry.dispose();  // sprites share three's one
    for (const mat of [].concat(c.material || [])) {
      if (mat.map) mat.map.dispose();
      mat.dispose();
    }
  });
  if (o.isInstancedMesh) o.dispose();
}

// ── one marker as RViz draws it (reusing the object it had when the type is the same) ──
function solid(old, m, geom) {
  const o = old || new THREE.Mesh(geom, new THREE.MeshLambertMaterial());
  setPose(o, m.p);
  o.scale.set(size(m.s[0]), size(m.s[1]), size(m.s[2]));
  paint(o.material, m.c);
  return o;
}

function arrow(old, m) {
  let o = old;
  if (!o) {
    const mat = new THREE.MeshLambertMaterial(), body = new THREE.Group();
    body.add(new THREE.Mesh(GEOM.shaft, mat), new THREE.Mesh(GEOM.head, mat));
    o = new THREE.Group();
    o.add(body);
  }
  const body = o.children[0], [shaft, head] = body.children, pts = m.pts || [];
  setPose(o, m.p);
  paint(shaft.material, m.c);
  if (pts.length >= 6) {
    // From points[0] to points[1]: scale.x is the shaft's diameter, scale.y the head's, scale.z (not 0) its length
    _a.set(pts[0], pts[1], pts[2]);
    const dir = _b.set(pts[3], pts[4], pts[5]).sub(_a), len = Math.max(dir.length(), 1e-4);
    const headLen = m.s[2] > 0 ? Math.min(m.s[2], len) : 0.23 * len;
    body.position.copy(_a);
    body.quaternion.setFromUnitVectors(X_AXIS, dir.normalize());
    shaft.scale.set(size(len - headLen), size(m.s[0]), size(m.s[0]));
    head.scale.set(size(headLen), size(m.s[1]), size(m.s[1]));
    head.position.set(len - headLen, 0, 0);
  } else {
    // Along the pose's x: scale.x long (77 % shaft, 23 % head), scale.y wide, scale.z high (the head twice that)
    body.position.set(0, 0, 0);
    body.quaternion.identity();
    shaft.scale.set(size(0.77 * m.s[0]), size(m.s[1]), size(m.s[2]));
    head.scale.set(size(0.23 * m.s[0]), size(2 * m.s[1]), size(2 * m.s[2]));
    head.position.set(0.77 * m.s[0], 0, 0);
  }
  return o;
}

function instanced(old, geom, count) {
  // An InstancedMesh with room for `count`, made anew (with spare room) only when it has to grow
  if (old && old.geometry === geom && old.userData.room >= count) {
    old.count = count;
    return old;
  }
  const room = Math.max(8, Math.ceil(count * 1.5));
  const o = new THREE.InstancedMesh(geom, new THREE.MeshLambertMaterial(), room);
  o.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  o.userData.room = room;
  o.count = count;
  o.frustumCulled = false;  // spread out: the base geometry's bounds do not cover the instances
  return o;
}

function instanceColours(o, m, n, pointOf) {
  // Each point's colour (m.colors), else the marker's; the material is white so the instance colour shows as is
  const cols = m.cols && m.cols.length >= 4 * (pointOf(n - 1) + 1) ? m.cols : null;
  let alpha = m.c[3];
  if (cols && n) {
    alpha = 0;
    for (let i = 0; i < n; i++) alpha += cols[4 * pointOf(i) + 3];
    alpha /= n;
  }
  for (let i = 0; i < n; i++) {
    const j = 4 * pointOf(i);
    if (cols) _c.setRGB(cols[j], cols[j + 1], cols[j + 2]);
    else _c.setRGB(m.c[0], m.c[1], m.c[2]);
    o.setColorAt(i, _c);
  }
  if (o.instanceColor) o.instanceColor.needsUpdate = true;
  paint(o.material, [1, 1, 1, alpha]);
}

function list(old, m) {
  // CUBE_LIST / SPHERE_LIST: one shape of the marker's scale at each point
  const pts = m.pts || [], n = Math.floor(pts.length / 3);
  const o = instanced(old, m.t === M.SPHERE_LIST ? GEOM.sphere : GEOM.box, n);
  setPose(o, m.p);
  _s.set(size(m.s[0]), size(m.s[1]), size(m.s[2]));
  _q.identity();
  for (let i = 0; i < n; i++) o.setMatrixAt(i, _m.compose(_a.set(pts[3 * i], pts[3 * i + 1], pts[3 * i + 2]), _q, _s));
  o.instanceMatrix.needsUpdate = true;
  instanceColours(o, m, n, (i) => i);
  return o;
}

function lines(old, m) {
  // LINE_STRIP (point to point) / LINE_LIST (in pairs), scale.x wide, as thin bars (WebGL lines are 1 px)
  const pts = m.pts || [], n = Math.floor(pts.length / 3), starts = [];
  for (let i = 0; i + 1 < n; i += m.t === M.LINE_LIST ? 2 : 1) starts.push(i);
  const o = instanced(old, GEOM.segment, starts.length), w = size(m.s[0]);
  setPose(o, m.p);
  starts.forEach((i, k) => {
    _a.set(pts[3 * i], pts[3 * i + 1], pts[3 * i + 2]);
    const d = _b.set(pts[3 * i + 3], pts[3 * i + 4], pts[3 * i + 5]).sub(_a), len = d.length();
    _q.setFromUnitVectors(X_AXIS, len > 1e-9 ? d.divideScalar(len) : X_AXIS);
    o.setMatrixAt(k, _m.compose(_a, _q, _s.set(size(len), w, w)));
  });
  o.instanceMatrix.needsUpdate = true;
  instanceColours(o, m, starts.length, (k) => starts[k]);
  return o;
}

function vertexColours(m, n) {
  const col = new Float32Array(3 * n), cols = m.cols && m.cols.length >= 4 * n ? m.cols : null;
  for (let i = 0; i < n; i++) {
    for (let k = 0; k < 3; k++) col[3 * i + k] = cols ? cols[4 * i + k] : m.c[k];
  }
  return new THREE.BufferAttribute(col, 3);
}

function points(old, m) {
  // POINTS: squares scale.x wide (metres), each point's colour
  const pts = m.pts || [], n = Math.floor(pts.length / 3);
  const o = old || new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({vertexColors: true}));
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(pts.slice(0, 3 * n), 3));
  g.setAttribute("color", vertexColours(m, n));
  o.geometry.dispose();
  o.geometry = g;
  setPose(o, m.p);
  o.material.size = size(m.s[0]);
  paint(o.material, [1, 1, 1, m.c[3]]);
  o.frustumCulled = false;
  return o;
}

function triangles(old, m) {
  // TRIANGLE_LIST: every three points a triangle, scaled by the marker's scale
  const pts = m.pts || [], n = Math.floor(pts.length / 9) * 3;
  const o = old || new THREE.Mesh(new THREE.BufferGeometry(),
                                  new THREE.MeshLambertMaterial({vertexColors: true, side: THREE.DoubleSide}));
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(pts.slice(0, 3 * n), 3));
  g.setAttribute("color", vertexColours(m, n));
  g.computeVertexNormals();
  o.geometry.dispose();
  o.geometry = g;
  setPose(o, m.p);
  o.scale.set(size(m.s[0]), size(m.s[1]), size(m.s[2]));
  paint(o.material, [1, 1, 1, m.c[3]]);
  return o;
}

const FONT = "600 48px system-ui, -apple-system, 'Segoe UI', sans-serif";

function text(old, m) {
  // TEXT_VIEW_FACING: always facing the viewer, scale.z the height of a capital letter (about 34 of 64 px a line)
  const o = old || new THREE.Sprite(new THREE.SpriteMaterial({transparent: true, depthWrite: false}));
  o.position.set(m.p[0], m.p[1], m.p[2]);
  const c = m.c, key = `${m.txt}|${c[0]},${c[1]},${c[2]}`;
  if (o.userData.key !== key) {
    const rows = String(m.txt || "").split("\n");
    const canvas = o.userData.canvas || document.createElement("canvas"), ctx = canvas.getContext("2d");
    ctx.font = FONT;
    const w = Math.ceil(Math.max(1, ...rows.map((r) => ctx.measureText(r).width))) + 16, h = 64 * rows.length;
    if (canvas.width !== w || canvas.height !== h || !o.material.map) {
      canvas.width = w;
      canvas.height = h;
      if (o.material.map) o.material.map.dispose();
      o.material.map = new THREE.CanvasTexture(canvas);
      o.material.map.minFilter = THREE.LinearFilter;
      o.material.map.generateMipmaps = false;
      o.material.needsUpdate = true;
    }
    ctx.clearRect(0, 0, w, h);
    Object.assign(ctx, {font: FONT, textAlign: "center", textBaseline: "middle", lineWidth: 6,
                        strokeStyle: "rgba(0, 0, 0, 0.7)",  // a dark edge: readable on the light map too
                        fillStyle: `rgb(${Math.round(255 * c[0])}, ${Math.round(255 * c[1])}, ${Math.round(255 * c[2])})`});
    rows.forEach((r, i) => {
      ctx.strokeText(r, w / 2, 32 + 64 * i);
      ctx.fillText(r, w / 2, 32 + 64 * i);
    });
    o.material.map.needsUpdate = true;
    Object.assign(o.userData, {key, canvas, aspect: w / h, rows: rows.length});
  }
  const h = size(m.s[2]) * (64 / 34) * o.userData.rows;
  o.scale.set(h * o.userData.aspect, h, 1);
  o.material.opacity = c[3];
  o.renderOrder = 10;
  return o;
}

function build(old, m) {
  switch (m.t) {
    case M.CUBE: case M.MESH: return solid(old, m, GEOM.box);  // a mesh file cannot be loaded here: its box
    case M.SPHERE: return solid(old, m, GEOM.sphere);
    case M.CYLINDER: return solid(old, m, GEOM.cylinder);
    case M.ARROW: return arrow(old, m);
    case M.LINE_STRIP: case M.LINE_LIST: return lines(old, m);
    case M.CUBE_LIST: case M.SPHERE_LIST: return list(old, m);
    case M.POINTS: return points(old, m);
    case M.TEXT: return text(old, m);
    case M.TRIANGLES: return triangles(old, m);
    default: return null;
  }
}

function youModel() {
  // The wearer on the floor at base_footprint: a disc and an arrow the way they face (x)
  const mat = new THREE.MeshLambertMaterial({color: 0x3ca0ff}), g = new THREE.Group();
  const disc = new THREE.Mesh(GEOM.cylinder, mat), shaft = new THREE.Mesh(GEOM.shaft, mat),
        head = new THREE.Mesh(GEOM.head, mat);
  disc.scale.set(0.45, 0.45, 0.04);
  disc.position.z = 0.03;
  shaft.scale.set(0.45, 0.08, 0.04);
  shaft.position.z = 0.06;
  head.scale.set(0.25, 0.26, 0.04);
  head.position.set(0.45, 0, 0.06);
  g.add(disc, shaft, head);
  return g;
}

export class Scene3D {
  constructor(container, {status = null, onMode = null, onFollow = null} = {}) {
    Object.assign(this, {container, statusEl: status, onMode, onFollow, fixed: null, mode: null, es: null,
                         connected: false, follow: true, dirty: true, youPos: null, mapCenter: null,
                         mapInfo: null, scanTopic: null});
    this.topics = new Map();  // topic -> {group, objs: Map(ns|id -> object)}
    this.tfObjs = new Map();
    this.counts = {scan: 0, markers: {}};
    try {
      this.renderer = new THREE.WebGLRenderer({antialias: true});
    } catch (e) {
      container.classList.add("no3d");
      container.textContent = "This browser cannot draw the 3D view (WebGL is off or not supported).";
      this.renderer = null;
      return;
    }
    const r = this.renderer;
    r.outputColorSpace = THREE.LinearSRGBColorSpace;
    r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    r.setClearColor(0x1e1e24);  // visionnav_outdoor.rviz's background
    container.appendChild(r.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.05, 1000);
    this.camera.up.set(0, 0, 1);  // ROS: z up
    this.camera.position.set(5, 5, 7);
    this.scene.add(this.camera);
    // Light comes from the viewer, as in RViz (three's Lambert shading divides by pi)
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.55 * Math.PI));
    const sun = new THREE.DirectionalLight(0xffffff, 0.6 * Math.PI);
    sun.position.set(-0.4, 0.6, 0);
    this.camera.add(sun, sun.target);
    sun.target.position.set(0, 0, -1);
    this.controls = new OrbitControls(this.camera, r.domElement);
    Object.assign(this.controls, {enableDamping: true, dampingFactor: 0.2, screenSpacePanning: false,
                                  maxPolarAngle: Math.PI / 2 - 0.02, minDistance: 0.5, maxDistance: 400});
    this.controls.addEventListener("change", () => { this.dirty = true; });
    r.domElement.addEventListener("pointerdown", (e) => {
      if (e.button === 2 || e.shiftKey || e.ctrlKey || e.metaKey) this.setFollow(false);  // moving the view away
    });
    this.groups = {};
    for (const [key] of LAYERS) {
      this.groups[key] = new THREE.Group();
      this.scene.add(this.groups[key]);
    }
    this.you = youModel();
    this.you.visible = false;
    this.groups.you.add(this.you);
    new ResizeObserver(() => this._resize()).observe(container);
    this._resize();
    const loop = () => {
      requestAnimationFrame(loop);
      this._draw();
    };
    requestAnimationFrame(loop);
    setInterval(() => this._showStatus(), 500);
  }

  // ── the stream ──
  connect(url = "/api/scene") {
    if (this.es || !this.renderer) return;
    const es = (this.es = new EventSource(url));
    const handlers = {frame: this.setFrame, map: this.setMap, scan: this.setScan, markers: this.setMarkers,
                      route: this.setRoute, tf: this.setTF};
    for (const [name, fn] of Object.entries(handlers)) {
      es.addEventListener(name, (e) => {
        try {
          fn.call(this, JSON.parse(e.data));
        } catch (err) {
          console.warn("3D view:", name, err);
        }
        this.dirty = true;
      });
    }
    es.onopen = () => { this.connected = true; };
    es.onerror = () => { this.connected = false; };  // it reconnects by itself
  }

  disconnect() {
    if (this.es) this.es.close();
    this.es = null;
    this.connected = false;
  }

  // ── the layers ──
  setFrame(d) {
    const changed = d.fixed !== this.fixed || d.mode !== this.mode;
    this.clear();  // everything comes again, in this frame
    this.fixed = d.fixed;
    this.mode = d.mode;
    this._makeGrid();
    if (changed) {
      this.youPos = this.mapCenter = null;
      this._centre = true;  // centred on the wearer once they are known
      this.resetView();
      if (this.onMode) this.onMode(d.mode);
    }
  }

  setMap(d) {
    if (!d.png) {  // no map (any more): the brain stopped
      this._drop("mapObj");
      this.mapInfo = this.mapCenter = null;
      return;
    }
    const fixed = this.fixed, img = new Image();
    img.onload = () => {
      if (fixed !== this.fixed) return;  // the frame changed meanwhile
      const W = d.w * d.res, H = d.h * d.res, key = `${d.w}x${d.h}x${d.res}`;
      if (!this.mapObj || this.mapObj.userData.key !== key) {
        this._drop("mapObj");
        // RViz's map display: alpha 0.7, one texel per cell, row 0 at the origin (hence no flip)
        this.mapObj = new THREE.Mesh(new THREE.PlaneGeometry(W, H).translate(W / 2, H / 2, 0),
                                     new THREE.MeshBasicMaterial({transparent: true, opacity: 0.7, depthWrite: false}));
        this.mapObj.userData.key = key;
        this.mapObj.renderOrder = -1;
        this.groups.map.add(this.mapObj);
      }
      const tex = new THREE.Texture(img);
      Object.assign(tex, {flipY: false, magFilter: THREE.NearestFilter, minFilter: THREE.LinearFilter,
                          generateMipmaps: false, needsUpdate: true});
      const mat = this.mapObj.material;
      if (mat.map) mat.map.dispose();
      mat.map = tex;
      mat.needsUpdate = true;
      setPose(this.mapObj, d.p);
      this.mapObj.position.z -= 0.01;  // just under what stands on the floor
      this.mapInfo = `${d.w}×${d.h} cells of ${Math.round(d.res * 100)} cm`;
      this.mapCenter = [d.p[0] + W / 2, d.p[1] + H / 2];
      this.dirty = true;
    };
    img.src = "data:image/png;base64," + d.png;
  }

  setScan(d) {
    const pts = d.pts || [], n = pts.length / 3;
    if (!this.scanObj || this.scanObj.userData.room < n) {
      this._drop("scanObj");
      const room = Math.max(1024, Math.ceil(n * 1.5)), g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(3 * room), 3)
        .setUsage(THREE.DynamicDrawUsage));
      this.scanObj = new THREE.Points(g, new THREE.PointsMaterial({color: 0xff4040, size: 3, sizeAttenuation: false}));
      this.scanObj.userData.room = room;
      this.scanObj.frustumCulled = false;
      this.groups.scan.add(this.scanObj);
    }
    const attr = this.scanObj.geometry.getAttribute("position");
    attr.array.set(pts);
    attr.needsUpdate = true;
    this.scanObj.geometry.setDrawRange(0, n);
    this.counts.scan = n;
    this.scanTopic = d.topic;
  }

  setMarkers(d) {
    let tg = this.topics.get(d.topic);
    if (!tg) {
      tg = {group: new THREE.Group(), objs: new Map()};
      (this.groups[TOPIC_LAYER[d.topic]] || this.groups.objects).add(tg.group);
      this.topics.set(d.topic, tg);
    }
    const seen = new Set();
    for (const m of d.markers || []) {
      seen.add(m.k);
      const old = tg.objs.get(m.k);
      let o = null;
      try {
        o = build(old && old.userData.t === m.t ? old : null, m);
      } catch (err) {
        console.warn("3D view: marker", m, err);
      }
      if (old && o !== old) {
        tg.group.remove(old);
        dispose(old);
        tg.objs.delete(m.k);
      }
      if (o) {
        o.userData.t = m.t;
        if (o.parent !== tg.group) tg.group.add(o);
        tg.objs.set(m.k, o);
      }
    }
    for (const [k, o] of tg.objs) {
      if (!seen.has(k)) {
        tg.group.remove(o);
        dispose(o);
        tg.objs.delete(k);
      }
    }
    this.counts.markers[d.topic] = tg.objs.size;
  }

  setRoute(d) {
    // visionnav.rviz's Path display: red, 5 cm wide, 10 cm above the floor
    const o = lines(this.routeObj, {t: M.LINE_STRIP, p: [0, 0, 0.1, 0, 0, 0, 1], s: [0.05, 0, 0],
                                    c: [1, 50 / 255, 50 / 255, 1], pts: d.pts || []});
    if (o !== this.routeObj) {
      this._drop("routeObj");
      this.routeObj = o;
      this.groups.route.add(o);
    }
  }

  setTF(d) {
    const frames = d.frames || {};
    for (const [name, p] of Object.entries(frames)) {
      let axes = this.tfObjs.get(name);
      if (!axes) {
        axes = new THREE.AxesHelper(0.3);  // x red, y green, z blue, as RViz's TF display
        this.tfObjs.set(name, axes);
        this.groups.tf.add(axes);
      }
      setPose(axes, p);
    }
    for (const [name, axes] of this.tfObjs) {
      if (!(name in frames)) {
        this.groups.tf.remove(axes);
        axes.dispose();
        this.tfObjs.delete(name);
      }
    }
    const you = this.mode !== "outdoor" ? d.you : null;  // outdoors the camera AI draws the wearer itself
    this.you.visible = !!you;
    if (you) {
      setPose(this.you, you);
      this.youPos = you;
      if (this._centre) {
        this._centre = false;
        this.resetView();
      }
    }
  }

  clear() {
    for (const tg of this.topics.values()) {
      for (const o of tg.objs.values()) dispose(o);
      tg.objs.clear();
      tg.group.clear();
    }
    for (const name of ["mapObj", "scanObj", "routeObj", "gridObj"]) this._drop(name);
    for (const axes of this.tfObjs.values()) {
      axes.parent.remove(axes);
      axes.dispose();
    }
    this.tfObjs.clear();
    this.counts = {scan: 0, markers: {}};
    this.mapInfo = null;
    this.you.visible = false;
    this.dirty = true;
  }

  _drop(name) {
    const o = this[name];
    if (o) {
      if (o.parent) o.parent.remove(o);
      dispose(o);
    }
    this[name] = null;
  }

  _makeGrid() {
    // visionnav_outdoor.rviz: 1 m cells, 24 of them round the wearer; indoors 40 round the map's origin
    const n = this.mode === "outdoor" ? 24 : 40, g = new THREE.GridHelper(n, n, 0x6e6e6e, 0x6e6e6e);
    g.rotation.x = Math.PI / 2;  // GridHelper lies in x-z; ROS's floor is x-y
    Object.assign(g.material, {transparent: true, opacity: 0.4, depthWrite: false});
    this.gridObj = g;
    this.groups.grid.add(g);
  }

  setLayer(key, on) {
    if (this.groups && this.groups[key]) {
      this.groups[key].visible = on;
      this.dirty = true;
    }
  }

  // ── the view ──
  setFollow(on) {
    this.follow = on;
    if (this.onFollow) this.onFollow(on);
  }

  resetView() {
    if (!this.renderer) return;
    if (this.mode === "outdoor") return this._orbit([4, 0, 0], 13, 0.75, Math.PI);  // RViz's "Behind the wearer"
    const c = this.youPos || this.mapCenter || [0, 0];
    this._orbit([c[0], c[1], 0], 10, 0.785, 0.785);  // RViz's default orbit view
  }

  topView() {
    if (!this.renderer) return;
    const t = this.controls.target, p = this.camera.position;
    const yaw = this.mode === "outdoor" ? Math.PI : Math.atan2(p.y - t.y, p.x - t.x);
    this._orbit([t.x, t.y, t.z], Math.max(p.distanceTo(t), 6), 1.553, yaw);  // straight down, same way up
  }

  _orbit(f, dist, pitch, yaw) {
    this.controls.target.set(f[0], f[1], f[2]);
    this.camera.position.set(f[0] + dist * Math.cos(pitch) * Math.cos(yaw),
                             f[1] + dist * Math.cos(pitch) * Math.sin(yaw), f[2] + dist * Math.sin(pitch));
    this.controls.update();
    this.dirty = true;
  }

  _resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h || !this.renderer) return;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.dirty = true;
  }

  _draw() {
    if (this.follow && this.mode !== "outdoor" && this.youPos) {
      // Follow the wearer: target and camera glide along with them, the view's angle and zoom stay
      const t = this.controls.target, dx = this.youPos[0] - t.x, dy = this.youPos[1] - t.y;
      if (Math.abs(dx) + Math.abs(dy) > 0.002) {
        t.x += 0.12 * dx;
        t.y += 0.12 * dy;
        this.camera.position.x += 0.12 * dx;
        this.camera.position.y += 0.12 * dy;
        this.dirty = true;
      }
    }
    if (this.controls.update() || this.dirty) {
      this.renderer.render(this.scene, this.camera);
      this.dirty = false;
    }
  }

  _showStatus() {
    if (!this.statusEl) return;
    const markers = Object.values(this.counts.markers).reduce((a, b) => a + b, 0);
    const parts = [this.connected ? "live" : "connecting…"];
    if (this.fixed) parts.push(`fixed frame: ${this.fixed}`);
    parts.push(this.counts.scan ? `LiDAR: ${this.counts.scan} points (${this.scanTopic})` : "LiDAR: no scan");
    if (this.mode !== "outdoor") parts.push(this.mapInfo ? `map: ${this.mapInfo}` : "map: none yet");
    parts.push(`${markers} markers`);
    const text = parts.join(" · ");
    if (text !== this._statusText) this.statusEl.textContent = this._statusText = text;
  }
}

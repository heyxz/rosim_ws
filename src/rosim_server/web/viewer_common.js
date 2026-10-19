// Shared helpers for the debug and production WebGL viewers. Loaded as a
// classic script after three.min.js, so these names are global.
"use strict";

function poseToMatrix4(pos, quat) {
  const m = new THREE.Matrix4();
  if (!pos) return m;
  m.compose(new THREE.Vector3(pos[0], pos[1], pos[2]),
            new THREE.Quaternion(quat[0], quat[1], quat[2], quat[3]),
            new THREE.Vector3(1, 1, 1));
  return m;
}

// Decode the server's packed mesh payload: base64 of float32 vertices followed
// by uint32 triangle indices. Returns {vertices:Float32Array, indices:Uint32Array}.
function decodeMeshData(data, vcount, tcount) {
  const bin = atob(data);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  const vertices = new Float32Array(bytes.buffer, 0, vcount * 3);
  const indices = new Uint32Array(bytes.buffer, vcount * 12, tcount * 3);
  return { vertices, indices };
}

// three.js computeVertexNormals averages every face that shares a vertex. That
// rounds off machined edges and smears flat faces into their cylindrical
// neighbours (dark fans around bores, soft box edges). Split the normals at
// creases instead: average incident face normals only within `angleRad`, and
// de-index the geometry so each triangle corner can carry its own normal.
function buildCreasedGeometry(vertices, indices, angleRad) {
  const nTri = indices.length / 3;
  const nVert = vertices.length / 3;
  const cosA = Math.cos(angleRad);

  // Per-vertex incident-face lists in CSR form.
  const start = new Uint32Array(nVert + 1);
  for (let i = 0; i < indices.length; i++) start[indices[i] + 1]++;
  for (let v = 0; v < nVert; v++) start[v + 1] += start[v];
  const cursor = start.slice(0, nVert);
  const adj = new Uint32Array(indices.length);
  for (let i = 0; i < indices.length; i++) adj[cursor[indices[i]]++] = (i / 3) | 0;

  // Unnormalised face normals: |cross| = 2*area, so summing them gives
  // area-weighted vertex normals.
  const fn = new Float32Array(nTri * 3);
  for (let t = 0; t < nTri; t++) {
    const ia = indices[3 * t] * 3, ib = indices[3 * t + 1] * 3, ic = indices[3 * t + 2] * 3;
    const ax = vertices[ia], ay = vertices[ia + 1], az = vertices[ia + 2];
    const ux = vertices[ib] - ax, uy = vertices[ib + 1] - ay, uz = vertices[ib + 2] - az;
    const vx = vertices[ic] - ax, vy = vertices[ic + 1] - ay, vz = vertices[ic + 2] - az;
    fn[3 * t] = uy * vz - uz * vy;
    fn[3 * t + 1] = uz * vx - ux * vz;
    fn[3 * t + 2] = ux * vy - uy * vx;
  }

  const nCorner = nTri * 3;
  const positions = new Float32Array(nCorner * 3);
  const normals = new Float32Array(nCorner * 3);
  for (let t = 0; t < nTri; t++) {
    const nx = fn[3 * t], ny = fn[3 * t + 1], nz = fn[3 * t + 2];
    const nl = Math.sqrt(nx * nx + ny * ny + nz * nz) || 1;
    const tx = nx / nl, ty = ny / nl, tz = nz / nl;
    for (let c = 0; c < 3; c++) {
      const k = 3 * t + c, src = indices[k] * 3, v = indices[k];
      positions[3 * k] = vertices[src];
      positions[3 * k + 1] = vertices[src + 1];
      positions[3 * k + 2] = vertices[src + 2];
      let sx = 0, sy = 0, sz = 0;
      for (let j = start[v]; j < start[v + 1]; j++) {
        const f = adj[j] * 3;
        const gx = fn[f], gy = fn[f + 1], gz = fn[f + 2];
        const gl = Math.sqrt(gx * gx + gy * gy + gz * gz) || 1;
        if ((gx * tx + gy * ty + gz * tz) / gl >= cosA) {
          sx += gx; sy += gy; sz += gz;
        }
      }
      const sl = Math.sqrt(sx * sx + sy * sy + sz * sz) || 1;
      normals[3 * k] = sx / sl;
      normals[3 * k + 1] = sy / sl;
      normals[3 * k + 2] = sz / sl;
    }
  }

  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  g.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
  return g;
}

// Canvas-texture sprite used as the floating label over a selected seam.
function makeEdgeLabelSprite(text, color) {
  const c = document.createElement("canvas");
  c.width = 256; c.height = 64;
  const ctx = c.getContext("2d");
  const r = 12, w = c.width, h = c.height;
  ctx.beginPath();
  ctx.moveTo(r, 2);
  ctx.arcTo(w - 2, 2, w - 2, h - 2, r);
  ctx.arcTo(w - 2, h - 2, 2, h - 2, r);
  ctx.arcTo(2, h - 2, 2, 2, r);
  ctx.arcTo(2, 2, w - 2, 2, r);
  ctx.closePath();
  ctx.fillStyle = "rgba(6, 12, 20, 0.88)";
  ctx.fill();
  ctx.lineWidth = 3;
  ctx.strokeStyle = color;
  ctx.stroke();
  ctx.font = "bold 34px system-ui, -apple-system, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillStyle = "#9fdcff";
  ctx.fillText(text, w / 2, 34);

  const tex = new THREE.CanvasTexture(c);
  tex.minFilter = THREE.LinearFilter;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({
    map: tex, transparent: true, depthTest: false, depthWrite: false
  }));
  sp.scale.set(0.16, 0.04, 1);
  sp.renderOrder = 21;
  return sp;
}

// Selected-edge marker: a fat tube laid over the thin seam line, endpoint
// caps, and a floating label. depthTest is off so the marker stays visible
// where the seam is buried in the part surface or behind the torch.
// `points` is a flat [x,y,z,...] array in ROS-world coordinates.
function buildEdgeMarker(points, labelText, color, radius) {
  if (!points || points.length < 6) return null;

  const pts = [];
  for (let i = 0; i < points.length; i += 3)
    pts.push(new THREE.Vector3(points[i], points[i + 1], points[i + 2]));

  // Cap the tube's control points: edges can carry thousands of samples and
  // the tube only needs enough to follow the curve visually.
  const stride = Math.max(1, Math.ceil((pts.length - 1) / 127));
  const curvePts = [];
  for (let i = 0; i < pts.length; i += stride) {
    const p = pts[i];
    if (!curvePts.length || curvePts[curvePts.length - 1].distanceToSquared(p) > 1e-12)
      curvePts.push(p);
  }
  const last = pts[pts.length - 1];
  if (curvePts[curvePts.length - 1].distanceToSquared(last) > 1e-12)
    curvePts.push(last);
  if (curvePts.length < 2) return null;

  const group = new THREE.Group();
  const mat = new THREE.MeshBasicMaterial({
    color: color, transparent: true, opacity: 0.95,
    depthTest: false, depthWrite: false, toneMapped: false
  });

  const curve = new THREE.CatmullRomCurve3(curvePts, false, "centripetal");
  const tube = new THREE.Mesh(
    new THREE.TubeGeometry(curve, Math.min(256, curvePts.length * 3), radius, 8, false),
    mat);
  tube.renderOrder = 20;
  group.add(tube);

  const capGeo = new THREE.SphereGeometry(radius * 2, 16, 12);
  for (const p of [curvePts[0], curvePts[curvePts.length - 1]]) {
    const cap = new THREE.Mesh(capGeo, mat);
    cap.position.copy(p);
    cap.renderOrder = 20;
    group.add(cap);
  }

  const cssColor = "#" + color.toString(16).padStart(6, "0");
  const label = makeEdgeLabelSprite(labelText, cssColor);
  const mid = curvePts[Math.floor(curvePts.length / 2)];
  label.position.set(mid.x, mid.y, mid.z + 0.035);
  group.add(label);
  return group;
}

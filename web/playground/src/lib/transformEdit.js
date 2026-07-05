function maxId(node, seen = new Set()) {
  if (seen.has(node.id)) return 0;
  seen.add(node.id);
  return node.children.reduce((m, c) => Math.max(m, maxId(c, seen)), node.id);
}

function round4(v) {
  return Math.round(v * 10000) / 10000;
}

function replaceById(root, targetId, replacement) {
  const memo = new Map();
  function walk(node) {
    if (memo.has(node.id)) return memo.get(node.id);
    if (node.id === targetId) {
      memo.set(node.id, replacement);
      return replacement;
    }
    const children = node.children.map(walk);
    const changed = children.some((c, i) => c !== node.children[i]);
    const result = changed ? { ...node, children } : node;
    memo.set(node.id, result);
    return result;
  }
  return walk(root);
}

export function applyTranslate(root, id, delta) {
  const [dx, dy, dz] = delta.map(round4);

  function findNode(node) {
    if (node.id === id) return node;
    for (const c of node.children) {
      const found = findNode(c);
      if (found) return found;
    }
    return null;
  }

  const target = findNode(root);
  if (!target) return root;

  if (target.op === 'translate') {
    const [ox, oy, oz] = target.args;
    const merged = {
      ...target,
      args: [round4(ox + dx), round4(oy + dy), round4(oz + dz)],
    };
    return replaceById(root, id, merged);
  }

  let nextId = maxId(root) + 1;
  const wrapper = {
    id: nextId,
    op: 'translate',
    args: [dx, dy, dz],
    children: [target],
    shape: null,
  };
  return replaceById(root, id, wrapper);
}

export function applyRotate(root, id, axis, angle, pivot) {
  const [ax, ay, az] = axis;
  const a = round4(angle);

  function findNode(node) {
    if (node.id === id) return node;
    for (const c of node.children) {
      const found = findNode(c);
      if (found) return found;
    }
    return null;
  }

  const target = findNode(root);
  if (!target) return root;

  let nextId = maxId(root) + 1;
  const rotNode = {
    id: nextId++,
    op: 'rotate',
    args: [round4(ax), round4(ay), round4(az), a],
    children: [target],
    shape: null,
  };

  const [px, py, pz] = pivot;
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const len = Math.hypot(ax, ay, az);
  let wrapped = rotNode;

  if (len > 1e-9 && (Math.abs(px) > 1e-9 || Math.abs(py) > 1e-9 || Math.abs(pz) > 1e-9)) {
    const nx = ax / len, ny = ay / len, nz = az / len;
    const rpx = px * (c + nx * nx * (1 - c)) + py * (nx * ny * (1 - c) - nz * s) + pz * (nx * nz * (1 - c) + ny * s);
    const rpy = px * (ny * nx * (1 - c) + nz * s) + py * (c + ny * ny * (1 - c)) + pz * (ny * nz * (1 - c) - nx * s);
    const rpz = px * (nz * nx * (1 - c) - ny * s) + py * (nz * ny * (1 - c) + nx * s) + pz * (c + nz * nz * (1 - c));
    const tx = round4(px - rpx);
    const ty = round4(py - rpy);
    const tz = round4(pz - rpz);
    if (Math.abs(tx) > 1e-9 || Math.abs(ty) > 1e-9 || Math.abs(tz) > 1e-9) {
      wrapped = {
        id: nextId++,
        op: 'translate',
        args: [tx, ty, tz],
        children: [rotNode],
        shape: null,
      };
    }
  }

  return replaceById(root, id, wrapped);
}

export function applyScale(root, id, factors, pivot) {
  const [fx, fy, fz] = factors;

  function findNode(node) {
    if (node.id === id) return node;
    for (const c of node.children) {
      const found = findNode(c);
      if (found) return found;
    }
    return null;
  }

  const target = findNode(root);
  if (!target) return root;

  let nextId = maxId(root) + 1;
  const isUniform = Math.abs(fx - fy) < 1e-9 && Math.abs(fy - fz) < 1e-9;

  const scaleNode = isUniform
    ? { id: nextId++, op: 'uniformScale', args: [round4(fx)], children: [target], shape: null }
    : { id: nextId++, op: 'scale', args: [round4(fx), round4(fy), round4(fz)], children: [target], shape: null };

  const [px, py, pz] = pivot;
  let wrapped = scaleNode;

  if (Math.abs(px) > 1e-9 || Math.abs(py) > 1e-9 || Math.abs(pz) > 1e-9) {
    const tx = round4(px - fx * px);
    const ty = round4(py - fy * py);
    const tz = round4(pz - fz * pz);
    if (Math.abs(tx) > 1e-9 || Math.abs(ty) > 1e-9 || Math.abs(tz) > 1e-9) {
      wrapped = {
        id: nextId++,
        op: 'translate',
        args: [tx, ty, tz],
        children: [scaleNode],
        shape: null,
      };
    }
  }

  return replaceById(root, id, wrapped);
}

export function pathTo(root, targetId) {
  if (root.id === targetId) return [];
  for (let i = 0; i < root.children.length; i++) {
    const sub = pathTo(root.children[i], targetId);
    if (sub !== null) return [i, ...sub];
  }
  return null;
}

export function nodeAt(root, path) {
  let current = root;
  for (const i of path) {
    if (!current.children || i >= current.children.length) return null;
    current = current.children[i];
  }
  return current;
}

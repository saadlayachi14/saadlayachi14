// Generates the profile banner: a LiDAR-style point cloud with one 3D detection.
// Usage: node banner.mjs <out.svg>
import { writeFileSync } from "node:fs";

const W = 1280, H = 380;
const CX = 880, CY = 276, FLAT = 0.3, ZS = 0.95;

// Deterministic noise so the banner is identical on every run.
let seed = 20261113;
const rnd = () => {
  seed |= 0; seed = (seed + 0x6d2b79f5) | 0;
  let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
};
const jit = (a) => (rnd() - 0.5) * 2 * a;

const proj = (gx, gy, z = 0) => [CX + gx, CY + gy * FLAT - z * ZS];
const lerp = (a, b, t) => a + (b - a) * t;
const NEAR = [125, 249, 255], FAR = [124, 108, 255];
const colour = (t) => {
  const c = NEAR.map((n, i) => Math.round(lerp(n, FAR[i], Math.min(1, Math.max(0, t)))));
  return `rgb(${c[0]},${c[1]},${c[2]})`;
};

// The detected car: a box on the ground plane, turned 20 degrees so it reads as a volume.
const car = { cx: -36, cy: -262, len: 150, wid: 78, h: 46, yaw: -0.36 };
const cosY = Math.cos(car.yaw), sinY = Math.sin(car.yaw);
const toGround = (u, v) => [car.cx + u * cosY - v * sinY, car.cy + u * sinY + v * cosY];
const boxCorners = (hl, hw) => [[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]].map(([u, v]) => toGround(u, v));
const corners = boxCorners(car.len / 2, car.wid / 2);
const angles = corners.map(([x, y]) => Math.atan2(y, x));
const A0 = Math.min(...angles), A1 = Math.max(...angles);
const carNear = Math.min(...corners.map(([x, y]) => Math.hypot(x, y)));

const WALL_Y = -470;         // straight barrier on the far side of the track

const dots = [];
const add = (gx, gy, z, t, o, r) => {
  const [x, y] = proj(gx, gy, z);
  if (x < 380 || x > W + 4 || y < -4 || y > H + 4) return;
  dots.push({ x, y, t, o, r });
};

// Ground returns: one ring per laser beam.
for (let k = 0; k < 32; k++) {
  const R = 46 + Math.pow(k, 1.42) * 9.2;
  const n = Math.round(R * 1.05);
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2 - Math.PI + jit(0.004);
    if (rnd() < 0.07) continue;                               // dropped returns
    if (a > A0 && a < A1 && R > carNear) continue;            // shadow behind the car
    const rr = R + jit(1.1);
    const gx = rr * Math.cos(a), gy = rr * Math.sin(a);
    if (gy < WALL_Y) continue;                                // shadow behind the barrier
    const t = R / 680;
    add(gx, gy, 0, t, lerp(0.95, 0.3, t), lerp(1.55, 1.0, t));
  }
}

// Barrier: a low wall along the far side of the track.
for (let row = 0; row < 5; row++) {
  for (let gx = -520; gx < 420; gx += 6.2) {
    if (rnd() < 0.12) continue;
    const a = Math.atan2(WALL_Y, gx);
    if (a > A0 && a < A1) continue;                           // hidden by the car
    add(gx + jit(1.5), WALL_Y + jit(4), 5 + row * 8 + jit(1.2), 0.85, 0.55 - row * 0.05, 1.05);
  }
}

// Car returns: only the faces turned towards the sensor, plus a sparse roof.
const hl = car.len / 2, hw = car.wid / 2;
const faces = [
  { n: [-1, 0], pts: (s) => [-hl, lerp(-hw, hw, s)], count: 13 },
  { n: [1, 0], pts: (s) => [hl, lerp(-hw, hw, s)], count: 13 },
  { n: [0, -1], pts: (s) => [lerp(-hl, hl, s), -hw], count: 30 },
  { n: [0, 1], pts: (s) => [lerp(-hl, hl, s), hw], count: 30 },
];
for (const f of faces) {
  const [nx, ny] = [f.n[0] * cosY - f.n[1] * sinY, f.n[0] * sinY + f.n[1] * cosY];
  if (nx * -car.cx + ny * -car.cy <= 0) continue;             // facing away from the sensor
  for (let z = 5; z <= car.h; z += 6.5) {
    for (let i = 0; i <= f.count; i++) {
      const [u, v] = f.pts(i / f.count);
      const [gx, gy] = toGround(u, v);
      add(gx + jit(1.2), gy + jit(2.5), z + jit(1), 0.25, 0.95, 1.5);
    }
  }
}
for (let i = 0; i < 60; i++) {
  const [gx, gy] = toGround(lerp(-hl, hl, rnd()), lerp(-hw, hw, rnd()));
  add(gx, gy, car.h + jit(1.5), 0.25, 0.6, 1.2);
}

// 3D bounding box (with a small margin around the returns).
const B = { base: boxCorners(hl + 10, hw + 16), h: car.h + 12 };
const P = (x, y, z) => proj(x, y, z).map((v) => v.toFixed(1)).join(",");
const edges = [];
for (let i = 0; i < 4; i++) {
  const [ax, ay] = B.base[i], [bx, by] = B.base[(i + 1) % 4];
  edges.push(`M${P(ax, ay, 0)}L${P(bx, by, 0)}`, `M${P(ax, ay, B.h)}L${P(bx, by, B.h)}`, `M${P(ax, ay, 0)}L${P(ax, ay, B.h)}`);
}
// Anchor the label on the right-most top corner.
const top = B.base.map(([x, y]) => proj(x, y, B.h)).sort((p, q) => q[0] - p[0])[0];
const [lx, ly] = top;
// Group dots by colour/opacity bucket to keep the file small.
const groups = new Map();
for (const d of dots) {
  const key = `${colour(d.t)}|${d.o.toFixed(1)}`;
  if (!groups.has(key)) groups.set(key, []);
  groups.get(key).push(`<circle cx="${d.x.toFixed(1)}" cy="${d.y.toFixed(1)}" r="${d.r.toFixed(2)}"/>`);
}
const cloud = [...groups].map(([key, list]) => {
  const [fill, o] = key.split("|");
  return `<g fill="${fill}" fill-opacity="${o}">${list.join("")}</g>`;
}).join("\n");

const SANS = "'Segoe UI',system-ui,-apple-system,'Helvetica Neue',Helvetica,Arial,sans-serif";
const MONO = "ui-monospace,'SF Mono','Cascadia Mono',Consolas,'Liberation Mono',Menlo,monospace";

const svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}" role="img" aria-labelledby="t d">
<title id="t">Saad Layachi, AI engineer</title>
<desc id="d">A LiDAR point cloud of a race track with one car outlined by a 3D bounding box, next to the line: I take AI from demo to production.</desc>
<defs>
  <linearGradient id="bg" x1="0" y1="0" x2="1" y2="1">
    <stop offset="0" stop-color="#05070D"/><stop offset="1" stop-color="#0A1020"/>
  </linearGradient>
  <linearGradient id="veil" x1="0" y1="0" x2="1" y2="0">
    <stop offset="0" stop-color="#05070D"/><stop offset="0.36" stop-color="#05070D" stop-opacity="0.96"/>
    <stop offset="0.58" stop-color="#05070D" stop-opacity="0"/>
  </linearGradient>
  <radialGradient id="glow" cx="${CX}" cy="${CY}" r="420" gradientUnits="userSpaceOnUse">
    <stop offset="0" stop-color="#2A6BFF" stop-opacity="0.20"/><stop offset="1" stop-color="#2A6BFF" stop-opacity="0"/>
  </radialGradient>
  <clipPath id="clip"><rect width="${W}" height="${H}" rx="18"/></clipPath>
  <style>
    .box{animation:lock 4.5s ease-in-out infinite}
    @keyframes lock{0%,100%{opacity:1}50%{opacity:.55}}
    @media (prefers-reduced-motion:reduce){.box{animation:none}}
  </style>
</defs>
<g clip-path="url(#clip)">
  <rect width="${W}" height="${H}" fill="url(#bg)"/>
  <rect width="${W}" height="${H}" fill="url(#glow)"/>
  ${cloud}
  <circle cx="${CX}" cy="${CY}" r="3.2" fill="#fff"/>
  <circle cx="${CX}" cy="${CY}" r="9" fill="none" stroke="#fff" stroke-opacity="0.35"/>
  <g class="box" fill="none" stroke="#FFB454" stroke-width="1.6" stroke-linecap="round"><path d="${edges.join("")}"/></g>
  <path d="M${lx.toFixed(1)},${ly.toFixed(1)}l16,-16h10" fill="none" stroke="#FFB454" stroke-width="1.2"/>
  <text x="${(lx + 32).toFixed(1)}" y="${(ly - 12).toFixed(1)}" font-family="${MONO}" font-size="13" fill="#FFB454" letter-spacing="0.4" stroke="#070B14" stroke-width="5" stroke-linejoin="round" paint-order="stroke">3D detection · 78.6% AP · &lt;50 ms</text>
  <rect width="${W}" height="${H}" fill="url(#veil)"/>
  <text x="64" y="112" font-family="${MONO}" font-size="15" fill="#7DF9FF" letter-spacing="3">AI ENGINEER · INSA ROUEN</text>
  <text x="61" y="190" font-family="${SANS}" font-size="72" font-weight="700" fill="#F5F8FF" letter-spacing="-1.5">Saad Layachi</text>
  <text x="64" y="240" font-family="${SANS}" font-size="27" fill="#C4CEDF">I take AI from demo to production.</text>
  <rect x="64" y="272" width="44" height="2" fill="#7DF9FF"/>
  <text x="64" y="308" font-family="${MONO}" font-size="14.5" fill="#8794AB" letter-spacing="0.6">RAG · self-hosted LLMs · full-stack · 3D perception</text>
  <rect x="0.5" y="0.5" width="${W - 1}" height="${H - 1}" rx="18" fill="none" stroke="#FFFFFF" stroke-opacity="0.08"/>
</g>
</svg>
`;

writeFileSync(process.argv[2], svg);
console.log(`dots: ${dots.length}, groups: ${groups.size}, bytes: ${svg.length}`);

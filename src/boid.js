// A single boid and the flock that coordinates them.
//
// The actual flocking math lives in ./boids.js as pure, tested functions
// (the three Reynolds rules, neighbour finding, steering and integration).
// This file is the stateful wrapper the canvas demo drives: it owns the boid
// array, a uniform spatial hash so neighbour lookups stay cheap with hundreds
// of agents, the predator interaction, and toroidal wrapping.

import {
  flockForce,
  separation,
  alignment,
  cohesion,
  integrate,
  limit,
  DEFAULTS,
} from "./boids.js";

const TAU = Math.PI * 2;

export class Boid {
  constructor(x, y) {
    this.x = x;
    this.y = y;
    const a = Math.random() * TAU;
    this.vx = Math.cos(a);
    this.vy = Math.sin(a);
    this.hue = 190 + Math.random() * 60; // cyan → violet band
  }
}

export class Flock {
  constructor(width, height) {
    this.resize(width, height);
    this.boids = [];
    this.cfg = { ...DEFAULTS };
    this.predator = null; // {x, y} or null
  }

  resize(width, height) {
    this.width = width;
    this.height = height;
  }

  setCount(n) {
    const b = this.boids;
    while (b.length < n) b.push(new Boid(Math.random() * this.width, Math.random() * this.height));
    if (b.length > n) b.length = n;
  }

  // Build a spatial hash keyed by cell so each boid only compares against
  // neighbours within its vision radius.
  _hash() {
    const cell = this.cfg.vision;
    const cols = Math.max(1, Math.ceil(this.width / cell));
    const grid = new Map();
    for (const b of this.boids) {
      const cx = Math.min(cols - 1, Math.max(0, Math.floor(b.x / cell)));
      const cy = Math.max(0, Math.floor(b.y / cell));
      const key = cy * cols + cx;
      let bucket = grid.get(key);
      if (!bucket) grid.set(key, (bucket = []));
      bucket.push(b);
    }
    return { grid, cols, cell };
  }

  // Gather a boid's neighbours within vision using the spatial hash. Equivalent
  // to neighbors() in boids.js but only scans the 3x3 block of cells around it.
  _neighbors(b, grid, cols, cell) {
    const visSq = this.cfg.vision * this.cfg.vision;
    const gx = Math.min(cols - 1, Math.max(0, Math.floor(b.x / cell)));
    const gy = Math.max(0, Math.floor(b.y / cell));
    const out = [];
    for (let oy = gy - 1; oy <= gy + 1; oy++) {
      if (oy < 0) continue;
      for (let ox = gx - 1; ox <= gx + 1; ox++) {
        if (ox < 0 || ox >= cols) continue;
        const bucket = grid.get(oy * cols + ox);
        if (!bucket) continue;
        for (const o of bucket) {
          if (o === b) continue;
          const dx = b.x - o.x;
          const dy = b.y - o.y;
          const d2 = dx * dx + dy * dy;
          if (d2 > visSq || d2 === 0) continue;
          out.push(o);
        }
      }
    }
    return out;
  }

  step() {
    const cfg = this.cfg;
    const { maxSpeed, maxForce } = cfg;
    const { grid, cols, cell } = this._hash();

    for (const b of this.boids) {
      const near = this._neighbors(b, grid, cols, cell);

      // Three Reynolds rules → one clamped steering force (pure core).
      let force = flockForce(b, near, cfg);

      // Predator flee: an extra steering force away from the cursor, layered on
      // top of the flock force.
      if (this.predator) {
        const dx = b.x - this.predator.x;
        const dy = b.y - this.predator.y;
        const d2 = dx * dx + dy * dy;
        if (d2 < 140 * 140 && d2 > 0) {
          const flee = separation(b, [this.predator], maxSpeed, maxForce);
          force = limit({ x: force.x + flee.x * 2.4, y: force.y + flee.y * 2.4 }, maxForce * 3.4);
        }
      }

      // Integrate in place, keeping boids from stalling below half speed.
      integrate(b, force, cfg, { mutate: true, minSpeedFactor: 0.5 });

      // toroidal wrap
      if (b.x < 0) b.x += this.width; else if (b.x >= this.width) b.x -= this.width;
      if (b.y < 0) b.y += this.height; else if (b.y >= this.height) b.y -= this.height;
    }
  }
}

// Re-export the pure rules for convenience (e.g. debugging in the console).
export { separation, alignment, cohesion };

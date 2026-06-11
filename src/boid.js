// A single boid and the flock that coordinates them.
// Flocking follows Reynolds' three steering rules (separation, alignment,
// cohesion), accelerated with a uniform spatial hash so neighbour lookups
// stay cheap even with several hundred agents.

const TAU = Math.PI * 2;

export class Boid {
  constructor(x, y, w, h) {
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
    this.cfg = {
      sep: 1.6, ali: 1.0, coh: 0.9,
      vision: 60, maxSpeed: 3.4, maxForce: 0.06,
    };
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

  step() {
    const { sep, ali, coh, vision, maxSpeed, maxForce } = this.cfg;
    const visSq = vision * vision;
    const { grid, cols, cell } = this._hash();

    for (const b of this.boids) {
      let sx = 0, sy = 0;          // separation accumulator
      let ax = 0, ay = 0, an = 0;  // alignment
      let cx = 0, cy = 0, cn = 0;  // cohesion

      const gx = Math.min(cols - 1, Math.max(0, Math.floor(b.x / cell)));
      const gy = Math.max(0, Math.floor(b.y / cell));

      for (let oy = gy - 1; oy <= gy + 1; oy++) {
        if (oy < 0) continue;
        for (let ox = gx - 1; ox <= gx + 1; ox++) {
          if (ox < 0 || ox >= cols) continue;
          const bucket = grid.get(oy * cols + ox);
          if (!bucket) continue;
          for (const o of bucket) {
            if (o === b) continue;
            const dx = b.x - o.x, dy = b.y - o.y;
            const d2 = dx * dx + dy * dy;
            if (d2 > visSq || d2 === 0) continue;
            sx += dx / d2; sy += dy / d2;          // push harder when closer
            ax += o.vx; ay += o.vy; an++;
            cx += o.x; cy += o.y; cn++;
          }
        }
      }

      let fx = 0, fy = 0;

      // separation
      if (sx || sy) { const [vx, vy] = limitTo(sx, sy, maxSpeed, b); fx += (vx) * sep; fy += (vy) * sep; }
      // alignment
      if (an) { const [vx, vy] = limitTo(ax / an, ay / an, maxSpeed, b); fx += vx * ali; fy += vy * ali; }
      // cohesion
      if (cn) { const [vx, vy] = limitTo(cx / cn - b.x, cy / cn - b.y, maxSpeed, b); fx += vx * coh; fy += vy * coh; }

      // predator flee
      if (this.predator) {
        const dx = b.x - this.predator.x, dy = b.y - this.predator.y;
        const d2 = dx * dx + dy * dy;
        if (d2 < 140 * 140 && d2 > 0) {
          const [vx, vy] = limitTo(dx, dy, maxSpeed, b);
          const w = 2.4;
          fx += vx * w; fy += vy * w;
        }
      }

      // clamp the combined steering force
      const fm = Math.hypot(fx, fy);
      if (fm > maxForce) { fx = (fx / fm) * maxForce; fy = (fy / fm) * maxForce; }

      b.vx += fx; b.vy += fy;

      // clamp speed
      const sp = Math.hypot(b.vx, b.vy);
      if (sp > maxSpeed) { b.vx = (b.vx / sp) * maxSpeed; b.vy = (b.vy / sp) * maxSpeed; }
      else if (sp < maxSpeed * 0.5 && sp > 0) { b.vx = (b.vx / sp) * maxSpeed * 0.5; b.vy = (b.vy / sp) * maxSpeed * 0.5; }

      b.x += b.vx; b.y += b.vy;

      // toroidal wrap
      if (b.x < 0) b.x += this.width; else if (b.x >= this.width) b.x -= this.width;
      if (b.y < 0) b.y += this.height; else if (b.y >= this.height) b.y -= this.height;
    }
  }
}

// Normalise a desired direction to max speed, then return the steering vector
// (desired − current velocity). Keeps each rule on the same scale.
function limitTo(dx, dy, maxSpeed, b) {
  const m = Math.hypot(dx, dy);
  if (m === 0) return [0, 0];
  const dvx = (dx / m) * maxSpeed;
  const dvy = (dy / m) * maxSpeed;
  return [dvx - b.vx, dvy - b.vy];
}

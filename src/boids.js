// Framework-free core of the flocking simulation.
//
// Pure functions only: no canvas, no DOM, no globals. A "boid" here is any
// object with numeric { x, y, vx, vy }. The three classic Reynolds steering
// rules (separation, alignment, cohesion) each return a *steering force*
// vector { x, y } expressed as (desired velocity - current velocity), so they
// all live on the same scale and can be weighted and summed.
//
// Reference: Craig Reynolds, "Flocks, Herds, and Schools" (1987).

// ---- vector helpers --------------------------------------------------------

export function magnitude(v) {
  return Math.hypot(v.x, v.y);
}

// Scale a vector to a target length. A zero vector stays zero.
export function setMagnitude(v, len) {
  const m = magnitude(v);
  if (m === 0) return { x: 0, y: 0 };
  return { x: (v.x / m) * len, y: (v.y / m) * len };
}

// Clamp a vector's length to at most `max`. Shorter vectors pass through.
export function limit(v, max) {
  const m = magnitude(v);
  if (m > max && m > 0) return { x: (v.x / m) * max, y: (v.y / m) * max };
  return { x: v.x, y: v.y };
}

// Squared distance — avoids a sqrt in the hot neighbour loop.
export function distSq(a, b) {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return dx * dx + dy * dy;
}

// ---- neighbour finding -----------------------------------------------------

// Every boid in `boids`, excluding `self`, whose centre lies within `vision`.
// Brute-force O(n) scan; the demo layers a spatial hash on top for speed, but
// the result is identical to this reference set.
export function neighbors(self, boids, vision) {
  const visSq = vision * vision;
  const out = [];
  for (const other of boids) {
    if (other === self) continue;
    const d2 = distSq(self, other);
    if (d2 > visSq || d2 === 0) continue;
    out.push(other);
  }
  return out;
}

// Turn a desired velocity into a steering force relative to current velocity,
// matching Reynolds' steer = desired - velocity. The desired direction is
// normalised to maxSpeed, then the resulting force is clamped to maxForce.
function steerToward(desired, boid, maxSpeed, maxForce) {
  if (desired.x === 0 && desired.y === 0) return { x: 0, y: 0 };
  const aimed = setMagnitude(desired, maxSpeed);
  const force = { x: aimed.x - boid.vx, y: aimed.y - boid.vy };
  return limit(force, maxForce);
}

// ---- the three rules -------------------------------------------------------
// Each takes the boid, its neighbour list, and limits; each returns a steering
// force { x, y }. With no neighbours every rule returns the zero vector.

// SEPARATION: steer away from neighbours, weighted by inverse distance so the
// closest flockmates push hardest.
export function separation(boid, neighborList, maxSpeed, maxForce) {
  let dx = 0;
  let dy = 0;
  for (const other of neighborList) {
    const ox = boid.x - other.x;
    const oy = boid.y - other.y;
    const d2 = ox * ox + oy * oy;
    if (d2 === 0) continue;
    dx += ox / d2;
    dy += oy / d2;
  }
  return steerToward({ x: dx, y: dy }, boid, maxSpeed, maxForce);
}

// ALIGNMENT: steer toward the average heading (velocity) of neighbours.
export function alignment(boid, neighborList, maxSpeed, maxForce) {
  if (neighborList.length === 0) return { x: 0, y: 0 };
  let vx = 0;
  let vy = 0;
  for (const other of neighborList) {
    vx += other.vx;
    vy += other.vy;
  }
  vx /= neighborList.length;
  vy /= neighborList.length;
  return steerToward({ x: vx, y: vy }, boid, maxSpeed, maxForce);
}

// COHESION: steer toward the average position (centroid) of neighbours.
export function cohesion(boid, neighborList, maxSpeed, maxForce) {
  if (neighborList.length === 0) return { x: 0, y: 0 };
  let cx = 0;
  let cy = 0;
  for (const other of neighborList) {
    cx += other.x;
    cy += other.y;
  }
  cx /= neighborList.length;
  cy /= neighborList.length;
  const desired = { x: cx - boid.x, y: cy - boid.y };
  return steerToward(desired, boid, maxSpeed, maxForce);
}

// ---- combined update -------------------------------------------------------

// Default tuning, shared with the demo so the page and the tests run the same
// numbers.
export const DEFAULTS = {
  sep: 1.6,
  ali: 1.0,
  coh: 0.9,
  vision: 60,
  maxSpeed: 3.4,
  maxForce: 0.06,
};

// Sum the three weighted rules into one steering force, clamped to maxForce.
export function flockForce(boid, neighborList, cfg) {
  const { sep, ali, coh, maxSpeed, maxForce } = cfg;
  const s = separation(boid, neighborList, maxSpeed, maxForce);
  const a = alignment(boid, neighborList, maxSpeed, maxForce);
  const c = cohesion(boid, neighborList, maxSpeed, maxForce);
  const force = {
    x: s.x * sep + a.x * ali + c.x * coh,
    y: s.y * sep + a.y * ali + c.y * coh,
  };
  return limit(force, maxForce);
}

// Apply a steering force to a boid and integrate one step. Velocity is clamped
// to maxSpeed (and, if `minSpeedFactor` is given, nudged up to that fraction of
// maxSpeed so boids never stall). Returns the updated boid; if `mutate` is
// true the input boid is updated in place.
export function integrate(boid, force, cfg, opts = {}) {
  const { maxSpeed } = cfg;
  const { minSpeedFactor = 0, mutate = false } = opts;

  let vx = boid.vx + force.x;
  let vy = boid.vy + force.y;

  const speed = Math.hypot(vx, vy);
  if (speed > maxSpeed && speed > 0) {
    vx = (vx / speed) * maxSpeed;
    vy = (vy / speed) * maxSpeed;
  } else if (minSpeedFactor > 0 && speed > 0 && speed < maxSpeed * minSpeedFactor) {
    const min = maxSpeed * minSpeedFactor;
    vx = (vx / speed) * min;
    vy = (vy / speed) * min;
  }

  const target = mutate ? boid : { ...boid };
  target.vx = vx;
  target.vy = vy;
  target.x = boid.x + vx;
  target.y = boid.y + vy;
  return target;
}

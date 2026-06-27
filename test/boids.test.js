import { test } from "node:test";
import assert from "node:assert/strict";

import {
  separation,
  alignment,
  cohesion,
  neighbors,
  flockForce,
  integrate,
  limit,
  magnitude,
  DEFAULTS,
} from "../src/boids.js";

const cfg = DEFAULTS;
const { maxSpeed, maxForce } = cfg;

// A boid at rest (zero velocity) so a rule's steering force points purely in
// the direction of its desired velocity, with no current-velocity offset.
function restingBoid(x, y) {
  return { x, y, vx: 0, vy: 0 };
}

test("cohesion steers toward the centroid of neighbours", () => {
  // Boid at origin; neighbours sit to the right, so the centroid is to the
  // right and cohesion should push +x with no vertical component.
  const boid = restingBoid(0, 0);
  const near = [
    { x: 100, y: 10, vx: 0, vy: 0 },
    { x: 100, y: -10, vx: 0, vy: 0 },
  ]; // centroid = (100, 0)

  const f = cohesion(boid, near, maxSpeed, maxForce);

  assert.ok(f.x > 0, "cohesion should steer toward +x (the centroid)");
  assert.ok(Math.abs(f.y) < 1e-9, "no net vertical steering for a symmetric pair");
});

test("separation produces a force pointing away from a too-close neighbour", () => {
  // Neighbour is just to the right of the boid → boid should be pushed left.
  const boid = restingBoid(0, 0);
  const near = [{ x: 5, y: 0, vx: 0, vy: 0 }];

  const f = separation(boid, near, maxSpeed, maxForce);

  assert.ok(f.x < 0, "separation should steer away (-x) from a neighbour on the +x side");
  assert.ok(Math.abs(f.y) < 1e-9, "no vertical component for a purely horizontal neighbour");
});

test("alignment steers velocity toward the average heading of neighbours", () => {
  // Boid moving up (+y); neighbours all move right (+x). Steering = desired -
  // velocity, so it should gain +x velocity and shed +y velocity.
  const boid = { x: 0, y: 0, vx: 0, vy: maxSpeed };
  const near = [
    { x: 10, y: 0, vx: maxSpeed, vy: 0 },
    { x: -10, y: 0, vx: maxSpeed, vy: 0 },
  ]; // average heading points +x

  const f = alignment(boid, near, maxSpeed, maxForce);

  assert.ok(f.x > 0, "should steer toward the neighbours' +x heading");
  assert.ok(f.y < 0, "should steer away from its own +y heading");
});

test("speed is clamped to maxSpeed and steering force to maxForce", () => {
  // A neighbour far to the right yields a strong cohesion pull; the returned
  // force must still be capped at maxForce.
  const boid = restingBoid(0, 0);
  const near = [{ x: 500, y: 0, vx: 0, vy: 0 }];

  const force = flockForce(boid, near, cfg);
  assert.ok(
    magnitude(force) <= maxForce + 1e-9,
    `steering force ${magnitude(force)} should be <= maxForce ${maxForce}`,
  );

  // Integrate from a velocity already at the limit; resulting speed must not
  // exceed maxSpeed.
  const fast = { x: 0, y: 0, vx: maxSpeed, vy: 0 };
  const big = { x: maxForce * 100, y: 0 }; // far larger than any real force
  const moved = integrate(fast, big, cfg);
  const speed = Math.hypot(moved.vx, moved.vy);
  assert.ok(
    speed <= maxSpeed + 1e-9,
    `speed ${speed} should be clamped to maxSpeed ${maxSpeed}`,
  );

  // limit() itself never returns a longer vector than asked.
  const capped = limit({ x: 10, y: 0 }, 3);
  assert.equal(magnitude(capped), 3);
});

test("a boid with no neighbours keeps its velocity (rules contribute zero)", () => {
  const boid = { x: 0, y: 0, vx: 1.2, vy: -0.7 };
  const near = []; // nobody in range

  // Each rule returns the zero vector.
  assert.deepEqual(separation(boid, near, maxSpeed, maxForce), { x: 0, y: 0 });
  assert.deepEqual(alignment(boid, near, maxSpeed, maxForce), { x: 0, y: 0 });
  assert.deepEqual(cohesion(boid, near, maxSpeed, maxForce), { x: 0, y: 0 });

  // So the combined force is zero and integrating leaves velocity unchanged.
  const force = flockForce(boid, near, cfg);
  assert.deepEqual(force, { x: 0, y: 0 });

  const moved = integrate(boid, force, cfg);
  assert.equal(moved.vx, 1.2, "vx unchanged with no neighbours");
  assert.equal(moved.vy, -0.7, "vy unchanged with no neighbours");
});

test("neighbors() returns only boids within the vision radius, excluding self", () => {
  const self = restingBoid(0, 0);
  const inRange = { x: 30, y: 0, vx: 0, vy: 0 };
  const outOfRange = { x: 1000, y: 0, vx: 0, vy: 0 };
  const all = [self, inRange, outOfRange];

  const found = neighbors(self, all, cfg.vision);

  assert.equal(found.length, 1);
  assert.equal(found[0], inRange);
});

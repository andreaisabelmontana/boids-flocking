# Boids Flocking

An interactive **boids flocking** simulation that runs entirely in the browser. Hundreds of agents self-organise into flowing, starling-like murmurations from just three local steering rules — no central control.

**▶ Live:** https://andreaisabelmontana.github.io/boids-flocking/

> **Not an original idea.** This recreates the classic boids concept — I didn't invent it. I rebuilt it from scratch, my own way, to understand how it actually works.

## The three rules

Each boid steers using only the flockmates inside its **vision radius** `r`. Let `N` be that neighbour set. Every rule returns a *steering force* expressed as `desired − velocity` (Reynolds' formulation), so all three live on the same scale, can be weighted, and summed.

A desired direction `d` is converted to a force by aiming it at `maxSpeed` and subtracting current velocity, then clamping to `maxForce`:

```
steer(d) = limit( normalize(d) · maxSpeed − v , maxForce )
```

- **Separation** — steer away from crowding. Each neighbour contributes a vector pointing *away* from it, weighted by inverse-square distance so the closest push hardest:

  ```
  d = Σ (p − pₒ) / |p − pₒ|²      for o in N
  ```

- **Alignment** — steer toward the average heading of neighbours:

  ```
  d = (1/|N|) Σ vₒ                 for o in N
  ```

- **Cohesion** — steer toward the centroid (average position) of neighbours:

  ```
  d = (1/|N|) Σ pₒ − p             for o in N
  ```

The combined force is `sep·separation + ali·alignment + coh·cohesion`, clamped to `maxForce`. It's added to velocity, velocity is clamped to `maxSpeed`, and position integrates by one step. With **no neighbours** all three rules return the zero vector, so a lone boid keeps its velocity.

Balancing the weights produces emergent flocking — the sliders push the system from rigid columns to loose, chaotic swarms.

## Structure

The flocking math is a framework-free, dependency-free ES module of pure functions — no canvas, no DOM. The demo imports it, so the tested code *is* the simulation.

```
index.html      page + controls
styles.css
src/boids.js    pure core: the 3 rules, neighbour finding, steer + integrate (tested)
src/boid.js     stateful Flock wrapper: spatial hash, predator, toroidal wrap
src/main.js     canvas setup, controls, render loop
test/boids.test.js
```

A boid is just `{ x, y, vx, vy }`. The pure functions accept plain objects, which is exactly what the tests pass in.

## Features

- Real-time control of force weights, vision radius, speed, and flock size (up to 600 boids)
- **Cursor predator** mode — boids flee the pointer, carving holes through the flock
- Motion trails for the classic long-exposure murmuration look
- Toroidal (wrap-around) world and a uniform **spatial hash** so neighbour lookups stay fast at high counts

## Run the demo

No build step, no dependencies. Open `index.html` in any modern browser, or serve the folder:

```sh
python -m http.server      # then open http://localhost:8000
```

## Run the tests

The core is tested with Node's built-in runner — **no npm install, no dependencies** (Node ≥ 18; developed on Node 24):

```sh
node --test
```

Real output:

```
✔ cohesion steers toward the centroid of neighbours (0.749ms)
✔ separation produces a force pointing away from a too-close neighbour (0.1435ms)
✔ alignment steers velocity toward the average heading of neighbours (0.1621ms)
✔ speed is clamped to maxSpeed and steering force to maxForce (0.9954ms)
✔ a boid with no neighbours keeps its velocity (rules contribute zero) (0.6623ms)
✔ neighbors() returns only boids within the vision radius, excluding self (0.1607ms)
ℹ tests 6
ℹ suites 0
ℹ pass 6
ℹ fail 0
```

The tests check each rule's direction (centroid pull, away-from-crowding push, heading match), that force and speed are clamped to `maxForce`/`maxSpeed`, that a neighbourless boid drifts unchanged, and that neighbour finding respects the vision radius.

## License

MIT — see [LICENSE](LICENSE).

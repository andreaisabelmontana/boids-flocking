# Murmuration

An interactive **boids flocking** simulation that runs entirely in the browser. Hundreds of agents self-organise into flowing, starling-like murmurations from just three local steering rules — no central control.

**▶ Live:** https://andreaisabelmontana.github.io/murmuration/

## The three rules

Each boid steers using only what it can see within its vision radius:

- **Separation** — steer away from crowding nearby flockmates
- **Alignment** — steer toward the average heading of neighbours
- **Cohesion** — steer toward the average position of neighbours

Balancing the three produces emergent flocking. The sliders let you push the system from rigid columns to loose, chaotic swarms.

## Features

- Real-time control of force weights, vision radius, speed, and flock size (up to 600 boids)
- **Cursor predator** mode — boids flee the pointer, carving holes through the flock
- Motion trails for the classic long-exposure murmuration look
- Toroidal (wrap-around) world and a uniform **spatial hash** so neighbour lookups stay fast at high counts

## Tech

Vanilla JavaScript ES modules + Canvas 2D. No build step, no dependencies.

```
index.html
styles.css
src/boid.js   # Boid + Flock (spatial hash, steering)
src/main.js   # canvas setup, controls, render loop
```

Open `index.html` in any modern browser, or serve the folder with any static server.

## License

MIT — see [LICENSE](LICENSE).

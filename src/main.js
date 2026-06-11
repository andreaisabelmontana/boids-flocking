import { Flock } from "./boid.js";

const canvas = document.getElementById("stage");
const ctx = canvas.getContext("2d");

let DPR = Math.min(2, window.devicePixelRatio || 1);
const flock = new Flock(window.innerWidth, window.innerHeight);

function fit() {
  DPR = Math.min(2, window.devicePixelRatio || 1);
  canvas.width = window.innerWidth * DPR;
  canvas.height = window.innerHeight * DPR;
  canvas.style.width = window.innerWidth + "px";
  canvas.style.height = window.innerHeight + "px";
  ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
  flock.resize(window.innerWidth, window.innerHeight);
}
window.addEventListener("resize", fit);
fit();

flock.setCount(240);

// ---- controls ----
const ui = {
  count: document.getElementById("count"),
  sep: document.getElementById("sep"),
  ali: document.getElementById("ali"),
  coh: document.getElementById("coh"),
  vis: document.getElementById("vis"),
  spd: document.getElementById("spd"),
  trails: document.getElementById("trails"),
  predator: document.getElementById("predator"),
};
const out = (k) => document.querySelector(`[data-out="${k}"]`);

function sync() {
  flock.setCount(+ui.count.value);
  flock.cfg.sep = +ui.sep.value;
  flock.cfg.ali = +ui.ali.value;
  flock.cfg.coh = +ui.coh.value;
  flock.cfg.vision = +ui.vis.value;
  flock.cfg.maxSpeed = +ui.spd.value;
  out("count").textContent = ui.count.value;
  out("sep").textContent = (+ui.sep.value).toFixed(2);
  out("ali").textContent = (+ui.ali.value).toFixed(2);
  out("coh").textContent = (+ui.coh.value).toFixed(2);
  out("vis").textContent = ui.vis.value;
  out("spd").textContent = (+ui.spd.value).toFixed(1);
}
Object.values(ui).forEach((el) => el.addEventListener("input", sync));
sync();

// panel collapse
const panel = document.getElementById("panel");
document.getElementById("collapse").addEventListener("click", () => panel.classList.toggle("hidden"));

// cursor → predator / attractor
let pointer = null;
function onMove(e) {
  const t = e.touches ? e.touches[0] : e;
  pointer = { x: t.clientX, y: t.clientY };
}
window.addEventListener("mousemove", onMove);
window.addEventListener("touchmove", onMove, { passive: true });
window.addEventListener("mouseleave", () => (pointer = null));

// ---- render ----
function draw() {
  flock.predator = ui.predator.checked ? pointer : null;
  flock.step();

  if (ui.trails.checked) {
    ctx.fillStyle = "rgba(5, 6, 13, 0.18)";
    ctx.fillRect(0, 0, flock.width, flock.height);
  } else {
    ctx.clearRect(0, 0, flock.width, flock.height);
    ctx.fillStyle = "#05060d";
    ctx.fillRect(0, 0, flock.width, flock.height);
  }

  for (const b of flock.boids) {
    const ang = Math.atan2(b.vy, b.vx);
    ctx.save();
    ctx.translate(b.x, b.y);
    ctx.rotate(ang);
    ctx.fillStyle = `hsl(${b.hue}, 90%, 68%)`;
    ctx.beginPath();
    ctx.moveTo(7, 0);
    ctx.lineTo(-5, 3.6);
    ctx.lineTo(-2.5, 0);
    ctx.lineTo(-5, -3.6);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  // draw the predator marker
  if (flock.predator) {
    ctx.beginPath();
    ctx.arc(flock.predator.x, flock.predator.y, 8, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(255, 90, 110, 0.9)";
    ctx.fill();
  }

  requestAnimationFrame(draw);
}
draw();

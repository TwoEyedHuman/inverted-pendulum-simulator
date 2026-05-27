# Inverted Pendulum Simulator

> A real-time web demo of classic control theory — balance a pole on a cart using PID or LQR, tune the gains live, and apply disturbances to see how each controller recovers.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Repository Structure](#repository-structure)
3. [Technology Stack](#technology-stack)
4. [Environment Strategy](#environment-strategy)
5. [Pre-Flight Checklist](#pre-flight-checklist)
6. [Implementation Stories](#implementation-stories)
7. [Config Management](#config-management)
8. [Definition of Done](#definition-of-done)

---

## Architecture Overview

```
Browser (Canvas + Sliders)
        │
        │  WebSocket (simulation state JSON)
        │  HTTP (static assets)
        ▼
┌─────────────────────────┐
│      FastAPI (Python)   │
│                         │
│  ┌───────────────────┐  │
│  │  Simulation Loop  │  │
│  │  - Physics engine │  │
│  │  - PID controller │  │
│  │  - LQR controller │  │
│  └───────────────────┘  │
└─────────────────────────┘
        │
        ▼
   Docker Container
   (Fly.io — auto-stop on idle)
```

### Key Design Decisions

- **Single container, single user:** No session management or multi-tenancy needed. One global simulation loop, one WebSocket connection at a time. Simplifies everything.
- **Server-side physics:** The simulation runs on the server and streams lightweight state JSON (x, θ, ẋ, θ̇, F) to the browser at ~60fps. The browser only renders — no physics logic in JS.
- **Stateless on reconnect:** Fly.io machines spin down on idle. Each new connection starts a fresh simulation. This is an explicit design decision, not a limitation.
- **Controller switching without reset:** PID and LQR share the same state vector. Switching controllers mid-simulation is seamless. When switching away from PID, its integral accumulator is zeroed to avoid stale history poisoning the next PID session.
- **Vanilla JS frontend:** No build toolchain. FastAPI serves a single `index.html` with inline JS and Canvas rendering. Easy to iframe-embed on a portfolio site.

---

## Repository Structure

```
inverted-pendulum/
├── README.md
├── Dockerfile
├── fly.toml                    ← Fly.io config with auto_stop_machines
├── Makefile
├── .env.example                ← committed; .env is gitignored
│
├── app/
│   ├── main.py                 ← FastAPI app, WebSocket endpoint, static file serving
│   ├── simulation.py           ← Physics engine (nonlinear equations of motion)
│   ├── controllers/
│   │   ├── __init__.py
│   │   ├── pid.py              ← PID controller
│   │   └── lqr.py              ← LQR controller (linearized model + gain computation)
│   └── static/
│       └── index.html          ← Canvas renderer + WebSocket client + UI controls
│
└── tests/
    ├── test_simulation.py
    ├── test_pid.py
    └── test_lqr.py
```

---

## Technology Stack

| Layer | Technology | Reason |
|---|---|---|
| Simulation & Backend | Python 3.12 + FastAPI | NumPy/SciPy for control math; async WebSocket support |
| Physics / Control Math | NumPy, SciPy (`linalg.solve_continuous_are`) | LQR gain computation via algebraic Riccati equation |
| Frontend | Vanilla JS + HTML5 Canvas | No build step; trivially embeddable via iframe |
| WebSocket | FastAPI native (`websockets`) | Real-time state streaming at ~60fps |
| Containers | Docker | Single container, consistent across envs |
| Hosting | Fly.io | Auto-stop on idle, Docker-native |

---

## Environment Strategy

| | Local | Production (Fly.io) |
|---|---|---|
| Domain | `localhost:8000` | `your-app.fly.dev` |
| TLS | none | Fly.io managed |
| Config | `.env` file | `fly secrets set` / `fly.toml` |
| Deploy | `make dev` | `fly deploy` |

---

## Pre-Flight Checklist

Run before first build and after any environment change:

```bash
# Docker is running
docker info > /dev/null && echo "✓ Docker running" || echo "✗ Docker not running"

# Python deps available (for local dev without Docker)
python -m pip show fastapi numpy scipy > /dev/null && echo "✓ Python deps ok" || echo "✗ pip install -r requirements.txt"

# Port is free
lsof -i :8000 | grep LISTEN && echo "⚠ port 8000 in use" || echo "✓ port 8000 free"

# .env exists
test -f .env && echo "✓ .env found" || echo "✗ copy .env.example to .env"
```

---

## Implementation Stories

### Story Template

Each story is one Claude Code session. Keep them tight.

```
#### Story X.Y — Title

**Context:** What already exists. What this story builds on. (2-3 sentences max)

**Assumptions:**
- List explicit prerequisites — files, running services, installed tools
- If an assumption is wrong, the story will fail; fix the assumption first

**Tasks:**
- Imperative, specific, one action per bullet
- Include file paths

**Out of Scope:**
- Anything that might tempt scope creep

**Acceptance Criteria:**
- [ ] Unit tests pass
- [ ] Container builds and runs
- [ ] Observable behavior verified (curl, browser, WebSocket message)
```

---

### EPIC 1 — Repo Scaffolding & Physics Foundation

**Epic Goal:** The physics engine is implemented, tested, and numerically verified. No web server yet — just the math.

---

#### Story 1.1 — Repository Scaffold

**Context:** Starting from scratch.

**Assumptions:**
- Python 3.12 and Docker are installed locally
- `fly` CLI is installed (`curl -L https://fly.io/install.sh | sh`)

**Tasks:**
- Create directory structure per [Repository Structure](#repository-structure)
- `requirements.txt` — `fastapi`, `uvicorn`, `numpy`, `scipy`, `websockets`, `pytest`
- `Dockerfile` — Python 3.12 slim base, non-root user, `CMD ["uvicorn", "app.main:app"]`
- `.env.example` — `SIM_TICK_RATE_HZ=60`, `CORS_ORIGIN=*`
- `Makefile` targets: `dev` (uvicorn with reload), `docker-build`, `docker-run`, `test`, `preflight`
- `app/__init__.py`, `app/controllers/__init__.py`, `tests/` with empty `__init__.py`
- `.gitignore` — `__pycache__/`, `.env`, `*.pyc`, `.pytest_cache/`

**Out of Scope:** Any simulation logic, web server routes, frontend.

**Acceptance Criteria:**
- [ ] `make preflight` passes
- [ ] `docker build -t pendulum .` completes without error
- [ ] `make test` runs (zero tests, zero failures)
- [ ] `git status` shows no `.env` tracked

---

#### Story 1.2 — Nonlinear Physics Engine

**Context:** Story 1.1 complete. Scaffold exists with no simulation logic.

**Assumptions:**
- `numpy` is available
- State vector is `[x, x_dot, theta, theta_dot]` — theta=0 is upright

**System parameters to hardcode (tunable later):**
- Cart mass M = 1.0 kg
- Pole mass m = 0.1 kg
- Pole half-length l = 0.5 m
- Gravity g = 9.81 m/s²
- Friction (cart) b = 0.1 N·s/m

**Tasks:**
- `app/simulation.py` — `SimulationState` dataclass: `x`, `x_dot`, `theta`, `theta_dot`, `force`, `time`
- Implement `step(state, F, dt)` using the full nonlinear equations of motion for a cart-pole system (RK4 integrator — do not use Euler)
- Implement `is_failed(state)` — returns True if `|theta| > 0.5 rad` (~28°) or `|x| > 2.4 m` (cart hits track limit)
- Implement `reset()` — returns a new state with `theta = 0.05 rad` (slight offset to make balancing non-trivial)
- `app/simulation.py` must have no FastAPI imports — pure physics, no web dependencies

**Out of Scope:** Any controller logic, web server, visualization.

**Acceptance Criteria:**
- [ ] `tests/test_simulation.py` — zero-force simulation from upright falls and `is_failed()` returns True within 2 seconds of simulation time
- [ ] RK4 conserves energy within 1% over 1 second with no friction and no force (verifiable analytically)
- [ ] `reset()` state with zero force fails within expected time range
- [ ] No FastAPI or web imports anywhere in `simulation.py`

---

#### Story 1.3 — PID Controller

**Context:** Story 1.2 complete. Physics engine is tested and working.

**Assumptions:**
- State vector convention from Story 1.2 is stable
- PID controls pole angle only (theta). Cart position correction is out of scope for PID.

**Tasks:**
- `app/controllers/pid.py` — `PIDController` class
  - Constructor: `kp`, `ki`, `kd`, `output_clamp` (max force magnitude, default 20N)
  - `compute(theta, theta_dot, dt) -> float` — returns force F
  - `reset()` — zeros the integral accumulator (called on controller switch)
  - Derivative term uses `theta_dot` directly (already available in state — avoids numerical differentiation noise)
- `tests/test_pid.py`
  - Test that a positive theta produces a negative (corrective) force
  - Test that `reset()` zeroes accumulated integral
  - Test output clamping

**Out of Scope:** LQR, web server, any visualization.

**Acceptance Criteria:**
- [ ] All `tests/test_pid.py` tests pass
- [ ] PID + physics simulation: starting from `theta=0.05 rad`, a well-tuned set of gains (e.g. Kp=50, Ki=1, Kd=10) keeps `|theta| < 0.5 rad` for at least 10 seconds of simulation time (verified in test)
- [ ] `reset()` zeroes integral accumulator

---

#### Story 1.4 — LQR Controller

**Context:** Stories 1.2 and 1.3 complete. PID is verified working.

**Background for implementation:**
The LQR controller linearizes the nonlinear equations of motion around the upright equilibrium (theta=0), producing matrices A and B of the linear system `ẋ = Ax + Bu`. It then solves the continuous algebraic Riccati equation (CARE) to find the optimal gain matrix K, such that `F = -K @ state`. SciPy's `linalg.solve_continuous_are` handles the Riccati solve.

**Tasks:**
- `app/controllers/lqr.py` — `LQRController` class
  - Constructor: `q_weights` (list of 4 diagonal Q values, one per state), `r_weight` (scalar R — penalty on control effort)
  - `_compute_gains()` — linearize the cart-pole system, build A and B matrices, solve CARE via `scipy.linalg.solve_continuous_are`, store gain matrix K
  - `compute(state: SimulationState) -> float` — returns `float(-K @ state_vector)`, clamped to ±20N
  - `reset()` — no-op (LQR is stateless)
  - Default weights: Q = diag([1, 1, 10, 1]) (penalize angle heavily), R = 0.01
- `tests/test_lqr.py`
  - Test that default gains produce a stable controller (same 10-second simulation test as PID)
  - Test that increasing R weight reduces peak force magnitude
  - Test that K matrix is recomputed when weights change

**Out of Scope:** UI controls for Q/R, any web layer.

**Acceptance Criteria:**
- [ ] All `tests/test_lqr.py` tests pass
- [ ] Default LQR gains keep `|theta| < 0.5 rad` for 10+ seconds from `theta=0.05 rad`
- [ ] LQR recovery from a larger disturbance (`theta=0.2 rad`) demonstrably smoother than PID (lower peak force, fewer oscillations) — verified in test with a comparison fixture

---

### EPIC 1 Integration Gate

Before starting Epic 2:

- [ ] `make test` — all tests pass, zero failures
- [ ] Physics engine verified numerically (energy conservation test)
- [ ] Both controllers independently verified to stabilize the system in simulation
- [ ] LQR demonstrably smoother than PID on disturbance recovery (test assertion passes)
- [ ] No web or FastAPI imports in `simulation.py` or `controllers/`

---

### EPIC 2 — FastAPI Backend & WebSocket

**Epic Goal:** The simulation runs server-side and streams live state to a WebSocket client. Controllable via JSON messages.

---

#### Story 2.1 — FastAPI App & HTTP Health

**Context:** Epic 1 complete. Pure physics and controller logic is tested. No web layer exists yet.

**Assumptions:**
- `fastapi` and `uvicorn` are in `requirements.txt`
- `app/main.py` does not exist yet

**Tasks:**
- `app/main.py` — FastAPI app instance
- `GET /health` → `{"status": "ok"}`
- Serve `app/static/index.html` at `GET /` (create a placeholder HTML file for now)
- Configure CORS from `CORS_ORIGIN` env var
- Graceful shutdown — simulation loop (coming in 2.2) should stop cleanly on SIGTERM

**Out of Scope:** WebSocket, simulation loop, any real frontend.

**Acceptance Criteria:**
- [ ] `make dev` starts uvicorn without error
- [ ] `curl http://localhost:8000/health` → `{"status":"ok"}`
- [ ] `curl http://localhost:8000/` → returns HTML (placeholder is fine)
- [ ] `docker build && docker run -p 8000:8000` — same checks pass in container

---

#### Story 2.2 — Simulation Loop & WebSocket Endpoint

**Context:** Story 2.1 complete. FastAPI app serves health and static file.

**Assumptions:**
- `SimulationState`, `PIDController`, `LQRController` from Epic 1 are importable
- One WebSocket connection at a time — no concurrency handling needed

**Tasks:**
- `app/main.py` — `SimulationManager` class (or module-level singleton)
  - Holds current `SimulationState`, active controller (PID or LQR), and a running asyncio task
  - `start_loop()` — runs at `SIM_TICK_RATE_HZ`, calls `simulation.step()`, broadcasts state to connected WebSocket
  - `reset()` — resets state to `simulation.reset()`, zeroes PID accumulator
- `WS /ws` endpoint
  - On connect: start simulation loop if not running, send initial state
  - On disconnect: pause simulation loop
  - Outbound message schema: `{"x": float, "x_dot": float, "theta": float, "theta_dot": float, "force": float, "time": float, "failed": bool, "controller": "pid"|"lqr"}`
  - Inbound message schema (from browser): `{"action": "set_controller"|"set_pid_gains"|"set_lqr_weights"|"disturbance"|"reset", ...params}`
- On `set_controller` to PID: call `pid.reset()` before activating
- On `disturbance`: add an impulse of `{"magnitude": float}` to `theta_dot`
- On `failed`: broadcast state with `"failed": true`, loop keeps running (browser decides to reset)

**Out of Scope:** Frontend rendering, sliders, canvas — just the WebSocket protocol.

**Acceptance Criteria:**
- [ ] `wscat -c ws://localhost:8000/ws` (or equivalent) — receives state JSON at ~60fps
- [ ] Send `{"action": "set_controller", "controller": "lqr"}` — subsequent frames show `"controller": "lqr"`
- [ ] Send `{"action": "disturbance", "magnitude": 0.3}` — theta visibly spikes in subsequent frames
- [ ] Send `{"action": "reset"}` — state returns to near-zero theta
- [ ] Disconnect and reconnect — simulation resumes cleanly
- [ ] `tests/test_simulation.py` — add an async test that connects via WebSocket and verifies message schema

---

### EPIC 2 Integration Gate

- [ ] WebSocket streams state JSON at target tick rate
- [ ] All inbound actions (`set_controller`, `set_pid_gains`, `set_lqr_weights`, `disturbance`, `reset`) handled correctly
- [ ] PID integral accumulator is zeroed on controller switch (verified via WS test)
- [ ] Container runs cleanly: `docker run -p 8000:8000 pendulum` → WebSocket works
- [ ] Graceful shutdown: `docker stop` → container exits within 5 seconds

---

### EPIC 3 — Frontend: Canvas Renderer & Controls

**Epic Goal:** A browser page renders the simulation in real time and exposes full user controls. This is the portfolio-facing demo.

---

#### Story 3.1 — Canvas Renderer

**Context:** Epic 2 complete. WebSocket is streaming state. `app/static/index.html` is a placeholder.

**Assumptions:**
- State JSON schema from Story 2.2 is stable
- No JS framework — vanilla JS only

**Tasks:**
- `app/static/index.html` — replace placeholder with real page
- Canvas element sized to show track (±2.4m) with cart and pole drawn from state
  - Cart: rectangle centered at `x` (scaled to canvas)
  - Pole: line from cart center, angled at `theta`, length proportional to pole length
  - Track: horizontal line with end stops at ±2.4m
  - Pole color: green when `|theta| < 0.1 rad`, yellow when `< 0.3 rad`, red otherwise
- WebSocket client — connects on page load, updates canvas on each message at animation frame rate
- "FAILED" overlay displayed when `failed: true` in state
- No controls yet — just rendering

**Out of Scope:** Sliders, buttons, controller switching UI.

**Acceptance Criteria:**
- [ ] `http://localhost:8000/` — cart and pole visible, pole moves in real time
- [ ] Pole color changes correctly with angle thresholds
- [ ] "FAILED" overlay appears when pole falls
- [ ] Resizing the browser window — canvas rescales correctly
- [ ] No JS console errors

---

#### Story 3.2 — Controls UI: Controller Switch, Reset, Disturbance

**Context:** Story 3.1 complete. Canvas renders live simulation.

**Tasks:**
- Add to `index.html`:
  - Controller toggle — "PID" / "LQR" buttons (one active at a time); sends `set_controller` action
  - "Reset" button — sends `reset` action
  - "Nudge" button — sends `disturbance` with a fixed magnitude (e.g. 0.3 rad/s impulse to theta_dot). Alternatively, clicking the canvas sends a disturbance in the direction of the click relative to the pole — whichever feels more natural to implement
  - Active controller label displayed on canvas or adjacent
- "FAILED" overlay should include a "Reset" affordance (click anywhere or a button)

**Out of Scope:** Gain/weight tuning sliders (Story 3.3).

**Acceptance Criteria:**
- [ ] Toggle between PID and LQR mid-simulation — controller switches without reset
- [ ] "Nudge" visibly disturbs the pole
- [ ] "Reset" returns simulation to initial state
- [ ] "FAILED" overlay includes reset affordance and works
- [ ] No JS console errors

---

#### Story 3.3 — Gain Tuning Sliders

**Context:** Story 3.2 complete. Controller switching and disturbance work.

**Tasks:**
- PID sliders panel (visible when PID active):
  - Kp: range 0–200, default 50
  - Ki: range 0–10, default 1
  - Kd: range 0–50, default 10
  - Each slider sends `set_pid_gains` action on change with debounce (100ms)
  - Display current value next to each slider
- LQR sliders panel (visible when LQR active):
  - "Angle penalty" (maps to Q[2,2]): range 1–100, default 10
  - "Position penalty" (maps to Q[0,0]): range 1–20, default 1
  - "Effort penalty" (maps to R): range 0.001–1, default 0.01 (log scale slider)
  - Each slider sends `set_lqr_weights` action on change — backend recomputes K
  - Display current value next to each slider
- Panels swap when controller is toggled

**Out of Scope:** Advanced/raw Q matrix editing, Kalman filter controls.

**Acceptance Criteria:**
- [ ] Dragging Kp to 0 — PID immediately fails to balance (pole falls)
- [ ] Dragging Kp back up — recovery visible
- [ ] LQR effort slider — high R visibly reduces jerkiness of cart movement
- [ ] Sliders swap correctly when toggling controller
- [ ] No JS console errors; no WebSocket message flood (debounce working)

---

### EPIC 3 Integration Gate

- [ ] Full demo loop: page loads → simulation running → nudge → recovery visible
- [ ] Switch PID → LQR mid-wobble → smoother recovery visible
- [ ] Tune PID gains to instability → switch to LQR → rescue
- [ ] "FAILED" → reset → simulation restarts cleanly
- [ ] All controls work without page reload
- [ ] No JS console errors throughout

---

### EPIC 4 — Docker & Fly.io Deployment

**Epic Goal:** One `fly deploy` puts the demo live at a public URL, auto-stopping when idle.

---

#### Story 4.1 — Production Dockerfile & Local Smoke Test

**Context:** Epic 3 complete. Full demo works locally via `make dev`.

**Tasks:**
- Harden `Dockerfile`:
  - Multi-stage if beneficial, otherwise single-stage Python 3.12-slim
  - Non-root user
  - `HEALTHCHECK` via `curl http://localhost:8000/health`
  - `stop_grace_period` equivalent — uvicorn handles SIGTERM gracefully (already done in 2.1; verify here)
  - Expose port 8000
- `make docker-run` — builds and runs container, maps port 8000
- Verify full demo works inside container (not just `make dev`)

**Out of Scope:** Fly.io config, deployment.

**Acceptance Criteria:**
- [ ] `docker build -t pendulum .` succeeds
- [ ] `docker run -p 8000:8000 pendulum` — full demo works in browser at `http://localhost:8000`
- [ ] `docker stop` → container exits within 5 seconds
- [ ] `docker inspect pendulum | grep -i health` → healthcheck configured

---

#### Story 4.2 — Fly.io Config & Deploy

**Context:** Story 4.1 complete. Container verified locally.

**Assumptions:**
- `fly` CLI installed and authenticated (`fly auth whoami` returns your email)
- Fly.io account exists

**Tasks:**
- `fly launch` — generates initial `fly.toml`; do not deploy yet
- Edit `fly.toml`:
  - `auto_stop_machines = "stop"` — machine stops when no active connections
  - `auto_start_machines = true`
  - `min_machines_running = 0`
  - Set `[http_service]` internal port to 8000
  - Memory: 256MB is sufficient
- Set any required secrets: `fly secrets set CORS_ORIGIN=https://yourdomain.com`
- `fly deploy`
- Verify WebSocket works through Fly.io proxy (WebSocket over TLS requires `wss://`)
- Update `index.html` WebSocket URL to detect `wss://` vs `ws://` based on `window.location.protocol`

**Out of Scope:** Custom domain, iframe embed (Story 4.3).

**Acceptance Criteria:**
- [ ] `fly deploy` succeeds
- [ ] `https://your-app.fly.dev/health` → `{"status":"ok"}`
- [ ] Full demo works at `https://your-app.fly.dev/`
- [ ] Machine visible in Fly.io dashboard
- [ ] After 5 minutes of no browser connection, machine shows as stopped in dashboard
- [ ] New browser visit — machine restarts and demo loads (may take ~2s cold start)

---

#### Story 4.3 — iframe Embed & CORS

**Context:** Story 4.2 complete. App is live on Fly.io.

**Tasks:**
- Update `CORS_ORIGIN` to include your portfolio domain
- Test `<iframe src="https://your-app.fly.dev/" />` embedded on your portfolio site
- Ensure `X-Frame-Options` is not set to `DENY` (FastAPI default is fine; verify)
- Add minimal embed-friendly styling to `index.html`: no external scrollbars, canvas fills iframe cleanly
- Document the iframe snippet in this README under a new "Embedding" section

**Out of Scope:** Responsive iframe sizing on the portfolio side (that's portfolio work).

**Acceptance Criteria:**
- [ ] `<iframe>` on portfolio site renders the demo without scrollbars or layout bleed
- [ ] All controls (nudge, sliders, toggle) work inside the iframe
- [ ] No browser console CORS errors
- [ ] Demo still works standalone at `https://your-app.fly.dev/`

---

### EPIC 4 Integration Gate

- [ ] Full demo accessible at public Fly.io URL
- [ ] Auto-stop verified (machine stops after idle period)
- [ ] Auto-start verified (cold start from stopped state works)
- [ ] iframe embed works on portfolio site
- [ ] `fly deploy` from local works cleanly end-to-end

---

### EPIC 5 — Kalman Filter (Optional Extension)

**Epic Goal:** Add simulated sensor noise to the state and a Kalman filter that estimates the true state before passing it to the controller. Demonstrates the full observe → estimate → control loop.

---

#### Story 5.1 — Sensor Noise Model

**Context:** Epic 4 complete. App is deployed and working.

**Tasks:**
- `app/simulation.py` — add `add_noise(state, noise_std: dict) -> SimulationState` that adds Gaussian noise to `theta` and `x` measurements
- Default noise std: `theta=0.01 rad`, `x=0.005 m`
- Noise toggled via a new inbound WS action: `{"action": "set_noise", "enabled": bool}`
- Noisy measurements sent in WebSocket state as `theta_meas`, `x_meas` alongside true values
- Frontend: when noise enabled, render a faint "true" pole and a slightly transparent "measured" pole

**Acceptance Criteria:**
- [ ] Noise visible in rendered pole (slight jitter vs true position)
- [ ] Controllers still stabilize with noise enabled (gains may need adjustment)

---

#### Story 5.2 — Kalman Filter

**Context:** Story 5.1 complete. Noisy measurements flowing through the system.

**Tasks:**
- `app/controllers/kalman.py` — `KalmanFilter` class wrapping the linear Kalman filter equations
  - Prediction step: uses linearized A, B matrices from LQR (share the linearization)
  - Update step: uses noisy `theta_meas`, `x_meas`
  - Outputs estimated state passed to whichever controller is active
- Frontend: add "Kalman Filter" toggle — shows estimated state as a third rendering layer on canvas
- `tests/test_kalman.py` — verify estimated state converges to true state over time

**Acceptance Criteria:**
- [ ] With noise on and Kalman on, controllers stabilize as well as the no-noise case
- [ ] With noise on and Kalman off, visible degradation in controller performance
- [ ] Estimated state rendering visible and distinct on canvas

---

## Config Management

| Variable | Local | Production |
|---|---|---|
| `SIM_TICK_RATE_HZ` | `.env` | `fly.toml [env]` |
| `CORS_ORIGIN` | `.env` | `fly secrets set` |

Never commit `.env`. The `.env.example` file is committed with keys but no values.

---

## Definition of Done

A story is complete when:

- [ ] All acceptance criteria pass
- [ ] `make test` still passes after the change
- [ ] Container builds and runs cleanly
- [ ] No secrets committed
- [ ] README updated if any setup steps changed
- [ ] Epic integration gate passes before starting the next epic
import os
import signal
import asyncio
from dataclasses import replace
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse

from app.simulation import SimulationState, step, is_failed, reset as sim_reset
from app.controllers.pid import PIDController
from app.controllers.lqr import LQRController

SIM_TICK_RATE_HZ = float(os.getenv("SIM_TICK_RATE_HZ", "60"))

app = FastAPI()

cors_origin = os.getenv("CORS_ORIGIN", "*")
app.add_middleware(
    CORSMiddleware,
    allow_origins=[cors_origin],
    allow_methods=["*"],
    allow_headers=["*"],
)

_shutdown_event = asyncio.Event()


def _handle_sigterm(*_):
    _shutdown_event.set()


signal.signal(signal.SIGTERM, _handle_sigterm)


class SimulationManager:
    def __init__(self):
        self.state: SimulationState = sim_reset()
        self.pid = PIDController(kp=30.0, ki=1.0, kd=5.0)
        self.lqr = LQRController()
        self.controller_name: str = "lqr"
        self._ws: WebSocket | None = None
        self._task: asyncio.Task | None = None

    def _state_dict(self) -> dict:
        return {
            "x": self.state.x,
            "x_dot": self.state.x_dot,
            "theta": self.state.theta,
            "theta_dot": self.state.theta_dot,
            "force": self.state.force,
            "time": self.state.time,
            "failed": is_failed(self.state),
            "controller": self.controller_name,
        }

    async def start_loop(self, ws: WebSocket) -> None:
        self._ws = ws
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run())

    async def stop_loop(self) -> None:
        if self._task and not self._task.done():
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        self._task = None
        self._ws = None

    async def _run(self) -> None:
        dt = 1.0 / SIM_TICK_RATE_HZ
        while True:
            if self.controller_name == "pid":
                F = self.pid.compute(self.state.theta, self.state.theta_dot, dt)
            else:
                F = self.lqr.compute(self.state)
            self.state = step(self.state, F, dt)
            if self._ws is not None:
                try:
                    await self._ws.send_json(self._state_dict())
                except Exception:
                    break
            await asyncio.sleep(dt)

    def reset(self) -> None:
        self.state = sim_reset()
        self.pid.reset()


_manager = SimulationManager()


@app.get("/health")
async def health():
    return {"status": "ok"}


@app.get("/")
async def index():
    return FileResponse("app/static/index.html")


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    await ws.send_json(_manager._state_dict())
    await _manager.start_loop(ws)
    try:
        while True:
            data = await ws.receive_json()
            action = data.get("action")
            if action == "set_controller":
                name = data.get("controller", "lqr")
                if name == "pid":
                    _manager.pid.reset()
                _manager.controller_name = name
            elif action == "set_pid_gains":
                _manager.pid.kp = float(data.get("kp", _manager.pid.kp))
                _manager.pid.ki = float(data.get("ki", _manager.pid.ki))
                _manager.pid.kd = float(data.get("kd", _manager.pid.kd))
            elif action == "set_lqr_weights":
                q = data.get("q_weights", _manager.lqr.q_weights)
                r = data.get("r_weight", _manager.lqr.r_weight)
                _manager.lqr.update_weights(q, r)
            elif action == "disturbance":
                magnitude = float(data.get("magnitude", 0.0))
                _manager.state = replace(_manager.state, theta_dot=_manager.state.theta_dot + magnitude)
            elif action == "reset":
                _manager.reset()
    except WebSocketDisconnect:
        await _manager.stop_loop()

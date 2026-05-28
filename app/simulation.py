from dataclasses import dataclass
import numpy as np

M = 1.0   # cart mass (kg)
m = 0.1   # pole mass (kg)
l = 0.5   # pole half-length (m)
g = 9.81  # gravity (m/s²)
b = 0.1   # cart friction coefficient (N·s/m)


@dataclass
class SimulationState:
    x: float = 0.0
    x_dot: float = 0.0
    theta: float = 0.0
    theta_dot: float = 0.0
    force: float = 0.0
    time: float = 0.0


def _derivatives(sv: np.ndarray, F: float) -> np.ndarray:
    """Nonlinear cart-pole equations of motion. Returns d/dt [x, x_dot, theta, theta_dot]."""
    _, x_dot, theta, theta_dot = sv
    sin_th = np.sin(theta)
    cos_th = np.cos(theta)
    denom = M + m * sin_th ** 2

    x_ddot = (F - m * g * sin_th * cos_th + m * l * theta_dot ** 2 * sin_th - b * x_dot) / denom
    theta_ddot = (g * sin_th - x_ddot * cos_th) / l

    return np.array([x_dot, x_ddot, theta_dot, theta_ddot])


def step(state: SimulationState, F: float, dt: float) -> SimulationState:
    """Advance simulation by dt using RK4 integration."""
    sv = np.array([state.x, state.x_dot, state.theta, state.theta_dot])
    k1 = _derivatives(sv, F)
    k2 = _derivatives(sv + 0.5 * dt * k1, F)
    k3 = _derivatives(sv + 0.5 * dt * k2, F)
    k4 = _derivatives(sv + dt * k3, F)
    sv_new = sv + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return SimulationState(
        x=float(sv_new[0]),
        x_dot=float(sv_new[1]),
        theta=float(sv_new[2]),
        theta_dot=float(sv_new[3]),
        force=F,
        time=state.time + dt,
    )


def is_failed(state: SimulationState) -> bool:
    return abs(state.theta) > 0.5 or abs(state.x) > 2.4


def reset() -> SimulationState:
    return SimulationState(theta=0.05)

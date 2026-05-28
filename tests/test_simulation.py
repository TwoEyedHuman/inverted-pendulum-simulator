import math
import app.simulation as sim
from app.simulation import SimulationState, step, is_failed, reset


def _total_energy(state: SimulationState) -> float:
    """Mechanical energy of the cart-pole system."""
    x_dot = state.x_dot
    theta = state.theta
    theta_dot = state.theta_dot
    ke = (
        0.5 * (sim.M + sim.m) * x_dot ** 2
        + sim.m * x_dot * sim.l * theta_dot * math.cos(theta)
        + 0.5 * sim.m * sim.l ** 2 * theta_dot ** 2
    )
    pe = sim.m * sim.g * sim.l * math.cos(theta)
    return ke + pe


def test_zero_force_falls_and_fails():
    """Pendulum falls from reset() state and is_failed() triggers within 2 sim-seconds."""
    state = reset()
    dt = 1 / 60
    for _ in range(int(2.0 / dt) + 1):
        if is_failed(state):
            break
        state = step(state, 0.0, dt)
    assert is_failed(state), f"Expected failure within 2s, reached t={state.time:.3f}"
    assert state.time <= 2.0


def test_rk4_energy_conservation():
    """RK4 conserves mechanical energy within 1% over 1 second (zero friction, zero force)."""
    saved_b = sim.b
    sim.b = 0.0
    try:
        state = SimulationState(theta=0.05)
        e0 = _total_energy(state)
        dt = 1 / 60
        for _ in range(60):
            state = step(state, 0.0, dt)
        e1 = _total_energy(state)
        drift = abs(e1 - e0) / abs(e0)
        assert drift < 0.01, f"Energy drift {drift:.4%} exceeds 1% threshold"
    finally:
        sim.b = saved_b


def test_reset_fails_within_expected_time():
    """reset() state with zero force fails; small-angle approximation predicts ~0.7s."""
    state = reset()
    dt = 1 / 60
    for _ in range(int(2.0 / dt) + 1):
        if is_failed(state):
            break
        state = step(state, 0.0, dt)
    assert is_failed(state)
    # Linearised estimate: theta(t) = 0.05*cosh(sqrt(g/l)*t), fails at |theta|=0.5
    # => t ≈ arccosh(10) / sqrt(19.62) ≈ 0.68s; friction slows it slightly
    assert 0.4 < state.time < 2.0, f"Failure time {state.time:.3f}s outside expected range"

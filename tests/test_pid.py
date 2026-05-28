import pytest
from app.controllers.pid import PIDController
from app.simulation import SimulationState, step, is_failed


def test_positive_theta_produces_positive_corrective_force():
    # F = kp*theta: positive force needed to stabilize positive theta lean.
    # (Pushing cart right when pole leans right is the stabilizing action.)
    pid = PIDController(kp=50.0, ki=1.0, kd=10.0)
    force = pid.compute(theta=0.1, theta_dot=0.0, dt=0.01)
    assert force > 0, f"Expected positive corrective force, got {force}"


def test_reset_zeroes_integral():
    pid = PIDController(kp=10.0, ki=5.0, kd=1.0)
    pid.compute(theta=0.1, theta_dot=0.0, dt=0.1)
    pid.compute(theta=0.1, theta_dot=0.0, dt=0.1)
    pid.reset()
    # After reset, output must match a fresh controller (integral = 0 before this call)
    fresh = PIDController(kp=10.0, ki=5.0, kd=1.0)
    force_after_reset = pid.compute(theta=0.1, theta_dot=0.0, dt=0.1)
    force_fresh = fresh.compute(theta=0.1, theta_dot=0.0, dt=0.1)
    assert abs(force_after_reset - force_fresh) < 1e-9, (
        f"reset() did not zero integral: got {force_after_reset}, expected {force_fresh}"
    )


def test_output_clamping():
    pid = PIDController(kp=1000.0, ki=0.0, kd=0.0, output_clamp=20.0)
    force = pid.compute(theta=1.0, theta_dot=0.0, dt=0.01)
    assert force == pytest.approx(20.0), f"Expected clamped at 20, got {force}"

    pid2 = PIDController(kp=1000.0, ki=0.0, kd=0.0, output_clamp=20.0)
    force_neg = pid2.compute(theta=-1.0, theta_dot=0.0, dt=0.01)
    assert force_neg == pytest.approx(-20.0), f"Expected clamped at -20, got {force_neg}"


def test_stabilizes_10_seconds():
    """Kp=50, Ki=1, Kd=10 keeps |theta| < 0.5 rad for 10s from theta=0.05."""
    pid = PIDController(kp=50.0, ki=1.0, kd=10.0)
    state = SimulationState(theta=0.05)
    dt = 1 / 60
    steps = int(10.0 / dt)
    for _ in range(steps):
        force = pid.compute(state.theta, state.theta_dot, dt)
        state = step(state, force, dt)
        assert not is_failed(state), (
            f"System failed at t={state.time:.3f}s, theta={state.theta:.4f} rad"
        )

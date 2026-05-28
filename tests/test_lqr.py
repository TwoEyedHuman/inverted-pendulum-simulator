import numpy as np
import pytest

from app.controllers.lqr import LQRController
from app.controllers.pid import PIDController
from app.simulation import SimulationState, step, is_failed


def _run_sim(controller, initial_theta: float, seconds: float = 10.0, dt: float = 1 / 60):
    state = SimulationState(theta=initial_theta)
    forces = []
    failed = False
    steps = int(seconds / dt)
    for _ in range(steps):
        if hasattr(controller, "compute"):
            try:
                force = controller.compute(state)
            except TypeError:
                force = controller.compute(state.theta, state.theta_dot, dt)
        forces.append(force)
        state = step(state, force, dt)
        if is_failed(state):
            failed = True
            break
    return forces, failed


def test_stabilizes_10_seconds():
    lqr = LQRController()
    state = SimulationState(theta=0.05)
    dt = 1 / 60
    steps = int(10.0 / dt)
    for _ in range(steps):
        force = lqr.compute(state)
        state = step(state, force, dt)
        assert not is_failed(state), (
            f"System failed at t={state.time:.3f}s, theta={state.theta:.4f} rad"
        )


def test_increasing_r_reduces_peak_force():
    lqr_lo_r = LQRController(r_weight=0.001)
    lqr_hi_r = LQRController(r_weight=1.0)
    forces_lo, _ = _run_sim(lqr_lo_r, initial_theta=0.1, seconds=3.0)
    forces_hi, _ = _run_sim(lqr_hi_r, initial_theta=0.1, seconds=3.0)
    peak_lo = max(abs(f) for f in forces_lo)
    peak_hi = max(abs(f) for f in forces_hi)
    assert peak_hi < peak_lo, (
        f"Higher R should produce lower peak force: lo_R peak={peak_lo:.3f}, hi_R peak={peak_hi:.3f}"
    )


def test_k_recomputed_on_weight_change():
    lqr = LQRController()
    k_before = lqr._K.copy()
    lqr.update_weights(q_weights=[1.0, 1.0, 100.0, 1.0], r_weight=0.1)
    assert not np.allclose(k_before, lqr._K), "K matrix unchanged after weight update"


def test_lqr_smoother_than_pid():
    """LQR from theta=0.2 rad: lower total control effort and fewer oscillations than underdamped PID.

    Comparison fixture: PIDController(kp=200, ki=0, kd=5) — saturates at 20 N initially then
    oscillates. LQR makes one aggressive-but-optimal correction and settles, so its integrated
    squared force (the quantity LQR minimises) is lower, as are force-direction reversals.
    """
    dt = 1 / 60
    seconds = 5.0

    lqr = LQRController()
    pid_fixture = PIDController(kp=200.0, ki=0.0, kd=5.0)

    lqr_forces, lqr_failed = _run_sim(lqr, initial_theta=0.2, seconds=seconds, dt=dt)
    pid_forces, _ = _run_sim(pid_fixture, initial_theta=0.2, seconds=seconds, dt=dt)

    assert not lqr_failed, "LQR failed to stabilize from theta=0.2"

    lqr_effort = sum(f ** 2 for f in lqr_forces)
    pid_effort = sum(f ** 2 for f in pid_forces)
    assert lqr_effort < pid_effort, (
        f"LQR total effort ({lqr_effort:.1f}) not lower than PID fixture ({pid_effort:.1f})"
    )

    def sign_changes(forces):
        signs = [1 if f >= 0 else -1 for f in forces]
        return sum(1 for i in range(1, len(signs)) if signs[i] != signs[i - 1])

    lqr_osc = sign_changes(lqr_forces)
    pid_osc = sign_changes(pid_forces)
    assert lqr_osc < pid_osc, (
        f"LQR oscillations ({lqr_osc}) not fewer than PID fixture ({pid_osc})"
    )

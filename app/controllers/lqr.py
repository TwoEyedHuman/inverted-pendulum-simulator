import numpy as np
from scipy import linalg

from app.simulation import SimulationState, M, m, l, g, b

_DEFAULT_Q = [1.0, 1.0, 10.0, 1.0]
_DEFAULT_R = 0.01


class LQRController:
    def __init__(
        self,
        q_weights: list[float] | None = None,
        r_weight: float = _DEFAULT_R,
    ):
        self.q_weights = list(q_weights) if q_weights is not None else list(_DEFAULT_Q)
        self.r_weight = r_weight
        self._K: np.ndarray = np.zeros((1, 4))
        self._compute_gains()

    def _compute_gains(self) -> None:
        A = np.array([
            [0.0,       1.0,              0.0,              0.0],
            [0.0,  -b / M,        -m * g / M,              0.0],
            [0.0,       0.0,              0.0,              1.0],
            [0.0,  b / (M * l),  g * (M + m) / (M * l),   0.0],
        ])
        B = np.array([[0.0], [1.0 / M], [0.0], [-1.0 / (M * l)]])
        Q = np.diag(self.q_weights)
        R = np.array([[self.r_weight]])
        P = linalg.solve_continuous_are(A, B, Q, R)
        self._K = np.linalg.inv(R) @ B.T @ P

    def update_weights(self, q_weights: list[float], r_weight: float) -> None:
        self.q_weights = list(q_weights)
        self.r_weight = r_weight
        self._compute_gains()

    def compute(self, state: SimulationState) -> float:
        sv = np.array([state.x, state.x_dot, state.theta, state.theta_dot])
        raw = float(-(self._K @ sv)[0])
        return max(-20.0, min(20.0, raw))

    def reset(self) -> None:
        pass

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, output_clamp: float = 20.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_clamp = output_clamp
        self._integral = 0.0

    def compute(self, theta: float, theta_dot: float, dt: float) -> float:
        self._integral += theta * dt
        raw = self.kp * theta + self.ki * self._integral + self.kd * theta_dot
        return float(max(-self.output_clamp, min(self.output_clamp, raw)))

    def reset(self) -> None:
        self._integral = 0.0

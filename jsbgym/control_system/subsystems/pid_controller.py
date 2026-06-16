class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._last_error = 0.0
        self._integral = 0.0

    def reset(self):
        """Reset the PID controller state."""
        self._last_error = 0.0
        self._integral = 0.0

    def compute(self, measurement: float, setpoint, dt: float) -> float:
        """
        Compute the PID control output.
        Args:
            measurement: The current measured value.
            dt: Time step (if 0, derivative term is not used).
        Returns:
            Control output after applying PID formula and output limits.
        """
        error = setpoint - measurement
        self._integral += error * dt
        if dt != 0:
            derivative = (error - self._last_error) / dt
        else: 
            derivative = 0

        return self.kp * error + self.ki * self._integral + self.kd * derivative 
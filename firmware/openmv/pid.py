"""Small MicroPython-friendly PID controller with integral anti-windup."""

import time


class PID:
    def __init__(self, kp, ki, kd, integral_limit):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.integral_limit = float(integral_limit)
        self.reset()

    def reset(self):
        self._previous_error = None
        self._previous_time_us = None
        self._integral = 0.0

    def update(self, error):
        now_us = time.ticks_us()
        if self._previous_time_us is None:
            self._previous_time_us = now_us
            self._previous_error = error
            return 0.0

        dt = time.ticks_diff(now_us, self._previous_time_us) / 1_000_000
        if dt <= 0:
            return 0.0

        self._integral += 0.5 * (error + self._previous_error) * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))
        derivative = (error - self._previous_error) / dt

        self._previous_error = error
        self._previous_time_us = now_us
        return (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)


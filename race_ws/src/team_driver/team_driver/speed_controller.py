"""
Experimental RoboRacer speed controller V3 — gentle feedforward + P.

V3 keeps applying the baseline feedforward throttle instead of abruptly
switching between acceleration and zero / reverse torque on small overspeed.
It does not use reverse torque. Thus it may fail to slow enough for corners.

Use this in a separate development branch, with a short low-speed straight test.
Role 2 owns the submitted speed_controller.py and should review final changes.
"""

import math
IS_STUB = False

class SpeedController:
    def __init__(
        self,
        ff_throttle_per_mps=0.04,
        kp=0.012,
        deadband_mps=0.10,
        speed_filter_tau=0.10,
        max_throttle=0.30,
        throttle_slew_per_s=0.40,
        debug_every=10,
    ):
        self.ff = float(ff_throttle_per_mps)
        self.kp = float(kp)
        self.deadband = float(deadband_mps)
        self.tau = float(speed_filter_tau)
        self.max_throttle = float(max_throttle)
        self.slew = float(throttle_slew_per_s)
        self.debug_every = int(debug_every)
        if (self.ff <= 0 or self.kp < 0 or self.deadband < 0 or
                self.tau <= 0 or self.max_throttle <= 0 or
                self.max_throttle > 1 or self.slew <= 0):
            raise ValueError("Invalid speed-control parameters")
        self.reset()

    def reset(self):
        self.filtered_speed = None
        self.last_throttle = 0.0
        self.count = 0

    @staticmethod
    def clip(value, lo, hi):
        return max(lo, min(hi, value))

    def step(self, target_speed, measured_speed, dt):
        if not all(math.isfinite(v) for v in (target_speed, measured_speed, dt)):
            self.reset()
            return 0.0

        target = max(0.0, float(target_speed))
        measured = max(0.0, float(measured_speed))
        dt = self.clip(float(dt), 0.005, 0.10)

        if self.filtered_speed is None:
            self.filtered_speed = measured
        else:
            alpha = dt / (self.tau + dt)
            self.filtered_speed += alpha * (measured - self.filtered_speed)

        error = target - self.filtered_speed
        correction = 0.0 if abs(error) < self.deadband else self.kp * error
        baseline = self.ff * target

        # IMPORTANT: there is no active braking here. In particular, no abrupt
        # zero-throttle when speed is slightly above target.
        desired = self.clip(baseline + correction, 0.0, self.max_throttle)
        delta = self.clip(desired - self.last_throttle,
                          -self.slew * dt, self.slew * dt)
        throttle = self.clip(self.last_throttle + delta, 0.0, self.max_throttle)
        self.last_throttle = throttle

        return throttle

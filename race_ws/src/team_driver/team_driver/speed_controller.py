"""Experimental RoboRacer speed controller V3 — gentle feedforward + P.

DIAGNOSTIC/STRAIGHT-TEST VERSION, NOT A RACING BRAKE CONTROLLER.

Calibrated locally around 1.5 m/s from the user's AutoDRIVE test:
    constant throttle 0.06 -> steady ~1.5 m/s => feedforward ~0.04 / (m/s).
This relation is NOT known to be linear at higher speeds.

V3 keeps applying the baseline feedforward throttle instead of abruptly
switching between acceleration and zero / reverse torque on small overspeed.
It does not use reverse torque. Thus it may fail to slow enough for corners.

Use this in a separate development branch, with a short low-speed straight test.
Role 2 owns the submitted speed_controller.py and should review final changes.
"""

<<<<<<< HEAD
=======
import math

>>>>>>> 4873812cdd9a8ec4f1ad357dfd98eddc62a6348c
IS_STUB = False


class SpeedController:
<<<<<<< HEAD

    def __init__(self, ff_throttle_per_mps=0.04, max_throttle=1.0):
        # PLACEHOLDERS, NOT MEASURED. 0.12 per m/s reproduces the template's
        # constant crawl throttle (0.12) at a 1.0 m/s target. Role 2 replaces
        # these with the identified throttle-to-speed relationship.
        self.ff_throttle_per_mps = ff_throttle_per_mps
        

        self.kp = 0.20
        self.ki = 0.03

        self.max_int = 0.10
        self.int_err = 0.0
        self.max_throttle = max_throttle

    def step(self, target_speed, measured_speed, dt):
        """Return a throttle in [-1, 1].

        target_speed   [m/s]  what the planner/reactive driver wants
        measured_speed [m/s]  hypot of /odom linear velocity
        dt             [s]    time since the previous call (message time)
        """
        if target_speed<=0:
            self.reset()
            return 0.0
        err = target_speed-measured_speed
        if err < 0.0: self.int_err = max(0.0, self.int_err + err * dt)
        else: self.int_err += err*dt
        self.int_err = max(-self.max_int, min(self.max_int, self.int_err))

        

        ff = self.ff_throttle_per_mps * target_speed
        p_out = self.kp * err
        i_out = self.ki * self.int_err
        
        throttle = ff + p_out + i_out
        
        return float(max(-1.0, min(self.max_throttle, throttle)))

    def reset(self):
        """Clear internal state (the integral). Called when a run restarts."""
        self.int_err = 0.0
        pass
=======
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

        self.count += 1
        if self.debug_every > 0 and self.count % self.debug_every == 0:
            print(
                f"[SPEED-V3] target={target:.2f} "
                f"actual={measured:.2f} filtered={self.filtered_speed:.2f} "
                f"error={error:+.2f} ff={baseline:.3f} "
                f"throttle={throttle:+.3f}",
                flush=True,
            )
        return throttle
>>>>>>> 4873812cdd9a8ec4f1ad357dfd98eddc62a6348c

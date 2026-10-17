"""Speed controller - OWNED BY ROLE 2.

Turns a target speed into a throttle command. AutoDRIVE takes normalised motor
torque in [-1, 1], not a speed, and nothing else closes the speed loop.

This is a STUB. Replace the body of `step()` with feed-forward + PI on the
speed error (clamped integral, lift-off rule for overspeed). Keep the class
name and the two method signatures: the driver and your teammates rely on them.
When the real controller is in, set IS_STUB = False.
"""

IS_STUB = False


class SpeedController:

    def __init__(self, ff_throttle_per_mps=0.05, max_throttle=0.5):
        # PLACEHOLDERS, NOT MEASURED. 0.12 per m/s reproduces the template's
        # constant crawl throttle (0.12) at a 1.0 m/s target. Role 2 replaces
        # these with the identified throttle-to-speed relationship.
        self.ff_throttle_per_mps = ff_throttle_per_mps
        

        self.kp = 0.05
        self.ki = 0.04

        self.max_int = 2.0
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
        
        return float(max(0.0, min(self.max_throttle, throttle)))

    def reset(self):
        """Clear internal state (the integral). Called when a run restarts."""
        self.int_err = 0.0
        pass

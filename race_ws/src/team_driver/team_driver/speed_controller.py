"""Speed controller - OWNED BY ROLE 2.

Turns a target speed into a throttle command. AutoDRIVE takes normalised motor
torque in [-1, 1], not a speed, and nothing else closes the speed loop.

This is a STUB. Replace the body of `step()` with feed-forward + PI on the
speed error (clamped integral, lift-off rule for overspeed). Keep the class
name and the two method signatures: the driver and your teammates rely on them.
When the real controller is in, set IS_STUB = False.
"""

IS_STUB = True


class SpeedController:

    def __init__(self, ff_throttle_per_mps=0.12, max_throttle=0.2):
        # PLACEHOLDERS, NOT MEASURED. 0.12 per m/s reproduces the template's
        # constant crawl throttle (0.12) at a 1.0 m/s target. Role 2 replaces
        # these with the identified throttle-to-speed relationship.
        self.ff_throttle_per_mps = ff_throttle_per_mps
        self.max_throttle = max_throttle

    def step(self, target_speed, measured_speed, dt):
        """Return a throttle in [-1, 1].

        target_speed   [m/s]  what the planner/reactive driver wants
        measured_speed [m/s]  hypot of /odom linear velocity
        dt             [s]    time since the previous call (message time)
        """
        throttle = self.ff_throttle_per_mps * max(0.0, target_speed)
        return min(self.max_throttle, throttle)

    def reset(self):
        """Clear internal state (the integral). Called when a run restarts."""
        pass

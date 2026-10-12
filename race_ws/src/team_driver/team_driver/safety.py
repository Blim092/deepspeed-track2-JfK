"""Safety layer - OWNED BY ROLE 5.

Two jobs inside the driver:
  1. limit(): cap the target speed so the car can stop before the nearest
     obstacle ahead, v <= sqrt(2 * a_brake * max(0, d - margin)). a_brake comes
     from Role 2's measurements; measure the LiDAR-to-front-bumper distance
     instead of guessing it.
  2. line_blocked(): tell the driver that the planned line is blocked, so it
     falls back to the reactive driver (with hysteresis, so it does not flap).

This is a STUB: it changes nothing and never reports a blocked line. Replace
the bodies, keep the class name and signatures, set IS_STUB = False.

Scoring reminder: +10 s per collision on that lap; more than 10 collisions in
a run is a disqualification. Keep everything here cheap and vectorised; the
whole control callback has to stay well under 25 ms.
"""

from team_driver.common import finite_or

IS_STUB = True


class Safety:

    def limit(self, target_speed, scan):
        """Return the target speed [m/s], reduced if necessary."""
        return finite_or(target_speed, 0.0)

    def line_blocked(self, path_points_car_frame, scan):
        """Return True if LiDAR returns lie on the planned path.

        path_points_car_frame: [(x, y), ...] with +x forward, +y left, origin
        at the /odom pose.
        """
        return False

"""Planner - OWNED BY ROLE 4.

Path tracking on a known line (pure pursuit on the centreline, then a
curvature-based speed profile, then a racing line from the map).

This is a STUB: it returns straight ahead at a fixed low speed and an empty
path. Replace the methods, keep the class name and signatures, and set
IS_STUB = False when it really tracks a line.

Facts you need:
    Pure pursuit: delta = atan2(2 * L * sin(alpha), Ld), L = 0.324 m.
    steering = delta / 0.5236, clamped to [-1, 1].
    Map: maps/icra26_compete.pgm (220 x 476 cells, 5 cm).
    Centreline: maps/icra26_compete_centerline.csv (check its columns).
    Lap length 54.4 m. Check which point /odom reports before trusting it.
"""

IS_STUB = True


class Planner:

    def __init__(self, path_csv=None, crawl_speed=1.0):
        self.path_csv = path_csv         # Role 4: load the line here
        self.crawl_speed = crawl_speed   # [m/s] stub target speed

    def step(self, x, y, yaw, speed):
        """Return (steering, target_speed).

        x, y   [m]    ground-truth position from /odom (world frame)
        yaw    [rad]  heading
        speed  [m/s]  current speed
        steering is in [-1, 1], positive = left.
        """
        return 0.0, self.crawl_speed

    def path_ahead(self, x, y, yaw, n):
        """Return the next `n` points of the line as [(x, y), ...] in the
        WORLD frame. Role 5 uses them to check whether the line is blocked.
        Stub: no path."""
        return []

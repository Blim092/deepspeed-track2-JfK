"""Reactive (LiDAR-only) driver - OWNED BY ROLE 3.

Follow-the-gap with disparity extension. Needs no map and no memory; it is the
baseline that must finish laps and the fallback when the planner's line is
blocked.

This is a STUB: it drives straight at a fixed low target speed, exactly like
the original template. Replace `step()` and `gap_target()`, keep the class name
and signatures, and set IS_STUB = False when it really drives.

Facts you need:
    1080 beams over 270 deg, 0.25 deg apart, 0.06-10 m, 40 Hz.
    Index 0 = hard right, middle = straight ahead, positive angle = LEFT.
    steering = delta_rad / 0.5236, clamped to [-1, 1].
    Steering rate limit is 3.2 rad/s.
    The LiDAR sits 0.2733 m ahead of the rear axle.
"""

import numpy as np

from team_driver.common import clean_scan

IS_STUB = True


class Reactive:

    def __init__(self, max_range=10.0, crawl_speed=1.0):
        self.max_range = max_range
        self.crawl_speed = crawl_speed   # [m/s] stub target speed

    def step(self, scan):
        """Return (steering, target_speed).

        steering      [-1, 1], positive = left
        target_speed  [m/s]
        """
        return 0.0, self.crawl_speed

    def gap_target(self, scan):
        """Return (angle_rad, gap_distance_m) of the gap being aimed at.

        Used by Role 5's fallback logic, so it must be cheap enough to call on
        every scan. Stub: aim straight ahead and report the range there.
        """
        ranges, angles = clean_scan(scan, self.max_range)
        if len(ranges) == 0:
            return 0.0, 0.0
        ahead = int(np.argmin(np.abs(angles)))
        return 0.0, float(ranges[ahead])

"""Shared helpers for the team driver. Owned by Role 1.

Keep this file small and dependency-free (numpy only) so every module can use
it and it can be unit-tested without ROS.
"""

import math

import numpy as np


def clean_scan(scan, max_range=10.0):
    """Turn a raw LaserScan message into clean ranges plus each beam's angle.

    The LiDAR reports NaN and inf; arithmetic on those spreads silently, so
    they are removed here once. Index 0 is hard right, positive angles are to
    the LEFT. The beam count comes from the message, never hard-coded.
    """
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    ranges = np.nan_to_num(ranges, nan=0.0, posinf=max_range, neginf=0.0)
    ranges = np.clip(ranges, 0.0, max_range)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    return ranges, angles


def world_to_car(points, x, y, yaw):
    """Express world-frame (x, y) points in the car frame.

    Car frame: +x forward, +y left, origin at the pose (x, y). Note that the
    /odom pose may not be the same point as the LiDAR (the LiDAR is 0.2733 m
    ahead of the rear axle); whoever uses this for clearance checks must add
    that offset. Role 4 verifies which point /odom reports.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for px, py in points:
        dx, dy = px - x, py - y
        out.append((c * dx + s * dy, -s * dx + c * dy))
    return out


def finite_or(value, default=0.0):
    """Return float(value) if it is a finite number, otherwise `default`."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default

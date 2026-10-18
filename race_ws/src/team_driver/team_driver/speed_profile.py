"""
Offline speed profile for a closed-loop centreline.
"""

import math

# Provisional limits; replace with measured or validated values.
MAX_LATERAL_ACCEL = 5.0   # m/s^2
MAX_ACCEL = 1.2           # m/s^2
MAX_BRAKE = 1.0           # m/s^2
MAX_SPEED = 5.0           # m/s; conservative initial cap
CURVATURE_WINDOW = 5      # points on each side

def build_speed_profile(
    path,
    max_lateral_accel=MAX_LATERAL_ACCEL,
    max_accel=MAX_ACCEL,
    max_brake=MAX_BRAKE,
    max_speed=MAX_SPEED,
    smoothing_window=CURVATURE_WINDOW,
    corner_zones=None,
):
    # Return (curvature, speed_limit) arrays for a closed path.
    n = len(path)

    # Distance along each segment, including the closing segment.
    ds = [
        math.hypot(
            path[(i + 1) % n][0] - path[i][0],
            path[(i + 1) % n][1] - path[i][1],
        )
        for i in range(n)
    ]
    if any(d <= 1e-9 for d in ds):
        raise ValueError("Path contains duplicate consecutive points")

    # Signed discrete curvature from each point's neighbouring points.
    raw_curvature = []
    for i in range(n):
        ax, ay = path[(i - 1) % n]
        bx, by = path[i]
        cx, cy = path[(i + 1) % n]

        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ca = math.hypot(ax - cx, ay - cy)

        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        denominator = ab * bc * ca
        raw_curvature.append(
            2.0 * cross / denominator if denominator > 1e-12 else 0.0
        )

    # Circular moving-average smoothing.
    curvature = []
    w = smoothing_window
    for i in range(n):
        curvature.append(
            sum(raw_curvature[(i + offset) % n]
                for offset in range(-w, w + 1))
            / (2 * w + 1)
        )

    # Per-waypoint lateral acceleration limits. Zones override only their own
    # waypoint range; every other waypoint keeps max_lateral_accel.
    # corner_zones: [(name, start_index, end_index, lateral_acceleration), ...]
    lateral_limits = [float(max_lateral_accel)] * n
    for name, start, end, limit in (corner_zones or ()):
        for i in range(start, end + 1):
            lateral_limits[i] = float(limit)

    # Cornering speed limit: v = sqrt(local_a_lat / |curvature|).
    speeds = []
    for i, kappa in enumerate(curvature):
        if abs(kappa) < 1e-6:
            speeds.append(max_speed)
        else:
            speeds.append(
                min(max_speed, math.sqrt(lateral_limits[i] / abs(kappa)))
            )

    # Repeat around the loop until both constraints converge.
    # Each pass only lowers speeds, so it remains conservative.
    for _ in range(n):
        changed = False

        # Forward pass: acceleration limit.
        for i in range(n):
            j = (i + 1) % n
            allowed = math.sqrt(
                speeds[i] ** 2 + 2.0 * max_accel * ds[i]
            )
            if speeds[j] > allowed:
                speeds[j] = allowed
                changed = True

        # Backward pass: braking limit.
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            allowed = math.sqrt(
                speeds[j] ** 2 + 2.0 * max_brake * ds[i]
            )
            if speeds[i] > allowed:
                speeds[i] = allowed
                changed = True

        if not changed:
            break

    return curvature, speeds

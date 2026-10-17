import csv
import math

from team_driver.speed_profile import build_speed_profile

class Planner:
    """Follow a closed-loop centreline using pure pursuit."""

    # These index ranges are for icra26_compete_racingline_apex_reconstructed(1).csv
    # (272 path points). Keep the proven 1.75 elsewhere.
    # To run ONE controlled experiment, change only the C4 value below.
    CORNER_ZONES = (
        ("C1 approach",            57,  61, 6.85),
        ("C1 bottom U-turn",       62,  91, 8.85),
        ("C1 exit",                92, 110, 6.80),
        ("C2 approach", 138, 144, 2.35),
        ("C2 upper inner hairpin", 145, 170, 6.75),
        ("C3 approach",   171, 176, 2.25),
        ("C3 entry",      177, 184, 7.05),
        ("C3 tight turn", 185, 199, 6.75),
        ("C4 approach", 210, 219, 5.00),
        ("C4 upper sweeping turn", 220, 271, 14.85),
        ("C4 continuation",         0,   10, 13.50),
        ("Straight Part",         11,   40, 17.50)
    )
    EXPECTED_WAYPOINTS = 272

    WHEELBASE = 0.324
    MAX_STEERING_RAD = 0.5236

    def __init__(self, path_csv, target_speed=3.5):
        self.path = []

        with open(path_csv, "r", newline="") as file:
            for row in csv.reader(file):
                if not row or row[0].strip().startswith("#"):
                    continue
                if len(row) >= 2:
                    self.path.append((float(row[0]), float(row[1])))

        if len(self.path) < 2:
            raise ValueError("Centreline CSV must contain at least two points")

        self.target_speed = float(target_speed)
        self.last_index = None

        if len(self.path) != self.EXPECTED_WAYPOINTS:
            raise ValueError(
                f"Corner zones were calibrated for {self.EXPECTED_WAYPOINTS} "
                f"waypoints, but loaded {len(self.path)}. "
                "Re-identify corner indices for this CSV before racing."
            )

        self.curvature, self.speed_profile = build_speed_profile(
            self.path,
            max_lateral_accel=2.25,
            max_accel=1.6,
            max_brake=1.05,
            max_speed=7.0,
            corner_zones=self.CORNER_ZONES,
        )
        
        print("\n--- C1 APPROACH SPEED PROFILE ---")

        for i in range(0, 95, 5):
            print(
                f"Waypoint {i:3d}: "
                f"{self.speed_profile[i]:.2f} m/s"
            )
        
        
        print(
            f"[PROFILE] maximum calculated speed: "
            f"{max(self.speed_profile):.3f} m/s",
            flush=True
        )
        print(
            f"[CORNER-TUNING] points={len(self.path)} "
            f"planner_cap={self.target_speed:.2f} "
            f"profile_max={max(self.speed_profile):.2f} "
            f"profile_min={min(self.speed_profile):.2f} "
            f"zones={[(z[0], z[3]) for z in self.CORNER_ZONES]}",
            flush=True,
        )
    def step(self, x, y, yaw, speed):
        """Return (normalised steering, target speed)."""
        n = len(self.path)

        # Search globally on startup, then locally around the previous index.
        if self.last_index is None:
            nearest = min(
                range(n),
                key=lambda i: (self.path[i][0] - x) ** 2
                             + (self.path[i][1] - y) ** 2
            )
        else:
            window = 20
            candidates = {
                (self.last_index + offset) % n
                for offset in range(-window, window + 1)
            }
            nearest = min(
                candidates,
                key=lambda i: (self.path[i][0] - x) ** 2
                             + (self.path[i][1] - y) ** 2
            )

            # Recover globally if the local result is over 2 m away.
            distance_sq = (
                (self.path[nearest][0] - x) ** 2
                + (self.path[nearest][1] - y) ** 2
            )
            if distance_sq > 4.0:
                nearest = min(
                    range(n),
                    key=lambda i: (self.path[i][0] - x) ** 2
                                 + (self.path[i][1] - y) ** 2
                )

        self.last_index = nearest

        # Choose a lookahead distance, increasing slightly with speed.
        lookahead = max(1.1, 1.1 + 0.3 * abs(speed))
        target_index = nearest
        travelled = 0.0

        while travelled < lookahead:
            next_index = (target_index + 1) % n
            x1, y1 = self.path[target_index]
            x2, y2 = self.path[next_index]
            travelled += math.hypot(x2 - x1, y2 - y1)
            target_index = next_index

            if target_index == nearest:
                break

        tx, ty = self.path[target_index]
        dx, dy = tx - x, ty - y

        # Angle from the vehicle's forward direction to the target point.
        alpha = math.atan2(
            -math.sin(yaw) * dx + math.cos(yaw) * dy,
            math.cos(yaw) * dx + math.sin(yaw) * dy
        )

        # Pure-pursuit steering angle.
        distance = max(math.hypot(dx, dy), 0.01)
        delta = math.atan2(
            2.0 * self.WHEELBASE * math.sin(alpha),
            distance
        )

        # Convert radians to the simulator's normalised steering command.
        steering = delta / self.MAX_STEERING_RAD
        steering = max(-1.0, min(1.0, steering))

        window = 2
        upcoming_speeds = [
            self.speed_profile[(nearest + offset) % n]
            for offset in range (window + 1)
        ]
        profile_speed = min(upcoming_speeds)
        target_speed = min(self.target_speed, profile_speed)

        return steering, target_speed

    def path_ahead(self, x, y, yaw, n):
        """Return the next n centreline points in world coordinates."""
        if n <= 0:
            return []

        path_length = len(self.path)

        if self.last_index is None:
            nearest = min(
                range(path_length),
                key=lambda i: (self.path[i][0] - x) ** 2
                             + (self.path[i][1] - y) ** 2
            )
        else:
            window = 20
            candidates = {
                (self.last_index + offset) % path_length
                for offset in range(-window, window + 1)
            }

            nearest = min(
                candidates,
                key=lambda i: (self.path[i][0] - x) ** 2
                             + (self.path[i][1] - y) ** 2
            )

            distance_sq = (
                (self.path[nearest][0] - x) ** 2
                + (self.path[nearest][1] - y) ** 2
            )

            if distance_sq > 4.0:
                nearest = min(
                    range(path_length),
                    key=lambda i: (self.path[i][0] - x) ** 2
                                 + (self.path[i][1] - y) ** 2
                )

        self.last_index = nearest

        return [
            self.path[(nearest + offset) % path_length]
            for offset in range(1, n + 1)
        ]

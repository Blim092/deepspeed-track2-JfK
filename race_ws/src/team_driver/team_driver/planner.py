
import csv
import math


class Planner:
    """Follow a closed-loop centreline using pure pursuit."""

    WHEELBASE = 0.324
    MAX_STEERING_RAD = 0.5236

    def __init__(self, path_csv, target_speed=0.55):
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

        return steering, self.target_speed

    def path_ahead(self, x, y, yaw, n):
        """Return upcoming centreline points in world coordinates."""
        if n <= 0:
            return []

        nearest = min(
            range(len(self.path)),
            key=lambda i: (self.path[i][0] - x) ** 2
                         + (self.path[i][1] - y) ** 2
        )

        return [
            self.path[(nearest + offset) % len(self.path)]
            for offset in range(1, n + 1)
        ]

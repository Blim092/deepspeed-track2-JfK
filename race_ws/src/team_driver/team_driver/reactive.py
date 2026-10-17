"""NTU DeepSpeed Track 2 / Role 3: conservative feature-complete FTG candidate.

Built from the Stage 6 controller (completed 3/3 timed laps in isolated testing).
Includes disparity extension, guarded LiDAR safety bubble, guarded near-max
widest-gap preference, physical steering-rate limit and adaptive target speed.

FROZEN INTERFACE:
    Reactive.step(scan) -> (normalized_steering, target_speed)
    Reactive.gap_target(scan) -> (angle_rad, gap_distance_m)

To activate ROS parameters + /driver/markers visualization, Role 1 can
instantiate with Reactive(node=driver_node), or call reactive.attach_node(node).
Without a ROS node the driving logic remains fully functional, but marker
publishing and ROS parameter loading cannot operate.

This is a candidate, NOT a validated 10-lap submission. If it underperforms,
restore the known 3-lap Stage 6 file.
"""

import math
import numpy as np

IS_STUB = False


class Reactive:
    # Every driver tuning constant has a ROS parameter default here.
    # All names are prefixed with reactive. to avoid teammate collisions.
    DEFAULTS = {
        "max_look_deg": 75.0,
        "max_steering_rad": 0.5236,
        "min_gap_distance": 0.8,
        "min_gap_width_deg": 0.0,  # 0 preserves Stage 6 widest-gap behavior
        "disparity_threshold": 0.5,
        "car_half_width": 0.135,
        "disparity_margin": 0.05,
        "max_steering_rate_rad_s": 3.2,
        "nominal_scan_dt": 0.025,
        "bubble_enabled": True,
        "bubble_radius": 0.16,
        "bubble_trigger_distance": 1.05,
        "bubble_center_sector_deg": 25.0,
        "bubble_target_proximity_deg": 22.0,
        "bubble_max_target_shift_deg": 12.0,
        "near_max_enabled": True,
        "near_max_range_cap": 3.0,
        "near_max_ratio": 0.82,
        "near_max_min_width_deg": 12.0,
        "near_max_max_shift_deg": 8.0,
        "near_max_gain_m": 0.35,
        "min_target_speed": 0.46,
        "front_check_deg": 15.0,
        "front_slow_distance": 0.90,
        "front_full_distance": 2.80,
        "marker_enabled": True,
        "marker_every_scans": 4,
        "marker_max_length_m": 3.0,
        "debug_every_scans": 80,
    }

    def __init__(self, max_range=10.0, crawl_speed=0.5, node=None):
        self.max_range = float(max_range)
        self.crawl_speed = float(crawl_speed)
        for key, value in self.DEFAULTS.items():
            setattr(self, key, value)

        self._scan_count = 0
        self._last_disparities = 0
        self._last_extended_beams = 0
        self._last_scan_stamp = None
        self._last_dt = self.nominal_scan_dt
        self._previous_steer_rad = 0.0
        self._last_front_distance = 0.0
        self._last_bubble = "off"
        self._last_near_max = "off"
        self._last_baseline_angle = 0.0
        self._bubble_trigger_total = 0
        self._bubble_applied_total = 0
        self._near_applied_total = 0
        self._slowed_total = 0

        self._node = None
        self._marker_publisher = None
        self._Marker = None
        self._MarkerArray = None
        self._Point = None
        self._validate_config()
        if node is not None:
            self.attach_node(node)

    def attach_node(self, node):
        """Connect to Role 1's existing ROS node; never create another node.

        This method is optional so that raw-scan unit tests work without ROS.
        Call once at integration/startup, NOT inside the 40 Hz scan callback.
        """
        if self._node is not None:
            if self._node is node:
                return
            raise RuntimeError("Reactive already attached to a different ROS node")
        self._node = node

        for key, default in self.DEFAULTS.items():
            name = "reactive." + key
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            setattr(self, key, node.get_parameter(name).value)
        # Expose the existing public constructor arguments as parameters too.
        for name, default, attribute in (
            ("reactive.max_range", self.max_range, "max_range"),
            ("reactive.crawl_speed", self.crawl_speed, "crawl_speed"),
        ):
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            setattr(self, attribute, float(node.get_parameter(name).value))
        self._validate_config()

        if self.marker_enabled:
            try:
                from visualization_msgs.msg import Marker, MarkerArray
                from geometry_msgs.msg import Point
            except ImportError as exc:
                raise RuntimeError(
                    "ROS visualization_msgs and geometry_msgs are required "
                    "for /driver/markers; set reactive.marker_enabled=false "
                    "to disable publishing"
                ) from exc
            self._Marker, self._MarkerArray, self._Point = Marker, MarkerArray, Point
            self._marker_publisher = node.create_publisher(MarkerArray, "/driver/markers", 1)

    def _validate_config(self):
        if not (0.0 < self.max_look_deg <= 135.0):
            raise ValueError("reactive.max_look_deg must be in (0, 135]")
        if not (0.0 < self.max_steering_rad and self.max_steering_rate_rad_s > 0):
            raise ValueError("steering limits must be positive")
        if not (0.0 < self.min_gap_distance < self.max_range):
            raise ValueError("min_gap_distance must be below max_range")
        if not (0.0 < self.bubble_radius and self.bubble_trigger_distance > 0):
            raise ValueError("bubble dimensions must be positive")
        if not (0.0 < self.near_max_ratio <= 1.0 and self.near_max_range_cap > 0):
            raise ValueError("near-max ratio/cap out of range")
        if not (0.0 <= self.min_target_speed <= self.crawl_speed):
            raise ValueError("min_target_speed must be between 0 and crawl_speed")
        if not (0 <= self.front_slow_distance < self.front_full_distance):
            raise ValueError("front_slow_distance must be less than front_full_distance")
        if self.marker_every_scans < 1 or self.debug_every_scans < 1:
            raise ValueError("marker/debug scan intervals must be positive")

    def _process_scan(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        count = ranges.size
        increment = float(scan.angle_increment)
        if count == 0 or not np.isfinite(increment) or increment <= 0.0:
            return np.zeros(0, dtype=float), np.zeros(0, dtype=float)
        angles = float(scan.angle_min) + np.arange(count) * increment
        minimum = max(0.06, float(scan.range_min))
        cleaned = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        cleaned = np.clip(cleaned, 0.0, self.max_range)
        valid = (cleaned >= minimum) & (np.abs(angles) <= math.radians(self.max_look_deg))
        return np.where(valid, cleaned, 0.0), angles

    def _extend_disparities(self, distances, angle_increment):
        """Stage 6 disparity extension: detect edges from untouched original scan."""
        result = distances.copy()
        self._last_disparities = 0
        self._last_extended_beams = 0
        if distances.size < 2 or not math.isfinite(angle_increment) or angle_increment <= 0.0:
            return result
        edges = np.flatnonzero(
            (distances[:-1] > 0.0)
            & (distances[1:] > 0.0)
            & (np.abs(np.diff(distances)) >= self.disparity_threshold)
        )
        self._last_disparities = int(edges.size)
        radius = self.car_half_width + self.disparity_margin
        size = distances.size
        for i in edges:
            left = float(distances[i])
            right = float(distances[i + 1])
            near = min(left, right)
            half_angle = math.asin(min(1.0, radius / near))
            beams = max(1, int(math.ceil(half_angle / angle_increment)))
            if left < right:
                start = i + 1
                stop = min(size, i + 1 + beams)
            else:
                start = max(0, i + 1 - beams)
                stop = i + 1
            section = result[start:stop]
            np.minimum(section, near, out=section, where=section > 0.0)
        self._last_extended_beams = int(np.count_nonzero(
            (result > 0.0) & (result < distances - 1e-9)
        ))
        return result

    def _gap_runs(self, safe):
        changes = np.diff(np.r_[False, safe, False].astype(np.int8))
        starts = np.flatnonzero(changes == 1)
        ends = np.flatnonzero(changes == -1)
        return zip(starts, ends)

    def _widest_gap(self, distances, angles):
        """Exact Stage 6 preference, with optional minimum angular gap width."""
        best = None
        best_key = None
        inc = float(angles[1] - angles[0]) if len(angles) > 1 else 0.0
        for start, end in self._gap_runs(distances >= self.min_gap_distance):
            if math.degrees((end - start) * inc) < self.min_gap_width_deg:
                continue
            center = (int(start) + int(end) - 1) // 2
            key = (int(end - start), -abs(float(angles[center])))
            if best_key is None or key > best_key:
                best_key, best = key, (int(start), int(end), center)
        return best

    def _guarded_near_max(self, distances, angles, base):
        """Prefer widest *near-maximum* run, only when close to base direction.

        Selection stays within Stage 6's best continuous safe gap. If the
        new target would require a large departure, preserve the known
        baseline midpoint. This guards against Stage 8's wrong-corridor case.
        """
        self._last_near_max = "base"
        if not self.near_max_enabled:
            return base[2]
        start, end, base_i = base
        section = np.minimum(distances[start:end], self.near_max_range_cap)
        if section.size == 0:
            return base_i
        # Capped 90th percentile, not a single noisy maximum beam.
        threshold = float(np.percentile(section, 90)) * self.near_max_ratio
        base_depth = float(np.minimum(distances[base_i], self.near_max_range_cap))
        if base_depth >= threshold - self.near_max_gain_m:
            self._last_near_max = "base_clear"
            return base_i
        min_width = math.radians(self.near_max_min_width_deg)
        increment = float(angles[1] - angles[0]) if len(angles) > 1 else 0.0
        best = None
        best_key = None
        for s, e in self._gap_runs(section >= threshold):
            if (e - s) * increment < min_width:
                continue
            idx = start + (int(s) + int(e) - 1) // 2
            difference = abs(float(angles[idx] - angles[base_i]))
            if difference > math.radians(self.near_max_max_shift_deg):
                continue
            depth = float(np.minimum(distances[idx], self.near_max_range_cap))
            if depth < base_depth + self.near_max_gain_m:
                continue
            key = (int(e - s), -difference)
            if best_key is None or key > best_key:
                best, best_key = idx, key
        if best is not None and best != base_i:
            self._last_near_max = "applied"
            return best
        self._last_near_max = "no_safe_candidate"
        return base_i

    def _guarded_bubble(self, measured, distances, angles, selected_idx):
        """Close-range bubble around nearest obstacle, with safe fallback.

        Use raw valid returns to avoid reacting to an artificial disparity
        extension. Blank the obstacle's angular footprint and recalculate the
        widest gap. Reject drastic changes, avoiding Stage 4's closed gaps.
        """
        self._last_bubble = "disabled"
        if not self.bubble_enabled:
            return selected_idx, distances
        # Consider returns on a currently viable corridor near our target,
        # not the closest side-wall/duct anywhere within the forward sector.
        # Obstacles below min_gap_distance are already excluded by FTG.
        mask = ((measured >= self.min_gap_distance)
                & (distances >= self.min_gap_distance)
                & (np.abs(angles) <= math.radians(self.bubble_center_sector_deg))
                & (np.abs(angles - angles[selected_idx])
                   <= math.radians(self.bubble_target_proximity_deg)))
        if not np.any(mask):
            self._last_bubble = "no_returns"
            return selected_idx, distances
        closest = int(np.argmin(np.where(mask, measured, np.inf)))
        near_dist = float(measured[closest])
        near_angle = float(angles[closest])
        if near_dist > self.bubble_trigger_distance:
            self._last_bubble = "too_far"
            return selected_idx, distances
        if abs(near_angle - float(angles[selected_idx])) > math.radians(self.bubble_target_proximity_deg):
            self._last_bubble = "off_target"
            return selected_idx, distances
        self._bubble_trigger_total += 1
        half_angle = math.asin(min(1.0, self.bubble_radius / near_dist))
        bubble_mask = (np.abs(angles - near_angle) <= half_angle)
        proposed = distances.copy()
        proposed[bubble_mask] = 0.0
        gap = self._widest_gap(proposed, angles)
        if gap is None:
            self._last_bubble = "reject_no_gap"
            return selected_idx, distances
        candidate = gap[2]
        if abs(float(angles[candidate] - angles[selected_idx])) > math.radians(self.bubble_max_target_shift_deg):
            self._last_bubble = "reject_shift"
            return selected_idx, distances
        if proposed[candidate] < self.min_gap_distance:
            self._last_bubble = "reject_clearance"
            return selected_idx, distances
        if candidate == selected_idx:
            self._last_bubble = "unchanged"
            return selected_idx, distances
        self._last_bubble = "applied"
        self._bubble_applied_total += 1
        return candidate, proposed

    def gap_target(self, scan):
        """Frozen fallback interface: (angle in radians, range in metres)."""
        measured, angles = self._process_scan(scan)
        self._last_front_distance = 0.0
        self._last_bubble = "not_checked"
        self._last_near_max = "not_checked"
        if measured.size == 0:
            self._last_disparities = 0
            self._last_extended_beams = 0
            return 0.0, 0.0
        front_mask = ((np.abs(angles) <= math.radians(self.front_check_deg))
                      & (measured > 0.0))
        if np.any(front_mask):
            self._last_front_distance = float(np.min(measured[front_mask]))
        distances = self._extend_disparities(measured, float(scan.angle_increment))
        base = self._widest_gap(distances, angles)
        if base is None:
            return 0.0, 0.0
        self._last_baseline_angle = float(angles[base[2]])
        selected = self._guarded_near_max(distances, angles, base)
        if self._last_near_max == "applied":
            self._near_applied_total += 1
        selected, final_ranges = self._guarded_bubble(measured, distances, angles, selected)
        return float(angles[selected]), float(final_ranges[selected])

    def _scan_dt(self, scan):
        dt = self.nominal_scan_dt
        stamp = getattr(getattr(scan, "header", None), "stamp", None)
        sec = getattr(stamp, "sec", None)
        nsec = getattr(stamp, "nanosec", None)
        if sec is not None and nsec is not None:
            try:
                time_s = float(sec) + float(nsec) * 1e-9
            except (TypeError, ValueError, OverflowError):
                time_s = None
            if time_s is not None and math.isfinite(time_s):
                if self._last_scan_stamp is not None:
                    interval = time_s - self._last_scan_stamp
                    if 0.005 <= interval <= 0.10:
                        dt = interval
                self._last_scan_stamp = time_s
        self._last_dt = dt
        return dt

    def _rate_limited_steering(self, desired, dt):
        desired_rad = float(np.clip(desired, -1.0, 1.0)) * self.max_steering_rad
        max_delta = self.max_steering_rate_rad_s * dt
        difference = desired_rad - self._previous_steer_rad
        applied_rad = self._previous_steer_rad + float(np.clip(difference, -max_delta, max_delta))
        self._previous_steer_rad = applied_rad
        return float(np.clip(applied_rad / self.max_steering_rad, -1.0, 1.0))

    def _adaptive_speed(self, steering):
        lower = float(self.min_target_speed)
        upper = float(self.crawl_speed)
        front_fraction = float(np.clip(
            (self._last_front_distance - self.front_slow_distance)
            / (self.front_full_distance - self.front_slow_distance), 0.0, 1.0
        ))
        speed_from_front = lower + (upper - lower) * front_fraction
        speed_from_turn = upper - (upper - lower) * abs(steering)
        result = float(min(speed_from_front, speed_from_turn))
        if result < upper - 1e-6:
            self._slowed_total += 1
        return result

    def _publish_marker(self, scan, angle, distance):
        """RViz target arrow in LiDAR frame; requires attach_node()."""
        if self._marker_publisher is None or self._scan_count % self.marker_every_scans:
            return
        marker = self._Marker()
        scan_header = getattr(scan, "header", None)
        marker.header.frame_id = (getattr(scan_header, "frame_id", "") or "laser")
        stamp = getattr(scan_header, "stamp", None)
        if stamp is not None:
            marker.header.stamp = stamp
        marker.ns = "reactive_gap_target"
        marker.id = 0
        marker.type = self._Marker.ARROW
        marker.action = self._Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035  # arrow shaft diameter
        marker.scale.y = 0.08   # arrow head diameter
        marker.scale.z = 0.10   # arrow head length
        marker.color.r = 0.10 if distance > 0.0 else 1.0
        marker.color.g = 1.0 if distance > 0.0 else 0.10
        marker.color.b = 0.15
        marker.color.a = 0.95
        origin = self._Point()
        point = self._Point()
        length = min(distance, self.marker_max_length_m) if distance > 0 else 0.0
        point.x = length * math.cos(angle)
        point.y = length * math.sin(angle)
        marker.points = [origin, point]
        msg = self._MarkerArray()
        msg.markers = [marker]
        self._marker_publisher.publish(msg)

    def step(self, scan):
        angle, distance = self.gap_target(scan)
        self._scan_count += 1
        dt = self._scan_dt(scan)
        if distance <= 0.0:
            applied = self._rate_limited_steering(0.0, dt)
            speed = 0.0
        else:
            desired = float(np.clip(angle / self.max_steering_rad, -1.0, 1.0))
            applied = self._rate_limited_steering(desired, dt)
            speed = self._adaptive_speed(applied)
        self._publish_marker(scan, angle, distance)
        if self._scan_count == 1 or self._scan_count % self.debug_every_scans == 0:
            print(
                f"[FTG-COMBINED] scan={self._scan_count} "
                f"angle={math.degrees(angle):+.1f}deg gap={distance:.2f}m "
                f"front={self._last_front_distance:.2f}m "
                f"bubble={self._last_bubble} bubble_trials={self._bubble_trigger_total} "
                f"bubble_applied={self._bubble_applied_total} "
                f"near={self._last_near_max} near_applied={self._near_applied_total} "
                f"disparities={self._last_disparities} extended={self._last_extended_beams} "
                f"steer={applied:+.2f} target_speed={speed:.3f}",
                flush=True,
            )
        return applied, speed

#!/usr/bin/env python3
"""Team driver - integration wiring. OWNED BY ROLE 1; nobody else edits this file.

Per LiDAR scan (40 Hz):

    decide   planner (mode='planner') or reactive (mode='reactive')
    fallback planner -> reactive when Safety says the planned line is blocked
    safety   cap the target speed from the clearance ahead
    speed    SpeedController turns the target speed into a throttle
    publish  steering and throttle, clamped to [-1, 1], never NaN

The modules live in their own files, one owner each:

    speed_controller.py  Role 2      reactive.py  Role 3
    planner.py           Role 4      safety.py    Role 5
    common.py            Role 1

Rules this file keeps (see docs/06-rules.md):
  * `ros2 run team_driver driver` stays the entry point.
  * Every default lives HERE, in code: judging runs plain `ros2 run` with no
    arguments and no yaml.
  * Nothing here ever publishes /autodrive/reset_command.

AutoDRIVE takes normalised throttle, not speed:
    /autodrive/roboracer_1/throttle_command   Float32, [-1, 1]
    /autodrive/roboracer_1/steering_command   Float32, [-1, 1]  ->  +-0.5236 rad
"""

import math
import time

import numpy as np
import rclpy
import os
from team_driver.planner import Planner
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from team_driver import planner as planner_mod
from team_driver import reactive as reactive_mod
from team_driver import safety as safety_mod
from team_driver import speed_controller as speed_mod
from team_driver.common import finite_or, world_to_car
from team_driver.planner import Planner
from team_driver.reactive import Reactive
from team_driver.safety import Safety
from team_driver.speed_controller import SpeedController

# The steering command is normalised; this is what +-1.0 actually means.
MAX_STEERING_RAD = 0.5236


def devkit_qos() -> QoSProfile:
    """Match the QoS the AutoDRIVE bridge publishes and subscribes with.

    RELIABLE / KEEP_LAST(1) / VOLATILE. Get this wrong and the topics simply do
    not connect, and `ros2 topic list` still shows them.
    """
    return QoSProfile(
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class Driver(Node):

    NOMINAL_DT = 0.025   # [s] 40 Hz, used when message time is unusable

    def __init__(self):
        super().__init__('driver')

        # Defaults live in code. Retune from the command line if you like:
        #   ros2 run team_driver driver --ros-args -p mode:=planner
        self.declare_parameter('vehicle_ns', '/autodrive/roboracer_1')
        self.declare_parameter('mode', 'planner')           # or 'planner'
        self.declare_parameter('max_range', 10.0)            # [m] clip the scan
        self.declare_parameter('fault_throttle', 0.0)        # used if a step raises
        self.declare_parameter('callback_budget_s', 0.025)   # 40 Hz = 25 ms
        self.declare_parameter('path_points', 40)            # points for the blocked check

        self.mode = str(self.get_parameter('mode').value)
        if self.mode not in ('reactive', 'planner'):
            self.get_logger().warn(
                f"unknown mode '{self.mode}', using 'reactive'")
            self.mode = 'reactive'
        self.max_range = float(self.get_parameter('max_range').value)
        self.fault_throttle = float(self.get_parameter('fault_throttle').value)
        self.budget = float(self.get_parameter('callback_budget_s').value)
        self.path_points = int(self.get_parameter('path_points').value)

        ns = str(self.get_parameter('vehicle_ns').value).rstrip('/')
        qos = devkit_qos()

        self.throttle_pub = self.create_publisher(Float32, f'{ns}/throttle_command', qos)
        self.steering_pub = self.create_publisher(Float32, f'{ns}/steering_command', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/driver/markers', 1)

        self.create_subscription(LaserScan, f'{ns}/lidar', self.scan_callback, qos)
        self.create_subscription(Odometry, f'{ns}/odom', self.odom_callback, qos)
        self.create_subscription(Point, f'{ns}/ips', self.ips_callback, qos)

        # The four modules, each owned by one teammate.
        self.reactive = Reactive(max_range=self.max_range)
        self.planner = Planner(
            "/hackathon/maps/icra26_compete_racingline_apex_reconstructed.csv",
            target_speed=7.0,
        )
        self.safety = Safety()
        self.speed_ctrl = SpeedController()

        # Latest ground-truth pose and speed from /odom.
        self.position = None      # (x, y) in the `world` frame
        self.yaw = 0.0            # [rad]
        self.speed = 0.0          # [m/s]
        self.filtered_speed = 0.0
        self.last_steering = 0.0



        self._last_t = None
        self._source = None
        self._marker_divisor = 0
        self._cb_count = 0
        self._cb_worst = 0.0

        stubs = [name for name, mod in (
            ('speed_controller', speed_mod), ('reactive', reactive_mod),
            ('planner', planner_mod), ('safety', safety_mod))
            if getattr(mod, 'IS_STUB', False)]
        self.get_logger().info(f"team_driver up, mode='{self.mode}'")
        if stubs:
            self.get_logger().warn(
                'still STUBS (not driving yet): ' + ', '.join(stubs))

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        measured_speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )

        # Smooth speed changes across odometry messages.
        alpha = 0.20
        self.filtered_speed = (
            alpha * measured_speed
            + (1.0 - alpha) * self.filtered_speed
        )
        self.speed = self.filtered_speed

    def ips_callback(self, msg):
        """Position alone, at the same rate. /odom carries this plus velocity."""
        self.position = (msg.x, msg.y)

    def _dt(self, scan):
        """Seconds since the previous scan, from MESSAGE time.

        The simulator can run slower than real time (about half speed on some
        laptops), so wall-clock dt would be wrong for the controllers. Falls
        back to wall clock only if the message carries no usable stamp.
        """
        stamp = scan.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if t <= 0.0:
            t = time.monotonic()
        dt = self.NOMINAL_DT if self._last_t is None else t - self._last_t
        self._last_t = t
        if not (1e-3 <= dt <= 0.2):    # first call, stamp jump, or a pause
            dt = self.NOMINAL_DT
        return dt

    # ------------------------------------------------------------------
    # The control loop, once per LiDAR scan.
    # ------------------------------------------------------------------
    def scan_callback(self, scan):
        t0 = time.perf_counter()
        try:
            dt = self._dt(scan)
            steering, throttle = self.control(scan, dt)
        except Exception as exc:    # a dead node would leave the last command running
            steering, throttle = 0.0, self.fault_throttle
            self.get_logger().error(
                f'control step failed, publishing fault command: {exc!r}',
                throttle_duration_sec=2.0)

        self.publish(steering, throttle)

        self._marker_divisor = (self._marker_divisor + 1) % 10
        if self._marker_divisor == 0:
            self.publish_marker(finite_or(steering) * MAX_STEERING_RAD)

        elapsed = time.perf_counter() - t0
        self._cb_worst = max(self._cb_worst, elapsed)
        self._cb_count += 1
        if elapsed > self.budget:
            self.get_logger().warn(
                f'callback took {elapsed * 1000:.1f} ms '
                f'(budget {self.budget * 1000:.0f} ms)',
                throttle_duration_sec=5.0)
        if self._cb_count % 400 == 0:
            self.get_logger().info(
                f'worst callback in last 400 scans: {self._cb_worst * 1000:.1f} ms')
            self._cb_worst = 0.0

    def control(self, scan, dt):
        """decide -> fallback -> safety cap -> speed controller."""
        source = 'reactive'
        if self.mode == 'planner' and self.position is not None:
            x, y = self.position
            steering, v_target = self.planner.step(x, y, self.yaw, self.speed)
            source = 'planner'
            path = self.planner.path_ahead(x, y, self.yaw, self.path_points)
            car_path = world_to_car(path, x, y, self.yaw)
            if self.safety.line_blocked(car_path, scan):
                steering, v_target = self.reactive.step(scan)
                source = 'reactive (line blocked)'
        else:
            steering, v_target = self.reactive.step(scan)

        if source != self._source:
            self.get_logger().info(f'steering source: {source}')
            self._source = source

        v_target = finite_or(v_target, 0.0)
        v_target = finite_or(self.safety.limit(v_target, scan), 0.0)
        throttle = self.speed_ctrl.step(v_target, self.speed, dt)
        return finite_or(steering), finite_or(throttle)
    # ==================================================================
    # THIS IS THE PART YOU WRITE.
    # ==================================================================
    def plan(self, ranges, angles):
        """Follow the centreline with smoothed steering."""
        if self.position is None:
            return 0.0, 0.0

        x, y = self.position
        steering, target_speed = self.planner.step(
            x, y, self.yaw, self.speed
        )

        # Smooth abrupt steering changes.
        steering = float(np.clip(steering, -1.0, 1.0))
        smoothing = 0.25
        steering = (
            smoothing * steering
            + (1.0 - smoothing) * self.last_steering
        )
        self.last_steering = steering

        # Conservative throttle control.
        throttle = 0.10

        return steering, throttle

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------
    def publish(self, steering, throttle):
        steering = float(np.clip(finite_or(steering), -1.0, 1.0))
        throttle = float(np.clip(finite_or(throttle), -1.0, 1.0))
        self.steering_pub.publish(Float32(data=steering))
        self.throttle_pub.publish(Float32(data=throttle))

    def publish_marker(self, target_angle):
        """Draw where the car thinks it is going. Add /driver/markers in RViz."""
        marker = Marker()
        marker.header.frame_id = 'roboracer_1'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'team_driver'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = 1.5, 0.15, 0.15
        marker.color.g, marker.color.b, marker.color.a = 0.8, 1.0, 0.9
        marker.pose.orientation.z = math.sin(target_angle / 2.0)
        marker.pose.orientation.w = math.cos(target_angle / 2.0)
        array = MarkerArray()
        array.markers.append(marker)
        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

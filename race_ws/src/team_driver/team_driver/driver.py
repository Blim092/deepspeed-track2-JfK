#!/usr/bin/env python3
"""YOUR DRIVER GOES HERE.

This is the node the judges run. Keep `driver` as the executable name and keep
publishing to the two AutoDRIVE command topics; everything else is yours to
change - rewrite this file completely if you want to.

--------------------------------------------------------------------------
THIS TEMPLATE DOES NOT DRIVE
--------------------------------------------------------------------------
It is wiring, not a driver. It connects to the simulator, subscribes to the
sensors, and then publishes a slow constant throttle with the wheels straight.
It will roll forward off the line and into the first thing in front of it.
That is deliberate and it is the whole point: **there is no algorithm here and
no algorithm is shipped anywhere else in this repository.** Writing one is the
hackathon.

What the template is good for is proving your setup works. If the car moves
when you run it, then the image, the bridge, the QoS settings, the workspace
and your commands are all correct, and every problem left is yours.

    ros2 run team_driver driver
    ros2 launch team_driver driver.launch.py

`docs/04-algorithms.md` lists the approaches worth starting from - reactive
ones that need nothing but the LiDAR, planners that follow a line, model-based
control, and learned policies - with what each needs and where each breaks.
Pick one and replace `plan()` below.

--------------------------------------------------------------------------
THE CONTROL INTERFACE IS NOT A SPEED REQUEST
--------------------------------------------------------------------------
AutoDRIVE takes a normalised throttle, not a target speed:

    /autodrive/roboracer_1/throttle_command   Float32, [-1, 1]
    /autodrive/roboracer_1/steering_command   Float32, [-1, 1]  ->  +-0.5236 rad

-1 is full reverse, 0 is coasting, +1 is full torque. There is no speed
controller between you and the motor, so "go at 3 m/s" is a control problem you
now own. Whatever algorithm you pick will decide a *target* speed; turning that
into a throttle is a separate job, and it is worth several seconds a lap. See
`docs/04-algorithms.md` §4.1.

--------------------------------------------------------------------------
WHAT YOU MAY READ  (see docs/06-rules.md)
--------------------------------------------------------------------------
    .../lidar            1080 beams over 270 deg, 0.06-10 m
    .../imu              orientation, angular velocity, linear acceleration
    .../left_encoder     wheel angle, 16 PPR x 120
    .../right_encoder
    .../front_camera     RGB image (headless runs do not produce one)
    .../throttle         actuator feedback
    .../steering
    .../ips              ground-truth position - ALLOWED here, see below
    .../odom             ground-truth pose and velocity - ALLOWED, RECOMMENDED
    .../lap_count        .../lap_time  .../last_lap_time  .../best_lap_time
    .../collision_count
    /tf, /tf_static

The AutoDRIVE competition rules mark the pose and race-telemetry topics
"restricted": legal for development, not at race time. **The hackathon does not
adopt that restriction for reading.** Every input above is yours to use during
a scored run, ground-truth pose included. What stays restricted is *publishing*
`/autodrive/reset_command`, which is the referee's (rule 31).
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

# The steering command is normalised; this is what +-1.0 actually means.
MAX_STEERING_RAD = 0.5236


def devkit_qos() -> QoSProfile:
    """Match the QoS the AutoDRIVE bridge publishes and subscribes with.

    RELIABLE / KEEP_LAST(1) / VOLATILE. Get this wrong and the topics simply do
    not connect: a BEST_EFFORT subscriber will never see a RELIABLE publisher's
    messages, and `ros2 topic list` will still cheerfully show the topic.
    """
    return QoSProfile(
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class Driver(Node):

    def __init__(self):
        super().__init__('driver')

        # Declared parameters can be retuned without editing code:
        #   ros2 run team_driver driver --ros-args -p crawl_throttle:=0.3
        # Add your own as you go; config/driver_params.yaml loads them.
        self.declare_parameter('vehicle_ns', '/autodrive/roboracer_1')
        self.declare_parameter('crawl_throttle', 0.12)    # [-1, 1], open loop
        self.declare_parameter('max_range', 10.0)         # [m] clip the scan here

        self.crawl_throttle = self.get_parameter('crawl_throttle').value
        self.max_range = self.get_parameter('max_range').value

        ns = str(self.get_parameter('vehicle_ns').value).rstrip('/')
        qos = devkit_qos()

        self.throttle_pub = self.create_publisher(Float32, f'{ns}/throttle_command', qos)
        self.steering_pub = self.create_publisher(Float32, f'{ns}/steering_command', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/driver/markers', 1)

        self.create_subscription(LaserScan, f'{ns}/lidar', self.scan_callback, qos)
        self.create_subscription(Odometry, f'{ns}/odom', self.odom_callback, qos)
        self.create_subscription(Point, f'{ns}/ips', self.ips_callback, qos)

        # Latest known pose and speed. Ground truth from the simulator, which
        # the hackathon rules allow you to use - so use it.
        self.position = None      # (x, y) in the `world` frame
        self.yaw = 0.0            # [rad]
        self.speed = 0.0          # [m/s]

        self._marker_divisor = 0
        self.get_logger().warn(
            'team_driver is up, but this template has no driving logic: it will '
            'crawl straight ahead until it hits something. Implement plan().')

    # ------------------------------------------------------------------
    # Where the car is. Free, accurate, and worth building on.
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def ips_callback(self, msg):
        """Position alone, at the same rate. /odom carries this plus velocity."""
        self.position = (msg.x, msg.y)

    # ------------------------------------------------------------------
    # The control loop, once per LiDAR scan (40 Hz).
    # ------------------------------------------------------------------
    def scan_callback(self, scan):
        ranges, angles = self.preprocess(scan)
        steering, throttle = self.plan(ranges, angles)
        self.publish(steering, throttle)

        self._marker_divisor = (self._marker_divisor + 1) % 10
        if self._marker_divisor == 0:
            self.publish_marker(steering * MAX_STEERING_RAD)

    def preprocess(self, scan):
        """Turn a raw scan into clean ranges plus the angle of each beam.

        Kept because every approach needs some version of it and the details
        are fiddly rather than interesting: the LiDAR reports NaN and inf, and
        arithmetic on those propagates silently through everything downstream.
        """
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)

        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        return ranges, angles

    # ==================================================================
    # THIS IS THE PART YOU WRITE.
    # ==================================================================
    def plan(self, ranges, angles):
        """Decide what the car should do, given the latest scan.

        Returns `(steering, throttle)`, both normalised to [-1, 1]. Steering
        is a fraction of full lock (MAX_STEERING_RAD); throttle is torque, not
        speed.

        Right now it returns "straight ahead, slowly", which is not driving:
        it ignores `ranges` entirely, so the car will hold its heading off the
        line and put itself into the first barrier it meets. Replace the whole
        method.

        You have more to work with than the scan. `self.position`, `self.yaw`
        and `self.speed` are ground truth from /odom and are yours to use, and
        nothing stops you subscribing to more topics, loading a map or a line
        you computed offline, or running a policy you trained. See
        docs/04-algorithms.md for the approaches and what each one needs.
        """
        return 0.0, self.crawl_throttle

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------
    def publish(self, steering, throttle):
        self.steering_pub.publish(Float32(data=float(np.clip(steering, -1.0, 1.0))))
        self.throttle_pub.publish(Float32(data=float(np.clip(throttle, -1.0, 1.0))))

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

#!/usr/bin/env python3
# ident_node.py: standalone identification tool. NOT part of driver.py.
import csv, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32            # CHECK: message type for throttle/steering
from nav_msgs.msg import Odometry
# from <template_module> import devkit_qos  # CHECK: copy the import from the template
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)


def devkit_qos() -> QoSProfile:
    # Copied from team_driver/driver.py so this tool does not depend on it.
    return QoSProfile(
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )
THROTTLE_TOPIC = "/autodrive/roboracer_1/throttle_command"  # CHECK
STEERING_TOPIC = "/autodrive/roboracer_1/steering_command"  # CHECK
ODOM_TOPIC     = "/autodrive/roboracer_1/odom"              # CHECK


class IdentNode(Node):
    def __init__(self):
        super().__init__("ident_node")
        self.declare_parameter("throttle", 0.2)
        self.declare_parameter("t_accel", 5.0)     # seconds of throttle before stopping/coasting
        self.declare_parameter("t_max", 10.0)      # hard time limit for the whole run
        self.declare_parameter("v_abort", 4.0)     # m/s: cut throttle if exceeded
        self.declare_parameter("coast", False)     # True: throttle 0 after t_accel, keep logging
        self.declare_parameter("out", "ident.csv")
        self.declare_parameter("settle", 1.0)      # s of stillness before starting

        self.thr = self.get_parameter("throttle").value
        self.t_accel = self.get_parameter("t_accel").value
        self.t_max = self.get_parameter("t_max").value
        self.v_abort = self.get_parameter("v_abort").value
        self.coast = self.get_parameter("coast").value
        self.out = self.get_parameter("out").value
        self.settle = self.get_parameter("settle").value

        qos = devkit_qos()  # CHECK: may need to be called differently
        self.pub_thr = self.create_publisher(Float32, THROTTLE_TOPIC, qos)
        self.pub_str = self.create_publisher(Float32, STEERING_TOPIC, qos)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, qos)
        self.create_timer(0.05, self.on_timer)     # 20 Hz command loop

        self.t_first = None      # sim time of first odom
        self.t0 = None           # sim time when throttle started
        self.rows = []
        self.cmd = 0.0
        self.phase = "settle"    # settle -> run -> done
        self.done = False

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if self.t_first is None:
            self.t_first = t
        if self.phase == "settle" and t - self.t_first >= self.settle:
            self.phase, self.t0 = "run", t
        if self.phase == "run":
            tr = t - self.t0
            self.rows.append((tr, v, self.cmd))
            if v > self.v_abort or tr > self.t_max:
                self.phase = "done"
            elif self.coast and tr > self.t_accel and v < 0.05:
                self.phase = "done"
            # decide the command (published by the timer)
            if self.phase == "run":
                if tr < self.t_accel:
                    self.cmd = self.thr
                else:
                    self.cmd = 0.0 if self.coast else 0.0

    def on_timer(self):
        if self.phase == "done":
            self.finish()
            return
        self.pub_thr.publish(Float32(data=float(self.cmd)))
        self.pub_str.publish(Float32(data=0.0))

    def finish(self):
        if self.done:
            return
        self.done = True
        self.pub_thr.publish(Float32(data=0.0))   # level-triggered: always leave it at 0
        with open(self.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "speed", "throttle_cmd"])
            w.writerows(self.rows)
        self.get_logger().info(f"wrote {len(self.rows)} rows to {self.out}")
        rclpy.shutdown()


def main():
    rclpy.init()
    rclpy.spin(IdentNode())

if __name__ == "__main__":
    main()

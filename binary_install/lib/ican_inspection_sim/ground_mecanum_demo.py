#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def yaw_from_odom(msg):
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GroundMecanumDemo(Node):
    def __init__(self):
        super().__init__("ground_mecanum_demo")
        self.declare_parameter(
            "waypoints",
            [
                1.125, -1.350,
                0.000, -1.350,
                0.000, -2.700,
                1.125, -1.925,
                1.125, -0.775,
                0.000, 0.000,
            ],
        )
        self.declare_parameter("target_yaw", 0.0)
        self.declare_parameter("position_tolerance", 0.08)
        self.declare_parameter("max_linear_speed", 0.35)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("xy_gain", 0.9)
        self.declare_parameter("yaw_gain", 1.8)

        flat_waypoints = list(self.get_parameter("waypoints").value)
        if len(flat_waypoints) % 2 != 0 or not flat_waypoints:
            raise RuntimeError("waypoints must contain x/y pairs")
        self.waypoints = [
            (float(flat_waypoints[index]), float(flat_waypoints[index + 1]))
            for index in range(0, len(flat_waypoints), 2)
        ]

        self.target_yaw = float(self.get_parameter("target_yaw").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.xy_gain = float(self.get_parameter("xy_gain").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)

        self.odom = None
        self.current_index = 0
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.timer = self.create_timer(0.05, self.on_timer)

        self.get_logger().info(
            f"Ground mecanum demo loaded with {len(self.waypoints)} waypoints"
        )

    def on_odom(self, msg):
        self.odom = msg

    def on_timer(self):
        cmd = Twist()
        if self.odom is None:
            self.cmd_pub.publish(cmd)
            return

        if self.current_index >= len(self.waypoints):
            self.cmd_pub.publish(cmd)
            return

        pose = self.odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = yaw_from_odom(self.odom)
        target_x, target_y = self.waypoints[self.current_index]
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)

        if distance <= self.position_tolerance:
            self.get_logger().info(
                f"Reached waypoint {self.current_index + 1}: ({target_x:.3f}, {target_y:.3f})"
            )
            self.current_index += 1
            self.cmd_pub.publish(cmd)
            return

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_x = cos_yaw * dx + sin_yaw * dy
        body_y = -sin_yaw * dx + cos_yaw * dy
        yaw_error = normalize_angle(self.target_yaw - yaw)

        cmd.linear.x = clamp(self.xy_gain * body_x, -self.max_linear_speed, self.max_linear_speed)
        cmd.linear.y = clamp(self.xy_gain * body_y, -self.max_linear_speed, self.max_linear_speed)
        cmd.angular.z = clamp(
            self.yaw_gain * yaw_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = GroundMecanumDemo()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

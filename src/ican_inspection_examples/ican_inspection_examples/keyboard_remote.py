#!/usr/bin/env python3
"""键盘遥控节点：一个进程内持续发布 Twist，避免触发飞行插件 0.5s 指令超时。

用法：
  ros2 run ican_inspection_examples keyboard_remote                       # 默认 /uav/cmd_vel
  ros2 run ican_inspection_examples keyboard_remote --ros-args -p cmd_topic:=/cmd_vel
  ros2 run ican_inspection_examples keyboard_remote --ros-args -p linear_speed:=0.15
"""

import select
import sys
import termios
import tty
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


HELP_TEXT = """
==================== 键盘遥控器（话题：{topic}）====================
  w / x : 前进 / 后退      a / d : 左移 / 右移（麦克纳姆轮横移）
  t / b : 上升 / 下降      j / l : 左转 / 右转（偏航）
  空格  : 悬停（全部速度归零，持续发布）
  q/Ctrl+C : 退出（退出前会连发零速度，但空中请先按 b 降落！）
  按键后速度保持，不需要回车；切换动作零空隙，不会触发掉高保护。
======================================================================
"""


class KeyboardRemoteNode(Node):
    """以固定频率持续发布当前速度的键盘遥控节点。"""

    def __init__(self) -> None:
        super().__init__("keyboard_remote")
        self.declare_parameter("cmd_topic", "/uav/cmd_vel")
        self.declare_parameter("cmd_rate_hz", 20.0)
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("vertical_speed", 0.25)
        self.declare_parameter("descend_speed", -0.15)
        self.declare_parameter("angular_speed", 0.3)

        topic = str(self.get_parameter("cmd_topic").value)
        self.publisher = self.create_publisher(Twist, topic, 10)

        self.cmd = Twist()
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.vertical_speed = float(self.get_parameter("vertical_speed").value)
        self.descend_speed = float(self.get_parameter("descend_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        rate_hz = max(2.0, float(self.get_parameter("cmd_rate_hz").value))
        self.period_sec = 1.0 / rate_hz

        self.get_logger().info(HELP_TEXT.format(topic=topic))

    def handle_key(self, key: str) -> bool:
        """根据按键更新当前速度。返回 False 表示请求退出。"""
        if key in ("q", "\x03"):
            return False
        if key == " ":
            self.cmd = Twist()
        elif key == "w":
            self.cmd.linear.x = self.linear_speed
        elif key == "x":
            self.cmd.linear.x = -self.linear_speed
        elif key == "a":
            self.cmd.linear.y = self.linear_speed
        elif key == "d":
            self.cmd.linear.y = -self.linear_speed
        elif key == "t":
            self.cmd.linear.z = self.vertical_speed
        elif key == "b":
            self.cmd.linear.z = self.descend_speed
        elif key == "j":
            self.cmd.angular.z = self.angular_speed
        elif key == "l":
            self.cmd.angular.z = -self.angular_speed
        return True

    def run(self, fd: int) -> None:
        running = True
        while rclpy.ok() and running:
            readable, _, _ = select.select([sys.stdin], [], [], self.period_sec)
            if readable:
                key = sys.stdin.read(1)
                running = self.handle_key(key)
            # 无论有没有按键都持续发布，指令流永不中断（看门狗要求 <0.5s 间隔）
            self.publisher.publish(self.cmd)
            rclpy.spin_once(self, timeout_sec=0.0)

    def publish_stop(self) -> None:
        for _ in range(10):
            self.publisher.publish(Twist())


def configure_terminal() -> Optional[Tuple[object, int]]:
    """切换终端为 cbreak 模式，返回 (旧属性, fd)；失败返回 None。"""
    if not sys.stdin.isatty():
        return None
    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old_attrs, fd


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardRemoteNode()

    terminal = configure_terminal()
    if terminal is None:
        node.get_logger().error("需要在真实终端中运行（不支持管道输入）。")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    old_attrs, fd = terminal
    try:
        node.run(fd)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        node.get_logger().info("已退出。若在空中退出，0.5s 后会触发掉高保护，请记得先降落。")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

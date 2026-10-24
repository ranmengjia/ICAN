#!/usr/bin/env python3
"""视觉检测实时调试节点。

订阅一路相机图像，运行 HSV 颜色检测，把标注画面发布出来，
用 rqt_image_view 查看 ~detection/image 即可实时调阈值。

用法:
  ros2 run ican_inspection_examples vision_debug_node \\
      --ros-args -p image_topic:=/uav/camera/image
"""

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from .target_detector import (
    BLUE_LOWER,
    BLUE_UPPER,
    YELLOW_LOWER,
    YELLOW_UPPER,
    detect_color_targets,
    draw_detections,
)


class VisionDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_debug_node")
        self.declare_parameter("image_topic", "/uav/camera/image")
        self.declare_parameter("min_area", 500.0)
        # HSV 阈值做成参数，调参无需改代码、无需重新编译
        self.declare_parameter(
            "hsv_lower",
            [int(v) for v in YELLOW_LOWER.tolist()],
        )
        self.declare_parameter(
            "hsv_upper",
            [int(v) for v in YELLOW_UPPER.tolist()],
        )
        # 第二套阈值：窗上照片（蓝色无人机照片）
        self.declare_parameter(
            "photo_hsv_lower",
            [int(v) for v in BLUE_LOWER.tolist()],
        )
        self.declare_parameter(
            "photo_hsv_upper",
            [int(v) for v in BLUE_UPPER.tolist()],
        )

        image_topic = str(self.get_parameter("image_topic").value)
        self.bridge = CvBridge()
        self.annotated_pub = self.create_publisher(
            Image, "~/detection/image", 10
        )
        self.mask_pub = self.create_publisher(Image, "~/detection/mask", 10)
        # 轻量检测结果话题：[cx0, cy0, area0, cx1, cy1, area1, ...]
        # 控制节点订阅它做视觉伺服，不必自己订阅大图像
        self.dets_pub = self.create_publisher(
            Float32MultiArray, "~/detections", 10
        )
        self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(f"视觉调试节点已启动，订阅 {image_topic}")

    def on_image(self, msg: Image) -> None:
        image_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        min_area = float(self.get_parameter("min_area").value)

        # 第一套：黄色（金色板子）
        lower1 = np.array(self.get_parameter("hsv_lower").value, dtype=np.uint8)
        upper1 = np.array(self.get_parameter("hsv_upper").value, dtype=np.uint8)
        dets_plate = detect_color_targets(image_bgr, lower1, upper1, min_area=min_area)

        # 第二套：蓝色（窗上照片）
        lower2 = np.array(self.get_parameter("photo_hsv_lower").value, dtype=np.uint8)
        upper2 = np.array(self.get_parameter("photo_hsv_upper").value, dtype=np.uint8)
        dets_photo = detect_color_targets(image_bgr, lower2, upper2, min_area=min_area)

        # 合并两套检测结果
        all_dets = dets_plate + dets_photo

        # 画框：板子用绿色，照片用青色
        annotated = image_bgr.copy()
        h, w = annotated.shape[:2]
        cv2.drawMarker(
            annotated, (w // 2, h // 2), (255, 0, 0),
            markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2,
        )
        for d in dets_plate:
            x = int(d.cx - d.width / 2)
            y = int(d.cy - d.height / 2)
            cv2.rectangle(annotated, (x, y), (x + int(d.width), y + int(d.height)), (0, 255, 0), 3)
            cv2.circle(annotated, (int(d.cx), int(d.cy)), 8, (0, 0, 255), -1)
            cv2.putText(annotated, f"P ({int(d.cx)},{int(d.cy)}) S={int(d.area)}",
                        (x, max(25, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        for d in dets_photo:
            x = int(d.cx - d.width / 2)
            y = int(d.cy - d.height / 2)
            cv2.rectangle(annotated, (x, y), (x + int(d.width), y + int(d.height)), (255, 255, 0), 3)
            cv2.circle(annotated, (int(d.cx), int(d.cy)), 8, (255, 0, 0), -1)
            cv2.putText(annotated, f"Photo ({int(d.cx)},{int(d.cy)}) S={int(d.area)}",
                        (x, max(25, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # 把检测结果摊平发布：[cx0, cy0, area0, label0, cx1, cy1, area1, label1, ...]
        # label: 0 = 黄色板子, 1 = 蓝色窗上照片
        # 控制节点按 label 区分类别，不再依赖画面位置（照片实际在画面下半部）
        flat = []
        for d in dets_plate:
            flat.extend([d.cx, d.cy, d.area, 0.0])
        for d in dets_photo:
            flat.extend([d.cx, d.cy, d.area, 1.0])
        det_msg = Float32MultiArray()
        det_msg.data = flat
        self.dets_pub.publish(det_msg)

        self.annotated_pub.publish(
            self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        )

        # 掩码也发一份，调 HSV 阈值时直接看黑白图最直观
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        self.mask_pub.publish(
            self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        )

        if all_dets:
            names = ", ".join(
                f"({d.cx:.0f},{d.cy:.0f},S={d.area:.0f})" for d in all_dets
            )
            self.get_logger().info(
                f"检测到 {len(all_dets)} 个目标: {names}",
                throttle_duration_sec=0.5,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""基于 HSV 颜色分割的目标检测器。

设计原则：纯函数、无 ROS 依赖，方便单独用图片/视频调试，
比赛节点和调试节点都可以复用同一套检测逻辑。
"""

from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np


@dataclass
class Detection:
    """一个被检测到的目标。"""

    cx: float           # 质心像素 x
    cy: float           # 质心像素 y
    width: float        # 外接框宽（像素）
    height: float       # 外接框高（像素）
    area: float         # 轮廓面积（像素²），可用来估计远近

    def offset_from_center(self, image_w: int, image_h: int) -> Tuple[float, float]:
        """相对画面中心的偏差（像素）。右为 x 正，下为 y 正。"""
        return self.cx - image_w / 2.0, self.cy - image_h / 2.0


# 空中任务点是金黄色平板，H 在 15~35 之间；S/V 下限过滤掉灰色地面
YELLOW_LOWER = np.array([15, 80, 120], dtype=np.uint8)
YELLOW_UPPER = np.array([35, 255, 255], dtype=np.uint8)

# 窗上目标照片是无人机照片，主色调为蓝色
# 实测仿真渲染：照片 H 84~120(mean109) S 36~245(mean175) V 94~217(mean125)
# 天空 H~105 S~38 V~209；用 S>=80 排除低饱和天空，V>=80 保留中亮度照片
BLUE_LOWER = np.array([90, 80, 80], dtype=np.uint8)
BLUE_UPPER = np.array([130, 255, 255], dtype=np.uint8)


def detect_color_targets(
    image_bgr: np.ndarray,
    lower_hsv: np.ndarray,
    upper_hsv: np.ndarray,
    min_area: float = 500.0,
) -> List[Detection]:
    """在 BGR 图像中找出指定 HSV 颜色范围内的目标。

    参数:
        image_bgr: OpenCV 格式 BGR 图像
        lower_hsv/upper_hsv: HSV 下/上界（OpenCV 口径：H 0~179）
        min_area: 轮廓面积下限，小于它的噪点直接丢弃
    返回:
        Detection 列表，按面积从大到小排列
    """
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # 形态学：开运算去小噪点，闭运算补小空洞
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    detections: List[Detection] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            continue
        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]
        detections.append(
            Detection(cx=cx, cy=cy, width=float(w), height=float(h), area=area)
        )

    detections.sort(key=lambda d: d.area, reverse=True)
    return detections


def draw_detections(
    image_bgr: np.ndarray, detections: List[Detection]
) -> np.ndarray:
    """把检测结果画到图像上：绿框 + 红色质心 + 画面中心十字。"""
    vis = image_bgr.copy()
    h, w = vis.shape[:2]
    # 画面中心参考十字
    cv2.drawMarker(
        vis, (w // 2, h // 2), (255, 0, 0),
        markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2,
    )
    for i, d in enumerate(detections):
        x = int(d.cx - d.width / 2)
        y = int(d.cy - d.height / 2)
        cv2.rectangle(
            vis, (x, y), (x + int(d.width), y + int(d.height)), (0, 255, 0), 3
        )
        cv2.circle(vis, (int(d.cx), int(d.cy)), 8, (0, 0, 255), -1)
        cv2.putText(
            vis,
            f"#{i} ({int(d.cx)},{int(d.cy)}) S={int(d.area)}",
            (x, max(25, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
    return vis

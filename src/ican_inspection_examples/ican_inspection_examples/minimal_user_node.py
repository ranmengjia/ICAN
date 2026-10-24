#!/usr/bin/env python3

import math
import time
from typing import Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from .target_detector import Detection


GroundWaypoint = Tuple[float, float, float]


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class MinimalUserNode(Node):
    """A small template showing the simulator interfaces without solving the task."""

    def __init__(self) -> None:
        super().__init__("minimal_user_node")
        self.declare_parameter("demo_mode", "cmd_vel_smoke_test")
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter("takeoff_seconds", 2.6)
        self.declare_parameter("takeoff_speed", 0.25)
        self.declare_parameter("move_seconds", 2.0)
        self.declare_parameter("move_speed", 0.12)
        self.declare_parameter("hover_seconds", 6.0)
        self.declare_parameter("land_seconds", 3.2)
        self.declare_parameter("land_speed", -0.20)
        self.declare_parameter("cmd_rate_hz", 25.0)
        self.declare_parameter("nav_retry_count", 1)
        self.declare_parameter("odom_timeout_sec", 30.0)

        # --- 空中闭环飞行参数 ---
        self.declare_parameter("takeoff_alt", 0.8)          # 起飞目标高度 (m)
        self.declare_parameter("flight_kp_xy", 1.0)         # 水平 P 增益
        self.declare_parameter("flight_kp_z", 2.0)          # 垂直 P 增益
        self.declare_parameter("arrival_thresh", 0.15)      # 到达判定阈值 (m)
        self.declare_parameter("flight_timeout_sec", 20.0)  # 单点飞行超时 (s)
        self.declare_parameter("max_flight_alt", 1.5)       # 飞行天花板 (m)
        self.declare_parameter("land_alt_threshold", 0.1)   # 落地判定阈值 (m)
        self.declare_parameter("max_horizontal_speed", 0.35)  # 水平速度上限 (m/s)
        self.declare_parameter("takeoff_settle_sec", 2.0)   # 起飞到位后悬停稳定时间 (s)
        self.declare_parameter("odom_sanity_limit", 100.0)  # 里程计看门狗阈值 (m)，超过判定物理爆炸

        # --- 任务三：空中点激光照射参数 ---
        self.declare_parameter("laser_alt", 0.9)       # 照射高度 (m)，规则要求 >=0.8
        self.declare_parameter("scan_radius", 0.10)    # 十字扫查半径 (m)
        self.declare_parameter("scan_dwell_sec", 0.4)  # 每个扫查点停留时间 (s)
        self.declare_parameter("scan_thresh", 0.08)    # 扫查到位阈值 (m)

        self.ground_task_waypoints: Dict[str, GroundWaypoint] = {
            "ground_0": (1.125, -1.350, 0.0),
            "ground_1": (0.000, -1.350, 0.0),
            "ground_2": (0.000, -2.700, 0.0),
            "ground_3": (1.125, -1.925, 0.0),
            "ground_4": (1.125, -0.775, 0.0),
        }
        # 空中巡检航点（x, y, z），z 为目标高度
        self.uav_task_waypoints: Dict[str, Tuple[float, float, float]] = {
            "uav_0": (1.125, -1.350, 1.0),   # ground_0 上空
            "uav_1": (0.000, -1.350, 1.0),   # ground_1 上空
            "uav_2": (0.000, -2.700, 1.0),   # ground_2 上空
            "uav_3": (1.125, -1.925, 1.0),   # ground_3 上空
            "uav_4": (1.125, -0.775, 1.0),   # ground_4 上空
        }
        # 任务三：金色空中任务板中心 (x, y)。
        # 注意板子只有 visual 没有 collision，向下激光测距无法判断是否照中。
        self.aerial_task_points: Dict[str, Tuple[float, float]] = {
            "aerial_1": (3.080, -0.320),
            "aerial_2": (3.080, -1.355),
        }
        # 窗户开口中心（开口 z 0.4~1.15，巡航取 0.9m）。板子区在窗户墙东侧，必须穿窗到达
        self.window_1_center = (1.925, -0.3275, 0.9)
        self.window_2_center = (1.925, -2.3725, 0.9)
        # --- 视觉伺服参数（任务三 B 方案） ---
        self.declare_parameter("vision_min_area", 500.0)    # 板子面积下限 (px)
        self.declare_parameter("vision_max_area", 20000.0)  # 上限：过滤地板图案误检
        self.declare_parameter("vision_kp_y", 0.35)         # 横向 P 增益 (m/s per 满幅偏差)
        self.declare_parameter("vision_center_tol_px", 25.0)  # 对准判定 (px)

        # --- 任务二/四：窗上目标照片识别参数 ---
        self.declare_parameter("photo_min_area", 5000.0)    # 照片面积下限 (px)
        self.declare_parameter("photo_max_area", 400000.0)  # 照片面积上限 (px)，窗前稳定约31万
        self.declare_parameter("photo_detect_sec", 3.0)     # 识别确认时间 (s)
        self.declare_parameter("dip_alt", 0.3)              # 明显下降幅度 (m)
        self.declare_parameter("dip_sec", 1.5)              # 下降/回升各用时间 (s)

        self.ground_odom: Optional[Odometry] = None
        self.uav_odom: Optional[Odometry] = None
        self.latest_plate_det: Optional[Detection] = None
        self.latest_photo_det: Optional[Detection] = None

        self.nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("nav_action_name").value)
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.uav_cmd_vel_pub = self.create_publisher(Twist, "/uav/cmd_vel", 10)
        self.laser_pub = self.create_publisher(Bool, "/uav/laser_fire", 10)
        self.create_subscription(Odometry, "/odom", self.on_ground_odom, 10)
        self.create_subscription(Odometry, "/uav/odom", self.on_uav_odom, 10)
        # 订阅视觉节点的轻量检测结果，而不是相机大图：
        # 800x800@30Hz 图像会淹没执行器回调队列，饿死里程计等关键回调
        self.create_subscription(
            Float32MultiArray, "/vision_debug_node/detections", self.on_detections, 10
        )

    def on_ground_odom(self, msg: Odometry) -> None:
        self.ground_odom = msg

    def on_uav_odom(self, msg: Odometry) -> None:
        self.uav_odom = msg

    def on_detections(self, msg: Float32MultiArray) -> None:
        """接收视觉节点的检测结果 [cx,cy,area,label, ...]。

        label: 0=黄色板子, 1=蓝色窗上照片。按 label 区分类别，
        不再依赖画面位置（照片和板子都可能出现在画面下半部）。
        面积上下限过滤：地板黄色图案约 4 万像素（误检），
        金色板子在 1~2.5m 外约 1~5 千像素，窗上照片约 10 万像素。
        """
        plate_min = float(self.get_parameter("vision_min_area").value)
        plate_max = float(self.get_parameter("vision_max_area").value)
        photo_min = float(self.get_parameter("photo_min_area").value)
        photo_max = float(self.get_parameter("photo_max_area").value)
        data = msg.data
        best_plate: Optional[Detection] = None
        best_photo: Optional[Detection] = None
        for i in range(0, len(data) - 3, 4):
            cx, cy, area, label = data[i], data[i + 1], data[i + 2], data[i + 3]
            if label < 0.5:
                # 黄色板子
                if plate_min <= area <= plate_max:
                    if best_plate is None or area > best_plate.area:
                        best_plate = Detection(cx=cx, cy=cy, width=0.0, height=0.0, area=area)
            else:
                # 蓝色窗上照片
                if photo_min <= area <= photo_max:
                    if best_photo is None or area > best_photo.area:
                        best_photo = Detection(cx=cx, cy=cy, width=0.0, height=0.0, area=area)
        self.latest_plate_det = best_plate
        self.latest_photo_det = best_photo

    def send_ground_nav_goal(self, waypoint: GroundWaypoint) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 navigate_to_pose action server 未就绪。")
            return False

        x, y, yaw = waypoint
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self.nav_client.send_goal_async(goal)
        if not self.wait_for_future(send_future):
            return False
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        if not self.wait_for_future(result_future):
            return False
        result = result_future.result()
        return result is not None and result.status == GoalStatus.STATUS_SUCCEEDED

    def wait_for_ground_odom(self) -> bool:
        """等待第一条 /odom 到达。订阅是异步的，启动瞬间 ground_odom 为 None。"""
        timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.ground_odom is not None:
                return True
            self.get_logger().info(
                "正在等待地面里程计 /odom ...", throttle_duration_sec=1.0
            )
        return False

    def wait_for_uav_odom(self) -> bool:
        """等待第一条 /uav/odom 到达。"""
        timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.uav_odom is not None:
                return True
            self.get_logger().info(
                "正在等待 UAV 里程计 /uav/odom ...", throttle_duration_sec=1.0
            )
        return False

    def _uav_pos(self) -> Tuple[float, float, float]:
        """从 /uav/odom 读取当前位置。"""
        if self.uav_odom is None:
            return 0.0, 0.0, 0.0
        p = self.uav_odom.pose.pose.position
        return p.x, p.y, p.z

    def _uav_yaw(self) -> float:
        """从 /uav/odom 读取当前偏航角 (rad)。"""
        if self.uav_odom is None:
            return 0.0
        o = self.uav_odom.pose.pose.orientation
        sin_yaw = 2.0 * (o.w * o.z + o.x * o.y)
        cos_yaw = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        return math.atan2(sin_yaw, cos_yaw)

    def _odom_sane(self, x: float, y: float, z: float) -> bool:
        """里程计看门狗：数值爆炸（撞墙/穿模后物理引擎 NaN 传播）时判定异常。

        天文数字位置意味着仿真状态已损坏，继续发指令只会让飞机彻底飞丢，
        必须立即停桨中止，而不是用错误坐标继续闭环。
        """
        limit = float(self.get_parameter("odom_sanity_limit").value)
        return abs(x) < limit and abs(y) < limit and abs(z) < limit + 5.0

    def _uav_velocity_cmd(
        self, dx: float, dy: float, dz: float, yaw: float
    ) -> Tuple[float, float, float]:
        """P 控制计算速度并做安全限幅：水平限速、垂直限速 ±0.3。

        dx/dy 是世界坐标系的目标偏差；/uav/cmd_vel 的 linear.x/y 是机体坐标系，
        必须把世界速度旋转 -yaw 转到机体坐标系，否则 yaw≠0 时会飞偏方向。
        """
        kp_xy = float(self.get_parameter("flight_kp_xy").value)
        kp_z = float(self.get_parameter("flight_kp_z").value)
        max_h = float(self.get_parameter("max_horizontal_speed").value)
        vx_world = kp_xy * dx
        vy_world = kp_xy * dy
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        vx_body = vx_world * cos_y + vy_world * sin_y
        vy_body = -vx_world * sin_y + vy_world * cos_y
        vx = max(-max_h, min(vx_body, max_h))
        vy = max(-max_h, min(vy_body, max_h))
        vz = max(-0.3, min(kp_z * dz, 0.3))
        return vx, vy, vz

    def uav_goto(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        thresh: Optional[float] = None,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        """P 闭环飞到目标点 (x, y, z)。返回是否成功到达。

        thresh/timeout_sec 不传则用参数 arrival_thresh / flight_timeout_sec，
        精细移动（如扫查）时传更小的阈值。
        """
        if not self.wait_for_uav_odom():
            self.get_logger().error("等待 /uav/odom 超时，uav_goto 失败。")
            return False

        kp_xy = float(self.get_parameter("flight_kp_xy").value)
        kp_z = float(self.get_parameter("flight_kp_z").value)
        if thresh is None:
            thresh = float(self.get_parameter("arrival_thresh").value)
        if timeout_sec is None:
            timeout_sec = float(self.get_parameter("flight_timeout_sec").value)
        cmd_rate_hz = float(self.get_parameter("cmd_rate_hz").value)
        max_alt = float(self.get_parameter("max_flight_alt").value)
        rate_sec = 1.0 / cmd_rate_hz

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            cx, cy, cz = self._uav_pos()
            yaw = self._uav_yaw()

            # 里程计看门狗：物理爆炸时立即停桨，不再用天文坐标闭环
            if not self._odom_sane(cx, cy, cz):
                self.stop_uav()
                self.get_logger().error(
                    f"/uav/odom 数值异常 ({cx:.1f}, {cy:.1f}, {cz:.1f})，"
                    f"判定物理状态损坏，立即停桨中止。"
                )
                return False

            dx = target_x - cx
            dy = target_y - cy
            dz = target_z - cz
            horiz = math.sqrt(dx * dx + dy * dy)

            # 水平与垂直分别判定：避免 z 还差 0.15m 时因 3D 欧氏距离恰好
            # 小于阈值而误判到达，导致没飞稳就进入下一个机动
            if horiz < thresh and abs(dz) < thresh:
                self.stop_uav()
                self.get_logger().info(
                    f"到达目标 ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})，"
                    f"当前 ({cx:.2f}, {cy:.2f}, {cz:.2f})，水平误差 {horiz:.2f}m"
                )
                return True

            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.linear.z = self._uav_velocity_cmd(
                dx, dy, dz, yaw
            )
            # 锁定 yaw：斜飞/受扰动后不锁航向会让速度控制失稳
            cmd.angular.z = 0.0
            # 限制高度不超过天花板
            if cz > max_alt and cmd.linear.z > 0:
                cmd.linear.z = 0.0
            self.uav_cmd_vel_pub.publish(cmd)
            time.sleep(rate_sec)

        self.stop_uav()
        err = math.sqrt(
            (target_x - cx) ** 2 + (target_y - cy) ** 2 + (target_z - cz) ** 2
        )
        self.get_logger().error(
            f"uav_goto 超时：目标 ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})，"
            f"当前 ({cx:.2f}, {cy:.2f}, {cz:.2f})，误差 {err:.2f}m"
        )
        return False

    def uav_takeoff(self) -> bool:
        """在当前位置原地起飞到指定高度。

        地面巡检结束后飞机可能在任意航点，不能硬编码飞回 (0,0)，
        否则 P 控制器会边上升边水平长距离机动，容易撞墙且超时。
        """
        takeoff_alt = float(self.get_parameter("takeoff_alt").value)
        cx, cy, _ = self._uav_pos()
        self.get_logger().info(f"在 ({cx:.2f}, {cy:.2f}) 原地起飞至 {takeoff_alt:.2f} m ...")
        if not self.uav_goto(cx, cy, takeoff_alt):
            return False
        # 到位后悬停稳定：让爬升动量和姿态完全收敛，再开始水平机动。
        # 否则还带着上升速度就斜飞，低速大姿态容易失稳撞地。
        settle = float(self.get_parameter("takeoff_settle_sec").value)
        self.get_logger().info(f"起飞到位，悬停稳定 {settle:.1f}s ...")
        cx, cy, cz = self._uav_pos()
        self.uav_hold(cx, cy, cz, settle)
        return True

    def uav_land(self) -> None:
        """先飞到当前位置正上方 0.6m，然后缓慢下降直到落地。"""
        if not self.wait_for_uav_odom():
            self.get_logger().error("等待 /uav/odom 超时，无法降落。")
            return

        cx, cy, cz = self._uav_pos()
        land_thresh = float(self.get_parameter("land_alt_threshold").value)
        cmd_rate_hz = float(self.get_parameter("cmd_rate_hz").value)
        rate_sec = 1.0 / cmd_rate_hz

        self.get_logger().info(f"当前 ({cx:.2f}, {cy:.2f}, {cz:.2f})，准备降落 ...")
        # 先飞到正上方 0.6m（保持当前 x,y）
        if not self.uav_goto(cx, cy, 0.6):
            self.get_logger().warning("未到达降落高度，强制下降。")
            return

        # 缓慢下降直到 z < land_thresh
        cmd = Twist()
        cmd.linear.z = -0.15
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            _, _, cz = self._uav_pos()
            if cz < land_thresh:
                self.stop_uav()
                self.get_logger().info(f"已落地，高度 {cz:.2f} m。")
                return
            self.uav_cmd_vel_pub.publish(cmd)
            time.sleep(rate_sec)

    def uav_hold(self, x: float, y: float, z: float, duration_sec: float) -> None:
        """在 (x, y, z) 悬停 duration_sec 秒。

        持续发布 P 修正速度：既喂饱 0.5s 看门狗，又能抗扰动把飞机拉回原位。
        与 publish_uav_cmd 发固定速度不同，这是"定点保持"。
        """
        kp_xy = float(self.get_parameter("flight_kp_xy").value)
        kp_z = float(self.get_parameter("flight_kp_z").value)
        cmd_rate_hz = float(self.get_parameter("cmd_rate_hz").value)
        rate_sec = 1.0 / cmd_rate_hz

        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            cx, cy, cz = self._uav_pos()
            yaw = self._uav_yaw()
            if not self._odom_sane(cx, cy, cz):
                self.stop_uav()
                self.get_logger().error("悬停期间 /uav/odom 异常，停止悬停。")
                return
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.linear.z = self._uav_velocity_cmd(
                x - cx, y - cy, z - cz, yaw
            )
            cmd.angular.z = 0.0
            self.uav_cmd_vel_pub.publish(cmd)
            time.sleep(rate_sec)

    def vision_align_lateral(self, timeout_sec: float = 8.0) -> bool:
        """视觉伺服：横向移动，把检测到的板子拉到画面中线（cx ≈ 宽/2）。

        原理：像素偏差 -> 横向速度。相机朝机头 +x，画面右侧对应机体 -y，
        所以 vy = -kp * offset_x / (宽/2)。只修横向；纵向距离交给里程计，
        因为板子进入正下方后相机就看不见了（垂直视场 ±40°）。
        """
        kp = float(self.get_parameter("vision_kp_y").value)
        tol = float(self.get_parameter("vision_center_tol_px").value)
        kp_z = float(self.get_parameter("flight_kp_z").value)
        cmd_rate_hz = float(self.get_parameter("cmd_rate_hz").value)
        rate_sec = 1.0 / cmd_rate_hz

        hx, _, hz = self._uav_pos()  # 锁住当前 x 和高度，只动 y
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            cx, cy, cz = self._uav_pos()
            cmd = Twist()
            cmd.linear.x = 1.0 * (hx - cx)
            cmd.linear.z = max(-0.3, min(kp_z * (hz - cz), 0.3))

            det = self.latest_plate_det
            if det is not None:
                img_w = 800  # 相机画面宽 800px
                off_x, _ = det.offset_from_center(img_w, img_w)
                if abs(off_x) <= tol:
                    self.stop_uav()
                    self.get_logger().info(
                        f"视觉对准完成：板心像素偏差 {off_x:.0f}px，当前 y={cy:.3f}"
                    )
                    return True
                vy = -kp * off_x / (img_w / 2.0)
                cmd.linear.y = max(-0.25, min(vy, 0.25))
                self.get_logger().info(
                    f"伺服中：板心偏差 {off_x:+.0f}px -> vy={cmd.linear.y:+.2f} m/s",
                    throttle_duration_sec=0.5,
                )
            else:
                self.get_logger().warning(
                    "视野内没有有效板子，悬停等待 ...", throttle_duration_sec=1.0
                )
            self.uav_cmd_vel_pub.publish(cmd)
            time.sleep(rate_sec)

        self.get_logger().warning("视觉对准超时，按当前位置继续。")
        return False

    def run_vision_servo_demo(self) -> None:
        """B 方案演示：故意瞄偏 30cm，视觉伺服拉回中线，再穿窗照射板 1。"""
        if not self.uav_takeoff():
            self.get_logger().error("起飞失败，演示中止。")
            return

        # 故意偏 +0.30m，制造明显偏差，方便观察视觉纠偏
        aim_y = self.window_1_center[1] + 0.30
        self.get_logger().info(f"故意瞄偏到 y={aim_y:.2f}（板子在 y=-0.33）...")
        self.uav_goto(1.20, aim_y, 0.9)

        self.get_logger().info("开始视觉伺服对准板 1，观察 Gazebo 中飞机横向挪动 ...")
        self.vision_align_lateral(timeout_sec=8.0)

        # 对准后垂直穿窗 1，照射板 1，然后返航
        self.uav_goto(*self.window_1_center)
        self.laser_scan_point(*self.aerial_task_points["aerial_1"])

        self.get_logger().info("返航 ...")
        self.uav_goto(2.60, self.window_1_center[1], 0.9)
        self.uav_goto(1.20, self.window_1_center[1], 0.9)
        self.uav_goto(0.0, 0.0, float(self.get_parameter("takeoff_alt").value))
        self.uav_land()
        self.get_logger().info("视觉伺服演示结束。")

    def detect_window_photo(self, timeout_sec: float = 5.0) -> bool:
        """悬停并确认检测到窗上照片，返回是否稳定识别到。"""
        detect_sec = float(self.get_parameter("photo_detect_sec").value)
        deadline = time.monotonic() + timeout_sec
        first_seen: Optional[float] = None

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            det = self.latest_photo_det
            if det is not None:
                if first_seen is None:
                    first_seen = time.monotonic()
                    self.get_logger().info(
                        f"发现窗上照片：像素({det.cx:.0f},{det.cy:.0f}) 面积={det.area:.0f}px"
                    )
                elif time.monotonic() - first_seen >= detect_sec:
                    self.get_logger().info(
                        f"照片稳定识别 {detect_sec:.1f}s，确认目标。"
                    )
                    return True
            else:
                first_seen = None
            time.sleep(0.05)
        self.get_logger().warning("未稳定识别到窗上照片。")
        return False

    def obvious_dip(self) -> None:
        """执行"明显下降"动作：先降 dip_alt 米，再升回原高度。"""
        dip = float(self.get_parameter("dip_alt").value)
        dip_sec = float(self.get_parameter("dip_sec").value)
        cx, cy, cz = self._uav_pos()

        # 安全下限：不能低于 0.25m（略高于地面，防止触地超时）
        target_low = max(0.25, cz - dip)
        self.get_logger().info(f"执行明显下降：从 {cz:.2f}m 降至 {target_low:.2f}m ...")
        # 快速下降
        self.uav_goto(cx, cy, target_low, thresh=0.05, timeout_sec=dip_sec + 2.0)
        # 悬停一下让裁判看清
        self.uav_hold(cx, cy, target_low, 0.5)
        # 回升
        self.get_logger().info(f"回升到 {cz:.2f}m ...")
        self.uav_goto(cx, cy, cz, thresh=0.05, timeout_sec=dip_sec + 2.0)
        self.get_logger().info("明显下降动作完成。")

    def run_window_photo(self) -> None:
        """任务二/四：窗上目标照片识别 + 明显下降 + 穿窗。"""
        if not self.uav_takeoff():
            self.get_logger().error("起飞失败，任务中止。")
            return

        # 飞到窗 1 西侧对准点，升高到 1.2m 以便看到窗户上方的照片
        # 照片在 z=1.40，相机在 0.9m 平视看不到，需要飞到 1.2m 让照片进入视野上半部
        w1_west_low = (1.20, self.window_1_center[1], self.window_1_center[2])
        w1_west_high = (1.20, self.window_1_center[1], 1.2)
        self.get_logger().info("前往窗 1 西侧对准点（升高到 1.2m 看照片） ...")
        if not self.uav_goto(*w1_west_high):
            self.get_logger().error("未能到达窗 1 对准点，任务中止。")
            self.uav_land()
            return

        # 悬停并识别窗上照片
        self.get_logger().info("悬停识别窗上照片 ...")
        self.uav_hold(*w1_west_high, 1.0)  # 先稳 1 秒让图像稳定
        if not self.detect_window_photo(timeout_sec=6.0):
            self.get_logger().warning("未识别到照片，仍继续穿窗（比赛时需按实际规则处理）。")

        # 降回穿窗高度 0.9m，准备做明显下降
        self.get_logger().info("降回 0.9m 准备明显下降 ...")
        if not self.uav_goto(*w1_west_low):
            self.get_logger().warning("未能精确回到 0.9m，继续尝试明显下降。")

        # 明显下降动作
        self.obvious_dip()

        # 垂直穿窗 1
        self.get_logger().info("垂直穿窗 1 ...")
        w1_east = (2.60, self.window_1_center[1], self.window_1_center[2])
        if not self.uav_goto(*w1_east):
            self.get_logger().error("穿窗 1 失败，任务中止。")
            self.uav_land()
            return

        # 可选：串联任务三照射板子（这里先只做穿窗后返回，保持任务二/四独立）
        self.get_logger().info("穿窗成功，准备返航 ...")

        # 原路穿窗返回
        self.get_logger().info("垂直穿窗 1 返回 ...")
        if not self.uav_goto(*w1_west_low):
            self.get_logger().error("穿窗返回失败，尝试直接飞往指挥中心。")
        self.uav_goto(0.0, 0.0, float(self.get_parameter("takeoff_alt").value))
        self.uav_land()
        self.get_logger().info("任务二/四演示结束。")

    def run_full_inspection(self) -> None:
        """全流程串联：地面巡检 → 穿窗 1 → 识别+明显下降 → 双板激光 → 穿窗 2 → 降落。"""
        # --- 阶段 1：地面巡检 ---
        self.get_logger().info("========== 阶段 1：地面巡检 ==========")
        ground_ok = self.run_ground_patrol()
        ground_total = len(self.ground_task_waypoints)

        # 安全门禁：地面阶段大面积失败说明 Nav2/TF 异常，
        # 此时飞机真实位置可能已被恢复行为推离航点，不能继续起飞
        if ground_ok < 2:
            self.get_logger().error(
                f"地面巡检仅 {ground_ok}/{ground_total} 成功，判定导航异常，"
                f"全流程中止（不冒然起飞）。请重启 Gazebo+Nav2 后再试。"
            )
            self.stop_ground()
            return

        # --- 阶段 2：空中任务 ---
        self.get_logger().info("========== 阶段 2：空中任务 ==========")
        if not self.uav_takeoff():
            self.get_logger().error("起飞失败，全流程中止，紧急停桨。")
            self.stop_uav()
            return

        # 窗 1 坐标 shorthand
        w1_west_low = (1.20, self.window_1_center[1], self.window_1_center[2])
        w1_east = (2.60, self.window_1_center[1], self.window_1_center[2])
        w2_west = (1.20, self.window_2_center[1], self.window_2_center[2])
        w2_east = (2.60, self.window_2_center[1], self.window_2_center[2])
        observe_alt = 1.2

        # 观察窗上照片：坚持"先垂直后水平"，绝不在低空带速斜飞。
        # 先在起飞点正上方原地爬高到观察高度，再平移到窗前。
        cx0, cy0, _ = self._uav_pos()
        self.get_logger().info(f"原地爬高到 {observe_alt:.1f}m 观察高度 ...")
        if not self.uav_goto(cx0, cy0, observe_alt):
            self.get_logger().error("爬高失败，直接尝试在巡航高度穿窗。")
        else:
            self.uav_hold(cx0, cy0, observe_alt, 1.0)
            self.get_logger().info("平移到窗 1 西侧观察点 ...")
            if not self.uav_goto(1.20, self.window_1_center[1], observe_alt):
                self.get_logger().error("未能到达窗 1 观察点，直接尝试穿窗。")
            else:
                # 悬停识别窗上照片
                self.get_logger().info("悬停识别窗上照片 ...")
                self.uav_hold(1.20, self.window_1_center[1], observe_alt, 1.0)
                if not self.detect_window_photo(timeout_sec=6.0):
                    self.get_logger().warning("未识别到照片，仍继续穿窗。")
                # 在窗前原地降回 0.9m 准备明显下降和穿窗（xy 不变）
                self.uav_goto(*w1_west_low)

        # 明显下降动作
        self.obvious_dip()

        # 垂直穿窗 1
        self.get_logger().info("垂直穿窗 1 ...")
        if not self.uav_goto(*w1_east):
            self.get_logger().error("穿窗 1 失败，任务中止，直接降落。")
            self.uav_land()
            return

        # --- 阶段 3：双板激光照射 ---
        self.get_logger().info("========== 阶段 3：激光照射 ==========")
        succeeded = 0
        for name, (tx, ty) in self.aerial_task_points.items():
            self.get_logger().info(f"--- 照射 {name}：({tx:.3f}, {ty:.3f}) ---")
            if self.laser_scan_point(tx, ty):
                succeeded += 1
        total = len(self.aerial_task_points)
        self.get_logger().info(f"激光照射结果：{succeeded}/{total} 完成。")

        # --- 阶段 4：穿窗 2 返回 ---
        self.get_logger().info("垂直穿窗 2 返回 ...")
        if not (self.uav_goto(*w2_east) and self.uav_goto(*w2_west)):
            self.get_logger().error("穿窗 2 返回失败，尝试直接飞往指挥中心。")

        # 返航降落
        self.uav_goto(0.0, 0.0, float(self.get_parameter("takeoff_alt").value))
        self.uav_land()
        self.get_logger().info("全流程演示结束。")

    def laser_scan_point(self, x: float, y: float) -> bool:
        """在一块金色板子上空悬停并做十字扫查，让激光点扫过板心 1~3 秒。"""
        alt = float(self.get_parameter("laser_alt").value)
        radius = float(self.get_parameter("scan_radius").value)
        dwell = float(self.get_parameter("scan_dwell_sec").value)
        thresh = float(self.get_parameter("scan_thresh").value)

        # 先精确到板心上空（阈值收紧到 8cm）
        if not self.uav_goto(x, y, alt, thresh=thresh, timeout_sec=15.0):
            self.get_logger().error(f"未能到达 ({x:.2f}, {y:.2f}) 上空，跳过照射。")
            return False

        self.set_laser(True)
        self.get_logger().info("激光开启，开始十字扫查 ...")
        # 中心 → 前 → 后 → 左 → 右 → 回中心
        offsets = [(0.0, 0.0), (radius, 0.0), (-radius, 0.0),
                   (0.0, radius), (0.0, -radius), (0.0, 0.0)]
        for dx, dy in offsets:
            if not rclpy.ok():
                break
            self.uav_goto(x + dx, y + dy, alt, thresh=thresh, timeout_sec=6.0)
            self.uav_hold(x + dx, y + dy, alt, dwell)
        self.set_laser(False)
        self.get_logger().info("扫查完成，激光关闭。")
        return True

    def run_aerial_laser(self) -> None:
        """任务三：穿窗 1 → 两块金色板激光照射 → 穿窗 2 → 返回指挥中心降落。"""
        if not self.uav_takeoff():
            self.get_logger().error("起飞失败，任务三中止。")
            return

        # 穿窗铁律：先到窗前对准点，再垂直穿越。斜切窗口容易撞边框被卡死
        w1_west = (1.20, self.window_1_center[1], self.window_1_center[2])
        w1_east = (2.60, self.window_1_center[1], self.window_1_center[2])
        w2_east = (2.60, self.window_2_center[1], self.window_2_center[2])
        w2_west = (1.20, self.window_2_center[1], self.window_2_center[2])

        self.get_logger().info("对准窗 1，垂直穿越进入板子区 ...")
        if not (self.uav_goto(*w1_west) and self.uav_goto(*w1_east)):
            self.get_logger().error("穿窗 1 失败，任务三中止，直接降落。")
            self.uav_land()
            return

        succeeded = 0
        for name, (tx, ty) in self.aerial_task_points.items():
            self.get_logger().info(f"--- 照射 {name}：({tx:.3f}, {ty:.3f}) ---")
            if self.laser_scan_point(tx, ty):
                succeeded += 1

        self.get_logger().info("对准窗 2，垂直穿越返航 ...")
        self.uav_goto(*w2_east)
        self.uav_goto(*w2_west)
        self.uav_goto(0.0, 0.0, float(self.get_parameter("takeoff_alt").value))
        self.uav_land()
        total = len(self.aerial_task_points)
        self.get_logger().info(f"任务三结束：{succeeded}/{total} 个板子完成照射。")

    def run_uav_patrol(self) -> None:
        """空中巡检：起飞 → 逐个空中航点 → 降落。"""
        if not self.uav_takeoff():
            self.get_logger().error("起飞失败，空中巡检中止。")
            return

        succeeded = 0
        for name, (tx, ty, tz) in self.uav_task_waypoints.items():
            self.get_logger().info(
                f"前往 {name}：({tx:.3f}, {ty:.3f}, {tz:.3f})"
            )
            if self.uav_goto(tx, ty, tz):
                succeeded += 1
                self.get_logger().info(f"{name} 到达成功，悬停 2s 准备识别 ...")
                # 悬停 2 秒（给识别留出时间）
                self.publish_uav_cmd(2.0)
            else:
                self.get_logger().error(f"{name} 到达失败，跳过该点。")

        total = len(self.uav_task_waypoints)
        self.get_logger().info(
            f"空中巡检结束：{succeeded}/{total} 个航点到达。"
        )
        self.uav_land()

    def run_ground_patrol(self) -> int:
        """依次前往 ground_0 ~ ground_4，每个点失败后重试，最后汇总结果。

        返回成功到达的航点数量，供全流程判断 Nav2 是否工作正常。
        """
        retry_count = int(self.get_parameter("nav_retry_count").value)

        if not self.wait_for_ground_odom():
            self.get_logger().error(
                f"等待 /odom 超时，地面巡检中止。请确认 Gazebo 正在运行。"
            )
            return 0

        succeeded = 0
        # Python 3.7+ 字典按插入顺序遍历，即 ground_0 → ground_4
        for name, waypoint in self.ground_task_waypoints.items():
            x, y, yaw = waypoint
            self.get_logger().info(
                f"前往 {name}：x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
            )

            ok = False
            total_attempts = retry_count + 1
            for attempt in range(1, total_attempts + 1):
                ok = self.send_ground_nav_goal(waypoint)
                if ok:
                    break
                self.get_logger().warning(
                    f"{name} 第 {attempt}/{total_attempts} 次导航失败。"
                )

            if ok:
                succeeded += 1
                self.get_logger().info(f"{name} 到达成功。")
            else:
                self.get_logger().error(f"{name} 最终失败，跳过该点继续巡检。")

        self.stop_ground()
        total = len(self.ground_task_waypoints)
        self.get_logger().info(
            f"地面巡检结束：{succeeded}/{total} 个航点到达。"
        )
        return succeeded

    def set_laser(self, enabled: bool) -> None:
        try:
            self.laser_pub.publish(Bool(data=enabled))
        except Exception:
            pass

    def publish_uav_cmd(
        self,
        duration_sec: float,
        *,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        linear_z: float = 0.0,
        angular_z: float = 0.0,
    ) -> None:
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.linear.z = linear_z
        cmd.angular.z = angular_z

        deadline = time.monotonic() + max(0.0, duration_sec)
        cmd_rate_hz = max(1.0, float(self.get_parameter("cmd_rate_hz").value))
        rate_sec = 1.0 / cmd_rate_hz
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.uav_cmd_vel_pub.publish(cmd)
            time.sleep(rate_sec)

    def stop_ground(self) -> None:
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass

    def stop_uav(self) -> None:
        try:
            self.uav_cmd_vel_pub.publish(Twist())
        except Exception:
            pass
        self.set_laser(False)

    def wait_for_future(self, future) -> bool:
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def run(self) -> None:
        demo_mode = str(self.get_parameter("demo_mode").value)
        self.get_logger().info("最小用户模板已启动。默认执行 /uav/cmd_vel 开环速度控制演示。")
        if demo_mode in {"cmd_vel_smoke_test", "flight_smoke_test"}:
            takeoff_seconds = float(self.get_parameter("takeoff_seconds").value)
            takeoff_speed = float(self.get_parameter("takeoff_speed").value)
            move_seconds = float(self.get_parameter("move_seconds").value)
            move_speed = float(self.get_parameter("move_speed").value)
            hover_seconds = float(self.get_parameter("hover_seconds").value)
            land_seconds = float(self.get_parameter("land_seconds").value)
            land_speed = float(self.get_parameter("land_speed").value)

            self.get_logger().info(
                f"上升：linear.z={takeoff_speed:.2f} m/s，持续 {takeoff_seconds:.1f}s。"
            )
            self.publish_uav_cmd(takeoff_seconds, linear_z=takeoff_speed)
            self.get_logger().info(
                f"前进：linear.x={move_speed:.2f} m/s，持续 {move_seconds:.1f}s。"
            )
            self.publish_uav_cmd(move_seconds, linear_x=move_speed)
            self.get_logger().info(f"悬停：发布 0 速度，持续 {hover_seconds:.1f}s。")
            self.publish_uav_cmd(hover_seconds)
            self.get_logger().info(
                f"下降：linear.z={land_speed:.2f} m/s，持续 {land_seconds:.1f}s。"
            )
            self.publish_uav_cmd(land_seconds, linear_z=land_speed)
            self.stop_uav()
            self.get_logger().info("/uav/cmd_vel 开环速度控制演示完成。")
            return
        if demo_mode == "ground_patrol":
            self.run_ground_patrol()
            return
        if demo_mode == "uav_patrol":
            self.run_uav_patrol()
            return
        if demo_mode == "aerial_laser":
            self.run_aerial_laser()
            return
        if demo_mode == "vision_servo_demo":
            self.run_vision_servo_demo()
            return
        if demo_mode == "window_photo":
            self.run_window_photo()
            return
        if demo_mode == "full_inspection":
            self.run_full_inspection()
            return
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MinimalUserNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_ground()
            node.stop_uav()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

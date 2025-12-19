#!/usr/bin/env python3
import math
import json
import statistics
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float32, Bool
from std_srvs.srv import SetBool
from std_msgs.msg import String


def angle_wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Supervisor(Node):
    def __init__(self):
        super().__init__("supervisor")

        # --------------------------
        # PARAMETERS
        # --------------------------
        self.map_path = self.declare_parameter(
            "map_geometry_path",
            "/home/pi/PPOR_BALL/map_geometry.json"
        ).get_parameter_value().string_value

        # grid resolution (for localization)
        self.grid_res = 0.05   # 5cm
        self.max_range = 2.0   # ToF max distance

        # scan (rotate) 관련
        self.scan_required_turn = 2 * math.pi - 0.15  # 360도 근처
        self.scan_speed = 0.5  # rad/s

        # navigation 파라미터
        self.linear_speed = 0.12   # m/s
        self.angular_speed = 0.5   # rad/s
        self.scan_interval = 0.25  # m
        self.goal_tol = 0.10       # m
        self.yaw_tol = 0.05        # rad
        self.dt = 0.05             # timer 주기와 맞춤 (0.05s)

        # --------------------------
        # MAP GEOMETRY LOAD
        # --------------------------
        self.walls = self.load_map(self.map_path)

        # --------------------------
        # SENSOR STATE
        # --------------------------
        self.imu_yaw = None  # rad

        # rotate_mapper가 만들어주는 스캔 파일
        self.scan_file = "/home/pi/PPOR_BALL/scan_current.json"

        # --------------------------
        # POSE STATE (Localization 결과)
        # --------------------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # imu_yaw 그대로 사용 (offset 없음)

        # --------------------------
        # NAVIGATION STATE
        # --------------------------
        self.nav_active = False
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None  # 안 쓸 수도 있음

        # 현재 세그먼트 거리 (이번 직진 구간)
        self.segment_dist = 0.0
        self.segment_travel = 0.0
        self.remaining_dist_at_plan = 0.0

        # 마지막 localization 기준 점 (옵션)
        self.last_loc_x = 0.0
        self.last_loc_y = 0.0

        # 스캔 이유 (manual / nav)
        self.scan_reason = "manual"

        # --------------------------
        # ROS I/O
        # --------------------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pose_pub = self.create_publisher(Pose2D, "/localization/pose", 10)

        self.create_subscription(Float32, "/imu/yaw", self.imu_cb, 10)
        self.create_subscription(Bool, "/robot/scan", self.manual_scan_cb, 10)
        self.create_subscription(Pose2D, "/robot/nav_goal", self.nav_goal_cb, 10)
        self.create_subscription(String, "/robot/manual_cmd", self.manual_cmd_cb, 10)

        # rotate_mapper 서비스
        self.mapper_cli = self.create_client(SetBool, "/rotate_mapper/toggle")

        # --------------------------
        # FSM
        # --------------------------
        self.state = "IDLE"
        self.scan_start_yaw = None

        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info(
            "[Supervisor] READY: nav + rotate_mapper + ToF/IMU localization"
        )

    # =====================================================================
    # SENSOR CALLBACKS
    # =====================================================================
    def imu_cb(self, msg: Float32):
        self.imu_yaw = msg.data

    def manual_scan_cb(self, msg: Bool):
        if msg.data:
            self.start_scan(reason="manual")

    def nav_goal_cb(self, msg: Pose2D):
        # 새 목표 수신
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_yaw = msg.theta  # 필요 없으면 무시해도 됨
        self.nav_active = True

        self.get_logger().info(
            f"[NAV] new goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

        # 바로 navigation 계획 시작
        if self.state in ["IDLE"]:
            self.state = "NAV_PLAN"

    # =====================================================================
    # FSM ENTRY / COMMON
    # =====================================================================
    def stop(self):
        self.cmd_pub.publish(Twist())

    def start_scan(self, reason="manual"):
        if self.imu_yaw is None:
            self.get_logger().warn("[SCAN] IMU yaw not ready")
            return
        if self.state not in ["IDLE", "NAV_DRIVE", "NAV_PLAN"]:
            # 다른 동작 중이면 무시 (단순화)
            return

        self.scan_reason = reason
        self.get_logger().info(f"[SCAN] Starting 360° scan ({reason})")

        # enable rotate_mapper
        self.call_mapper(True)

        # 스캔 시작 yaw 기록
        self.scan_start_yaw = self.imu_yaw
        self.state = "SCAN_ROTATE"

    def call_mapper(self, enable: bool):
        if not self.mapper_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("[SCAN] rotate_mapper not available")
            return
        req = SetBool.Request()
        req.data = enable
        self.mapper_cli.call_async(req)

    # =====================================================================
    # MAIN LOOP
    # =====================================================================
    def loop(self):
        if self.state == "IDLE":
            return

        if self.state == "NAV_PLAN":
            self.do_nav_plan()
        elif self.state == "NAV_ROTATE_TO_GOAL":
            self.do_nav_rotate()
        elif self.state == "NAV_DRIVE":
            self.do_nav_drive()
        elif self.state == "NAV_FINISH":
            self.do_nav_finish()

        elif self.state == "SCAN_ROTATE":
            self.do_scan_rotate()
        elif self.state == "SCAN_FINISH":
            self.do_scan_finish()

    # =====================================================================
    # NAVIGATION STATES
    # =====================================================================
    def do_nav_plan(self):
        if not self.nav_active or self.goal_x is None or self.goal_y is None:
            self.state = "IDLE"
            return

        # 현재 pose 기준 거리 계산 (마지막 localization 기준)
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        self.remaining_dist_at_plan = dist

        self.get_logger().info(f"[NAV] remaining distance: {dist:.3f} m")

        if dist <= self.goal_tol:
            self.state = "NAV_FINISH"
            return

        # 이번 세그먼트에서 갈 거리 = min(scan_interval, dist)
        self.segment_dist = min(self.scan_interval, dist)
        self.segment_travel = 0.0

        # 회전 목표 yaw
        self.target_yaw = math.atan2(dy, dx)
        self.state = "NAV_ROTATE_TO_GOAL"

    def do_nav_rotate(self):
        if self.imu_yaw is None:
            return

        yaw_err = angle_wrap(self.target_yaw - self.imu_yaw)

        if abs(yaw_err) < self.yaw_tol:
            self.stop()
            self.state = "NAV_DRIVE"
            return

        t = Twist()
        # 간단 proportional + saturation
        k = 1.0
        cmd = k * yaw_err
        cmd = max(-self.angular_speed, min(self.angular_speed, cmd))
        t.angular.z = cmd
        self.cmd_pub.publish(t)

    def do_nav_drive(self):
        # 세그먼트 거리 dead-reckon (속도 * dt)
        self.segment_travel += self.linear_speed * self.dt

        # 목표 세그먼트 도달?
        if self.segment_travel >= self.segment_dist:
            self.stop()

            # 이번 세그먼트가 goal 근처까지였는지 확인
            if self.remaining_dist_at_plan <= (self.scan_interval + self.goal_tol):
                # 목표 도착으로 간주
                self.state = "NAV_FINISH"
            else:
                # 구간 중간 → 스캔 후 재로컬라이즈
                self.start_scan(reason="nav")
            return

        # 계속 직진
        t = Twist()
        t.linear.x = self.linear_speed
        self.cmd_pub.publish(t)

    def do_nav_finish(self):
        self.stop()
        self.get_logger().info(
            f"[NAV] reached goal (~ within {self.goal_tol:.2f} m)"
        )
        self.nav_active = False
        self.state = "IDLE"

    # =====================================================================
    # SCAN STATES (rotate_mapper 연동)
    # =====================================================================
    def do_scan_rotate(self):
        if self.imu_yaw is None:
            return

        t = Twist()
        t.angular.z = self.scan_speed
        self.cmd_pub.publish(t)

        turned = abs(angle_wrap(self.imu_yaw - self.scan_start_yaw))
        if turned >= self.scan_required_turn:
            self.get_logger().info("[SCAN] 360° rotation completed")
            self.stop()
            self.state = "SCAN_FINISH"

    def do_scan_finish(self):
        # rotate_mapper 비활성화 → scan_current.json 저장됨
        self.call_mapper(False)

        # scan 데이터 읽기
        scan = self.read_scan_file(self.scan_file)
        if not scan:
            self.get_logger().warn("[SCAN] No scan data.")
            # 스캔 이유 따라 다음 상태
            self.state = "NAV_PLAN" if (self.nav_active and self.scan_reason == "nav") else "IDLE"
            return

        # FLRB median 계산 (summary나 raw에서)
        flrb = self.compute_flrb_from_scan(scan)
        if not flrb:
            self.get_logger().warn("[SCAN] FLRB invalid.")
            self.state = "NAV_PLAN" if (self.nav_active and self.scan_reason == "nav") else "IDLE"
            return

        if self.imu_yaw is None:
            self.state = "NAV_PLAN" if (self.nav_active and self.scan_reason == "nav") else "IDLE"
            return

        # localization 수행
        x, y, err = self.localize(flrb, self.imu_yaw)
        self.x, self.y, self.yaw = x, y, self.imu_yaw
        self.last_loc_x, self.last_loc_y = self.x, self.y

        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.yaw
        self.pose_pub.publish(pose)

        self.get_logger().info(
            f"[LOC] pose=({self.x:.2f},{self.y:.2f},{self.yaw:.2f}), err={err:.3f}"
        )

        # 스캔 이유에 따라 다음 상태
        if self.nav_active and self.scan_reason == "nav":
            # 다음 세그먼트 계획
            self.state = "NAV_PLAN"
        else:
            self.state = "IDLE"

    # =====================================================================
    # MAP LOADING
    # =====================================================================
    def load_map(self, path: str):
        try:
            with open(path, "r") as f:
                data = json.load(f)
            pts = data["points"]
            walls = []
            for i in range(len(pts)):
                x1, y1 = pts[i]["x"], pts[i]["y"]
                x2, y2 = pts[(i + 1) % len(pts)]["x"], pts[(i + 1) % len(pts)]["y"]
                walls.append(((x1, y1), (x2, y2)))
            self.get_logger().info(f"[MAP] loaded {len(walls)} walls")
            return walls
        except Exception as e:
            self.get_logger().error(f"[MAP] load error: {e}")
            return []

    # =====================================================================
    # SCAN FILE / FLRB
    # =====================================================================
    def read_scan_file(self, path: str):
        try:
            with open(path, "r") as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().warn(f"[SCAN] read error: {e}")
            return None

    def compute_flrb_from_scan(self, scan: dict):
        
        """
        scan: {
          "front": <median or None>,
          "left":  ...,
          "right": ...,
          "back":  ...,
          "raw": {
            "front": [...],
            ...
          }
        }
        우선 summary(front/left/...)를 쓰고, 필요시 raw에서 median 다시 계산.
        """
        
        out = {}
        for key in ["front", "left", "right", "back"]:
            v = scan.get(key, None)
            if v is not None:
                out[key] = float(v)
                continue

            # summary가 없다면 raw에서 median
            raw_list = scan.get("raw", {}).get(key, [])
            raw_list = [x for x in raw_list if x > 0.02]
            if len(raw_list) == 0:
                out[key] = None
            else:
                out[key] = float(statistics.median(raw_list))

        # 네 개가 전부 None이면 실패
        if all(v is None for v in out.values()):
            return None
        return out

    # =====================================================================
    # LOCALIZATION (grid search)
    # =====================================================================
    def localize(self, flrb: dict, yaw: float):
        xs = []
        ys = []
        for (p1, p2) in self.walls:
            xs += [p1[0], p2[0]]
            ys += [p1[1], p2[1]]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        margin = 0.1
        min_x -= margin
        max_x += margin
        min_y -= margin
        max_y += margin

        best_err = float("inf")
        best_xy = (self.x, self.y)

        dirs = {
            "front": 0.0,
            "left": math.pi / 2,
            "right": -math.pi / 2,
            "back": math.pi,
        }

        gx = min_x
        while gx <= max_x:
            gy = min_y
            while gy <= max_y:
                err = 0.0
                used = 0

                for k, ofs in dirs.items():
                    meas = flrb.get(k, None)
                    if meas is None:
                        continue

                    theta = angle_wrap(yaw + ofs)
                    exp_d = self.raycast(gx, gy, theta)

                    dv = meas - exp_d
                    err += dv * dv
                    used += 1

                if used > 0 and err < best_err:
                    best_err = err
                    best_xy = (gx, gy)

                gy += self.grid_res
            gx += self.grid_res

        return best_xy[0], best_xy[1], best_err

    # =====================================================================
    # RAYCAST
    # =====================================================================
    def raycast(self, x: float, y: float, theta: float) -> float:
        dx = math.cos(theta)
        dy = math.sin(theta)
        min_t = None

        for (p1, p2) in self.walls:
            x1, y1 = p1
            x2, y2 = p2
            sx, sy = x2 - x1, y2 - y1

            denom = dx * sy - dy * sx
            if abs(denom) < 1e-8:
                continue

            t = ((x1 - x) * sy - (y1 - y) * sx) / denom
            if t < 0:
                continue

            u = ((x1 - x) * dy - (y1 - y) * dx) / denom
            if u < 0 or u > 1:
                continue

            if min_t is None or t < min_t:
                min_t = t

        if min_t is None:
            return self.max_range
        return float(min_t)


def main(args=None):
    rclpy.init(args=args)
    node = Supervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

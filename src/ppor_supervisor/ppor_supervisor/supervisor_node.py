# ppor_supervisor/supervisor_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import math, json, os

# Navigation states
STATE_IDLE            = 0
STATE_NAV_ROTATE1     = 1
STATE_NAV_DRIVE       = 2
STATE_NAV_ROTATE2     = 3
STATE_FINISHED        = 4

# Scan / Localization states
STATE_SCAN_START      = 10
STATE_SCAN_ROTATE     = 11
STATE_SCAN_FINISH     = 12


class Supervisor(Node):
    def __init__(self):
        super().__init__("supervisor")

        # Pub/Sub
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        # rotate mapper client
        self.mapper_cli = self.create_client(SetBool, "/rotate_mapper/toggle")

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Nav goal
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.yaw_goal = None

        # Scan goal
        self.scan_start_yaw = 0.0
        self.target_yaw = 0.0

        # FSM
        self.state = STATE_IDLE

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info("Supervisor with scan/localization ready!")

    # ------------------------
    # UTILITY
    # ------------------------
    def angle_diff(self, a, b):
        d = a - b
        return (d + math.pi) % (2*math.pi) - math.pi

    # ------------------------
    # ODOM CALLBACK
    # ------------------------
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    # ------------------------
    # NAVIGATION ENTRYPOINT
    # ------------------------
    def navigate_to(self, x_goal, y_goal, yaw_goal=None):
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.yaw_goal = yaw_goal

        dx = x_goal - self.x
        dy = y_goal - self.y
        self.target_yaw = math.atan2(dy, dx)

        self.state = STATE_NAV_ROTATE1
        self.get_logger().info(f"[NAV] Rotate→Drive start")

    # ------------------------
    # SCAN ENTRYPOINT
    # ------------------------
    def start_scan(self):
        self.get_logger().info("[SCAN] Start scan mapping")

        # mapping start
        req = SetBool.Request()
        req.data = True
        self.mapper_cli.call_async(req)

        self.scan_start_yaw = self.yaw
        self.target_yaw = self.yaw + 2 * math.pi   # 360° 회전

        self.state = STATE_SCAN_ROTATE

    # ------------------------
    # FSM UPDATE
    # ------------------------
    def update(self):
        if self.state == STATE_IDLE:
            return

        elif self.state == STATE_NAV_ROTATE1:
            self.do_nav_rotate1()

        elif self.state == STATE_NAV_DRIVE:
            self.do_nav_drive()

        elif self.state == STATE_NAV_ROTATE2:
            self.do_nav_rotate2()

        elif self.state == STATE_SCAN_ROTATE:
            self.do_scan_rotate()

        elif self.state == STATE_SCAN_FINISH:
            self.do_scan_finish()

        elif self.state == STATE_FINISHED:
            self.stop_robot()

    # ------------------------
    # LOW-LEVEL CONTROL
    # ------------------------
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # --- NAVIGATION ---
    def do_nav_rotate1(self):
        yaw_err = self.angle_diff(self.target_yaw, self.yaw)

        msg = Twist()
        msg.angular.z = 0.6 * yaw_err
        self.cmd_pub.publish(msg)

        if abs(yaw_err) < 0.05:
            self.stop_robot()
            self.state = STATE_NAV_DRIVE
            self.get_logger().info("[NAV] Initial rotation done")

    def do_nav_drive(self):
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.03:
            self.stop_robot()
            self.get_logger().info("[NAV] Drive complete")

            if self.yaw_goal is not None:
                self.target_yaw = self.yaw_goal
                self.state = STATE_NAV_ROTATE2
            else:
                self.state = STATE_FINISHED
            return

        msg = Twist()
        msg.linear.x = 0.17
        self.cmd_pub.publish(msg)

    def do_nav_rotate2(self):
        yaw_err = self.angle_diff(self.target_yaw, self.yaw)

        msg = Twist()
        msg.angular.z = 0.5 * yaw_err
        self.cmd_pub.publish(msg)

        if abs(yaw_err) < 0.05:
            self.stop_robot()
            self.state = STATE_FINISHED
            self.get_logger().info("[NAV] Final yaw alignment complete")

    # --- SCAN / LOCALIZATION ---
    def do_scan_rotate(self):
        yaw_err = self.angle_diff(self.target_yaw, self.yaw)

        msg = Twist()
        msg.angular.z = 0.5
        self.cmd_pub.publish(msg)

        # 360-degree rotation finished?
        if abs(yaw_err) < 0.05:
            self.stop_robot()

            # mapping stop
            req = SetBool.Request()
            req.data = False
            self.mapper_cli.call_async(req)

            self.get_logger().info("[SCAN] Rotation finished, starting ICP")
            self.state = STATE_SCAN_FINISH

    def do_scan_finish(self):
        # HERE: ICP 수행 + odom 보정
        self.apply_icp_correction()
        self.state = STATE_IDLE
        self.get_logger().info("[SCAN] Localization complete")

    # ------------------------
    # ICP PLACEHOLDER
    # ------------------------
    def apply_icp_correction(self):
        # map_reference.json vs map_current.json
        # open3d로 Δx,Δy,Δθ 계산 후 self.x,self.y,self.yaw 보정
        self.get_logger().info("[ICP] (placeholder) pose correction applied")
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Supervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

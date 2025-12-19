import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class WheelOdomNode(Node):
    def __init__(self):
        super().__init__("wheel_odom")

        self.wheel_radius = self.declare_parameter("wheel_radius", 0.03).get_parameter_value().double_value
        self.wheel_base = self.declare_parameter("wheel_base", 0.15).get_parameter_value().double_value
        self.ticks_per_rev = self.declare_parameter("ticks_per_rev", 1024).get_parameter_value().integer_value

        self.left_ticks_prev = None
        self.right_ticks_prev = None
        self.left_ticks = None
        self.right_ticks = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.sub_left = self.create_subscription(Int32, "/wheel/left_ticks", self.left_cb, 10)
        self.sub_right = self.create_subscription(Int32, "/wheel/right_ticks", self.right_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update_odom)

    def left_cb(self, msg: Int32):
        self.left_ticks = msg.data

    def right_cb(self, msg: Int32):
        self.right_ticks = msg.data

    def update_odom(self):
        if self.left_ticks is None or self.right_ticks is None:
            return

        if self.left_ticks_prev is None:
            self.left_ticks_prev = self.left_ticks
            self.right_ticks_prev = self.right_ticks
            self.last_time = self.get_clock().now()
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        dL_ticks = self.left_ticks - self.left_ticks_prev
        dR_ticks = self.right_ticks - self.right_ticks_prev

        self.left_ticks_prev = self.left_ticks
        self.right_ticks_prev = self.right_ticks
        self.last_time = current_time

        meters_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev
        dL = dL_ticks * meters_per_tick
        dR = dR_ticks * meters_per_tick

        dS = (dL + dR) / 2.0
        dTheta = (dR - dL) / self.wheel_base

        dx = dS * math.cos(self.theta + dTheta / 2.0)
        dy = dS * math.sin(self.theta + dTheta / 2.0)

        self.x += dx
        self.y += dy
        self.theta += dTheta

        q = yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = dS / dt
        odom.twist.twist.angular.z = dTheta / dt

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

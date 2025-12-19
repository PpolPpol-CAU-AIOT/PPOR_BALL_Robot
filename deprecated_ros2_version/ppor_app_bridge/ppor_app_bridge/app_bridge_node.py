#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import paho.mqtt.client as mqtt


class AppBridgeNode(Node):
    def __init__(self):
        super().__init__("app_bridge")

        # --------------------------
        # ROS -> 앱으로 보낼 위치 데이터
        # --------------------------
        self.pose_sub = self.create_subscription(
            Pose2D,
            "/localization/pose",
            self.pose_cb,
            10
        )

        # --------------------------
        # 앱 -> Supervisor (goal 명령)
        # --------------------------
        self.nav_goal_pub = self.create_publisher(
            Pose2D,
            "/robot/nav_goal",
            10
        )

        self.manual_pub = self.create_publisher(
            String,
            "/robot/manual_cmd",
            10
        )


        # --------------------------
        # MQTT 초기화
        # --------------------------
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.mqtt_on_connect
        self.mqtt_client.on_message = self.mqtt_on_message

        broker = "pporball-hub.local"
        self.mqtt_client.connect(broker, 8080, 60)

        # 앱 -> 로봇 명령 토픽
        self.mqtt_client.subscribe("pporball/cmd")

        self.mqtt_client.loop_start()

        self.get_logger().info("[AppBridge] READY (using /localization/pose, nav_goal enabled)")

    # =====================================================================
    # ROS -> MQTT (로봇 절대좌표 전송)
    # =====================================================================
    def pose_cb(self, msg: Pose2D):
        data = {
            "x": msg.x,
            "y": msg.y,
            "yaw": msg.theta,
        }
        payload = json.dumps(data)
        self.mqtt_client.publish("pporball/pose", payload)

    # =====================================================================
    # MQTT -> ROS (앱 명령 수신)
    # =====================================================================
    def mqtt_on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"[MQTT] Connected with code {rc}")

    def mqtt_on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
        except:
            self.get_logger().warn("[MQTT] Invalid JSON")
            return

        cmd = data.get("cmd", "")

        # ------------------------------
        # GOAL 명령 처리
        # ------------------------------
        if cmd == "GO_TO":
            gx = float(data.get("x", 0.0))
            gy = float(data.get("y", 0.0))
            gyaw = float(data.get("yaw", 0.0))

            pose = Pose2D()
            pose.x = gx
            pose.y = gy
            pose.theta = gyaw

            self.nav_goal_pub.publish(pose)
            self.get_logger().info(f"[NAV] Goal received: ({gx}, {gy}, {gyaw})")
            return

        # ------------------------------
        # 수동 스캔 트리거
        # ------------------------------
        if cmd == "SCAN":
            # Bool 메시지를 쓰지 않고 Supervisor가 직접 /app/scan=True 를 받는 구조라면
            # mqtt -> /app/scan publisher를 따로 만들어야 하는데
            # 일단 여기서는 Supervisor 수정을 반영하여 Bool publish가 필요하면 추가해줄 수 있음.
            pass
        
        if cmd =="front":
            self.get_logger().info(f"[MQTT] front command received")
            self.manual_pub.publish(String(data="front"))
            return
        if cmd =="back":
            self.get_logger().info(f"[MQTT] back command received")
            self.manual_pub.publish(String(data="back"))
            return
        if cmd =="left":
            self.get_logger().info(f"[MQTT] left command received")
            self.manual_pub.publish(String(data="left"))
            return
        if cmd =="right":
            self.get_logger().info(f"[MQTT] right command received")
            self.manual_pub.publish(String(data="right"))
            return
        if cmd =="dance":
            self.get_logger().info(f"[MQTT] dance command received")
            self.manual_pub.publish(String(data="dance"))
            return
        if cmd =="stop":
            self.get_logger().info(f"[MQTT] stop command received")
            self.manual_pub.publish(String(data="stop"))
            return
    



def main(args=None):
    rclpy.init(args=args)
    node = AppBridgeNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

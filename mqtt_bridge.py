#!/usr/bin/env python3
import json
import time
import threading
import paho.mqtt.client as mqtt


class MQTTBridge:
    def __init__(self, supervisor,
                 broker="172.30.1.65",
                 port=1883,
                 cmd_topic="robot/cmd",
                 pose_topic="robot/status",
                 pose_interval=0.2):

        self.sup = supervisor
        self.broker = broker
        self.port = port
        self.cmd_topic = cmd_topic
        self.pose_topic = pose_topic
        self.pose_interval = pose_interval

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # pose publish thread
        self.running = True
        self.pose_thread = threading.Thread(
                target=self._pose_loop, daemon=True
            )
        self.pose_thread.start()

    # ------------------------------------------------------
    # MQTT 연결 시작
    # ------------------------------------------------------
    def start(self):
        print(f"[MQTT] Connecting to {self.broker}:{self.port} ...")
        self.client.connect_async(self.broker, self.port, 60)
        self.client.loop_start()

    # ------------------------------------------------------
    # 연결 완료
    # ------------------------------------------------------
    def on_connect(self, client, userdata, flags, rc):
        print(f"[MQTT] Connected with rc={rc}")
        client.subscribe(self.cmd_topic)
        print(f"[MQTT] Subscribed -> {self.cmd_topic}")

    # ------------------------------------------------------
    # 앱에서 명령 수신
    # ------------------------------------------------------
    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
        except Exception as e:
            print(f"[MQTT] Invalid JSON ({e}): {msg.payload!r}")
            return

        direction = data.get("direction")
        cmd = data.get("cmd")

        print(f"[MQTT] cmd={cmd} || direction={direction}")
        
        # --------------------------
        # GO TO goal
        # --------------------------
        if cmd == "GO_TO":
            try:
                x = float(data.get("x", 0))
                y = float(data.get("y", 0))
                yaw = float(data.get("yaw", 0))
            except (TypeError, ValueError):
                print("[MQTT] Invalid GO_TO payload:", data)
                return

            self.sup.set_goal(x, y, yaw)
            return

        # --------------------------
        # Manual SCAN
        # --------------------------
        if cmd == "SCAN":
            self.sup.request_manual_scan()
            return

        # --------------------------
        # manual control
        # --------------------------
        motorR = 100
        motorL = 100
        hight = 100
        if direction == "up":
            self.sup.arduino.send(motorR, motorL, 0)
            return
        if direction == "down":
            self.sup.arduino.send(-motorR, -motorL, 0)
            return
        if direction == "left":
            self.sup.arduino.send(-motorR, motorL, 0)
            return
        if direction == "right":
            self.sup.arduino.send(motorR, -motorL, 0)
            return
        if direction == "stop":
            self.sup.arduino.send(0,0,0)
            self.sup.arduino.stop()
            time.sleep(0.5)
            return
        if direction == "high":
            self.sup.arduino.send(0,0,hight)
            return
        if direction == "low":
            self.sup.arduino.send(0,0,-hight)
            return
        
        if direction == "dance":
            # Supervisor에 dance 함수가 있다면 호출하면 됨
            try:
                self.sup.dance()
            except:
                print("[MQTT] dance() not implemented in supervisor")
            return

        print(f"[MQTT] Unknown cmd={cmd}")
        return

    # ------------------------------------------------------
    # pose 송신 (앱으로)
    # ------------------------------------------------------
    def _pose_loop(self):
        while self.running:
            pose = {
               "location": { 
			        "x": self.sup.x,
               		"y": self.sup.y,
                	"yaw": self.sup.yaw
            	},
	        }
            try:
                self.client.publish(self.pose_topic, json.dumps(pose))
            except Exception as e:
                print("[MQTT] pose publish error:", e)
            time.sleep(self.pose_interval)


    # ------------------------------------------------------
    # 종료
    # ------------------------------------------------------
    def stop(self):
        self.running = False
        self.client.loop_stop()
        self.client.disconnect()

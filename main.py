from supervisor import Supervisor
from imu_reader import ImuReader
from tof_reader import ToFReader
from rotate_mapper_node import RotateMapper
from arduino_bridge_node import ArduinoBridge
from mqtt_bridge import MQTTBridge
import time

arduino = ArduinoBridge()
imu = ImuReader()
tof = ToFReader()
mapper = RotateMapper(tof, imu)

sup = Supervisor(arduino, imu, tof, mapper)
mqtt = MQTTBridge(sup)
mqtt.start()

# 메인 루프
while True:
    sup.update()
    time.sleep(0.05)

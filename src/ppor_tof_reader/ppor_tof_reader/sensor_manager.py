import time
import board
import busio
import digitalio
import adafruit_vl53l0x

class ToFSensorManager:
    def __init__(self, gpio_map, address_map):
        """
        SCL, SDA는 기본 I2C 핀 (GPIO 3, GPIO 2)
        gpio_map = { "front":17, "left":27, "right":22, "back":23 }
        address_map = { "front":0x30, "left":0x31, "right":0x32, "back":0x33 }
        """
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensors = {}
        self.gpio_map = gpio_map
        self.address_map = address_map

        # Create XSHUT pins -> XSHUT 핀들을 Digital 출력으로 설정
        # board.D17, board.D27 이런 식으로 핀 접근
        self.xshut_pins = {}
        for name, pin in gpio_map.items():
            p = digitalio.DigitalInOut(getattr(board, f"D{pin}", None))
            p.direction = digitalio.Direction.OUTPUT
            self.xshut_pins[name] = p

        self.init_sensors()

    def init_sensors(self):
        # 모든 센서 종료
        for pin in self.xshut_pins.values():
            pin.value = False
        time.sleep(0.1)

        # 모든 센서 켜고 주소 할당하기
        for name, pin in self.xshut_pins.items():
            pin.value = True
            time.sleep(0.05)

            sensor = adafruit_vl53l0x.VL53L0X(self.i2c)
            sensor.set_address(self.address_map[name])
            self.sensors[name] = sensor
            time.sleep(0.05)

    def read_all(self):
        data = {}
        for name, sensor in self.sensors.items():
            try:
                data[name] = sensor.range / 1000.0  # meters
            except:
                data[name] = None
        return data

import time
import smbus2
import lgpio
import adafruit_vl53l0x
import board
import busio

VL53L0X_DEFAULT_ADDR = 0x29
MAX_DISTANCE = 2.0

class ToFSensor:
    def __init__(self, bus, addr):
        self.bus = bus
        self.addr = addr

    def read_range(self):
        try:
            data = self.bus.read_i2c_block_data(self.addr, 0x14, 12)
            return (data[10] << 8) | data[11]
        except:
            return None

class EwmaFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.value = None

    def update(self, new):
        if new is None:
            return self.value  
        
        if self.value is None:
            self.value = new
        else:
            self.value = self.alpha * new + (1 - self.alpha) * self.value
        
        return self.value

class ToFSensorManager:
    def __init__(self, gpio_map, address_map):
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # XSHUT 제어용 GPIO
        self.chip = lgpio.gpiochip_open(0)
        self.GPIO_FRONT = 5
        self.GPIO_LEFT  = 16
        #self.GPIO_RIGHT = 6
        self.GPIO_BACK  = 26

        self.pins = {
            "front": self.GPIO_FRONT,
            "left":  self.GPIO_LEFT,
        #    "right": self.GPIO_RIGHT,
            "back":  self.GPIO_BACK,
        }

        # 1) 전부 LOW로 내려서 센서 OFF (우리 기준 "부팅 리셋")
        self._reset_all_xshut()

        # 2) 센서 올리면서 주소 재배정 + 객체 생성
        self.sensors = {}
        self._init_sensor("front", new_addr=0x30)
        self._init_sensor("left",  new_addr=0x31)
        #self._init_sensor("right", new_addr=0x32)
        self._init_sensor("back",  new_addr=0x33)

        print("[INFO] 모든 ToF 센서 초기화 완료")
        def fmt(v): return f"{v:.3f}" if v is not None else "None"
        print("[INFO] 모든 ToF 센서 초기화 완료")


    def _reset_all_xshut(self):
        # XSHUT 핀 모두 OUTPUT + LOW
        for name, pin in self.pins.items():
            lgpio.gpio_claim_output(self.chip, pin, 0)
            lgpio.gpio_write(self.chip, pin, 0)

        time.sleep(0.1)  # 완전히 꺼질 시간
        print("[ToF] XSHUT all LOW (센서 리셋 완료)")


    def _init_sensor(self, name, new_addr):
        # 모두 끄기
        pin = self.pins[name]
        print(f"[ToF] INIT {name} (GPIO {pin}) -> addr {hex(new_addr)}")


        # 센서 하나씩 켜고 주소 변경
        lgpio.gpio_write(self.chip, pin, 1)
        time.sleep(0.5)  # 반드시 50ms 필요
        # 기본 주소(0x29)로 드라이버 생성
        try:
            sensor = adafruit_vl53l0x.VL53L0X(self.i2c)
            print(f"[ToF][{name}] 기본 주소(0x29) 접근 성공")

        except Exception as e:
            print(f"[ToF][{name}] 기본 주소(0x29) 접근 실패 -> {e}")
            # 실패했어도 그냥 넘어가고, 나중에 없는 센서로 취급
            self.sensors[name] = None

        try:
            sensor.set_address(new_addr)
            print(f"[OK] {name}: 주소 변경 성공 -> {hex(new_addr)}")
        except Exception as e:
            print(f"[FAIL] {name}: 주소 변경 실패 -> {e}")
            lgpio.gpio_write(self.chip, pin, 0)
            return

        self.sensors[name] = sensor


    def read_all(self):
        data = {}
        for name, sensor in self.sensors.items():
            if sensor is None:
                data[name] = None
                continue
            try:
                data[name] = sensor.range / 1000.0 
            except Exception as e:
                print(f"[ToF][{name}] read error: {e}")
                data[name] = None
                
        self.update_display(data)
        return data
    

    def update_value(self, row, text):
        y = row + 4   # 값이 들어가는 줄: 박스 기준 4, 5, 6, 7
        x = 12        # 값 위치 조절
        #print(f"\033[{y};{x}H{text:8}", end="")

    def update_display(self, data):
        front = data.get("front")
        left  = data.get("left")
        #right = data.get("right")
        back  = data.get("back")

        self.update_value(0, f"{front:.2f}" if front is not None else "---")
        self.update_value(1, f"{left:.2f}"  if left is not None else "---")
        #self.update_value(2, f"{right:.2f}" if right is not None else "---")
        self.update_value(3, f"{back:.2f}"  if back is not None else "---")
        print("", end="", flush=True)
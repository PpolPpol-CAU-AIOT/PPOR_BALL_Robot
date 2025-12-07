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
        self.gpio_map = gpio_map        # {"front":17, "left":27, "right":22}
        self.address_map = address_map  # {"front":0x30, "left":0x31, "right":0x32}

        self.chip = lgpio.gpiochip_open(0)
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # XSHUT 핀 설정
        for _, pin in self.gpio_map.items():
            lgpio.gpio_claim_output(self.chip, pin, 0)

        self.sensors = {}
        self.filters = {
            "front": EwmaFilter(alpha=0.3),
            "left": EwmaFilter(alpha=0.3),
            "right": EwmaFilter(alpha=0.3),
        }
        self.init_all_sensors()

    def _write_reg(self, addr, reg, val):
        self.bus.write_byte_data(addr, reg, val)


    def init_all_sensors(self):
        # 모두 끄기
        for pin in self.gpio_map.values():
            lgpio.gpio_write(self.chip, pin, 0)
        time.sleep(0.1)

        print("[INFO] VL53L0X 초기화 시작…")

        # 센서 하나씩 켜고 주소 변경
        for name, pin in self.gpio_map.items():
            new_addr = self.address_map[name]

            print(f"\n[INIT] {name} 부팅, GPIO {pin} → {hex(new_addr)}")

            # XSHUT HIGH -> 부팅
            lgpio.gpio_write(self.chip, pin, 1)
            time.sleep(0.05)  # 반드시 50ms 필요

            # 기본 주소(0x29)로 드라이버 생성
            try:
                sensor = adafruit_vl53l0x.VL53L0X(self.i2c)
            except Exception as e:
                print(f"[FAIL] {name}: 기본 주소(0x29) 접근 실패 → {e}")
                continue

            try:
                sensor.set_address(new_addr)
            except Exception as e:
                print(f"[FAIL] {name}: 주소 변경 실패 → {e}")
                continue

            self.sensors[name] = sensor
            print(f"[OK] {name} 초기화 완료 → {hex(new_addr)}")

        print("\n[INFO] 모든 센서 초기화 완료.\n")
        self.draw_static_box()


    def read_all(self):
        data = {}
        for name, sensor in self.sensors.items():
            try:
                mm = sensor.range
                # 에러 제거
                if mm > 8100 or mm == 0:
                    data[name] = None
                    continue
                dist = mm / 1000.0
                # 상한
                if dist > MAX_DISTANCE:
                    dist = MAX_DISTANCE

                data[name] = self.filters[name].update(dist)
                
            except Exception:
                data[name] = None
        self.update_display(data)
        return data
    
    def draw_static_box(self):
        print("\033[2J\033[H", end="")
        print("┌──────────────────────────────┐")
        print("│       TOF DISTANCES          │")
        print("├──────────────────────────────┤")
        print("│ front :       │")
        print("│ left  :       │")
        print("│ right :       │")
        print("└──────────────────────────────┘")
    
    def update_value(self, row, text):
        y = row + 4   # 값이 들어가는 줄: 박스 기준 4, 5, 6
        x = 12        # 값 위치 조절
        print(f"\033[{y};{x}H{text:8}", end="")

    def update_display(self, data):
        self.update_value(0, f"{data['front']:.2f}" if data['front'] else "---")
        self.update_value(1, f"{data['left']:.2f}"  if data['left'] else "---")
        self.update_value(2, f"{data['right']:.2f}" if data['right'] else "---")
        print("", end="", flush=True)

        """
        gpio_map: { "front":17, "left":27, "right":22 }
        address_map: { "front":0x30, "left":0x31, "right":0x32 }
        """


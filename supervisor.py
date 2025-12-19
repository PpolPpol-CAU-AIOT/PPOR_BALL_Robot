import time
import math
import json
import os
import statistics

def angle_wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
class Supervisor:
    def __init__(self, arduino_brindge, imu_reader, tof_reader, mapper,
                    map_path="/home/pi/Desktop/PPOR_BALL/map_geometry.json", dt = 0.05):
        self.arduino = arduino_brindge
        self.imu = imu_reader
        self.tof = tof_reader
        self.mapper = mapper


        # 업데이트 주기
        self.dt = dt

        self.walls = self.load_map(map_path)

        self.imu_yaw = 0.0
        self.x = 0.45
        self.y = 0.45
        self.yaw = 0.0

        # 네비게이션 파라미터
        self.linear_speed = 0.12
        self.angular_speed = 0.5
        self.goal_tol = 0.10
        self.scan_interval = 0.25
        self.yaw_tol = 0.05

        # 스캔 파라미터
        self.scan_required_turn = 2 * math.pi - 0.15
        self.scan_speed = 1
        self.scan_start_yaw = None

        # goal
        self.nav_active = False
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None

        self.segment_dist = 0.0
        self.segment_travel = 0.0
        self.remaining_dist_at_plan = 0.0

        self.scan_reason = None
        self.scan_file = "/home/pi/Desktop/PPOR_BALL/scan_current.json"

        # FSM 시작 상태
        self.state = "IDLE"

        self.scan_accum_turn = 0.0   # 지금까지 총 회전한 각도
        self.scan_prev_yaw = None    # 이전 프레임의 yaw
    
    def set_goal(self, x, y, yaw=0.0):
        """앱에서 Navigation Goal을 넣는 경우"""
        self.goal_x = x
        self.goal_y = y
        self.goal_yaw = yaw
        self.nav_active = True

        print(f"[NAV] new goal: ({x:.2f}, {y:.2f})")
        if self.state == "IDLE":
            self.state = "NAV_PLAN"

    def request_manual_scan(self):
        """앱에서 스캔 요청시 호출"""
        self.start_scan(reason="manual")

    
    # 반복 구간 -> 요청 
    def update(self):
        """Supervisor 한 프레임 실행"""
        # 센서 업데이트
        self.imu_yaw = self.imu.get_yaw()

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

    # 경로 찾기
    def do_nav_plan(self):
        if not self.nav_active:
            self.state = "IDLE"
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        self.remaining_dist_at_plan = dist

        print(f"[NAV] remaining dist = {dist:.3f}")

        if dist <= self.goal_tol:
            self.state = "NAV_FINISH"
            return

        self.segment_dist = min(self.scan_interval, dist)
        self.segment_travel = 0.0

        self.target_yaw = math.atan2(dy, dx)
        self.state = "NAV_ROTATE_TO_GOAL"
    
    def do_nav_rotate(self):
        yaw_err = angle_wrap(self.target_yaw - self.imu_yaw)

        if abs(yaw_err) < self.yaw_tol:
            self.arduino.stop()
            self.state = "NAV_DRIVE"
            return

        # 간단 proportional 회전
        sign = 1 if yaw_err > 0 else -1
        pwm = int(self.angular_speed * 70 * sign)
        self.arduino.send(pwm, -pwm, 0)

    def do_nav_drive(self):
        self.segment_travel += self.linear_speed * self.dt

        if self.segment_travel >= self.segment_dist:
            self.arduino.stop()

            if self.remaining_dist_at_plan <= (self.scan_interval + self.goal_tol):
                self.state = "NAV_FINISH"
            else:
                self.start_scan(reason="nav")
            return

        # 직진 = 양 바퀴 동일 PWM
        pwm = int(self.linear_speed * 100)
        self.arduino.send(pwm, pwm, 0)

    def do_nav_finish(self):
        self.arduino.stop()
        print("[NAV] reached goal")
        self.nav_active = False
        self.state = "IDLE"


    # 주변 환경 스캔
    def start_scan(self, reason="manual"):
        if self.imu_yaw is None:
            print("[SCAN] IMU not ready")
            return

        self.scan_reason = reason
        print(f"[SCAN] Start 360 scan ({reason})")

        self.mapper.start()

        self.scan_start_yaw = self.imu_yaw
        self.scan_prev_yaw = self.imu_yaw
        self.scan_accum_turn = 0.0
        self.state = "SCAN_ROTATE"
    
    def do_scan_rotate(self):
        # 계속 회전
        pwm = int(self.scan_speed * 100)
        self.arduino.send(pwm, -pwm, 0)

        dists = self.tof.get_distances()
        # 이전 yaw 값과 비교하여 이동한 각도 누적
        delta = angle_wrap(self.imu_yaw - self.scan_prev_yaw)
        self.scan_accum_turn += abs(delta)
        self.scan_prev_yaw = self.imu_yaw

        turned = self.scan_accum_turn
        def fmt(v): return f"{v:.2f}" if v is not None else "---"
        print(f"scanning... turned : {turned:.2f} | req : {self.scan_required_turn:.2f} | ToF [F:{fmt(dists['front'])} L:{fmt(dists['left'])}  B:{fmt(dists['back'])}] \n", flush=True)
        
        if turned >= self.scan_required_turn:
            print("\n[SCAN] Rotation complete")
            self.arduino.stop()
            self.state = "SCAN_FINISH"

    def do_scan_finish(self):
        self.mapper.stop()

        scan = self.read_scan_file(self.scan_file)
        if not scan:
            print("[SCAN] no scan data!")
            self.state = "NAV_PLAN" if self.nav_active else "IDLE"
            return

        flrb = self.compute_flrb_from_scan(scan)
        if not flrb:
            print("[SCAN] invalid FLRB!")
            self.state = "NAV_PLAN" if self.nav_active else "IDLE"
            return

        # 로컬라이제이션
        x, y, err = self.localize(flrb, self.imu_yaw)
        self.x, self.y = x, y
        self.yaw = self.imu_yaw

        print(f"[LOC] ({x:.2f}, {y:.2f}, yaw={self.yaw:.2f}), err={err:.3f}")

        if self.nav_active and self.scan_reason == "nav":
            self.state = "NAV_PLAN"
        else:
            self.state = "IDLE"

    
    def load_map(self, path):
        try:
            with open(path, "r") as f:
                data = json.load(f)
            pts = data["points"]
            walls = []
            for i in range(len(pts)):
                x1, y1 = pts[i]["x"], pts[i]["y"]
                x2, y2 = pts[(i+1) % len(pts)]["x"], pts[(i+1) % len(pts)]["y"]
                walls.append(((x1, y1), (x2, y2)))
            print(f"[MAP] loaded {len(walls)} walls")
            return walls
        except Exception as e:
            print("[MAP] load error:", e)
            return []
        
    # 파일 읽기
    def read_scan_file(self, path):
        try:
            with open(path, "r") as f:
                return json.load(f)
        except Exception:
            return None

    def compute_flrb_from_scan(self, scan):
        out = {}
        #for key in ["front", "left", "right", "back"]:
        for key in ["front", "left", "back"]:
            v = scan.get(key)
            if v is not None:
                out[key] = float(v)
            else:
                raw = scan["raw"].get(key, [])
                raw = [x for x in raw if x > 0.02]
                out[key] = float(statistics.median(raw)) if raw else None

        if all(v is None for v in out.values()):
            return None

        return out
    
    
    # 로컬라이제이션
    
    def localize(self, flrb, yaw):
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
        best_pos = (self.x, self.y)

        dirs = {
            "front": 0.0,
            "left": math.pi/2,
            #"right": -math.pi/2,
            "back": math.pi
        }

        gx = min_x
        while gx <= max_x:
            gy = min_y
            while gy <= max_y:
                err = 0
                used = 0

                for k, ofs in dirs.items():
                    meas = flrb[k]
                    if meas is None:
                        continue

                    theta = angle_wrap(yaw + ofs)
                    exp_d = self.raycast(gx, gy, theta)
                    dv = meas - exp_d
                    err += dv * dv
                    used += 1

                if used > 0 and err < best_err:
                    best_err = err
                    best_pos = (gx, gy)

                gy += 0.05
            gx += 0.05

        return best_pos[0], best_pos[1], best_err

    # ============================================================
    # Raycast
    # ============================================================

    def raycast(self, x, y, theta):
        dx = math.cos(theta)
        dy = math.sin(theta)
        min_t = None

        for (p1, p2) in self.walls:
            x1, y1 = p1
            x2, y2 = p2

            sx = x2 - x1
            sy = y2 - y1

            denom = dx*sy - dy*sx
            if abs(denom) < 1e-8:
                continue

            t = ((x1 - x)*sy - (y1 - y)*sx) / denom
            if t < 0:
                continue

            u = ((x1 - x)*dy - (y1 - y)*dx) / denom
            if u < 0 or u > 1:
                continue

            if min_t is None or t < min_t:
                min_t = t

        return min_t if min_t is not None else 2.0

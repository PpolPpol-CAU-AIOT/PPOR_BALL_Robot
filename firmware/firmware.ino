/*********************************************************************
  2-Wheel Robot Motor + Servo Driver (NO IMU / NO PID)

  - Motor Driver: TB6612FNG
  - Servos: 2개 (같이 움직임)
  - 외부 명령: 시리얼로 "r{r},l{l},h{h}\n" 수신
      r, l, h ∈ [-100, 100]
      r/l : 오른/왼 모터 속도(-100=최대 후진, 100=최대 전진)
      h   : 서보 각도 제어 (머리 좌우 등)
*********************************************************************/

#include <Servo.h>

// ============================= PIN MAP =============================
#define PIN_AIN1  2
#define PIN_AIN2  4
#define PIN_BIN1  10
#define PIN_BIN2  12
#define PIN_PWMA  3    // PWM
#define PIN_PWMB  11   // PWM
#define PIN_STBY  9    // TB6612FNG STBY

#define PIN_SERVO1  5
#define PIN_SERVO2  6
// ==================================================================

// 서보 객체
Servo servo1;
Servo servo2;

// ==================== 시리얼 명령 상태 ====================
int cmdR = 0;   // -100 ~ 100 (오른쪽 모터)
int cmdL = 0;   // -100 ~ 100 (왼쪽 모터)
int cmdH = 0;   // -100 ~ 100 (서보/머리)

unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT_MS = 7000;  // 1초 동안 명령 없으면 정지

// ==================== 함수 선언 ====================
void driveMotorsRaw(int rightPower, int leftPower);
int  percentToPwm(int v);
void setServosPercent(int h);
void applyCommand(int rPercent, int lPercent, int hPercent);
void parseLine(String line);
void handleSerial();
void stopAll();

// ======================= SETUP =========================
void setup() {
  Serial.begin(115200);

  // === 모터 핀 설정 ===
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);  // 모터 드라이버 활성화

  // === 서보 설정 ===
  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  setServosPercent(0);  // 중앙

  stopAll();

  Serial.println("Simple motor+servo driver ready.");
  lastCmdTime = millis();
}

// ======================= LOOP ==========================
void loop() {
  // 1) 시리얼 명령 처리 (r/l/h 갱신)
  handleSerial();

  // 2) 타임아웃 처리: 명령이 오래 안 오면 정지
  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    stopAll();
  }
}

// ======================= MOTOR CONTROL =======================

// 오른쪽/왼쪽 모터를 각각 PWM으로 제어 (-255 ~ 255)
void driveMotorsRaw(int rightPower, int leftPower) {
  rightPower = constrain(rightPower, -255, 255);
  leftPower  = constrain(leftPower,  -255, 255);

  // 오른쪽 모터 (A 채널)
  if (rightPower > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, rightPower);
  } else if (rightPower < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, -rightPower);
  } else {
    // 브레이크
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, 0);
  }

  // 왼쪽 모터 (B 채널)
  if (leftPower > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, leftPower);
  } else if (leftPower < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    analogWrite(PIN_PWMB, -leftPower);
  } else {
    // 브레이크
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, HIGH);
    analogWrite(PIN_PWMB, 0);
  }
}

// -100~100 → -255~255 매핑
int percentToPwm(int v) {
  v = constrain(v, -100, 100);
  return map(v, -100, 100, -255, 255);
}

// ======================= SERVO CONTROL =======================

// h ∈ [-100,100] → 서보 각도 (예: 70° ~ 130° 사이로 움직임)
void setServosPercent(int h) {
  h = constrain(h, -100, 100);
  int angle = map(h, -100, 100, 70, 130);  // 필요하면 범위 튜닝
  servo1.write(angle);
  servo2.write(angle);
}

// ======================= SERIAL PARSE ========================

void applyCommand(int rPercent, int lPercent, int hPercent) {
  cmdR = constrain(rPercent, -100, 100);
  cmdL = constrain(lPercent, -100, 100);
  cmdH = constrain(hPercent, -100, 100);
  lastCmdTime = millis();

  // 모터/서보에 바로 반영
  int pwmR = percentToPwm(cmdR);
  int pwmL = percentToPwm(cmdL);
  driveMotorsRaw(pwmR, pwmL);
  setServosPercent(cmdH);

  // 디버그 필요하면 주석 해제
  /*
  Serial.print("CMD -> r=");
  Serial.print(cmdR);
  Serial.print(", l=");
  Serial.print(cmdL);
  Serial.print(", h=");
  Serial.println(cmdH);
  */
}

void parseLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  int rStart = line.indexOf('r');
  int lStart = line.indexOf(",l");
  int hStart = line.indexOf(",h");

  if (rStart == -1 || lStart == -1 || hStart == -1) {
    Serial.print("Parse error: ");
    Serial.println(line);
    return;
  }

  String rStr = line.substring(rStart + 1, lStart);
  String lStr = line.substring(lStart + 2, hStart);
  String hStr = line.substring(hStart + 2);

  int r = rStr.toInt();
  int l = lStr.toInt();
  int h = hStr.toInt();

  applyCommand(r, l, h);
}

void handleSerial() {
  // 라즈베리파이에서 한 줄 단위로 보낸다고 가정 ("...\n")
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    parseLine(line);
  }
}

void stopAll() {
  cmdR = 0;
  cmdL = 0;
  // cmdH는 머리 각도 유지하고 싶으면 유지, 중앙으로 돌리고 싶으면 아래 활성화
  // cmdH = 0;

  driveMotorsRaw(0, 0);
  // setServosPercent(cmdH);  // 필요하면 중앙 유지 등
}

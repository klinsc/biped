#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_CH   0

// ช่วงที่คุณใช้อยู่
#define SERVO_MIN  200
#define SERVO_MAX  500

// แปลงมุมเป็นพัลส์
uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

// ขยับแบบนุ่ม: ไล่มุมทีละ stepDeg พร้อมหน่วงเวลา stepDelayMs
void moveServoSmooth(int fromDeg, int toDeg, int stepDeg = 2, int stepDelayMs = 15) {
  fromDeg = constrain(fromDeg, 0, 180);
  toDeg   = constrain(toDeg,   0, 180);

  if (fromDeg < toDeg) {
    for (int a = fromDeg; a <= toDeg; a += stepDeg) {
      pwm.setPWM(SERVO_CH, 0, angleToPulse(a));
      delay(stepDelayMs);
    }
  } else {
    for (int a = fromDeg; a >= toDeg; a -= stepDeg) {
      pwm.setPWM(SERVO_CH, 0, angleToPulse(a));
      delay(stepDelayMs);
    }
  }
  pwm.setPWM(SERVO_CH, 0, angleToPulse(toDeg)); // จบที่ค่าปลายทางพอดี
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);       // SDA=21, SCL=22
  pwm.begin();
  pwm.setPWMFreq(50);       // 50Hz servo
  delay(200);

  // เริ่มที่กลางก่อน ลดการกระชากตอนเปิด
  pwm.setPWM(SERVO_CH, 0, angleToPulse(90));
  delay(800);
}

void loop() {
  // ใช้ช่วงแคบก่อนเพื่อลดแรงกระชาก
  moveServoSmooth(90, 120, 2, 15);
  delay(500);

  moveServoSmooth(120, 60, 2, 15);
  delay(500);

  moveServoSmooth(60, 90, 2, 15);
  delay(1000);
}

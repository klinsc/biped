#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40);
#define SERVO_FREQ 50

// ====== CHANNEL MAP (ขาซ้าย: ล่าง -> บน) ======
const uint8_t CH_ANKLE_ROLL  = 0; // Servo#1 แนวขวาง L/R
const uint8_t CH_ANKLE_PITCH = 1; // Servo#2 หน้า/หลัง
const uint8_t CH_KNEE_PITCH  = 2; // Servo#3 หน้า/หลัง
const uint8_t CH_HIP_PITCH   = 3; // Servo#4 หน้า/หลัง

// ====== ปรับทิศทางตามการติดตั้งจริง ======
// ถ้าสั่ง + แล้วมันหมุนผิดทาง ให้สลับเป็น -1
int DIR_ANKLE_ROLL  = +1;
int DIR_ANKLE_PITCH = +1;
int DIR_KNEE_PITCH  = -1;
int DIR_HIP_PITCH   = +1;

// ====== จำกัดช่วงปลอดภัย (กันชนสุด) ======
const int US_CENTER = 1500;
const int US_MIN    = 1000;   // ปรับได้ภายหลังตาม servo ของคุณ
const int US_MAX    = 2000;

// แปลง "องศาแบบคร่าวๆ" เป็น us (ตรงนี้ใช้เพื่อเทสง่าย)
// 90deg ~ 1500us, ช่วง +-45deg ~ +-400us (ประมาณการ)
int degToUs(int degFromCenter) {
  // degFromCenter: -45..+45 แนะนำช่วงเทส
  long us = US_CENTER + (long)degFromCenter * 8; // 1deg ~ 8us (ปรับได้)
  if (us < US_MIN) us = US_MIN;
  if (us > US_MAX) us = US_MAX;
  return (int)us;
}

void writeServo(uint8_t ch, int us) {
  if (us < US_MIN) us = US_MIN;
  if (us > US_MAX) us = US_MAX;
  pwm.writeMicroseconds(ch, us);
}

void setAllCenter() {
  writeServo(CH_ANKLE_ROLL,  US_CENTER);
  writeServo(CH_ANKLE_PITCH, US_CENTER);
  writeServo(CH_KNEE_PITCH,  US_CENTER);
  writeServo(CH_HIP_PITCH,   US_CENTER);
}

// ขยับทีละข้อแบบนุ่ม
void sweepOne(uint8_t ch, int dir, int fromDeg, int toDeg, int stepDeg=1, int dly=20) {
  if (fromDeg < toDeg) {
    for (int d = fromDeg; d <= toDeg; d += stepDeg) {
      writeServo(ch, degToUs(dir * d));
      delay(dly);
    }
  } else {
    for (int d = fromDeg; d >= toDeg; d -= stepDeg) {
      writeServo(ch, degToUs(dir * d));
      delay(dly);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(300);

  // Phase: ตั้งท่ากลางเพื่อใส่ฮอร์น/เริ่มประกอบ
  setAllCenter();
  delay(1000);

  Serial.println("Left leg 4DOF test: AnkleRoll, AnklePitch, KneePitch, HipPitch");
}

void loop() {
  // === ท่ายืนเริ่มต้น (ชั่วคราว) ===
  // ตอนแรกให้ทุกตัวอยู่ center ก่อน
  setAllCenter();
  delay(800);

  // 1) ทดสอบ Ankle Roll ซ้าย/ขวา เล็กๆ
  sweepOne(CH_ANKLE_ROLL, DIR_ANKLE_ROLL, 0, +10, 1, 20);
  sweepOne(CH_ANKLE_ROLL, DIR_ANKLE_ROLL, +10, -10, 1, 20);
  sweepOne(CH_ANKLE_ROLL, DIR_ANKLE_ROLL, -10, 0, 1, 20);
  delay(400);

  // 2) ทดสอบ Ankle Pitch ก้ม/เงย เล็กๆ
  sweepOne(CH_ANKLE_PITCH, DIR_ANKLE_PITCH, 0, +10, 1, 20);
  sweepOne(CH_ANKLE_PITCH, DIR_ANKLE_PITCH, +10, -10, 1, 20);
  sweepOne(CH_ANKLE_PITCH, DIR_ANKLE_PITCH, -10, 0, 1, 20);
  delay(400);

  // 3) ทดสอบ Knee Pitch งอเข่าเล็กน้อย (แนะนำทิศเดียวก่อน)
  sweepOne(CH_KNEE_PITCH, DIR_KNEE_PITCH, 0, +15, 1, 25);
  sweepOne(CH_KNEE_PITCH, DIR_KNEE_PITCH, +15, 0, 1, 25);
  delay(400);

  // 4) ทดสอบ Hip Pitch ขยับสะโพกหน้า/หลังเล็กๆ
  sweepOne(CH_HIP_PITCH, DIR_HIP_PITCH, 0, +10, 1, 20);
  sweepOne(CH_HIP_PITCH, DIR_HIP_PITCH, +10, -10, 1, 20);
  sweepOne(CH_HIP_PITCH, DIR_HIP_PITCH, -10, 0, 1, 20);

  delay(1200);
}

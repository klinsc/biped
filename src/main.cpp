#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40);
#define SERVO_FREQ 50

// ====== แก้เลขช่องให้ตรงของคุณ ======
const uint8_t R_ANKLE_ROLL  = 4; // ข้อเท้าขวา แนวขวาง L/R
const uint8_t R_ANKLE_PITCH = 5; // ข้อเท้าขวา หน้า/หลัง
const uint8_t R_KNEE_PITCH  = 6; // เข่าขวา หน้า/หลัง
const uint8_t R_HIP_PITCH   = 7; // สะโพกขวา หน้า/หลัง

// ====== ทิศทาง (+1/-1) ======
int DIR_R_ANKLE_ROLL  = -1;
int DIR_R_ANKLE_PITCH = -1;
int DIR_R_KNEE_PITCH  = +1;
int DIR_R_HIP_PITCH   = -1;

// ====== microseconds ======
const int US_CENTER = 1500;
const int US_MIN = 1000;
const int US_MAX = 2000;

void writeServo(uint8_t ch, int us) {
  if (us < US_MIN) us = US_MIN;
  if (us > US_MAX) us = US_MAX;
  pwm.writeMicroseconds(ch, us);
}

void setRightCenter() {
  writeServo(R_ANKLE_ROLL,  US_CENTER);
  writeServo(R_ANKLE_PITCH, US_CENTER);
  writeServo(R_KNEE_PITCH,  US_CENTER);
  writeServo(R_HIP_PITCH,   US_CENTER);
}

// 1 deg ~ 8 us (ใช้เทสทิศทางพอ)
int degToUs(int degFromCenter) {
  long us = US_CENTER + (long)degFromCenter * 8;
  if (us < US_MIN) us = US_MIN;
  if (us > US_MAX) us = US_MAX;
  return (int)us;
}

void bump(uint8_t ch, int dir, int deg = 10, int holdMs = 1200) {
  writeServo(ch, degToUs(dir * deg));
  delay(holdMs);
  writeServo(ch, US_CENTER);
  delay(800);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(300);

  setRightCenter();
  delay(800);

  Serial.println("RIGHT LEG direction test (watch motion):");
  Serial.println("1) Ankle Roll  (+deg)");
  Serial.println("2) Ankle Pitch (+deg)");
  Serial.println("3) Knee Pitch  (+deg)");
  Serial.println("4) Hip Pitch   (+deg)");
}

void loop() {
  // 1) ข้อเท้าแนวขวาง: +deg ควรเอียง "ไปซ้ายของหุ่น" หรือ "ไปขวาของหุ่น" (คุณบอกผมที)
  bump(R_ANKLE_ROLL, DIR_R_ANKLE_ROLL);

  // 2) ข้อเท้าหน้า/หลัง: +deg ควร "ปลายเท้าโน้มไปหน้า"
  bump(R_ANKLE_PITCH, DIR_R_ANKLE_PITCH);

  // 3) เข่า: +deg ควร "งอไปหน้า"
  bump(R_KNEE_PITCH, DIR_R_KNEE_PITCH, 12);

  // 4) สะโพกหน้า/หลัง: +deg ควร "ขาทั้งข้างไปหน้า"
  bump(R_HIP_PITCH, DIR_R_HIP_PITCH);

  Serial.println("Cycle done. Repeat...");
  delay(1500);
}

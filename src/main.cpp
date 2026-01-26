#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40);

#define SERVO_MIN     200
#define SERVO_MAX     500
#define SERVO_CENTER  ((SERVO_MIN + SERVO_MAX) / 2)

// เพิ่มให้เห็นชัดขึ้น แต่ยังไม่แรงมาก
#define TEST_OFFSET   50    // แนะนำ 40–60

#define CH_START      0
#define CH_END        4     // 0..4 (5 ตัว)

// เวลาสังเกต (ms)
#define T_CENTER_1    1200
#define T_PLUS        1800
#define T_CENTER_2    800
#define T_MINUS       1800
#define T_CENTER_3    1200
#define T_GAP         1200

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(300);

  Serial.println("=== Auto Servo Classifier (No button) ===");
  Serial.println("180deg: moves a bit then holds position.");
  Serial.println("360deg: rotates continuously during +OFFSET/-OFFSET.");
  Serial.println("Tip: remove horns / no load for safety.");
  Serial.println("------------------------------------------");
}

void setAllCenter() {
  for (int ch = CH_START; ch <= CH_END; ch++) {
    pwm.setPWM(ch, 0, SERVO_CENTER);
  }
}

void testOneChannel(int ch) {
  Serial.print("Testing CH");
  Serial.println(ch);

  // 1) Center
  pwm.setPWM(ch, 0, SERVO_CENTER);
  delay(T_CENTER_1);

  // 2) +Offset
  pwm.setPWM(ch, 0, SERVO_CENTER + TEST_OFFSET);
  delay(T_PLUS);

  // 3) Center
  pwm.setPWM(ch, 0, SERVO_CENTER);
  delay(T_CENTER_2);

  // 4) -Offset
  pwm.setPWM(ch, 0, SERVO_CENTER - TEST_OFFSET);
  delay(T_MINUS);

  // 5) Center
  pwm.setPWM(ch, 0, SERVO_CENTER);
  delay(T_CENTER_3);

  Serial.println("----");
  delay(T_GAP);
}

void loop() {
  setAllCenter();
  delay(800);

  for (int ch = CH_START; ch <= CH_END; ch++) {
    testOneChannel(ch);
  }

  Serial.println("=== Cycle complete. Restarting in 3s ===");
  delay(3000);
}

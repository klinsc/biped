#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40);
#define SERVO_FREQ 50

// ===== LEFT LEG =====
#define L_ANKLE_ROLL   0
#define L_ANKLE_PITCH  1
#define L_KNEE_PITCH   2
#define L_HIP_PITCH    3
#define L_HIP_ROLL     8   // เพิ่ม HIP_ROLL

// ===== RIGHT LEG =====
#define R_ANKLE_ROLL   4
#define R_ANKLE_PITCH  5
#define R_KNEE_PITCH   6
#define R_HIP_PITCH    7
#define R_HIP_ROLL     9   // เพิ่ม HIP_ROLL

// ===== microseconds =====
const int US_CENTER = 1500;
const int US_MIN = 1000;
const int US_MAX = 2000;

// ===== DIR (+1 / -1) =====
// ถ้าข้อไหนผิด → เปลี่ยนเครื่องหมายตัวนั้นตัวเดียว
int DIR_L_ANKLE_ROLL  = +1;
int DIR_L_ANKLE_PITCH = +1;
int DIR_L_KNEE_PITCH  = +1;
int DIR_L_HIP_PITCH   = +1;
int DIR_L_HIP_ROLL    = +1;

int DIR_R_ANKLE_ROLL  = +1;
int DIR_R_ANKLE_PITCH = +1;
int DIR_R_KNEE_PITCH  = +1;
int DIR_R_HIP_PITCH   = +1;
int DIR_R_HIP_ROLL    = +1;

// ===== helper =====
int degToUs(int deg) {
  long us = US_CENTER + (long)deg * 8; // ~8us/deg
  if (us < US_MIN) us = US_MIN;
  if (us > US_MAX) us = US_MAX;
  return us;
}

void bump(uint8_t ch, int dir, int deg = 10) {
  pwm.writeMicroseconds(ch, degToUs(dir * deg));
  delay(1200);
  pwm.writeMicroseconds(ch, US_CENTER);
  delay(800);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(300);

  Serial.println("=== Direction Check : Legs + HIP_ROLL ===");
}

void loop() {
  // ----- LEFT LEG -----
  Serial.println("LEFT: Ankle Roll");
  bump(L_ANKLE_ROLL, DIR_L_ANKLE_ROLL);

  Serial.println("LEFT: Ankle Pitch");
  bump(L_ANKLE_PITCH, DIR_L_ANKLE_PITCH);

  Serial.println("LEFT: Knee Pitch");
  bump(L_KNEE_PITCH, DIR_L_KNEE_PITCH);

  Serial.println("LEFT: Hip Pitch");
  bump(L_HIP_PITCH, DIR_L_HIP_PITCH);

  Serial.println("LEFT: Hip Roll");
  bump(L_HIP_ROLL, DIR_L_HIP_ROLL);

  delay(1500);

  // ----- RIGHT LEG -----
  Serial.println("RIGHT: Ankle Roll");
  bump(R_ANKLE_ROLL, DIR_R_ANKLE_ROLL);

  Serial.println("RIGHT: Ankle Pitch");
  bump(R_ANKLE_PITCH, DIR_R_ANKLE_PITCH);

  Serial.println("RIGHT: Knee Pitch");
  bump(R_KNEE_PITCH, DIR_R_KNEE_PITCH);

  Serial.println("RIGHT: Hip Pitch");
  bump(R_HIP_PITCH, DIR_R_HIP_PITCH);

  Serial.println("RIGHT: Hip Roll");
  bump(R_HIP_ROLL, DIR_R_HIP_ROLL);

  Serial.println("=== Cycle complete ===");
  delay(3000);
}

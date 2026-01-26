#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40);

#define SERVO_FREQ 50  // Hz

uint16_t usToTicks(uint16_t us) {
  // 50Hz => 20,000 us ต่อคาบ
  // PCA9685 12-bit => 4096 ticks ต่อคาบ
  return (uint32_t)us * 4096 / 20000;
}

void setup() {
  Wire.begin(21, 22);       // ESP32
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(300);

  // เซนเตอร์ = 1500us
  uint16_t center = usToTicks(1500);

  for (int ch = 0; ch < 16; ch++) {
    pwm.setPWM(ch, 0, center);
    delay(10);
  }
}

void loop() {}

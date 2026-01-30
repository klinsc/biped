#ifndef SERVOHANDLER_H
#define SERVOHANDLER_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

class ServoHandler {
public:
  ServoHandler();
  void begin();
  void setServoDeg(uint8_t ch, int dir, float deg);
  void setCenter(uint8_t ch);
  int centerUsForChannel(uint8_t ch);

private:
  Adafruit_PWMServoDriver pwm;

  int degToUs(int deg);
  int getOffsetDegForChannel(uint8_t ch);
  int clampTargetDeg(uint8_t ch, int targetDeg);
};

#endif

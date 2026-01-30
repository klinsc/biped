#include "ServoHandler.h"
#include "I2CManager.h"
#include <math.h>

// Servo constants
static const int SERVO_FREQ = 50;
static const int US_CENTER = 1500;
static const int US_MIN = 1000;
static const int US_MAX = 2000;

// Soft limits (deg around offset)
static const int MAX_SOFT_DEG = 50;
static const int CH_MIN_DEG[10] = {-MAX_SOFT_DEG, -MAX_SOFT_DEG, -MAX_SOFT_DEG, -MAX_SOFT_DEG, -MAX_SOFT_DEG,
                                   -MAX_SOFT_DEG, -MAX_SOFT_DEG, -MAX_SOFT_DEG, -MAX_SOFT_DEG, -MAX_SOFT_DEG};
static const int CH_MAX_DEG[10] = {MAX_SOFT_DEG, MAX_SOFT_DEG, MAX_SOFT_DEG, MAX_SOFT_DEG, MAX_SOFT_DEG, MAX_SOFT_DEG, MAX_SOFT_DEG, MAX_SOFT_DEG, MAX_SOFT_DEG, MAX_SOFT_DEG};

// Channel -> DIR index
static const int CHANNEL_TO_DIR_IDX[10] = {
    0, 1, 2, 3, 5, 6, 7, 8, 4, 9};

extern int OFFSETS_DEG[];

ServoHandler::ServoHandler() : pwm(0x40) {}

void ServoHandler::begin()
{
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
}

int ServoHandler::degToUs(int deg)
{
  long us = US_CENTER + (long)deg * 8; // ~8us/deg
  if (us < US_MIN)
    us = US_MIN;
  if (us > US_MAX)
    us = US_MAX;
  return us;
}

int ServoHandler::getOffsetDegForChannel(uint8_t ch)
{
  if (ch >= 10)
    return 0;
  int idx = CHANNEL_TO_DIR_IDX[ch];
  if (idx < 0 || idx >= 10)
    return 0;
  return OFFSETS_DEG[idx];
}

int ServoHandler::centerUsForChannel(uint8_t ch)
{
  int offset = getOffsetDegForChannel(ch);
  return degToUs(offset);
}

int ServoHandler::clampTargetDeg(uint8_t ch, int targetDeg)
{
  if (ch >= 10)
    return targetDeg;
  int offset = getOffsetDegForChannel(ch);
  int minAbs = offset + CH_MIN_DEG[ch];
  int maxAbs = offset + CH_MAX_DEG[ch];
  if (targetDeg < minAbs)
    return minAbs;
  if (targetDeg > maxAbs)
    return maxAbs;
  return targetDeg;
}

void ServoHandler::setServoDeg(uint8_t ch, int dir, float deg)
{
  int offset = getOffsetDegForChannel(ch);
  int target = (int)roundf(offset + (dir * deg));
  target = clampTargetDeg(ch, target);
  I2CManager::Lock lock;
  pwm.writeMicroseconds(ch, degToUs(target));
}

void ServoHandler::setCenter(uint8_t ch)
{
  I2CManager::Lock lock;
  pwm.writeMicroseconds(ch, centerUsForChannel(ch));
}

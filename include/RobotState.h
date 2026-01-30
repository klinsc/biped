#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <Arduino.h>

// สถานะการทดสอบข้อต่อ (Test Motion State)
enum MotionAction { IDLE, BUMP_START, BUMP_WAIT, BUMP_RETURN, BUMP_FINISH };

struct RobotState {
    // Shared Flags
    volatile bool estopActive = false;
    volatile bool pidActive = true;
    volatile bool pidSuspendCal = false;

    // Test Motion Context
    volatile MotionAction currentAction = IDLE;
    volatile int testChannel = -1;
    volatile int testDir = 1;
    volatile int testDeg = 10;
    unsigned long actionTimer = 0;

    // System flags
    volatile bool pendingRestart = false;
    volatile bool imuCalibrateRequested = false;
};

extern RobotState globalState;

#endif

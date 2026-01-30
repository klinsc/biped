#include <Wire.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <Preferences.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "RobotState.h"
#include "I2CManager.h"
#include "RobotJoints.h"
#include "ServoHandler.h"
#include "WebHandler.h"

// ===== Safety =====
int MAX_DEG_PER_CMD = 25;           // limit per command
unsigned long MIN_CMD_INTERVAL_MS = 250; // rate limit
unsigned long lastCmdMs = 0;
int OFFSET_MIN_DEG = -45;
int OFFSET_MAX_DEG = 45;

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

Preferences prefs;
Adafruit_MPU6050 mpu;
RobotState globalState;
ServoHandler servos;
WebHandler web;
SemaphoreHandle_t I2CManager::mutex = nullptr;

// ===== IMU =====
volatile float IMU_ROLL = 0.0f;
volatile float IMU_PITCH = 0.0f;
volatile float IMU_LAST_DT = 0.0f;
volatile bool IMU_UPDATED = false;
volatile bool IMU_VALID = false;
volatile bool IMU_READY = false;
unsigned long lastImuMs = 0;
float GYRO_BIAS_X = 0.0f;
float GYRO_BIAS_Y = 0.0f;
float GYRO_BIAS_Z = 0.0f;
unsigned long lastImuMicros = 0;
const float IMU_ALPHA = 0.98f;
const unsigned long IMU_DT_US = 5000; // 200 Hz
const float IMU_MAX_ABS_ROLL = 60.0f;
const float IMU_MAX_ABS_PITCH = 60.0f;
const unsigned long IMU_STALE_MS = 200;

// ===== PID (soft defaults) =====
volatile float PID_KP = 0.6f;
volatile float PID_KI = 0.02f;
volatile float PID_KD = 0.08f;
const float PID_MAX_OUT_DEG = 12.0f;
const float PID_I_LIMIT = 20.0f;
const float PID_ANKLE_GAIN = 0.6f;
const float PID_HIP_GAIN = 0.4f;
int PID_ROLL_SIGN = 1;
int PID_PITCH_SIGN = 1;


// ===== DIR (+1 / -1) =====
// ถ้าข้อไหนผิด → เปลี่ยนเครื่องหมายตัวนั้นตัวเดียว
int DIR_L_ANKLE_ROLL  = +1;
int DIR_L_ANKLE_PITCH = +1;
int DIR_L_KNEE_PITCH  = -1;
int DIR_L_HIP_PITCH   = +1;
int DIR_L_HIP_ROLL    = +1;

int DIR_R_ANKLE_ROLL  = -1;
int DIR_R_ANKLE_PITCH = -1;
int DIR_R_KNEE_PITCH  = +1;
int DIR_R_HIP_PITCH   = -1;
int DIR_R_HIP_ROLL    = -1;

// ===== DIR registry =====
enum DirIndex {
  DIR_L_ANKLE_ROLL_IDX,
  DIR_L_ANKLE_PITCH_IDX,
  DIR_L_KNEE_PITCH_IDX,
  DIR_L_HIP_PITCH_IDX,
  DIR_L_HIP_ROLL_IDX,
  DIR_R_ANKLE_ROLL_IDX,
  DIR_R_ANKLE_PITCH_IDX,
  DIR_R_KNEE_PITCH_IDX,
  DIR_R_HIP_PITCH_IDX,
  DIR_R_HIP_ROLL_IDX,
  DIR_COUNT
};

const char* DIR_NAMES[DIR_COUNT] = {
  "L_ANKLE_ROLL",
  "L_ANKLE_PITCH",
  "L_KNEE_PITCH",
  "L_HIP_PITCH",
  "L_HIP_ROLL",
  "R_ANKLE_ROLL",
  "R_ANKLE_PITCH",
  "R_KNEE_PITCH",
  "R_HIP_PITCH",
  "R_HIP_ROLL"
};

int* DIR_PTRS[DIR_COUNT] = {
  &DIR_L_ANKLE_ROLL,
  &DIR_L_ANKLE_PITCH,
  &DIR_L_KNEE_PITCH,
  &DIR_L_HIP_PITCH,
  &DIR_L_HIP_ROLL,
  &DIR_R_ANKLE_ROLL,
  &DIR_R_ANKLE_PITCH,
  &DIR_R_KNEE_PITCH,
  &DIR_R_HIP_PITCH,
  &DIR_R_HIP_ROLL
};

int OFFSETS_DEG[DIR_COUNT] = {0};

// ===== Mixer Output =====
float pidOutput[DIR_COUNT] = {0};
float testOutput[DIR_COUNT] = {0};

void bump(uint8_t ch, int dir, int deg = 10);
void applyEmergencyStop(bool active);
void resetPidState();
void applyBalancePid(float dt);
bool imuIsValid(float roll, float pitch);
void loadDirs();
void saveDir(int idx);
void startTestMotion(uint8_t ch, int dir, int deg);
void updateTestMotion();

int findDirIndex(const String& name) {
  for (int i = 0; i < DIR_COUNT; i++) {
    if (name == DIR_NAMES[i]) return i;
  }
  return -1;
}

String buildStateJson() {
  String json;
  json.reserve(256);
  json += "{";
  int snapshot[DIR_COUNT];
  portENTER_CRITICAL(&dataMux);
  for (int i = 0; i < DIR_COUNT; i++) snapshot[i] = *DIR_PTRS[i];
  portEXIT_CRITICAL(&dataMux);
  for (int i = 0; i < DIR_COUNT; i++) {
    json += "\"" + String(DIR_NAMES[i]) + "\":" + String(snapshot[i]);
    if (i < DIR_COUNT - 1) json += ",";
  }
  json += "}";
  return json;
}

String buildPidJson() {
  String json;
  json.reserve(96);
  float kp, ki, kd;
  portENTER_CRITICAL(&dataMux);
  kp = PID_KP;
  ki = PID_KI;
  kd = PID_KD;
  portEXIT_CRITICAL(&dataMux);
  json += "{";
  json += "\"kp\":" + String(kp, 3) + ",";
  json += "\"ki\":" + String(ki, 3) + ",";
  json += "\"kd\":" + String(kd, 3);
  json += "}";
  return json;
}

String buildSafetyJson() {
  String json;
  json.reserve(32);
  bool estop;
  portENTER_CRITICAL(&dataMux);
  estop = globalState.estopActive;
  portEXIT_CRITICAL(&dataMux);
  json += "{";
  json += "\"estop\":" + String(estop ? 1 : 0);
  json += "}";
  return json;
}

String buildOffsetsJson() {
  String json;
  json.reserve(256);
  int snapshot[DIR_COUNT];
  portENTER_CRITICAL(&dataMux);
  for (int i = 0; i < DIR_COUNT; i++) snapshot[i] = OFFSETS_DEG[i];
  portEXIT_CRITICAL(&dataMux);
  json += "{";
  for (int i = 0; i < DIR_COUNT; i++) {
    json += "\"" + String(DIR_NAMES[i]) + "\":" + String(snapshot[i]);
    if (i < DIR_COUNT - 1) json += ",";
  }
  json += "}";
  return json;
}

String buildImuJson() {
  String json;
  json.reserve(64);
  float roll, pitch;
  portENTER_CRITICAL(&dataMux);
  roll = IMU_ROLL;
  pitch = IMU_PITCH;
  portEXIT_CRITICAL(&dataMux);
  json += "{";
  json += "\"roll\":" + String(roll, 2) + ",";
  json += "\"pitch\":" + String(pitch, 2);
  json += "}";
  return json;
}

void loadOffsets() {
  prefs.begin("offsets", true);
  for (int i = 0; i < DIR_COUNT; i++) {
    String key = "o" + String(i);
    OFFSETS_DEG[i] = prefs.getInt(key.c_str(), 0);
  }
  prefs.end();
}

void saveOffset(int idx) {
  prefs.begin("offsets", false);
  String key = "o" + String(idx);
  prefs.putInt(key.c_str(), OFFSETS_DEG[idx]);
  prefs.end();
}

void loadDirs() {
  prefs.begin("dirs", true);
  for (int i = 0; i < DIR_COUNT; i++) {
    String key = "d" + String(i);
    int defVal = *DIR_PTRS[i];
    int val = prefs.getInt(key.c_str(), defVal);
    if (val != 1 && val != -1) val = defVal;
    *DIR_PTRS[i] = val;
  }
  prefs.end();
}

void saveDir(int idx) {
  if (idx < 0 || idx >= DIR_COUNT) return;
  prefs.begin("dirs", false);
  String key = "d" + String(idx);
  prefs.putInt(key.c_str(), *DIR_PTRS[idx]);
  prefs.end();
}

void calibrateGyroBias() {
  const int samples = 300;
  float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
  sensors_event_t a, g, temp;
  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(2);
  }
  GYRO_BIAS_X = sumX / samples;
  GYRO_BIAS_Y = sumY / samples;
  GYRO_BIAS_Z = sumZ / samples;
}

void setupImu() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    IMU_READY = false;
    IMU_VALID = false;
    return;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  calibrateGyroBias();
  lastImuMicros = micros();
  lastImuMs = millis();
  IMU_READY = true;
  IMU_VALID = true;
}

void updateImu() {
  if (!IMU_READY) return;
  unsigned long now = micros();
  if (now - lastImuMicros < IMU_DT_US) return;
  float dt = (now - lastImuMicros) / 1000000.0f;
  lastImuMicros = now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float axf = a.acceleration.x;
  float ayf = a.acceleration.y;
  float azf = a.acceleration.z;

  float rollAcc = atan2f(ayf, azf) * 57.2958f;
  float pitchAcc = atan2f(-axf, sqrtf(ayf * ayf + azf * azf)) * 57.2958f;

  float gxDps = (g.gyro.x - GYRO_BIAS_X) * 57.2958f;
  float gyDps = (g.gyro.y - GYRO_BIAS_Y) * 57.2958f;

  float roll = IMU_ROLL + gxDps * dt;
  float pitch = IMU_PITCH + gyDps * dt;

  float nextRoll = IMU_ALPHA * roll + (1.0f - IMU_ALPHA) * rollAcc;
  float nextPitch = IMU_ALPHA * pitch + (1.0f - IMU_ALPHA) * pitchAcc;
  bool valid = imuIsValid(nextRoll, nextPitch);
  portENTER_CRITICAL(&dataMux);
  IMU_ROLL = nextRoll;
  IMU_PITCH = nextPitch;
  IMU_LAST_DT = dt;
  IMU_VALID = valid;
  lastImuMs = millis();
  IMU_UPDATED = true;
  portEXIT_CRITICAL(&dataMux);
}


// ===== helper =====
bool imuIsValid(float roll, float pitch) {
  if (!isfinite(roll) || !isfinite(pitch)) return false;
  if (fabsf(roll) > IMU_MAX_ABS_ROLL) return false;
  if (fabsf(pitch) > IMU_MAX_ABS_PITCH) return false;
  return true;
}

void startTestMotion(uint8_t ch, int dir, int deg) {
  portENTER_CRITICAL(&dataMux);
  globalState.testChannel = ch;
  globalState.testDir = dir;
  globalState.testDeg = deg;
  globalState.currentAction = BUMP_START;
  globalState.actionTimer = 0;
  portEXIT_CRITICAL(&dataMux);
}

void updateTestMotion() {
  if (globalState.currentAction == IDLE) return;
  unsigned long now = millis();
  int ch = globalState.testChannel;
  if (ch < 0 || ch >= 10) {
    globalState.currentAction = IDLE;
    return;
  }

  switch (globalState.currentAction) {
    case BUMP_START:
      testOutput[ch] = globalState.testDir * globalState.testDeg;
      globalState.actionTimer = now;
      globalState.currentAction = BUMP_WAIT;
      break;

    case BUMP_WAIT:
      if (now - globalState.actionTimer >= 1200) {
        globalState.currentAction = BUMP_RETURN;
      }
      break;

    case BUMP_RETURN:
      testOutput[ch] = 0;
      globalState.actionTimer = now;
      globalState.currentAction = BUMP_FINISH;
      break;

    case BUMP_FINISH:
      if (now - globalState.actionTimer >= 800) {
        globalState.currentAction = IDLE;
      }
      break;

    default:
      break;
  }
}    

// ===== PID state =====
float pidRollI = 0.0f;
float pidPitchI = 0.0f;
float pidRollPrevErr = 0.0f;
float pidPitchPrevErr = 0.0f;

void resetPidState() {
  pidRollI = 0.0f;
  pidPitchI = 0.0f;
  pidRollPrevErr = 0.0f;
  pidPitchPrevErr = 0.0f;
}

void applyBalancePid(float dt) {
  if (!globalState.pidActive || globalState.pidSuspendCal || globalState.estopActive) return;
  if (dt <= 0.0f) return;

  float errRoll = 0.0f - IMU_ROLL;
  float errPitch = 0.0f - IMU_PITCH;

  pidRollI += errRoll * dt;
  pidPitchI += errPitch * dt;
  if (pidRollI > PID_I_LIMIT) pidRollI = PID_I_LIMIT;
  if (pidRollI < -PID_I_LIMIT) pidRollI = -PID_I_LIMIT;
  if (pidPitchI > PID_I_LIMIT) pidPitchI = PID_I_LIMIT;
  if (pidPitchI < -PID_I_LIMIT) pidPitchI = -PID_I_LIMIT;

  float dRoll = (errRoll - pidRollPrevErr) / dt;
  float dPitch = (errPitch - pidPitchPrevErr) / dt;
  pidRollPrevErr = errRoll;
  pidPitchPrevErr = errPitch;

  float outRoll = (PID_KP * errRoll) + (PID_KI * pidRollI) + (PID_KD * dRoll);
  float outPitch = (PID_KP * errPitch) + (PID_KI * pidPitchI) + (PID_KD * dPitch);

  if (outRoll > PID_MAX_OUT_DEG) outRoll = PID_MAX_OUT_DEG;
  if (outRoll < -PID_MAX_OUT_DEG) outRoll = -PID_MAX_OUT_DEG;
  if (outPitch > PID_MAX_OUT_DEG) outPitch = PID_MAX_OUT_DEG;
  if (outPitch < -PID_MAX_OUT_DEG) outPitch = -PID_MAX_OUT_DEG;

  outRoll *= PID_ROLL_SIGN;
  outPitch *= PID_PITCH_SIGN;

  // Roll: left/right opposite
  pidOutput[L_ANKLE_ROLL] = DIR_L_ANKLE_ROLL * outRoll * PID_ANKLE_GAIN;
  pidOutput[R_ANKLE_ROLL] = DIR_R_ANKLE_ROLL * -outRoll * PID_ANKLE_GAIN;
  pidOutput[L_HIP_ROLL]   = DIR_L_HIP_ROLL   * outRoll * PID_HIP_GAIN;
  pidOutput[R_HIP_ROLL]   = DIR_R_HIP_ROLL   * -outRoll * PID_HIP_GAIN;

  // Pitch: left/right same direction
  pidOutput[L_ANKLE_PITCH] = DIR_L_ANKLE_PITCH * outPitch * PID_ANKLE_GAIN;
  pidOutput[R_ANKLE_PITCH] = DIR_R_ANKLE_PITCH * outPitch * PID_ANKLE_GAIN;
  pidOutput[L_HIP_PITCH]   = DIR_L_HIP_PITCH   * outPitch * PID_HIP_GAIN;
  pidOutput[R_HIP_PITCH]   = DIR_R_HIP_PITCH   * outPitch * PID_HIP_GAIN;
}

void applyServos() {
  bool estop;
  portENTER_CRITICAL(&dataMux);
  estop = globalState.estopActive;
  portEXIT_CRITICAL(&dataMux);

  static bool prevEstop = false;

  if (estop) {
    if (!prevEstop) {
      // First frame of E-STOP: Clear states and Center servos safely in this thread
      resetPidState();
      globalState.currentAction = IDLE;
      for (int i = 0; i < DIR_COUNT; i++) {
        pidOutput[i] = 0;
        testOutput[i] = 0;
        servos.setCenter(i);
      }
      prevEstop = true;
    }
    return;
  }
  prevEstop = false;

  for (int i = 0; i < DIR_COUNT; i++) {
    float finalDeg = pidOutput[i] + testOutput[i];
    // Use dir=1 because directions are already calculated in the outputs
    servos.setServoDeg(i, 1, finalDeg);
  }
}

void applyEmergencyStop(bool active) {
  portENTER_CRITICAL(&dataMux);
  globalState.estopActive = active;
  portEXIT_CRITICAL(&dataMux);
  // Actual stopping logic is handled in applyServos (Loop Thread) to prevent races
}

void bump(uint8_t ch, int dir, int deg) {
  if (globalState.estopActive) return;
  startTestMotion(ch, dir, deg);
}

void setup() {
  Serial.begin(115200);
  I2CManager::begin();
  servos.begin();
  delay(300);

  setupImu();
  loadOffsets();
  loadDirs();

  web.begin();

  Serial.println("=== Direction Check : Legs + HIP_ROLL ===");
}

void loop() {
  updateImu();
  if (globalState.imuCalibrateRequested) {
    globalState.imuCalibrateRequested = false;
    applyEmergencyStop(true);
    globalState.pidSuspendCal = true;
    resetPidState();
    calibrateGyroBias();
    lastImuMicros = micros();
    IMU_UPDATED = false;
    IMU_VALID = false;
    lastImuMs = millis();
    globalState.pidSuspendCal = false;
  }
  updateTestMotion();
  if (millis() - lastImuMs > IMU_STALE_MS) {
    applyEmergencyStop(true);
  }
  if (IMU_UPDATED) {
    IMU_UPDATED = false;
    if (IMU_VALID) {
      applyBalancePid(IMU_LAST_DT);
    } else {
      applyEmergencyStop(true);
    }
  }
  
  applyServos();

  if (globalState.pendingRestart) {
    globalState.pendingRestart = false;
    ESP.restart();
  }
  delay(1);
}

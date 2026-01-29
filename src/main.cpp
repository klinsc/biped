#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <Update.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_PWMServoDriver pwm(0x40);
#define SERVO_FREQ 50

// ===== WiFi =====
const char* WIFI_SSID = "Boomx_2.4G";
const char* WIFI_PASS = "11111111";
AsyncWebServer server(80);
const IPAddress STATIC_IP(192, 168, 1, 163);
const IPAddress GATEWAY(192, 168, 1, 1);
const IPAddress SUBNET(255, 255, 255, 0);
const IPAddress DNS1(192, 168, 1, 1);
const IPAddress DNS2(8, 8, 8, 8);

// ===== Safety =====
volatile bool ESTOP_ACTIVE = false;
const int MAX_DEG_PER_CMD = 25;           // limit per command
const unsigned long MIN_CMD_INTERVAL_MS = 250; // rate limit
unsigned long lastCmdMs = 0;
const int OFFSET_MIN_DEG = -45;
const int OFFSET_MAX_DEG = 45;

Preferences prefs;
Adafruit_MPU6050 mpu;

// ===== IMU =====
volatile float IMU_ROLL = 0.0f;
volatile float IMU_PITCH = 0.0f;
float GYRO_BIAS_X = 0.0f;
float GYRO_BIAS_Y = 0.0f;
float GYRO_BIAS_Z = 0.0f;
unsigned long lastImuMicros = 0;
const float IMU_ALPHA = 0.98f;
const unsigned long IMU_DT_US = 5000; // 200 Hz

// ===== PID (soft defaults) =====
volatile float PID_KP = 0.6f;
volatile float PID_KI = 0.02f;
volatile float PID_KD = 0.08f;

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

const int CHANNEL_TO_DIR_IDX[10] = {
  DIR_L_ANKLE_ROLL_IDX,
  DIR_L_ANKLE_PITCH_IDX,
  DIR_L_KNEE_PITCH_IDX,
  DIR_L_HIP_PITCH_IDX,
  DIR_R_ANKLE_ROLL_IDX,
  DIR_R_ANKLE_PITCH_IDX,
  DIR_R_KNEE_PITCH_IDX,
  DIR_R_HIP_PITCH_IDX,
  DIR_L_HIP_ROLL_IDX,
  DIR_R_HIP_ROLL_IDX
};

void bump(uint8_t ch, int dir, int deg = 10);
void applyEmergencyStop(bool active);

int findDirIndex(const String& name) {
  for (int i = 0; i < DIR_COUNT; i++) {
    if (name == DIR_NAMES[i]) return i;
  }
  return -1;
}

String buildStateJson() {
  String json = "{";
  for (int i = 0; i < DIR_COUNT; i++) {
    json += "\"" + String(DIR_NAMES[i]) + "\":" + String(*DIR_PTRS[i]);
    if (i < DIR_COUNT - 1) json += ",";
  }
  json += "}";
  return json;
}

String buildPidJson() {
  String json = "{";
  json += "\"kp\":" + String(PID_KP, 3) + ",";
  json += "\"ki\":" + String(PID_KI, 3) + ",";
  json += "\"kd\":" + String(PID_KD, 3);
  json += "}";
  return json;
}

String buildSafetyJson() {
  String json = "{";
  json += "\"estop\":" + String(ESTOP_ACTIVE ? 1 : 0);
  json += "}";
  return json;
}

String buildOffsetsJson() {
  String json = "{";
  for (int i = 0; i < DIR_COUNT; i++) {
    json += "\"" + String(DIR_NAMES[i]) + "\":" + String(OFFSETS_DEG[i]);
    if (i < DIR_COUNT - 1) json += ",";
  }
  json += "}";
  return json;
}

String buildImuJson() {
  String json = "{";
  json += "\"roll\":" + String(IMU_ROLL, 2) + ",";
  json += "\"pitch\":" + String(IMU_PITCH, 2);
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
    return;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  calibrateGyroBias();
  lastImuMicros = micros();
}

void updateImu() {
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

  IMU_ROLL = IMU_ALPHA * roll + (1.0f - IMU_ALPHA) * rollAcc;
  IMU_PITCH = IMU_ALPHA * pitch + (1.0f - IMU_ALPHA) * pitchAcc;
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>DIR Tuning</title>
  <style>
    body{font-family:Arial,Helvetica,sans-serif;margin:20px;background:#0b0f14;color:#e6edf3}
    h2{margin-bottom:8px}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}
    .card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:12px}
    .name{font-weight:bold;margin-bottom:8px}
    .row{display:flex;align-items:center;gap:10px}
    button{border:0;border-radius:6px;padding:8px 12px;cursor:pointer}
    .neg{background:#da3633;color:#fff}
    .pos{background:#238636;color:#fff}
    .val{min-width:24px;text-align:center;font-weight:bold}
    .muted{opacity:.7;font-size:12px}
    .viewer{perspective:800px;max-width:320px}
    .cube{width:160px;height:160px;position:relative;transform-style:preserve-3d;transition:transform .1s linear}
    .face{position:absolute;width:160px;height:160px;background:#0f1720;border:1px solid #30363d;opacity:.9}
    .front{transform:translateZ(80px)}
    .back{transform:rotateY(180deg) translateZ(80px)}
    .right{transform:rotateY(90deg) translateZ(80px)}
    .left{transform:rotateY(-90deg) translateZ(80px)}
    .top{transform:rotateX(90deg) translateZ(80px)}
    .bottom{transform:rotateX(-90deg) translateZ(80px)}
  </style>
</head>
<body>
  <h2>DIR (+1 / -1) Tuning</h2>
  <div class="muted">แตะปุ่มเพื่อสลับค่าแบบ realtime</div>
  <div id="grid" class="grid" style="margin-top:12px"></div>

  <h2 style="margin-top:18px">Manual Test</h2>
  <div class="muted">กดเพื่อเพิ่ม/ลดมุมทีละข้อต่อ (ใช้ค่า DIR ปัจจุบัน)</div>
  <div class="row" style="margin-top:8px">
    <span class="muted">Step (deg)</span>
    <input id="degStep" type="number" step="1" min="1" max="45" value="10" style="width:80px" />
  </div>
  <div class="row" style="margin-top:8px">
    <button class="neg" onclick="setEStop(true)">EMERGENCY STOP</button>
    <button class="pos" onclick="setEStop(false)">RESET</button>
    <span id="estopState" class="muted"></span>
  </div>
  <div id="testgrid" class="grid" style="margin-top:12px"></div>

  <h2 style="margin-top:18px">Offset (deg)</h2>
  <div class="muted">ปรับศูนย์กลไกของแต่ละข้อก่อนจูน PID</div>
  <div id="offsetgrid" class="grid" style="margin-top:12px"></div>

  <h2 style="margin-top:18px">PID Tuning</h2>
  <div class="muted">ปรับได้จริงแบบ realtime (ค่าเริ่มต้นนุ่มๆ)</div>
  <div class="grid" style="margin-top:12px">
    <div class="card">
      <div class="name">Kp</div>
      <div class="row">
        <input id="kp" type="number" step="0.01" style="width:100%" />
        <button class="pos" onclick="savePid()">Set</button>
      </div>
    </div>
    <div class="card">
      <div class="name">Ki</div>
      <div class="row">
        <input id="ki" type="number" step="0.01" style="width:100%" />
        <button class="pos" onclick="savePid()">Set</button>
      </div>
    </div>
    <div class="card">
      <div class="name">Kd</div>
      <div class="row">
        <input id="kd" type="number" step="0.01" style="width:100%" />
        <button class="pos" onclick="savePid()">Set</button>
      </div>
    </div>
  </div>

  <h2 style="margin-top:18px">IMU 3D View</h2>
  <div class="muted">แสดงทิศทางจาก GY-87 (roll/pitch)</div>
  <div class="row" style="margin-top:8px">
    <div class="viewer">
      <div id="cube" class="cube">
        <div class="face front"></div>
        <div class="face back"></div>
        <div class="face right"></div>
        <div class="face left"></div>
        <div class="face top"></div>
        <div class="face bottom"></div>
      </div>
    </div>
    <div style="margin-left:16px">
      <div class="muted">Roll: <span id="imuRoll">0</span>°</div>
      <div class="muted">Pitch: <span id="imuPitch">0</span>°</div>
    </div>
  </div>

  <script>
    const names = [
      "L_ANKLE_ROLL","L_ANKLE_PITCH","L_KNEE_PITCH","L_HIP_PITCH","L_HIP_ROLL",
      "R_ANKLE_ROLL","R_ANKLE_PITCH","R_KNEE_PITCH","R_HIP_PITCH","R_HIP_ROLL"
    ];

    const grid = document.getElementById('grid');
    const offsetgrid = document.getElementById('offsetgrid');

    function card(name, val){
      const div = document.createElement('div');
      div.className = 'card';
      div.innerHTML = `
        <div class="name">${name}</div>
        <div class="row">
          <button class="neg" onclick="setDir('${name}',-1)">-1</button>
          <div class="val" id="val-${name}">${val}</div>
          <button class="pos" onclick="setDir('${name}',+1)">+1</button>
        </div>
      `;
      return div;
    }

    function testCard(name){
      const div = document.createElement('div');
      div.className = 'card';
      div.innerHTML = `
        <div class="name">${name}</div>
        <div class="row">
          <button class="neg" onclick="testJoint('${name}',-1)">-</button>
          <button class="pos" onclick="testJoint('${name}',+1)">+</button>
        </div>
      `;
      return div;
    }

    function offsetCard(name, val){
      const div = document.createElement('div');
      div.className = 'card';
      div.innerHTML = `
        <div class="name">${name}</div>
        <div class="row">
          <button class="neg" onclick="setOffset('${name}',-1)">-1</button>
          <div class="val" id="off-${name}">${val}</div>
          <button class="pos" onclick="setOffset('${name}',+1)">+1</button>
        </div>
      `;
      return div;
    }

    async function load(){
      const res = await fetch('/state');
      const data = await res.json();
      grid.innerHTML='';
      names.forEach(n => grid.appendChild(card(n, data[n])));
      const offRes = await fetch('/offsets');
      const off = await offRes.json();
      offsetgrid.innerHTML='';
      names.forEach(n => offsetgrid.appendChild(offsetCard(n, off[n] ?? 0)));
      const testgrid = document.getElementById('testgrid');
      testgrid.innerHTML='';
      names.forEach(n => testgrid.appendChild(testCard(n)));
    }

    async function loadPid(){
      const res = await fetch('/pid');
      const pid = await res.json();
      document.getElementById('kp').value = pid.kp;
      document.getElementById('ki').value = pid.ki;
      document.getElementById('kd').value = pid.kd;
    }

    async function loadSafety(){
      const res = await fetch('/safety');
      const s = await res.json();
      document.getElementById('estopState').textContent = s.estop ? 'ESTOP: ON' : 'ESTOP: OFF';
    }

    async function savePid(){
      const kp = document.getElementById('kp').value;
      const ki = document.getElementById('ki').value;
      const kd = document.getElementById('kd').value;
      await fetch(`/set_pid?kp=${encodeURIComponent(kp)}&ki=${encodeURIComponent(ki)}&kd=${encodeURIComponent(kd)}`);
    }

    async function setDir(name, val){
      await fetch(`/set?joint=${encodeURIComponent(name)}&dir=${val}`);
      document.getElementById(`val-${name}`).textContent = val;
    }

    async function setOffset(name, delta){
      const current = parseInt(document.getElementById(`off-${name}`).textContent || '0', 10);
      const next = current + delta;
      await fetch(`/set_offset?joint=${encodeURIComponent(name)}&offset=${encodeURIComponent(next)}`);
      document.getElementById(`off-${name}`).textContent = next;
    }

    async function loadImu(){
      try {
        const res = await fetch('/imu');
        const imu = await res.json();
        const roll = imu.roll || 0;
        const pitch = imu.pitch || 0;
        document.getElementById('imuRoll').textContent = roll.toFixed(1);
        document.getElementById('imuPitch').textContent = pitch.toFixed(1);
        const cube = document.getElementById('cube');
        // mirror view (like looking at a mirror)
        cube.style.transform = `rotateX(${pitch}deg) rotateZ(${roll}deg)`;
      } catch (e) {
        // ignore
      }
    }

    async function testJoint(name, sign){
      const deg = document.getElementById('degStep').value || 10;
      await fetch(`/test?joint=${encodeURIComponent(name)}&deg=${encodeURIComponent(deg)}&sign=${sign}`);
    }

    async function setEStop(on){
      await fetch(`/estop?on=${on ? 1 : 0}`);
      loadSafety();
    }

    load();
    loadPid();
    loadSafety();
    loadImu();
    setInterval(load, 2000);
    setInterval(loadSafety, 2000);
    setInterval(loadImu, 100);
  </script>
</body>
</html>
)HTML";

const char UPDATE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>OTA Update</title>
  <style>
    body{font-family:Arial,Helvetica,sans-serif;margin:20px;background:#0b0f14;color:#e6edf3}
    .card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:16px;max-width:480px}
    input[type=file]{width:100%;margin:8px 0}
    button{border:0;border-radius:6px;padding:8px 12px;cursor:pointer;background:#238636;color:#fff}
  </style>
</head>
<body>
  <h2>OTA Update</h2>
  <div class="card">
    <form method="POST" action="/update" enctype="multipart/form-data">
      <input type="file" name="update" accept=".bin" required />
      <button type="submit">Upload</button>
    </form>
    <div style="margin-top:8px" class="muted">อย่าปิดไฟระหว่างอัปโหลด</div>
  </div>
</body>
</html>
)HTML";

const char UPDATE_DONE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <meta http-equiv="refresh" content="3; url=/" />
  <title>OTA Done</title>
  <style>
    body{font-family:Arial,Helvetica,sans-serif;margin:20px;background:#0b0f14;color:#e6edf3}
    .card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:16px;max-width:480px}
  </style>
</head>
<body>
  <h2>Upload complete</h2>
  <div class="card">
    <div>กำลังรีสตาร์ต... จะกลับไปหน้าแรกอัตโนมัติ</div>
  </div>
</body>
</html>
)HTML";

void setupWifiAndWeb() {
  WiFi.mode(WIFI_STA);
  WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS1, DNS2);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", INDEX_HTML);
  });

  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", buildStateJson());
  });

  server.on("/pid", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", buildPidJson());
  });

  server.on("/safety", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", buildSafetyJson());
  });

  server.on("/offsets", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", buildOffsetsJson());
  });

  server.on("/imu", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", buildImuJson());
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("joint") || !request->hasParam("dir")) {
      request->send(400, "text/plain", "missing args");
      return;
    }
    String joint = request->getParam("joint")->value();
    int dir = request->getParam("dir")->value().toInt();
    if (dir != 1 && dir != -1) {
      request->send(400, "text/plain", "dir must be 1 or -1");
      return;
    }
    int idx = findDirIndex(joint);
    if (idx < 0) {
      request->send(404, "text/plain", "joint not found");
      return;
    }
    *DIR_PTRS[idx] = dir;
    request->send(200, "application/json", buildStateJson());
  });

  server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (ESTOP_ACTIVE) {
      request->send(423, "text/plain", "estop active");
      return;
    }
    unsigned long now = millis();
    if (now - lastCmdMs < MIN_CMD_INTERVAL_MS) {
      request->send(429, "text/plain", "rate limited");
      return;
    }
    if (!request->hasParam("joint")) {
      request->send(400, "text/plain", "missing joint");
      return;
    }
    String joint = request->getParam("joint")->value();
    int idx = findDirIndex(joint);
    if (idx < 0) {
      request->send(404, "text/plain", "joint not found");
      return;
    }
    int dir = *DIR_PTRS[idx];
    int sign = 1;
    int deg = 10;
    if (request->hasParam("sign")) {
      sign = request->getParam("sign")->value().toInt();
      if (sign != 1 && sign != -1) sign = 1;
    }
    if (request->hasParam("deg")) {
      deg = request->getParam("deg")->value().toInt();
      if (deg < 1) deg = 1;
      if (deg > MAX_DEG_PER_CMD) deg = MAX_DEG_PER_CMD;
    }
    uint8_t ch = 255;
    if (joint == "L_ANKLE_ROLL") ch = L_ANKLE_ROLL;
    else if (joint == "L_ANKLE_PITCH") ch = L_ANKLE_PITCH;
    else if (joint == "L_KNEE_PITCH") ch = L_KNEE_PITCH;
    else if (joint == "L_HIP_PITCH") ch = L_HIP_PITCH;
    else if (joint == "L_HIP_ROLL") ch = L_HIP_ROLL;
    else if (joint == "R_ANKLE_ROLL") ch = R_ANKLE_ROLL;
    else if (joint == "R_ANKLE_PITCH") ch = R_ANKLE_PITCH;
    else if (joint == "R_KNEE_PITCH") ch = R_KNEE_PITCH;
    else if (joint == "R_HIP_PITCH") ch = R_HIP_PITCH;
    else if (joint == "R_HIP_ROLL") ch = R_HIP_ROLL;

    if (ch == 255) {
      request->send(404, "text/plain", "channel not found");
      return;
    }
    lastCmdMs = now;
    bump(ch, dir * sign, deg);
    request->send(200, "text/plain", "ok");
  });

  server.on("/set_offset", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("joint") || !request->hasParam("offset")) {
      request->send(400, "text/plain", "missing args");
      return;
    }
    String joint = request->getParam("joint")->value();
    int offset = request->getParam("offset")->value().toInt();
    if (offset < OFFSET_MIN_DEG) offset = OFFSET_MIN_DEG;
    if (offset > OFFSET_MAX_DEG) offset = OFFSET_MAX_DEG;
    int idx = findDirIndex(joint);
    if (idx < 0) {
      request->send(404, "text/plain", "joint not found");
      return;
    }
    OFFSETS_DEG[idx] = offset;
    saveOffset(idx);
    request->send(200, "application/json", buildOffsetsJson());
  });

  server.on("/estop", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("on")) {
      request->send(400, "text/plain", "missing on");
      return;
    }
    bool on = request->getParam("on")->value().toInt() == 1;
    applyEmergencyStop(on);
    request->send(200, "application/json", buildSafetyJson());
  });

  server.on("/set_pid", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("kp") || !request->hasParam("ki") || !request->hasParam("kd")) {
      request->send(400, "text/plain", "missing args");
      return;
    }
    PID_KP = request->getParam("kp")->value().toFloat();
    PID_KI = request->getParam("ki")->value().toFloat();
    PID_KD = request->getParam("kd")->value().toFloat();
    request->send(200, "application/json", buildPidJson());
  });

  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", UPDATE_HTML);
  });

  server.on(
    "/update",
    HTTP_POST,
    [](AsyncWebServerRequest *request) {
      bool ok = !Update.hasError();
      request->send(ok ? 200 : 500, "text/html", ok ? UPDATE_DONE_HTML : "FAIL");
      if (ok) {
        delay(100);
        ESP.restart();
      }
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (!index) {
        applyEmergencyStop(true);
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
        }
      }
      if (!Update.hasError()) {
        if (Update.write(data, len) != len) {
          Update.printError(Serial);
        }
      }
      if (final) {
        if (!Update.end(true)) {
          Update.printError(Serial);
        }
      }
    }
  );

  server.begin();
}

// ===== helper =====
int degToUs(int deg) {
  long us = US_CENTER + (long)deg * 8; // ~8us/deg
  if (us < US_MIN) us = US_MIN;
  if (us > US_MAX) us = US_MAX;
  return us;
}

int getOffsetDegForChannel(uint8_t ch) {
  if (ch >= 10) return 0;
  int idx = CHANNEL_TO_DIR_IDX[ch];
  if (idx < 0 || idx >= DIR_COUNT) return 0;
  return OFFSETS_DEG[idx];
}

int centerUsForChannel(uint8_t ch) {
  int offset = getOffsetDegForChannel(ch);
  return degToUs(offset);
}

void delayWithWeb(unsigned long ms) {
  delay(ms);
}

void applyEmergencyStop(bool active) {
  ESTOP_ACTIVE = active;
  if (active) {
    pwm.writeMicroseconds(L_ANKLE_ROLL, centerUsForChannel(L_ANKLE_ROLL));
    pwm.writeMicroseconds(L_ANKLE_PITCH, centerUsForChannel(L_ANKLE_PITCH));
    pwm.writeMicroseconds(L_KNEE_PITCH, centerUsForChannel(L_KNEE_PITCH));
    pwm.writeMicroseconds(L_HIP_PITCH, centerUsForChannel(L_HIP_PITCH));
    pwm.writeMicroseconds(L_HIP_ROLL, centerUsForChannel(L_HIP_ROLL));
    pwm.writeMicroseconds(R_ANKLE_ROLL, centerUsForChannel(R_ANKLE_ROLL));
    pwm.writeMicroseconds(R_ANKLE_PITCH, centerUsForChannel(R_ANKLE_PITCH));
    pwm.writeMicroseconds(R_KNEE_PITCH, centerUsForChannel(R_KNEE_PITCH));
    pwm.writeMicroseconds(R_HIP_PITCH, centerUsForChannel(R_HIP_PITCH));
    pwm.writeMicroseconds(R_HIP_ROLL, centerUsForChannel(R_HIP_ROLL));
  }
}

void bump(uint8_t ch, int dir, int deg) {
  if (ESTOP_ACTIVE) return;
  int offset = getOffsetDegForChannel(ch);
  pwm.writeMicroseconds(ch, degToUs(offset + (dir * deg)));
  delayWithWeb(1200);
  pwm.writeMicroseconds(ch, centerUsForChannel(ch));
  delayWithWeb(800);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(300);

  setupImu();
  loadOffsets();

  setupWifiAndWeb();

  Serial.println("=== Direction Check : Legs + HIP_ROLL ===");
}

void loop() {
  updateImu();
  delay(1);
}

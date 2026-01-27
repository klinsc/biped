#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WebServer.h>

Adafruit_PWMServoDriver pwm(0x40);
#define SERVO_FREQ 50

// ===== WiFi =====
const char* WIFI_SSID = "Boomx_2.4G";
const char* WIFI_PASS = "11111111";
WebServer server(80);

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
int DIR_L_KNEE_PITCH  = +1;
int DIR_L_HIP_PITCH   = +1;
int DIR_L_HIP_ROLL    = +1;

int DIR_R_ANKLE_ROLL  = +1;
int DIR_R_ANKLE_PITCH = +1;
int DIR_R_KNEE_PITCH  = +1;
int DIR_R_HIP_PITCH   = +1;
int DIR_R_HIP_ROLL    = +1;

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

void bump(uint8_t ch, int dir, int deg = 10);

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
  <div id="testgrid" class="grid" style="margin-top:12px"></div>

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

  <script>
    const names = [
      "L_ANKLE_ROLL","L_ANKLE_PITCH","L_KNEE_PITCH","L_HIP_PITCH","L_HIP_ROLL",
      "R_ANKLE_ROLL","R_ANKLE_PITCH","R_KNEE_PITCH","R_HIP_PITCH","R_HIP_ROLL"
    ];

    const grid = document.getElementById('grid');

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

    async function load(){
      const res = await fetch('/state');
      const data = await res.json();
      grid.innerHTML='';
      names.forEach(n => grid.appendChild(card(n, data[n])));
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

    async function testJoint(name, sign){
      const deg = document.getElementById('degStep').value || 10;
      await fetch(`/test?joint=${encodeURIComponent(name)}&deg=${encodeURIComponent(deg)}&sign=${sign}`);
    }

    load();
    loadPid();
    setInterval(load, 2000);
  </script>
</body>
</html>
)HTML";

void setupWifiAndWeb() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/state", []() {
    server.send(200, "application/json", buildStateJson());
  });

  server.on("/pid", []() {
    server.send(200, "application/json", buildPidJson());
  });

  server.on("/set", []() {
    if (!server.hasArg("joint") || !server.hasArg("dir")) {
      server.send(400, "text/plain", "missing args");
      return;
    }
    String joint = server.arg("joint");
    int dir = server.arg("dir").toInt();
    if (dir != 1 && dir != -1) {
      server.send(400, "text/plain", "dir must be 1 or -1");
      return;
    }
    int idx = findDirIndex(joint);
    if (idx < 0) {
      server.send(404, "text/plain", "joint not found");
      return;
    }
    *DIR_PTRS[idx] = dir;
    server.send(200, "application/json", buildStateJson());
  });

  server.on("/test", []() {
    if (!server.hasArg("joint")) {
      server.send(400, "text/plain", "missing joint");
      return;
    }
    String joint = server.arg("joint");
    int idx = findDirIndex(joint);
    if (idx < 0) {
      server.send(404, "text/plain", "joint not found");
      return;
    }
    int dir = *DIR_PTRS[idx];
    int sign = 1;
    int deg = 10;
    if (server.hasArg("sign")) {
      sign = server.arg("sign").toInt();
      if (sign != 1 && sign != -1) sign = 1;
    }
    if (server.hasArg("deg")) {
      deg = server.arg("deg").toInt();
      if (deg < 1) deg = 1;
      if (deg > 45) deg = 45;
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
      server.send(404, "text/plain", "channel not found");
      return;
    }
    bump(ch, dir * sign, deg);
    server.send(200, "text/plain", "ok");
  });

  server.on("/set_pid", []() {
    if (!server.hasArg("kp") || !server.hasArg("ki") || !server.hasArg("kd")) {
      server.send(400, "text/plain", "missing args");
      return;
    }
    PID_KP = server.arg("kp").toFloat();
    PID_KI = server.arg("ki").toFloat();
    PID_KD = server.arg("kd").toFloat();
    server.send(200, "application/json", buildPidJson());
  });

  server.begin();
}

// ===== helper =====
int degToUs(int deg) {
  long us = US_CENTER + (long)deg * 8; // ~8us/deg
  if (us < US_MIN) us = US_MIN;
  if (us > US_MAX) us = US_MAX;
  return us;
}

void delayWithWeb(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    server.handleClient();
    delay(5);
  }
}

void bump(uint8_t ch, int dir, int deg) {
  pwm.writeMicroseconds(ch, degToUs(dir * deg));
  delayWithWeb(1200);
  pwm.writeMicroseconds(ch, US_CENTER);
  delayWithWeb(800);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(300);

  setupWifiAndWeb();

  Serial.println("=== Direction Check : Legs + HIP_ROLL ===");
}

void loop() {
  server.handleClient();
  delay(5);
}

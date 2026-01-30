#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <Update.h>
#include <freertos/portmacro.h>
#include "WebHandler.h"
#include "RobotState.h"
#include "RobotJoints.h"

// ===== WiFi =====
static const char* WIFI_SSID = "Boomx_2.4G";
static const char* WIFI_PASS = "11111111";
static const IPAddress STATIC_IP(192, 168, 1, 163);
static const IPAddress GATEWAY(192, 168, 1, 1);
static const IPAddress SUBNET(255, 255, 255, 0);
static const IPAddress DNS1(192, 168, 1, 1);
static const IPAddress DNS2(8, 8, 8, 8);

extern portMUX_TYPE dataMux;
extern RobotState globalState;

extern volatile bool IMU_READY;
extern unsigned long lastCmdMs;
extern unsigned long MIN_CMD_INTERVAL_MS;
extern int MAX_DEG_PER_CMD;
extern int OFFSET_MIN_DEG;
extern int OFFSET_MAX_DEG;

extern volatile float PID_KP;
extern volatile float PID_KI;
extern volatile float PID_KD;

extern int* DIR_PTRS[];
extern int OFFSETS_DEG[];

extern int findDirIndex(const String& name);
extern String buildStateJson();
extern String buildPidJson();
extern String buildSafetyJson();
extern String buildOffsetsJson();
extern String buildImuJson();

extern void saveDir(int idx);
extern void saveOffset(int idx);
extern void startTestMotion(uint8_t ch, int dir, int deg);
extern void applyEmergencyStop(bool active);
extern void resetPidState();

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
  <div class="row" style="margin-top:8px">
    <button id="btnTogglePid" class="neg" onclick="togglePid()">Disable PID</button>
    <span id="pidState" class="muted"></span>
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
    <button class="pos" id="btnCalibrateImu" onclick="calibrateImu()">Calibrate IMU</button>
    <span id="imuCalStatus" class="muted"></span>
  </div>
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
      document.getElementById('pidState').textContent = s.pid ? 'PID: ON' : 'PID: OFF';
      
      const btnPid = document.getElementById('btnTogglePid');
      if (s.pid) {
          btnPid.className = 'neg';
          btnPid.textContent = "Disable PID";
      } else {
          btnPid.className = 'pos';
          btnPid.textContent = "Enable PID";
      }
    }
    
    async function togglePid(){
      const res = await fetch('/toggle_pid');
      loadSafety();
    }

    async function savePid(){
      const kp = document.getElementById('kp').value;
      const ki = document.getElementById('ki').value;
      const kd = document.getElementById('kd').value;
      await fetch(`/set_pid?kp=${encodeURIComponent(kp)}&ki=${encodeURIComponent(ki)}&kd=${encodeURIComponent(kd)}`);
    }

    async function setDir(name, val){
      try {
        const res = await fetch(`/set?joint=${encodeURIComponent(name)}&dir=${val}`);
        if (!res.ok) throw new Error('fail');
        const data = await res.json();
        names.forEach(n => {
          const el = document.getElementById(`val-${n}`);
          if (el && data[n] != null) el.textContent = data[n];
        });
      } catch (e) {
        load();
      }
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

    async function calibrateImu(){
      const btn = document.getElementById('btnCalibrateImu');
      const status = document.getElementById('imuCalStatus');
      btn.disabled = true;
      status.textContent = 'Calibrating...';
      try {
        const res = await fetch('/calibrate_imu');
        if (!res.ok) throw new Error('fail');
        status.textContent = 'Done';
      } catch (e) {
        status.textContent = 'Failed';
      } finally {
        setTimeout(() => { status.textContent = ''; }, 1500);
        btn.disabled = false;
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

WebHandler::WebHandler() : server(80) {}

void WebHandler::begin() {
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

  server.on("/calibrate_imu", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!IMU_READY) {
      request->send(500, "text/plain", "imu not ready");
      return;
    }
    globalState.imuCalibrateRequested = true;
    request->send(200, "text/plain", "ok");
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
    portENTER_CRITICAL(&dataMux);
    *DIR_PTRS[idx] = dir;
    portEXIT_CRITICAL(&dataMux);
    saveDir(idx);
    request->send(200, "application/json", buildStateJson());
  });

  server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (globalState.estopActive) {
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
    if (globalState.currentAction != IDLE) {
      request->send(429, "text/plain", "busy");
      return;
    }
    lastCmdMs = now;
    startTestMotion(ch, dir * sign, deg);
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
    portENTER_CRITICAL(&dataMux);
    OFFSETS_DEG[idx] = offset;
    portEXIT_CRITICAL(&dataMux);
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

  server.on("/toggle_pid", HTTP_GET, [](AsyncWebServerRequest *request) {
    portENTER_CRITICAL(&dataMux);
    globalState.pidActive = !globalState.pidActive;
    portEXIT_CRITICAL(&dataMux);
    request->send(200, "application/json", buildSafetyJson());
  });

  server.on("/set_pid", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("kp") || !request->hasParam("ki") || !request->hasParam("kd")) {
      request->send(400, "text/plain", "missing args");
      return;
    }
    portENTER_CRITICAL(&dataMux);
    PID_KP = request->getParam("kp")->value().toFloat();
    PID_KI = request->getParam("ki")->value().toFloat();
    PID_KD = request->getParam("kd")->value().toFloat();
    portEXIT_CRITICAL(&dataMux);
    resetPidState();
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
        globalState.pendingRestart = true;
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

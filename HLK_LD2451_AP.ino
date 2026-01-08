#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// -------------------- Аппаратные настройки (можно менять) --------------------
static const uint32_t RADAR_BAUD = 115200;
static const int RADAR_RX_PIN = 16; // ESP32 RX2 <- TX радара
static const int RADAR_TX_PIN = 17; // ESP32 TX2 -> RX радара
static const int BUZZER_PIN = 25;   // Пищалка

// Если у вас пассивная пищалка (нужна частота) - true. Активная - false.
static const bool BUZZER_PASSIVE_DEFAULT = true;
static const bool BUZZER_ACTIVE_HIGH_DEFAULT = true;

static const char *AP_SSID = "LD2451_TEST";
static const char *AP_PASS = "12345678"; // минимум 8 символов

// -------------------- Протокол HLK-LD2451 --------------------
static const uint8_t FRAME_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t FRAME_TAIL[4] = {0xF8, 0xF7, 0xF6, 0xF5};
static const size_t RX_BUF_MAX = 512;

// -------------------- Конфигурация (сохраняется в NVS) --------------------
struct Config {
  // Фильтры
  bool laneFilterEnabled = true;
  bool angleFilterEnabled = false;
  float laneHalfWidthM = 0.8f;
  float laneCenterOffsetM = 0.0f;
  float angleCenterDeg = 0.0f;
  float angleHalfWidthDeg = 25.0f;

  // Выбор первичной цели
  bool primaryBySNR = false; // false = ближайшая

  // Ограничения целей
  float maxDistanceM = 10.0f;
  uint8_t minSNR = 8;

  // Лейн-лок (удержание цели)
  bool laneLockEnabled = true;
  float laneLockSeconds = 2.0f;
  float laneLockMaxDistDeltaM = 1.0f;
  float laneLockMaxAngleDeltaDeg = 8.0f;

  // Логика предупреждения
  bool useAlarmMode = true; // true = ALARM, false = TTC
  uint32_t debounceMs = 250;
  uint32_t cooldownMs = 1500;
  float ttcWarnS = 2.0f;
  float ttcCriticalS = 1.0f;
  float distWarnM = 3.0f;
  float distCriticalM = 1.5f;

  // Сглаживание
  float emaAlphaDistance = 0.35f; // 0..1
  float emaAlphaTTC = 0.35f;      // 0..1

  // Пищалка
  bool buzzerEnabled = true;
  bool buzzerPassive = BUZZER_PASSIVE_DEFAULT;
  bool buzzerActiveHigh = BUZZER_ACTIVE_HIGH_DEFAULT;
  uint16_t buzzerToneHz = 2600;
};

Config cfg;
Preferences prefs;

// -------------------- Состояние --------------------
struct Target {
  uint8_t angleRaw = 0;
  float angleDeg = 0.0f;
  float distanceM = 0.0f;
  int16_t speed = 0; // сырой
  uint8_t snr = 0;
  float yM = 0.0f;
};

static Target targets[16];
static uint8_t targetCount = 0;
static bool alarmFlag = false;

static uint32_t frameCount = 0;
static uint32_t errorCount = 0;

static uint8_t rxBuf[RX_BUF_MAX];
static size_t rxLen = 0;

static bool primaryValid = false;
static Target primaryTarget;
static uint32_t primaryLastMs = 0;
static float primaryLastDistance = 0.0f;
static float closingSpeed = 0.0f;
static float ttcRaw = -1.0f;
static float ttcEma = -1.0f;
static float distEma = -1.0f;

// Лейн-лок
static bool lockActive = false;
static Target lockTarget;
static uint32_t lockUntilMs = 0;

// Предупреждение
static bool warnActive = false;
static uint32_t warnConditionSinceMs = 0;
static uint32_t cooldownUntilMs = 0;
static uint32_t warnHoldUntilMs = 0;
static bool mute = false;

// Пищалка
static bool buzzerOn = false;
static uint32_t buzzerNextMs = 0;
static uint16_t buzzerPatternMs = 0;
static uint16_t buzzerSilenceMs = 0;

// Web
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// -------------------- Веб-страница --------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LD2451 Radar Tuner</title>
<style>
body{font-family:Arial, sans-serif;margin:0;padding:0;background:#0b0f14;color:#e6edf3}
header{padding:12px 16px;background:#121821;border-bottom:1px solid #1e293b}
main{display:grid;grid-template-columns:1fr 360px;gap:12px;padding:12px}
section{background:#0f172a;border:1px solid #1f2937;border-radius:8px;padding:12px}
label{display:block;margin:8px 0 2px}
input[type=range]{width:100%}
input,select,button{width:100%;padding:6px;border-radius:6px;border:1px solid #334155;background:#0b1220;color:#e2e8f0}
button{cursor:pointer}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.small{font-size:12px;color:#94a3b8}
#radar{width:100%;height:360px;background:#0b1220;border:1px solid #1f2937;border-radius:6px}
.tag{display:inline-block;padding:4px 8px;border-radius:999px;background:#1e293b;margin-right:6px}
</style>
</head>
<body>
<header>
  <strong>HLK-LD2451</strong> <span class="small">локальная точка доступа</span>
</header>
<main>
  <section>
    <canvas id="radar" width="600" height="360"></canvas>
    <div class="small" id="stats"></div>
    <div>
      <span class="tag" id="warnTag">warning: off</span>
      <span class="tag" id="modeTag">mode: ALARM</span>
      <span class="tag" id="primaryTag">primary: none</span>
    </div>
  </section>
  <section>
    <h3>Фильтры</h3>
    <label><input type="checkbox" id="laneFilter"> Lane filter</label>
    <label>Half width (m) <span id="laneHalfWidthV"></span></label>
    <input type="range" min="0.2" max="3" step="0.05" id="laneHalfWidth">
    <label>Center offset (m) <span id="laneCenterV"></span></label>
    <input type="range" min="-2" max="2" step="0.05" id="laneCenter">

    <label><input type="checkbox" id="angleFilter"> Angle filter</label>
    <label>Angle center (deg) <span id="angleCenterV"></span></label>
    <input type="range" min="-60" max="60" step="1" id="angleCenter">
    <label>Angle half width (deg) <span id="angleHalfV"></span></label>
    <input type="range" min="5" max="60" step="1" id="angleHalf">

    <label>Primary by SNR</label>
    <select id="primaryMode">
      <option value="0">Nearest</option>
      <option value="1">Strongest SNR</option>
    </select>

    <h3>Предупреждения</h3>
    <label>Mode</label>
    <select id="warnMode">
      <option value="1">ALARM</option>
      <option value="0">TTC</option>
    </select>
    <div class="grid2">
      <div>
        <label>Debounce (ms)</label>
        <input type="number" id="debounceMs" min="0" max="5000">
      </div>
      <div>
        <label>Cooldown (ms)</label>
        <input type="number" id="cooldownMs" min="0" max="10000">
      </div>
    </div>

    <div class="grid2">
      <div>
        <label>TTC warn (s)</label>
        <input type="number" id="ttcWarn" step="0.1">
      </div>
      <div>
        <label>TTC critical (s)</label>
        <input type="number" id="ttcCritical" step="0.1">
      </div>
    </div>

    <div class="grid2">
      <div>
        <label>Dist warn (m)</label>
        <input type="number" id="distWarn" step="0.1">
      </div>
      <div>
        <label>Dist critical (m)</label>
        <input type="number" id="distCritical" step="0.1">
      </div>
    </div>

    <h3>Ограничения / лейн-лок</h3>
    <div class="grid2">
      <div>
        <label>Max distance (m)</label>
        <input type="number" id="maxDistance" step="0.1">
      </div>
      <div>
        <label>Min SNR</label>
        <input type="number" id="minSnr" min="0" max="255">
      </div>
    </div>
    <label><input type="checkbox" id="laneLock"> Lane lock</label>
    <div class="grid2">
      <div>
        <label>Lock seconds</label>
        <input type="number" id="lockSeconds" step="0.1">
      </div>
      <div>
        <label>Lock Δdist (m)</label>
        <input type="number" id="lockDeltaDist" step="0.1">
      </div>
    </div>
    <label>Lock Δangle (deg)</label>
    <input type="number" id="lockDeltaAngle" step="0.5">

    <h3>Сглаживание</h3>
    <div class="grid2">
      <div>
        <label>EMA distance</label>
        <input type="number" id="emaDist" step="0.05" min="0" max="1">
      </div>
      <div>
        <label>EMA TTC</label>
        <input type="number" id="emaTtc" step="0.05" min="0" max="1">
      </div>
    </div>

    <h3>Пищалка</h3>
    <label><input type="checkbox" id="buzzerEnabled"> Enabled</label>
    <label><input type="checkbox" id="buzzerPassive"> Passive (tone)</label>
    <label>Tone Hz</label>
    <input type="number" id="buzzerTone" step="50" min="200" max="8000">

    <div class="grid2">
      <button id="btnMute">Mute</button>
      <button id="btnBeep">Test beep</button>
    </div>
    <div class="grid2">
      <button id="btnReset">Reset stats</button>
      <button id="btnSave">Save config</button>
    </div>
    <button id="btnLoad">Load config</button>
  </section>
</main>
<script>
const qs = (id) => document.getElementById(id);
const state = { targets: [] };
let ws;

function connectWS(){
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (ev)=>{
    const data = JSON.parse(ev.data);
    if(data.type === 'telemetry'){
      state.targets = data.targets || [];
      updateStats(data);
      drawRadar(data);
    }else if(data.type === 'config'){
      applyConfig(data.config);
    }
  };
}

function updateStats(d){
  qs('stats').innerText = `frames: ${d.frame_count}  errors: ${d.error_count}  fps: ${d.fps.toFixed(1)}`;
  qs('warnTag').innerText = `warning: ${d.warn_active ? 'ON' : 'off'}  ttc:${format(d.ttc)}  dist:${format(d.primary_distance)}`;
  qs('modeTag').innerText = `mode: ${d.mode}`;
  qs('primaryTag').innerText = d.primary_valid ? `primary: ${d.primary_distance.toFixed(2)}m @ ${d.primary_angle.toFixed(1)}°` : 'primary: none';
}

function format(v){
  if(v === null || v === undefined || v < 0) return '-';
  return v.toFixed(2);
}

function drawRadar(d){
  const c = qs('radar');
  const ctx = c.getContext('2d');
  ctx.clearRect(0,0,c.width,c.height);
  ctx.fillStyle = '#0b1220';
  ctx.fillRect(0,0,c.width,c.height);

  const maxDist = Math.max(d.max_distance || 10, 1);
  const scaleX = (c.width - 40) / maxDist;
  const scaleY = (c.height - 40) / (d.lane_half_width*2 + 2);

  // lane corridor
  const laneCenter = d.lane_center;
  const laneHalf = d.lane_half_width;
  ctx.strokeStyle = '#334155';
  ctx.strokeRect(20, 20 + (c.height/2 - (laneCenter + laneHalf)*scaleY),
    maxDist * scaleX, (laneHalf*2)*scaleY);

  // axis
  ctx.strokeStyle = '#1f2937';
  ctx.beginPath();
  ctx.moveTo(20, c.height/2);
  ctx.lineTo(c.width-20, c.height/2);
  ctx.stroke();

  d.targets.forEach(t => {
    const x = 20 + t.distance * scaleX;
    const y = c.height/2 - (t.y_m) * scaleY;
    ctx.fillStyle = t.is_primary ? '#f59e0b' : '#38bdf8';
    ctx.beginPath();
    ctx.arc(x, y, t.is_primary ? 6 : 4, 0, Math.PI*2);
    ctx.fill();
  });
}

async function loadConfig(){
  const res = await fetch('/api/config');
  const data = await res.json();
  applyConfig(data);
}

function applyConfig(c){
  qs('laneFilter').checked = c.laneFilterEnabled;
  qs('angleFilter').checked = c.angleFilterEnabled;
  qs('laneHalfWidth').value = c.laneHalfWidthM;
  qs('laneCenter').value = c.laneCenterOffsetM;
  qs('angleCenter').value = c.angleCenterDeg;
  qs('angleHalf').value = c.angleHalfWidthDeg;
  qs('primaryMode').value = c.primaryBySNR ? '1' : '0';
  qs('warnMode').value = c.useAlarmMode ? '1' : '0';
  qs('debounceMs').value = c.debounceMs;
  qs('cooldownMs').value = c.cooldownMs;
  qs('ttcWarn').value = c.ttcWarnS;
  qs('ttcCritical').value = c.ttcCriticalS;
  qs('distWarn').value = c.distWarnM;
  qs('distCritical').value = c.distCriticalM;
  qs('maxDistance').value = c.maxDistanceM;
  qs('minSnr').value = c.minSNR;
  qs('laneLock').checked = c.laneLockEnabled;
  qs('lockSeconds').value = c.laneLockSeconds;
  qs('lockDeltaDist').value = c.laneLockMaxDistDeltaM;
  qs('lockDeltaAngle').value = c.laneLockMaxAngleDeltaDeg;
  qs('emaDist').value = c.emaAlphaDistance;
  qs('emaTtc').value = c.emaAlphaTTC;
  qs('buzzerEnabled').checked = c.buzzerEnabled;
  qs('buzzerPassive').checked = c.buzzerPassive;
  qs('buzzerTone').value = c.buzzerToneHz;
  updateLabels();
}

function gatherConfig(){
  return {
    laneFilterEnabled: qs('laneFilter').checked,
    angleFilterEnabled: qs('angleFilter').checked,
    laneHalfWidthM: parseFloat(qs('laneHalfWidth').value),
    laneCenterOffsetM: parseFloat(qs('laneCenter').value),
    angleCenterDeg: parseFloat(qs('angleCenter').value),
    angleHalfWidthDeg: parseFloat(qs('angleHalf').value),
    primaryBySNR: qs('primaryMode').value === '1',
    useAlarmMode: qs('warnMode').value === '1',
    debounceMs: parseInt(qs('debounceMs').value),
    cooldownMs: parseInt(qs('cooldownMs').value),
    ttcWarnS: parseFloat(qs('ttcWarn').value),
    ttcCriticalS: parseFloat(qs('ttcCritical').value),
    distWarnM: parseFloat(qs('distWarn').value),
    distCriticalM: parseFloat(qs('distCritical').value),
    maxDistanceM: parseFloat(qs('maxDistance').value),
    minSNR: parseInt(qs('minSnr').value),
    laneLockEnabled: qs('laneLock').checked,
    laneLockSeconds: parseFloat(qs('lockSeconds').value),
    laneLockMaxDistDeltaM: parseFloat(qs('lockDeltaDist').value),
    laneLockMaxAngleDeltaDeg: parseFloat(qs('lockDeltaAngle').value),
    emaAlphaDistance: parseFloat(qs('emaDist').value),
    emaAlphaTTC: parseFloat(qs('emaTtc').value),
    buzzerEnabled: qs('buzzerEnabled').checked,
    buzzerPassive: qs('buzzerPassive').checked,
    buzzerToneHz: parseInt(qs('buzzerTone').value)
  };
}

function updateLabels(){
  qs('laneHalfWidthV').innerText = qs('laneHalfWidth').value;
  qs('laneCenterV').innerText = qs('laneCenter').value;
  qs('angleCenterV').innerText = qs('angleCenter').value;
  qs('angleHalfV').innerText = qs('angleHalf').value;
}

qs('laneHalfWidth').oninput = updateLabels;
qs('laneCenter').oninput = updateLabels;
qs('angleCenter').oninput = updateLabels;
qs('angleHalf').oninput = updateLabels;

qs('btnSave').onclick = async ()=>{
  await fetch('/api/config', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(gatherConfig())});
};
qs('btnLoad').onclick = ()=> loadConfig();
qs('btnReset').onclick = ()=> fetch('/api/reset');
qs('btnBeep').onclick = ()=> fetch('/api/beep');
qs('btnMute').onclick = ()=> fetch('/api/mute');

connectWS();
loadConfig();
</script>
</body>
</html>
)HTML";

// -------------------- Утилиты --------------------
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static float toRadians(float deg) {
  return deg * 0.0174532925f;
}

#if defined(ESP_ARDUINO_VERSION_MAJOR)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void ledcAttachCompat() {
  ledcAttach(BUZZER_PIN, cfg.buzzerToneHz, 8);
}

static void ledcDetachCompat() {
  ledcDetach(BUZZER_PIN);
}

static void ledcWriteToneCompat(uint32_t freq) {
  ledcWriteTone(BUZZER_PIN, freq);
}
#else
static const uint8_t BUZZER_LEDC_CHANNEL = 0;
static void ledcAttachCompat() {
  ledcSetup(BUZZER_LEDC_CHANNEL, cfg.buzzerToneHz, 8);
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
}

static void ledcDetachCompat() {
  ledcDetachPin(BUZZER_PIN);
}

static void ledcWriteToneCompat(uint32_t freq) {
  ledcWriteTone(BUZZER_LEDC_CHANNEL, freq);
}
#endif
#else
// Если версия ядра не определена, пробуем новый API (ESP32 core 3.x).
static void ledcAttachCompat() {
  ledcAttach(BUZZER_PIN, cfg.buzzerToneHz, 8);
}

static void ledcDetachCompat() {
  ledcDetach(BUZZER_PIN);
}

static void ledcWriteToneCompat(uint32_t freq) {
  ledcWriteTone(BUZZER_PIN, freq);
}
#endif

static void buzzerWrite(bool on) {
  if (!cfg.buzzerEnabled || mute) {
    if (cfg.buzzerPassive) {
      ledcWriteToneCompat(0);
    } else {
      digitalWrite(BUZZER_PIN, cfg.buzzerActiveHigh ? LOW : HIGH);
    }
    return;
  }
  if (cfg.buzzerPassive) {
    if (on) {
      ledcWriteToneCompat(cfg.buzzerToneHz);
    } else {
      ledcWriteToneCompat(0);
    }
  } else {
    digitalWrite(BUZZER_PIN, on ? (cfg.buzzerActiveHigh ? HIGH : LOW)
                               : (cfg.buzzerActiveHigh ? LOW : HIGH));
  }
}

static void applyBuzzerConfig() {
  pinMode(BUZZER_PIN, OUTPUT);
  if (cfg.buzzerPassive) {
    ledcAttachCompat();
    ledcWriteToneCompat(0);
  } else {
    ledcDetachCompat();
    digitalWrite(BUZZER_PIN, cfg.buzzerActiveHigh ? LOW : HIGH);
  }
}

static void setBuzzerPattern(uint16_t onMs, uint16_t offMs) {
  buzzerPatternMs = onMs;
  buzzerSilenceMs = offMs;
  buzzerOn = false;
  buzzerNextMs = millis();
}

static void updateBuzzer() {
  if (!cfg.buzzerEnabled || mute) {
    buzzerWrite(false);
    return;
  }
  uint32_t now = millis();
  if (now < buzzerNextMs) return;

  if (buzzerPatternMs == 0 && buzzerSilenceMs == 0) {
    buzzerWrite(false);
    return;
  }

  if (buzzerPatternMs > 0 && buzzerSilenceMs == 0) {
    // непрерывный тон
    buzzerWrite(true);
    return;
  }

  buzzerOn = !buzzerOn;
  buzzerWrite(buzzerOn);
  buzzerNextMs = now + (buzzerOn ? buzzerPatternMs : buzzerSilenceMs);
}

// -------------------- Парсер кадров --------------------
static int findHeader(const uint8_t *buf, size_t len) {
  for (size_t i = 0; i + 3 < len; ++i) {
    if (buf[i] == FRAME_HEADER[0] && buf[i + 1] == FRAME_HEADER[1] &&
        buf[i + 2] == FRAME_HEADER[2] && buf[i + 3] == FRAME_HEADER[3]) {
      return (int)i;
    }
  }
  return -1;
}

static bool parseFrame(const uint8_t *frame, size_t totalLen) {
  // frame = header(4) + len(2) + payload + tail(4)
  if (totalLen < 10) return false;
  if (memcmp(frame, FRAME_HEADER, 4) != 0) return false;
  if (memcmp(frame + totalLen - 4, FRAME_TAIL, 4) != 0) return false;

  uint16_t payloadLen = frame[4] | (frame[5] << 8); // little-endian
  if (payloadLen + 10 != totalLen) return false;

  const uint8_t *payload = frame + 6;
  if (payloadLen < 2) return false;

  alarmFlag = payload[0] == 0x01;
  uint8_t count = payload[1];

  size_t expected = 2 + count * 5;
  if (expected > payloadLen) {
    return false;
  }

  targetCount = 0;
  for (uint8_t i = 0; i < count && i < 16; ++i) {
    size_t idx = 2 + i * 5;
    uint8_t angleRaw = payload[idx];
    uint8_t dist = payload[idx + 1];
    int16_t speed = (int16_t)(payload[idx + 2] | (payload[idx + 3] << 8));
    uint8_t snr = payload[idx + 4];

    Target t;
    t.angleRaw = angleRaw;
    t.angleDeg = (float)angleRaw - 128.0f;
    t.distanceM = (float)dist;
    t.speed = speed;
    t.snr = snr;
    t.yM = t.distanceM * tanf(toRadians(t.angleDeg));
    targets[targetCount++] = t;
  }

  return true;
}

static void processRx() {
  while (Serial2.available()) {
    if (rxLen >= RX_BUF_MAX) {
      rxLen = 0; // переполнение
      errorCount++;
    }
    rxBuf[rxLen++] = (uint8_t)Serial2.read();
  }

  while (rxLen >= 10) {
    int hdrIdx = findHeader(rxBuf, rxLen);
    if (hdrIdx < 0) {
      // оставим последние 3 байта на случай частичного заголовка
      if (rxLen > 3) {
        memmove(rxBuf, rxBuf + rxLen - 3, 3);
        rxLen = 3;
      }
      break;
    }
    if (hdrIdx > 0) {
      memmove(rxBuf, rxBuf + hdrIdx, rxLen - hdrIdx);
      rxLen -= hdrIdx;
    }

    if (rxLen < 6) break; // ждём длину

    uint16_t payloadLen = rxBuf[4] | (rxBuf[5] << 8);
    size_t totalLen = 4 + 2 + payloadLen + 4;
    if (totalLen > RX_BUF_MAX || payloadLen > 256) {
      // подозрительная длина
      memmove(rxBuf, rxBuf + 1, rxLen - 1);
      rxLen -= 1;
      errorCount++;
      continue;
    }

    if (rxLen < totalLen) break;

    if (parseFrame(rxBuf, totalLen)) {
      frameCount++;
    } else {
      errorCount++;
    }

    memmove(rxBuf, rxBuf + totalLen, rxLen - totalLen);
    rxLen -= totalLen;
  }
}

// Небольшой тест: прогоняем пример байтов
static void testParser() {
  uint8_t sample[] = {
      0xF4, 0xF3, 0xF2, 0xF1, // header
      0x07, 0x00,             // payload len = 7
      0x01,                   // alarm
      0x01,                   // 1 target
      0x80,                   // angle_raw (0 deg)
      0x05,                   // distance 5 m
      0x10, 0x00,             // speed
      0x20,                   // snr
      0xF8, 0xF7, 0xF6, 0xF5  // tail
  };

  if (parseFrame(sample, sizeof(sample))) {
    Serial.println("[TEST] Parser OK, target count: " + String(targetCount));
  } else {
    Serial.println("[TEST] Parser FAIL");
  }
}

// -------------------- Логика выбора цели --------------------
static bool targetPassFilters(const Target &t) {
  if (t.distanceM <= 0.0f || t.distanceM > cfg.maxDistanceM) return false;
  if (t.snr < cfg.minSNR) return false;

  bool ok = true;
  if (cfg.laneFilterEnabled) {
    float dy = fabsf(t.yM - cfg.laneCenterOffsetM);
    ok &= dy <= cfg.laneHalfWidthM;
  }
  if (cfg.angleFilterEnabled) {
    float da = fabsf(t.angleDeg - cfg.angleCenterDeg);
    ok &= da <= cfg.angleHalfWidthDeg;
  }
  return ok;
}

static bool choosePrimary(Target &out) {
  Target best;
  bool found = false;

  if (cfg.laneLockEnabled && lockActive && millis() < lockUntilMs) {
    // ищем ближайшего к предыдущей цели
    float bestScore = 1e9f;
    for (uint8_t i = 0; i < targetCount; ++i) {
      const Target &t = targets[i];
      if (!targetPassFilters(t)) continue;
      float dDist = t.distanceM - lockTarget.distanceM;
      float dAng = t.angleDeg - lockTarget.angleDeg;
      if (fabsf(dDist) > cfg.laneLockMaxDistDeltaM) continue;
      if (fabsf(dAng) > cfg.laneLockMaxAngleDeltaDeg) continue;
      float score = dDist * dDist + dAng * dAng;
      if (score < bestScore) {
        bestScore = score;
        best = t;
        found = true;
      }
    }
    if (found) {
      lockTarget = best;
      lockUntilMs = millis() + (uint32_t)(cfg.laneLockSeconds * 1000.0f);
      out = best;
      return true;
    }
  }

  for (uint8_t i = 0; i < targetCount; ++i) {
    const Target &t = targets[i];
    if (!targetPassFilters(t)) continue;
    if (!found) {
      best = t;
      found = true;
      continue;
    }
    if (cfg.primaryBySNR) {
      if (t.snr > best.snr) best = t;
    } else {
      if (t.distanceM < best.distanceM) best = t;
    }
  }

  if (found && cfg.laneLockEnabled) {
    lockActive = true;
    lockTarget = best;
    lockUntilMs = millis() + (uint32_t)(cfg.laneLockSeconds * 1000.0f);
  } else if (!found) {
    lockActive = false;
  }

  if (found) out = best;
  return found;
}

// -------------------- Предупреждение / TTC --------------------
static void updatePrimaryMetrics() {
  if (!primaryValid) {
    ttcRaw = -1.0f;
    closingSpeed = 0.0f;
    distEma = -1.0f;
    ttcEma = -1.0f;
    return;
  }

  uint32_t now = millis();
  float dt = (now - primaryLastMs) / 1000.0f;
  if (dt > 0.05f) {
    closingSpeed = (primaryLastDistance - primaryTarget.distanceM) / dt;
    primaryLastDistance = primaryTarget.distanceM;
    primaryLastMs = now;
    if (closingSpeed > 0.05f) {
      ttcRaw = primaryTarget.distanceM / closingSpeed;
    } else {
      ttcRaw = -1.0f;
    }
  }

  // сглаживание
  if (distEma < 0) distEma = primaryTarget.distanceM;
  distEma = cfg.emaAlphaDistance * primaryTarget.distanceM + (1.0f - cfg.emaAlphaDistance) * distEma;

  if (ttcRaw > 0) {
    if (ttcEma < 0) ttcEma = ttcRaw;
    ttcEma = cfg.emaAlphaTTC * ttcRaw + (1.0f - cfg.emaAlphaTTC) * ttcEma;
  }
}

static bool evaluateWarningCondition(uint8_t &severity) {
  severity = 0;
  if (cfg.useAlarmMode) {
    if (alarmFlag) {
      severity = 2; // в ALARM режиме просто сильное предупреждение
      return true;
    }
    return false;
  }

  if (!primaryValid) return false;

  float useDist = distEma > 0 ? distEma : primaryTarget.distanceM;
  float useTtc = ttcEma > 0 ? ttcEma : ttcRaw;

  bool warn = false;
  if (useDist > 0 && useDist <= cfg.distWarnM) warn = true;
  if (useTtc > 0 && useTtc <= cfg.ttcWarnS) warn = true;

  bool critical = false;
  if (useDist > 0 && useDist <= cfg.distCriticalM) critical = true;
  if (useTtc > 0 && useTtc <= cfg.ttcCriticalS) critical = true;

  if (critical) severity = 2;
  else if (warn) severity = 1;

  return warn || critical;
}

static void updateWarning() {
  uint8_t severity = 0;
  bool condition = evaluateWarningCondition(severity);
  uint32_t now = millis();

  if (now < warnHoldUntilMs) {
    warnActive = true;
    return; // держим текущий паттерн
  }

  if (now < cooldownUntilMs) {
    warnActive = false;
    setBuzzerPattern(0, 0);
    return;
  }

  if (condition) {
    if (warnConditionSinceMs == 0) warnConditionSinceMs = now;
    if (now - warnConditionSinceMs >= cfg.debounceMs) {
      warnActive = true;
      // паттерны
      if (severity >= 2) {
        setBuzzerPattern(0, 0); // непрерывный тон
      } else {
        setBuzzerPattern(120, 180);
      }
      warnHoldUntilMs = now + 600;
      cooldownUntilMs = now + cfg.cooldownMs;
      warnConditionSinceMs = 0;
      return;
    }
  } else {
    warnConditionSinceMs = 0;
  }

  warnActive = false;
  setBuzzerPattern(0, 0);
}

// -------------------- Конфиг --------------------
static void saveConfig() {
  prefs.putBytes("config", &cfg, sizeof(cfg));
}

static void loadConfig() {
  size_t len = prefs.getBytesLength("config");
  if (len == sizeof(cfg)) {
    prefs.getBytes("config", &cfg, sizeof(cfg));
  }
}

static String configToJson() {
  StaticJsonDocument<768> doc;
  doc["laneFilterEnabled"] = cfg.laneFilterEnabled;
  doc["angleFilterEnabled"] = cfg.angleFilterEnabled;
  doc["laneHalfWidthM"] = cfg.laneHalfWidthM;
  doc["laneCenterOffsetM"] = cfg.laneCenterOffsetM;
  doc["angleCenterDeg"] = cfg.angleCenterDeg;
  doc["angleHalfWidthDeg"] = cfg.angleHalfWidthDeg;
  doc["primaryBySNR"] = cfg.primaryBySNR;
  doc["maxDistanceM"] = cfg.maxDistanceM;
  doc["minSNR"] = cfg.minSNR;
  doc["laneLockEnabled"] = cfg.laneLockEnabled;
  doc["laneLockSeconds"] = cfg.laneLockSeconds;
  doc["laneLockMaxDistDeltaM"] = cfg.laneLockMaxDistDeltaM;
  doc["laneLockMaxAngleDeltaDeg"] = cfg.laneLockMaxAngleDeltaDeg;
  doc["useAlarmMode"] = cfg.useAlarmMode;
  doc["debounceMs"] = cfg.debounceMs;
  doc["cooldownMs"] = cfg.cooldownMs;
  doc["ttcWarnS"] = cfg.ttcWarnS;
  doc["ttcCriticalS"] = cfg.ttcCriticalS;
  doc["distWarnM"] = cfg.distWarnM;
  doc["distCriticalM"] = cfg.distCriticalM;
  doc["emaAlphaDistance"] = cfg.emaAlphaDistance;
  doc["emaAlphaTTC"] = cfg.emaAlphaTTC;
  doc["buzzerEnabled"] = cfg.buzzerEnabled;
  doc["buzzerPassive"] = cfg.buzzerPassive;
  doc["buzzerActiveHigh"] = cfg.buzzerActiveHigh;
  doc["buzzerToneHz"] = cfg.buzzerToneHz;

  String out;
  serializeJson(doc, out);
  return out;
}

static void applyConfigFromJson(const String &body) {
  StaticJsonDocument<768> doc;
  if (deserializeJson(doc, body) != DeserializationError::Ok) return;
  cfg.laneFilterEnabled = doc["laneFilterEnabled"] | cfg.laneFilterEnabled;
  cfg.angleFilterEnabled = doc["angleFilterEnabled"] | cfg.angleFilterEnabled;
  cfg.laneHalfWidthM = doc["laneHalfWidthM"] | cfg.laneHalfWidthM;
  cfg.laneCenterOffsetM = doc["laneCenterOffsetM"] | cfg.laneCenterOffsetM;
  cfg.angleCenterDeg = doc["angleCenterDeg"] | cfg.angleCenterDeg;
  cfg.angleHalfWidthDeg = doc["angleHalfWidthDeg"] | cfg.angleHalfWidthDeg;
  cfg.primaryBySNR = doc["primaryBySNR"] | cfg.primaryBySNR;
  cfg.maxDistanceM = doc["maxDistanceM"] | cfg.maxDistanceM;
  cfg.minSNR = doc["minSNR"] | cfg.minSNR;
  cfg.laneLockEnabled = doc["laneLockEnabled"] | cfg.laneLockEnabled;
  cfg.laneLockSeconds = doc["laneLockSeconds"] | cfg.laneLockSeconds;
  cfg.laneLockMaxDistDeltaM = doc["laneLockMaxDistDeltaM"] | cfg.laneLockMaxDistDeltaM;
  cfg.laneLockMaxAngleDeltaDeg = doc["laneLockMaxAngleDeltaDeg"] | cfg.laneLockMaxAngleDeltaDeg;
  cfg.useAlarmMode = doc["useAlarmMode"] | cfg.useAlarmMode;
  cfg.debounceMs = doc["debounceMs"] | cfg.debounceMs;
  cfg.cooldownMs = doc["cooldownMs"] | cfg.cooldownMs;
  cfg.ttcWarnS = doc["ttcWarnS"] | cfg.ttcWarnS;
  cfg.ttcCriticalS = doc["ttcCriticalS"] | cfg.ttcCriticalS;
  cfg.distWarnM = doc["distWarnM"] | cfg.distWarnM;
  cfg.distCriticalM = doc["distCriticalM"] | cfg.distCriticalM;
  cfg.emaAlphaDistance = clampf(doc["emaAlphaDistance"] | cfg.emaAlphaDistance, 0.0f, 1.0f);
  cfg.emaAlphaTTC = clampf(doc["emaAlphaTTC"] | cfg.emaAlphaTTC, 0.0f, 1.0f);
  cfg.buzzerEnabled = doc["buzzerEnabled"] | cfg.buzzerEnabled;
  cfg.buzzerPassive = doc["buzzerPassive"] | cfg.buzzerPassive;
  cfg.buzzerActiveHigh = doc["buzzerActiveHigh"] | cfg.buzzerActiveHigh;
  cfg.buzzerToneHz = doc["buzzerToneHz"] | cfg.buzzerToneHz;
  applyBuzzerConfig();
}

// -------------------- WebSocket / API --------------------
static void notifyConfig(AsyncWebSocketClient *client = nullptr) {
  String json = "{\"type\":\"config\",\"config\":" + configToJson() + "}";
  if (client) {
    client->text(json);
  } else {
    ws.textAll(json);
  }
}

static void notifyTelemetry(float fps) {
  StaticJsonDocument<1536> doc;
  doc["type"] = "telemetry";
  doc["alarm"] = alarmFlag;
  doc["frame_count"] = frameCount;
  doc["error_count"] = errorCount;
  doc["fps"] = fps;
  doc["warn_active"] = warnActive;
  doc["mode"] = cfg.useAlarmMode ? "ALARM" : "TTC";
  doc["primary_valid"] = primaryValid;
  doc["primary_distance"] = primaryValid ? primaryTarget.distanceM : -1;
  doc["primary_angle"] = primaryValid ? primaryTarget.angleDeg : 0;
  doc["primary_snr"] = primaryValid ? primaryTarget.snr : 0;
  doc["primary_y"] = primaryValid ? primaryTarget.yM : 0;
  doc["closing_speed"] = closingSpeed;
  doc["ttc"] = ttcEma > 0 ? ttcEma : ttcRaw;
  doc["lane_half_width"] = cfg.laneHalfWidthM;
  doc["lane_center"] = cfg.laneCenterOffsetM;
  doc["max_distance"] = cfg.maxDistanceM;

  JsonArray arr = doc.createNestedArray("targets");
  for (uint8_t i = 0; i < targetCount; ++i) {
    JsonObject t = arr.createNestedObject();
    t["angle"] = targets[i].angleDeg;
    t["distance"] = targets[i].distanceM;
    t["speed"] = targets[i].speed;
    t["snr"] = targets[i].snr;
    t["y_m"] = targets[i].yM;
    bool isPrimary = primaryValid && fabsf(targets[i].distanceM - primaryTarget.distanceM) < 0.01f &&
                     fabsf(targets[i].angleDeg - primaryTarget.angleDeg) < 0.1f;
    t["is_primary"] = isPrimary;
  }

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

static void setupServer() {
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      notifyConfig(client);
    }
  });
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });

  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", configToJson());
  });

  server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr,
            [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t, size_t) {
              String body;
              body.reserve(len);
              for (size_t i = 0; i < len; ++i) body += (char)data[i];
              applyConfigFromJson(body);
              saveConfig();
              request->send(200, "application/json", configToJson());
              notifyConfig();
            });

  server.on("/api/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
    frameCount = 0;
    errorCount = 0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/api/beep", HTTP_GET, [](AsyncWebServerRequest *request) {
    setBuzzerPattern(120, 120);
    buzzerNextMs = millis();
    request->send(200, "text/plain", "OK");
  });

  server.on("/api/mute", HTTP_GET, [](AsyncWebServerRequest *request) {
    mute = !mute;
    request->send(200, "text/plain", mute ? "MUTED" : "UNMUTED");
  });

  server.begin();
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);
  Serial.println("HLK-LD2451 radar starting...");

  Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  prefs.begin("ld2451", false);
  loadConfig();
  applyBuzzerConfig();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println("AP IP: " + WiFi.softAPIP().toString());

  setupServer();

  // тест парсера (можно закомментировать)
  testParser();

  primaryLastMs = millis();
  primaryLastDistance = 0.0f;
}

void loop() {
  processRx();

  Target chosen;
  primaryValid = choosePrimary(chosen);
  if (primaryValid) {
    primaryTarget = chosen;
  }

  updatePrimaryMetrics();
  updateWarning();
  updateBuzzer();

  // телеметрия ~10 Гц
  static uint32_t lastTelemetryMs = 0;
  static uint32_t lastFrameCount = 0;
  uint32_t now = millis();
  if (now - lastTelemetryMs >= 100) {
    float fps = (frameCount - lastFrameCount) / ((now - lastTelemetryMs) / 1000.0f + 0.0001f);
    lastFrameCount = frameCount;
    lastTelemetryMs = now;
    notifyTelemetry(fps);
  }

  ws.cleanupClients();
}

/*
===== Краткая инструкция для новичка =====
1) Установите библиотеки: ESPAsyncWebServer, AsyncTCP, ArduinoJson.
2) Подключите радар к RX2/TX2 (3.3V TTL), пищалку к BUZZER_PIN.
3) Залейте скетч, найдите Wi-Fi сеть LD2451_TEST (пароль 12345678).
4) Откройте браузер: http://192.168.4.1
5) Настройте фильтры: lane half width, angle gate, и т.д.

===== Безопасность =====
- Не используйте экран во время движения.
- Используйте устройство только как экспериментальный помощник.
*/

#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

#include "wifidata.h"
#include "whitelist.h"
#include "pins.h"

// Watchdog Timeout in Sekunden
#define WDT_TIMEOUT 15

// WS2812 Status-LED (falls in pins.h nicht definiert)
#ifndef WS2812_PIN
#define WS2812_PIN 16
#endif

// LED Helligkeit (0-255, gedimmt schont den 3,3V-Regler)
#define LED_BRIGHTNESS 25
#define LED_FLASH_MS   40

// RFM69 Konstanten
#define FREQUENCY_KHZ     868300
#define DATA_RATE         20000
#define REG_OPMODE        0x01
#define REG_IRQFLAGS2     0x28
#define REG_PAYLOADLENGTH 0x38

#define PAYLOAD_SIZE 47
#define FRAME_LENGTH 41
#define EPSILON      0.00001

// Web-Dashboard & Verlaufshistorie
#define MAX_IDS      32
#define HISTORY_LEN  30  // Letzte 30 Messwerte pro Gerät für SVG-Sparkline

bool logOnlyFailed = true;

unsigned long lastStatusPublish = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long ledOffTime = 0;
bool ledFlashActive = false;

uint8_t m_payload[64];
uint16_t reconcount = 0;

struct Frame {
  uint16_t ID;
  uint32_t TotalSeconds;
  uint32_t OnSeconds;
  double Consumption;
  float Power;
  float MaximumPower;
  uint16_t NumberOfResets;
  bool IsOn;
  uint8_t Reception;
  bool IsValid;
  uint16_t CRC;
};

struct Tracker {
  uint16_t ID;
  uint16_t LastResets;
  double LastConsumption;
  float LastPower;
  float MaxPower;
  uint32_t TotalSeconds;
  uint32_t OnSeconds;
  bool IsOn;
  float LastRSSI;
  bool Initialized;
  unsigned long LastSeen;
  uint16_t History[HISTORY_LEN]; // Power * 10 als Ganzzahl (spart RAM)
  uint8_t HistoryCount;
  uint8_t HistoryHead;
} trackers[MAX_IDS] = {0};

WiFiClient espClient;
PubSubClient client(espClient);
WebServer server(80);

// --- Modernes Web-Dashboard im Flash (PROGMEM -> 0 Bytes RAM-Verbrauch) ---
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>EC3000 Energy Bridge</title>
<style>
  :root {
    --bg: #0f172a; --card: #1e293b; --border: #334155;
    --text: #f8fafc; --muted: #94a3b8; --accent: #38bdf8;
    --green: #22c55e; --red: #ef4444;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: var(--bg); color: var(--text); padding: 1.25rem;
  }
  header {
    max-width: 1200px; margin: 0 auto 1.5rem auto;
    display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center;
    gap: 1rem; padding-bottom: 1rem; border-bottom: 1px solid var(--border);
  }
  h1 { font-size: 1.4rem; font-weight: 700; display: flex; align-items: center; gap: 0.6rem; }
  .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--green); box-shadow: 0 0 8px var(--green); }
  .dot.offline { background: var(--red); box-shadow: 0 0 8px var(--red); }
  .sys-pills { display: flex; flex-wrap: wrap; gap: 0.5rem; font-size: 0.8rem; color: var(--muted); }
  .pill { background: var(--card); border: 1px solid var(--border); padding: 0.35rem 0.7rem; border-radius: 999px; }
  .pill b { color: var(--text); font-weight: 600; }
  .grid {
    max-width: 1200px; margin: 0 auto;
    display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 1rem;
  }
  .card {
    background: var(--card); border: 1px solid var(--border); border-radius: 14px;
    padding: 1.2rem; display: flex; flex-direction: column; gap: 0.9rem;
    transition: opacity 0.3s ease;
  }
  .card.stale { opacity: 0.55; }
  .card-top { display: flex; justify-content: space-between; align-items: flex-start; gap: 0.5rem; }
  .dev-name { font-weight: 700; font-size: 1.1rem; color: var(--text); }
  .dev-id { font-family: monospace; font-size: 0.78rem; color: var(--accent); opacity: 0.85; }
  .badge {
    font-size: 0.72rem; font-weight: 700; padding: 0.25rem 0.65rem;
    border-radius: 999px; text-transform: uppercase; letter-spacing: 0.04em; white-space: nowrap;
  }
  .badge.on { background: rgba(34, 197, 94, 0.15); color: var(--green); border: 1px solid rgba(34, 197, 94, 0.4); }
  .badge.off { background: rgba(148, 163, 184, 0.12); color: var(--muted); border: 1px solid var(--border); }
  .power-row { display: flex; align-items: baseline; justify-content: space-between; }
  .power-val { font-size: 2.2rem; font-weight: 800; letter-spacing: -0.02em; }
  .power-unit { font-size: 1rem; color: var(--muted); margin-left: 0.2rem; }
  .max-val { font-size: 0.8rem; color: var(--muted); }
  .spark-wrap {
    width: 100%; height: 54px; background: rgba(15, 23, 42, 0.55);
    border-radius: 8px; padding: 4px; border: 1px solid rgba(51, 65, 85, 0.6);
  }
  svg { width: 100%; height: 100%; overflow: visible; }
  .stats {
    display: grid; grid-template-columns: repeat(3, 1fr); gap: 0.5rem;
    padding-top: 0.7rem; border-top: 1px solid var(--border); font-size: 0.8rem;
  }
  .stat span { display: block; color: var(--muted); font-size: 0.7rem; }
  .stat b { font-weight: 600; }
  .empty { text-align: center; color: var(--muted); padding: 3rem; grid-column: 1 / -1; }
</style>
</head>
<body>
<header>
  <h1><span id="connDot" class="dot"></span> EC3000 MQTT Bridge</h1>
  <div class="sys-pills">
    <div class="pill">Uptime: <b id="uptime">-</b></div>
    <div class="pill">WiFi: <b id="wifiRssi">-</b></div>
    <div class="pill">MQTT: <b id="mqttState">-</b></div>
    <div class="pill">Heap: <b id="heap">-</b></div>
    <div class="pill">Reset: <b id="resetReason">-</b></div>
  </div>
</header>
<main class="grid" id="grid">
  <div class="empty">Warte auf erste Funkpakete auf 868,3 MHz...</div>
</main>

<script>
function fmtDuration(sec) {
  const d = Math.floor(sec / 86400);
  const h = Math.floor((sec % 86400) / 3600);
  const m = Math.floor((sec % 3600) / 60);
  if (d > 0) return `${d}d ${h}h ${m}m`;
  if (h > 0) return `${h}h ${m}m`;
  return `${m}m ${sec % 60}s`;
}

function buildSparkline(vals, id) {
  if (!vals || vals.length < 2) {
    return `<svg viewBox="0 0 100 30"><line x1="0" y1="25" x2="100" y2="25" stroke="#334155" stroke-width="1.5" stroke-dasharray="3 3"/></svg>`;
  }
  const min = Math.min(...vals);
  const max = Math.max(...vals);
  const range = (max - min) || 1;
  const w = 240, h = 44, pad = 3;
  const pts = vals.map((v, i) => {
    const x = (i / (vals.length - 1)) * w;
    const y = h - pad - ((v - min) / range) * (h - pad * 2);
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  });
  const linePts = pts.join(' ');
  const areaPts = `0,${h} ${linePts} ${w},${h}`;
  return `
    <svg viewBox="0 0 ${w} ${h}" preserveAspectRatio="none">
      <defs>
        <linearGradient id="g_${id}" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0%" stop-color="#38bdf8" stop-opacity="0.35"/>
          <stop offset="100%" stop-color="#38bdf8" stop-opacity="0.0"/>
        </linearGradient>
      </defs>
      <polygon points="${areaPts}" fill="url(#g_${id})"/>
      <polyline points="${linePts}" fill="none" stroke="#38bdf8" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
    </svg>`;
}

async function refresh() {
  try {
    const res = await fetch('/api/data');
    const data = await res.json();
    document.getElementById('connDot').className = 'dot' + (data.mqtt ? '' : ' offline');
    document.getElementById('uptime').textContent = fmtDuration(data.uptime);
    document.getElementById('wifiRssi').textContent = data.wifiRssi + ' dBm';
    document.getElementById('mqttState').textContent = data.mqtt ? 'Verbunden' : 'Getrennt';
    document.getElementById('heap').textContent = Math.round(data.freeHeap / 1024) + ' KB (min ' + Math.round(data.minHeap / 1024) + ')';
    document.getElementById('resetReason').textContent = data.resetReason;

    const grid = document.getElementById('grid');
    if (!data.devices || data.devices.length === 0) return;

    grid.innerHTML = data.devices.map(d => {
      const stale = d.ageSec > 120;
      return `
        <div class="card ${stale ? 'stale' : ''}">
          <div class="card-top">
            <div>
              <div class="dev-name">${d.name}</div>
              <div class="dev-id">ID: 0x${d.id}</div>
            </div>
            <span class="badge ${d.isOn ? 'on' : 'off'}">${d.isOn ? 'Aktiv' : 'Aus'} · vor ${d.ageSec}s</span>
          </div>
          <div class="power-row">
            <div><span class="power-val">${d.power.toFixed(1)}</span><span class="power-unit">W</span></div>
            <div class="max-val">Max: ${d.maxPower.toFixed(1)} W</div>
          </div>
          <div class="spark-wrap">${buildSparkline(d.history, d.id)}</div>
          <div class="stats">
            <div class="stat"><span>Verbrauch</span><b>${d.consumption.toFixed(3)} kWh</b></div>
            <div class="stat"><span>Laufzeit</span><b>${fmtDuration(d.onSeconds)}</b></div>
            <div class="stat"><span>Funk-RSSI</span><b>${d.rssi.toFixed(0)} dBm</b></div>
          </div>
        </div>`;
    }).join('');
  } catch (e) {
    document.getElementById('connDot').className = 'dot offline';
  }
}

refresh();
setInterval(refresh, 2500);
</script>
</body>
</html>
)rawliteral";

// --- WS2812 Hilfsfunktionen ---
void setStatusLed(uint8_t r, uint8_t g, uint8_t b) {
  ledFlashActive = false;
  rgbLedWrite(WS2812_PIN, r, g, b);
}

void flashStatusLed(uint8_t r, uint8_t g, uint8_t b, uint16_t durationMs = LED_FLASH_MS) {
  rgbLedWrite(WS2812_PIN, r, g, b);
  ledOffTime = millis() + durationMs;
  ledFlashActive = true;
}

void updateStatusLed() {
  if (ledFlashActive && (long)(millis() - ledOffTime) >= 0) {
    rgbLedWrite(WS2812_PIN, 0, 0, 0);
    ledFlashActive = false;
  }
}

// --- Whitelist & Klarname-Abfrage ---
const char* getDeviceName(uint16_t id) {
  for (size_t i = 0; i < whitelistSize; i++) {
    if (whitelist[i].id == id) {
      return whitelist[i].name;
    }
  }
  return nullptr;
}

bool isWhitelisted(uint16_t id) {
  return getDeviceName(id) != nullptr;
}

// --- RFM69 SPI-Zugriff ---
void WriteReg(uint8_t addr, uint8_t value) {
  digitalWrite(RFM69_CS, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  digitalWrite(RFM69_CS, HIGH);
}

uint8_t ReadReg(uint8_t addr) {
  digitalWrite(RFM69_CS, LOW);
  SPI.transfer(addr & 0x7F);
  uint8_t result = SPI.transfer(0);
  digitalWrite(RFM69_CS, HIGH);
  return result;
}

uint8_t GetByteFromFifo() {
  return ReadReg(0x00);
}

// --- EC3000 Dekodierung ---
byte Count1bits(uint32_t v) {
  byte c;
  for (c = 0; v; c++) {
    v &= v - 1;
  }
  return c;
}

void DescramblePayload(byte* payload) {
  byte ctr = PAYLOAD_SIZE;
  uint8_t inpbyte, outbyte = 0;
  uint32_t scramshift = 0xF185D3AC;
  while (ctr--) {
    inpbyte = *payload;
    for (byte bit = 0; bit < 8; ++bit) {
      byte ibit = (inpbyte & 0x80) >> 7;
      byte obit = ibit ^ (Count1bits(scramshift & 0x31801) & 0x01);
      scramshift = (scramshift << 1) | ibit;
      inpbyte <<= 1;
      outbyte = (outbyte << 1) | obit;
    }
    *payload++ = outbyte ^ 0xFF;
  }
}

uint16_t UpdateCRC(uint16_t crc, byte data) {
  data ^= crc & 0xFF;
  data ^= data << 4;
  return ((((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

void Del0BitsAndRevBits(byte* payload, byte blen) {
  uint8_t sval, dval, bit;
  uint8_t si, sbi, di, dbi, n1bits;

  di = dval = dbi = n1bits = 0;
  for (si = 0; si < blen; ++si) {
    sval = payload[si];
    for (sbi = 0; sbi < 8; ++sbi) {
      bit = sval & 0x80;
      sval <<= 1;
      if (n1bits >= 5 && bit == 0) {
        n1bits = 0;
        continue;
      }
      if (bit) n1bits++;
      else n1bits = 0;
      dval = (dval >> 1) | bit;
      dbi++;
      if (dbi == 8) {
        payload[di++] = dval;
        dval = dbi = 0;
      }
    }
  }
  if (dbi) payload[di] = dval >> (uint8_t)(8 - dbi);
}

void ShiftLeft(byte* payload, byte blen, byte shift) {
  uint8_t offs, bits, slen, i;
  uint16_t wbuf;

  if (shift == 0) return;
  offs = shift / 8;
  bits = 8 - shift % 8;
  slen = blen - offs - 1;
  wbuf = payload[offs];
  for (i = 0; i < slen; ++i) {
    wbuf = (wbuf << 8) | payload[i + offs + 1];
    payload[i] = wbuf >> bits;
  }
  payload[slen] = wbuf << (uint8_t)(8 - bits);
}

uint16_t ShiftReverse(byte* payload) {
  byte rblen = 47;
  uint16_t i, ec3klen;
  uint16_t crc;

  ec3klen = rblen - 1;
  Del0BitsAndRevBits(payload + 1, ec3klen);
  crc = 0xFFFF;
  if (ec3klen >= FRAME_LENGTH) {
    for (i = 0; i < FRAME_LENGTH; ++i) {
      crc = UpdateCRC(crc, payload[1 + i]);
    }
  }
  ShiftLeft(payload, rblen, 4 + 8);
  return crc;
}

void DecodeFrame(byte* payload, struct Frame* frame) {
  frame->IsValid = false;
  DescramblePayload(payload);
  frame->CRC = ShiftReverse(payload);
  uint32_t mustBeZero = payload[4] + payload[5] + payload[8] + payload[9] + payload[10] + payload[31] + payload[32];

  frame->ID = (payload[0] << 8) | payload[1];

  if (frame->CRC == 0xF0B8 && mustBeZero == 0 && frame->ID != 0x0000) {
    frame->IsValid = true;
  }

  frame->TotalSeconds = ((uint32_t)payload[29] << 20) | ((uint32_t)payload[30] << 12) | ((uint32_t)payload[2] << 8) | (uint32_t)payload[3];
  frame->OnSeconds = ((uint32_t)payload[35] << 20) | ((uint32_t)payload[36] << 12) | ((uint32_t)payload[6] << 8) | (uint32_t)payload[7];

  uint64_t cons = 0;
  cons |= payload[14];
  cons |= (uint16_t)payload[13] << 8;
  cons |= (uint16_t)payload[12] << 16;
  cons |= (uint32_t)(payload[11] & 0x0F) << 24;
  cons |= (uint64_t)payload[34] << 28;
  cons |= (uint64_t)payload[33] << 36;

  frame->Consumption = round(cons / 3600000.0 * 1000.0) / 1000.0;
  frame->Power = round(((uint16_t)payload[15] << 8 | payload[16]) / 10.0 * 10) / 10.0;
  frame->MaximumPower = round(((uint16_t)payload[17] << 8 | payload[18]) / 10.0 * 10) / 10.0;

  frame->NumberOfResets = (payload[36] << 4) | (payload[37] >> 4);
  frame->IsOn = (payload[37] & 0x08) != 0;
  frame->Reception = 0;
}

// --- Home Assistant MQTT Discovery (inkl. Klarnamen) ---
void publishDiscoveryEntity(uint16_t id, const char* component, const char* entityKey, const char* name,
                            const char* valueTpl, const char* unit, const char* devClass, const char* stateClass, bool isBinary) {
  char topic[96];
  char payload[640];
  char extra[160] = "";

  snprintf(topic, sizeof(topic), "homeassistant/%s/ec3000_%04X/%s/config", component, id, entityKey);

  if (isBinary) {
    snprintf(extra, sizeof(extra), "\"payload_on\":\"1\",\"payload_off\":\"0\",");
  } else {
    int pos = 0;
    if (unit && unit[0] != '\0') pos += snprintf(extra + pos, sizeof(extra) - pos, "\"unit_of_measurement\":\"%s\",", unit);
    if (devClass && devClass[0] != '\0') pos += snprintf(extra + pos, sizeof(extra) - pos, "\"device_class\":\"%s\",", devClass);
    if (stateClass && stateClass[0] != '\0') pos += snprintf(extra + pos, sizeof(extra) - pos, "\"state_class\":\"%s\",", stateClass);
  }

  const char* devName = getDeviceName(id);
  char fallbackName[24];
  if (!devName) {
    snprintf(fallbackName, sizeof(fallbackName), "EC3000 %04X", id);
    devName = fallbackName;
  }

  snprintf(payload, sizeof(payload),
           "{\"name\":\"%s\",\"state_topic\":\"EC3000/%04X\",\"unique_id\":\"%04x_%s\","
           "\"value_template\":\"{{ value_json.%s }}\",%s"
           "\"device\":{\"identifiers\":[\"ec3000_%04X\"],\"name\":\"%s\","
           "\"manufacturer\":\"DIY\",\"model\":\"EC3000 Energy Monitor\"}}",
           name, id, id, entityKey, valueTpl, extra, id, devName);

  client.publish(topic, payload, true);
  client.loop();
}

void publishDiscoveryMessages(uint16_t id) {
  publishDiscoveryEntity(id, "sensor", "power", "Power", "Power", "W", "power", nullptr, false);
  publishDiscoveryEntity(id, "sensor", "energy", "Energy", "Consumption", "kWh", "energy", "total_increasing", false);
  publishDiscoveryEntity(id, "sensor", "total_seconds", "Total Seconds", "TotalSeconds", "s", "duration", "total_increasing", false);
  publishDiscoveryEntity(id, "sensor", "on_seconds", "On Seconds", "OnSeconds", "s", "duration", "total_increasing", false);
  publishDiscoveryEntity(id, "sensor", "max_power", "Maximum Power", "MaximumPower", "W", "power", nullptr, false);
  publishDiscoveryEntity(id, "sensor", "resets", "Number of Resets", "NumberOfResets", nullptr, nullptr, nullptr, false);
  publishDiscoveryEntity(id, "binary_sensor", "is_on", "Is On", "IsOn", nullptr, nullptr, nullptr, true);
  publishDiscoveryEntity(id, "sensor", "rssi", "RSSI", "RSSI", "dBm", "signal_strength", nullptr, false);

  Serial.printf("Published discovery messages for %s (0x%04X)\n", getDeviceName(id), id);
}

// --- Nicht-blockierendes Verbindungsmanagement ---
void handleConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    setStatusLed(LED_BRIGHTNESS, 0, 0);
    static unsigned long lastWifiAttempt = 0;
    if (millis() - lastWifiAttempt > 10000) {
      lastWifiAttempt = millis();
      Serial.println("WiFi disconnected, reconnecting...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }
    return;
  }

  if (!client.connected()) {
    setStatusLed(LED_BRIGHTNESS, 0, 0);
    if (millis() - lastMqttReconnectAttempt > 5000) {
      lastMqttReconnectAttempt = millis();
      Serial.print("Connecting to MQTT...");
      char clientId[32];
      snprintf(clientId, sizeof(clientId), "ESP32S2-EC3000-%04X", (uint16_t)random(0xffff));

      if (client.connect(clientId, mqtt_user, mqtt_password)) {
        Serial.println("connected");
        setStatusLed(0, 0, 0);
        reconcount++;
        for (size_t i = 0; i < whitelistSize; i++) {
          publishDiscoveryMessages(whitelist[i].id);
        }
      } else {
        Serial.printf("failed, rc=%d\n", client.state());
      }
    }
  }
}

// --- Tracker & Historien-Aktualisierung ---
void pushHistory(Tracker* t, float power) {
  uint16_t val = (uint16_t)round(power * 10.0f);
  t->History[t->HistoryHead] = val;
  t->HistoryHead = (t->HistoryHead + 1) % HISTORY_LEN;
  if (t->HistoryCount < HISTORY_LEN) {
    t->HistoryCount++;
  }
}

bool updateAndCheckTracker(const Frame* frame, float rssi, double* lastConsumption) {
  unsigned long now = millis();

  for (int i = 0; i < MAX_IDS; i++) {
    if (trackers[i].Initialized && trackers[i].ID == frame->ID) {
      *lastConsumption = trackers[i].LastConsumption;
      double delta = frame->Consumption - *lastConsumption;

      if (trackers[i].LastConsumption != 0.0 && fabs(delta) >= EPSILON) {
        if (delta < 0.0 || delta > 0.025) {
          return false;
        }
      }

      trackers[i].LastConsumption = frame->Consumption;
      trackers[i].LastPower = frame->Power;
      trackers[i].MaxPower = frame->MaximumPower;
      trackers[i].TotalSeconds = frame->TotalSeconds;
      trackers[i].OnSeconds = frame->OnSeconds;
      trackers[i].IsOn = frame->IsOn;
      trackers[i].LastResets = frame->NumberOfResets;
      trackers[i].LastRSSI = rssi;
      trackers[i].LastSeen = now;
      pushHistory(&trackers[i], frame->Power);
      return true;
    }
  }

  for (int i = 0; i < MAX_IDS; i++) {
    if (!trackers[i].Initialized) {
      trackers[i].ID = frame->ID;
      trackers[i].LastResets = frame->NumberOfResets;
      trackers[i].LastConsumption = frame->Consumption;
      trackers[i].LastPower = frame->Power;
      trackers[i].MaxPower = frame->MaximumPower;
      trackers[i].TotalSeconds = frame->TotalSeconds;
      trackers[i].OnSeconds = frame->OnSeconds;
      trackers[i].IsOn = frame->IsOn;
      trackers[i].LastRSSI = rssi;
      trackers[i].Initialized = true;
      trackers[i].LastSeen = now;
      trackers[i].HistoryCount = 0;
      trackers[i].HistoryHead = 0;
      pushHistory(&trackers[i], frame->Power);
      *lastConsumption = frame->Consumption;
      return true;
    }
  }
  return false;
}

// --- WebServer Endpunkte (speichersicher per Chunked Transfer) ---
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleApiData() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  char buf[416];
  snprintf(buf, sizeof(buf),
           "{\"uptime\":%lu,\"wifiRssi\":%d,\"mqtt\":%s,\"freeHeap\":%u,\"minHeap\":%u,\"resetReason\":%d,\"devices\":[",
           millis() / 1000, WiFi.RSSI(), client.connected() ? "true" : "false",
           ESP.getFreeHeap(), ESP.getMinFreeHeap(), (int)esp_reset_reason());
  server.sendContent(buf);

  unsigned long now = millis();
  bool firstDev = true;

  for (int i = 0; i < MAX_IDS; i++) {
    if (!trackers[i].Initialized) continue;

    if (!firstDev) server.sendContent(",");
    firstDev = false;

    unsigned long ageSec = (now - trackers[i].LastSeen) / 1000;
    const char* devName = getDeviceName(trackers[i].ID);

    snprintf(buf, sizeof(buf),
             "{\"id\":\"%04X\",\"name\":\"%s\",\"power\":%.1f,\"maxPower\":%.1f,\"consumption\":%.3f,"
             "\"onSeconds\":%lu,\"isOn\":%s,\"rssi\":%.1f,\"ageSec\":%lu,\"history\":[",
             trackers[i].ID, devName ? devName : "Unbekannt",
             trackers[i].LastPower, trackers[i].MaxPower, trackers[i].LastConsumption,
             trackers[i].OnSeconds, trackers[i].IsOn ? "true" : "false", trackers[i].LastRSSI, ageSec);
    server.sendContent(buf);

    uint8_t count = trackers[i].HistoryCount;
    uint8_t start = (count < HISTORY_LEN) ? 0 : trackers[i].HistoryHead;
    char numBuf[12];
    for (uint8_t h = 0; h < count; h++) {
      uint8_t idx = (start + h) % HISTORY_LEN;
      float val = trackers[i].History[idx] / 10.0f;
      snprintf(numBuf, sizeof(numBuf), "%s%.1f", (h > 0) ? "," : "", val);
      server.sendContent(numBuf);
    }
    server.sendContent("]}");
  }

  server.sendContent("]}");
  server.sendContent("");
}

void printFrame(struct Frame* frame, float rssi) {
  const char* devName = getDeviceName(frame->ID);
  Serial.printf("RSSI: %.2f dBm | %s (0x%04X) | Total: %lus | On: %lus | Cons: %.3f kWh | Power: %.1f W | Max: %.1f W | Resets: %u | On: %s | CRC: 0x%04X\n",
                rssi, devName ? devName : "Unknown", frame->ID, frame->TotalSeconds, frame->OnSeconds, frame->Consumption,
                frame->Power, frame->MaximumPower, frame->NumberOfResets,
                frame->IsOn ? "Yes" : "No", frame->CRC);
}

void publishSystemStatus() {
  if (!client.connected()) return;

  char payload[256];
  IPAddress ip = WiFi.localIP();
  snprintf(payload, sizeof(payload),
           "{\"Uptime\":%lu,\"WiFiRSSI\":%d,\"IP\":\"%u.%u.%u.%u\",\"FreeHeap\":%u,\"MinFreeHeap\":%u,\"Reconnects\":%u,\"ResetReason\":%d}",
           millis() / 1000, WiFi.RSSI(), ip[0], ip[1], ip[2], ip[3],
           ESP.getFreeHeap(), ESP.getMinFreeHeap(), reconcount, (int)esp_reset_reason());

  Serial.println(payload);
  client.publish("EC3000/system", payload);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setStatusLed(0, 0, LED_BRIGHTNESS); // Blau: Systemstart

  // Hardware-Watchdog aktivieren (ESP-IDF v5 / Arduino Core 3.x API)
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(NULL);

  // WLAN starten
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(ssid, password);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
    esp_task_wdt_reset();
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.println(WiFi.localIP());
  }

  // WebServer starten
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleApiData);
  server.begin();
  Serial.println("Web dashboard started on port 80");

  client.setBufferSize(1024);
  client.setServer(mqtt_server, mqtt_port);

  // RFM69 Reset & SPI Start
  pinMode(RFM69_CS, OUTPUT);
  digitalWrite(RFM69_CS, HIGH);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  SPI.begin(RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CS);
  delay(10);

  WriteReg(REG_OPMODE, 0x04);
  delay(10);

  WriteReg(REG_PAYLOADLENGTH, 0x0A);
  if (ReadReg(REG_PAYLOADLENGTH) != 0x0A) {
    Serial.println("RFM69 detection failed at step 1!");
    setStatusLed(LED_BRIGHTNESS, 0, LED_BRIGHTNESS);
    while (1) { delay(1000); }
  }
  WriteReg(REG_PAYLOADLENGTH, 0x40);
  if (ReadReg(REG_PAYLOADLENGTH) != 0x40) {
    Serial.println("RFM69 detection failed at step 2!");
    setStatusLed(LED_BRIGHTNESS, 0, LED_BRIGHTNESS);
    while (1) { delay(1000); }
  }
  Serial.println("RFM69 detected!");

  unsigned long f = (((FREQUENCY_KHZ * 1000UL) << 2) / (32000000UL >> 11)) << 6;
  WriteReg(0x07, f >> 16);
  WriteReg(0x08, f >> 8);
  WriteReg(0x09, f);

  uint16_t r = ((32000000UL + (DATA_RATE / 2)) / DATA_RATE);
  WriteReg(0x03, r >> 8);
  WriteReg(0x04, r & 0xFF);

  WriteReg(0x02, 0x00);
  WriteReg(0x05, 0x01);
  WriteReg(0x06, 0x48);
  WriteReg(0x11, 0x9F);
  WriteReg(0x13, 0x0F);
  WriteReg(0x18, 0x80);
  WriteReg(0x19, 0x51); // RxBw ~125 kHz
  WriteReg(0x1E, 0x0C); // AFC AutoOn + AutoClearOn
  WriteReg(0x28, 0x10);
  WriteReg(0x29, 220);
  WriteReg(0x2E, 0xA0);
  WriteReg(0x2F, 0x13);
  WriteReg(0x30, 0xF1);
  WriteReg(0x31, 0x85);
  WriteReg(0x32, 0xD3);
  WriteReg(0x33, 0xAC);
  WriteReg(0x37, 0x00);
  WriteReg(0x38, 64);
  WriteReg(0x3C, 0x8F);
  WriteReg(0x3D, 0x10);
  WriteReg(0x6F, 0x30);

  WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | 0x10);
  Serial.println("RFM69 initialized in RX mode!");

  handleConnections();
}

void loop() {
  esp_task_wdt_reset();
  updateStatusLed();
  handleConnections();
  server.handleClient();

  if (client.connected()) {
    client.loop();
  }

  if (millis() - lastStatusPublish > 30000) {
    publishSystemStatus();
    lastStatusPublish = millis();
  }

  uint8_t irqFlags2 = ReadReg(REG_IRQFLAGS2);

  if (irqFlags2 & 0x04) {
    uint8_t idx = 0;
    while ((ReadReg(REG_IRQFLAGS2) & 0x40) && idx < 64) {
      m_payload[idx++] = GetByteFromFifo();
    }

    if (ReadReg(REG_IRQFLAGS2) & 0x50) {
      WriteReg(REG_IRQFLAGS2, 0x10);
      WriteReg(0x3D, 0x12);
    }

    if (idx >= PAYLOAD_SIZE) {
      uint8_t ec3k_payload[PAYLOAD_SIZE];
      memcpy(ec3k_payload, m_payload, PAYLOAD_SIZE);

      struct Frame frame = {0};
      DecodeFrame(ec3k_payload, &frame);

      float rssi = -(ReadReg(0x27) / 2.0f);

      if (!isWhitelisted(frame.ID)) {
        return;
      }

      bool valid = true;
      char reason[128] = "";
      int rPos = 0;

      if (!frame.IsValid) {
        valid = false;
        rPos += snprintf(reason + rPos, sizeof(reason) - rPos, "CRC invalid; ");
      }
      if (frame.OnSeconds > frame.TotalSeconds) {
        valid = false;
        rPos += snprintf(reason + rPos, sizeof(reason) - rPos, "OnSeconds > TotalSeconds; ");
      }
      if (!frame.IsOn && frame.Power != 0) {
        valid = false;
        rPos += snprintf(reason + rPos, sizeof(reason) - rPos, "IsOn=No but Power>0; ");
      }
      if (frame.Power > 3500) {
        valid = false;
        rPos += snprintf(reason + rPos, sizeof(reason) - rPos, "Power too high; ");
      }

      double lastConsumption = 0.0;
      if (valid && !updateAndCheckTracker(&frame, rssi, &lastConsumption)) {
        valid = false;
        rPos += snprintf(reason + rPos, sizeof(reason) - rPos, "Consumption invalid (last=%.3f); ", lastConsumption);
      }

      if (!valid) {
        flashStatusLed(LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
        printFrame(&frame, rssi);
        Serial.print("Discarded: ");
        Serial.println(reason);
        if (client.connected()) {
          client.publish("EC3000/debug", reason);
        }
        return;
      }

      flashStatusLed(0, LED_BRIGHTNESS, 0);

      if (!logOnlyFailed) {
        printFrame(&frame, rssi);
      }

      if (client.connected()) {
        char topic[32];
        char payload[256];
        snprintf(topic, sizeof(topic), "EC3000/%04X", frame.ID);
        snprintf(payload, sizeof(payload),
                 "{\"TotalSeconds\":%lu,\"OnSeconds\":%lu,\"Consumption\":%.3f,\"Power\":%.1f,\"MaximumPower\":%.1f,\"NumberOfResets\":%u,\"IsOn\":%d,\"CRC\":\"0x%04X\",\"RSSI\":%.2f}",
                 frame.TotalSeconds, frame.OnSeconds, frame.Consumption, frame.Power, frame.MaximumPower, frame.NumberOfResets, frame.IsOn ? 1 : 0, frame.CRC, rssi);
        client.publish(topic, payload);
      }
    }
  } else if (irqFlags2 & 0x10) {
    WriteReg(REG_IRQFLAGS2, 0x10);
    WriteReg(0x3D, 0x12);
  }
}

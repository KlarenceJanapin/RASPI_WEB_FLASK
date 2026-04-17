#include <SPI.h>
#include <LoRa.h>
#define BRIDGE_MODE 1
#if !BRIDGE_MODE
#include <WiFi.h>
#include <WebServer.h>
#endif

#if !BRIDGE_MODE
const char* ssid = "HUAWEI-2.4G-j67T";
const char* password = "4jg9QM5S";

WebServer server(80);
#endif

static const long LORA_FREQUENCY_HZ = 433E6;
static const int LORA_SCK = 18;
static const int LORA_MISO = 19;
static const int LORA_MOSI = 23;
static const int LORA_CS = 5;
static const int LORA_RST = 27;
static const int LORA_DIO0 = 33;

static const int minThrottleUs = 1000;
static const int maxThrottleUs = 2000;

bool loraOk = false;
unsigned long lastLoraRxMs = 0;
unsigned long loraRxCount = 0;
unsigned long loraTxCount = 0;
unsigned long lastLoraTxMs = 0;
uint8_t lastCmdSent = 0;
long lastLoraRssi = 0;
float lastLoraSnr = 0.0f;

double curLat = 0, curLon = 0;
float curHead = 0, distToWP = 0;
float targetBrng = 0, relBearing = 0, xte = 0;
int currentWP = 0;
int throttleLeftUs = 1500;
int throttleRightUs = 1500;
bool manualOverride = false;
uint8_t gpsValid = 0;
uint8_t gpsSats = 0;
uint8_t intensity = 0;
String directionText = "STRAIGHT";
float waterTempC = 0.0f;
float waterPh = 0.0f;
float waterTdsPpm = 0.0f;

uint16_t cmdThrLUi = minThrottleUs;
uint16_t cmdThrRUi = minThrottleUs;

struct Waypoint { double lat; double lng; };
Waypoint path[50];
int pathLen = 0;

#pragma pack(push, 1)
struct TelemetryPacket {
  uint8_t type;
  int32_t latE7;
  int32_t lonE7;
  int16_t headDeciDeg;
  int16_t tbrngDeciDeg;
  int16_t relBearingDeciDeg;
  uint16_t distDeciM;
  int16_t xteDeciM;
  uint8_t wp;
  uint16_t thrL;
  uint16_t thrR;
  uint8_t manual;
  uint8_t gpsValid;
  uint8_t gpsSats;
  uint8_t intensity;
  uint8_t direction;
  int16_t tempCentiC;
  uint16_t tdsPpm;
  uint16_t phCenti;
};

struct CommandPacket {
  uint8_t type;
  uint8_t cmd;
  uint16_t seq;
  int32_t latE7;
  int32_t lonE7;
  uint16_t thrL;
  uint16_t thrR;
};

struct AckPacket {
  uint8_t type;
  uint8_t from;
  uint8_t ackType;
  uint16_t seq;
  uint8_t code;
};
#pragma pack(pop)

enum : uint8_t {
  PKT_TELEM = 0xA1,
  PKT_CMD = 0xB1,
  PKT_TEXT = 0xC1,
  PKT_ACK = 0xD1,
};

enum : uint8_t {
  NODE_CONTROLLER = 1,
  NODE_VEHICLE = 2,
};

enum : uint8_t {
  CMD_SET_THR = 1,
  CMD_START = 2,
  CMD_CLEAR = 3,
  CMD_CALIBRATE = 4,
  CMD_ADD_WP = 5,
};

enum : uint8_t {
  DIR_STRAIGHT = 0,
  DIR_LEFT = 1,
  DIR_RIGHT = 2,
  DIR_REACHED = 3,
};

const char* directionTextFromEnum(uint8_t dir) {
  switch (dir) {
    case DIR_LEFT: return "LEFT";
    case DIR_RIGHT: return "RIGHT";
    case DIR_REACHED: return "REACHED";
    default: return "STRAIGHT";
  }
}

bool initLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY_HZ)) return false;
  LoRa.setSyncWord(0x12);
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();
  return true;
}

bool loraSendBytes(const uint8_t* data, size_t len) {
  if (!loraOk) return false;
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write(data, len);
  bool ok = LoRa.endPacket() == 1;
  LoRa.receive();
  if (ok) {
    loraTxCount++;
    lastLoraTxMs = millis();
  }
  return ok;
}

uint16_t cmdSeqCounter = 1;
uint16_t pendingSeq = 0;
uint8_t pendingCmd = 0;
uint16_t pendingThrL = minThrottleUs;
uint16_t pendingThrR = minThrottleUs;
int32_t pendingLatE7 = 0;
int32_t pendingLonE7 = 0;
bool pendingActive = false;
unsigned long pendingLastSendMs = 0;
uint8_t pendingRetries = 0;
unsigned long pendingStartMs = 0;
unsigned long lastCmdAckMs = 0;
uint16_t lastAckSeq = 0;
uint8_t lastAckCode = 0;

bool loraSendCommandPacket(uint8_t cmd, uint16_t seq, uint16_t thrLUi, uint16_t thrRUi, int32_t latE7, int32_t lonE7) {
  CommandPacket p{};
  p.type = PKT_CMD;
  p.cmd = cmd;
  p.seq = seq;
  p.latE7 = latE7;
  p.lonE7 = lonE7;
  p.thrL = thrLUi;
  p.thrR = thrRUi;
  lastCmdSent = cmd;
  return loraSendBytes((const uint8_t*)&p, sizeof(p));
}

void queueCommand(uint8_t cmd, uint16_t thrLUi, uint16_t thrRUi, int32_t latE7, int32_t lonE7) {
  cmdSeqCounter++;
  if (cmdSeqCounter == 0) cmdSeqCounter = 1;
  pendingSeq = cmdSeqCounter;
  pendingCmd = cmd;
  pendingThrL = thrLUi;
  pendingThrR = thrRUi;
  pendingLatE7 = latE7;
  pendingLonE7 = lonE7;
  pendingActive = true;
  pendingLastSendMs = 0;
  pendingRetries = 0;
  pendingStartMs = millis();
}

void servicePendingCommand() {
  if (!pendingActive) return;
  unsigned long now = millis();
  if (pendingRetries >= 12) {
    pendingActive = false;
    return;
  }
  if (pendingLastSendMs != 0 && now - pendingLastSendMs < 280) return;
  loraSendCommandPacket(pendingCmd, pendingSeq, pendingThrL, pendingThrR, pendingLatE7, pendingLonE7);
  pendingLastSendMs = now;
  pendingRetries++;
}

bool loraSendText(const char* msg, uint8_t len) {
  if (!msg) return false;
  if (len == 0) return false;
  if (len > 60) len = 60;
  uint8_t buf[3 + 60];
  buf[0] = PKT_TEXT;
  buf[1] = NODE_CONTROLLER;
  buf[2] = len;
  memcpy(&buf[3], msg, len);
  return loraSendBytes(buf, 3 + len);
}

void loraSendTextBurst(const char* msg, uint8_t len) {
  loraSendText(msg, len);
}

uint16_t streamThrL = minThrottleUs;
uint16_t streamThrR = minThrottleUs;
bool streamThrottleDirty = false;
unsigned long lastThrottleSendMs = 0;

void setThrottleTarget(uint16_t leftUi, uint16_t rightUi) {
  streamThrL = leftUi;
  streamThrR = rightUi;
  streamThrottleDirty = true;
}

void serviceThrottleStream() {
  if (pendingActive) return;
  if (!streamThrottleDirty) return;
  unsigned long now = millis();
  if (now - lastThrottleSendMs < 160) return;
  loraSendCommandPacket(CMD_SET_THR, 0, streamThrL, streamThrR, 0, 0);
  lastThrottleSendMs = now;
}

#if BRIDGE_MODE
static bool jsonGetString(const char* json, const char* key, char* out, size_t outLen) {
  if (!json || !key || !out || outLen == 0) return false;
  out[0] = 0;
  char pat[32];
  size_t klen = strlen(key);
  if (klen + 2 >= sizeof(pat)) return false;
  pat[0] = '\"';
  memcpy(&pat[1], key, klen);
  pat[1 + klen] = '\"';
  pat[2 + klen] = 0;
  const char* p = strstr(json, pat);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (*p != '\"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '\"' && i + 1 < outLen) {
    out[i++] = *p++;
  }
  out[i] = 0;
  return i > 0;
}

static bool jsonGetLong(const char* json, const char* key, long* out) {
  if (!json || !key || !out) return false;
  char pat[32];
  size_t klen = strlen(key);
  if (klen + 2 >= sizeof(pat)) return false;
  pat[0] = '\"';
  memcpy(&pat[1], key, klen);
  pat[1 + klen] = '\"';
  pat[2 + klen] = 0;
  const char* p = strstr(json, pat);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  char* endp = nullptr;
  long v = strtol(p, &endp, 10);
  if (endp == p) return false;
  *out = v;
  return true;
}

static bool jsonGetDouble(const char* json, const char* key, double* out) {
  if (!json || !key || !out) return false;
  char pat[32];
  size_t klen = strlen(key);
  if (klen + 2 >= sizeof(pat)) return false;
  pat[0] = '\"';
  memcpy(&pat[1], key, klen);
  pat[1 + klen] = '\"';
  pat[2 + klen] = 0;
  const char* p = strstr(json, pat);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  char* endp = nullptr;
  double v = strtod(p, &endp);
  if (endp == p) return false;
  *out = v;
  return true;
}

static void piSendTelemetryJson() {
  unsigned long now = millis();
  unsigned long rxAge = (lastLoraRxMs == 0) ? 999999UL : (unsigned long)(now - lastLoraRxMs);
  unsigned long txAge = (lastLoraTxMs == 0) ? 999999UL : (unsigned long)(now - lastLoraTxMs);
  unsigned long cmdAckAge = (lastCmdAckMs == 0) ? 999999UL : (unsigned long)(now - lastCmdAckMs);
  Serial.print("{\"type\":\"telem\"");
  Serial.print(",\"lat\":"); Serial.print(curLat, 6);
  Serial.print(",\"lng\":"); Serial.print(curLon, 6);
  Serial.print(",\"head\":"); Serial.print(curHead, 1);
  Serial.print(",\"tbrng\":"); Serial.print(targetBrng, 1);
  Serial.print(",\"relBearing\":"); Serial.print(relBearing, 1);
  Serial.print(",\"dist\":"); Serial.print(distToWP, 1);
  Serial.print(",\"xte\":"); Serial.print(xte, 1);
  Serial.print(",\"wp\":"); Serial.print(currentWP);
  Serial.print(",\"direction\":\""); Serial.print(directionText); Serial.print("\"");
  Serial.print(",\"intensity\":"); Serial.print((int)intensity);
  Serial.print(",\"thrL\":"); Serial.print(throttleLeftUs);
  Serial.print(",\"thrR\":"); Serial.print(throttleRightUs);
  Serial.print(",\"manual\":"); Serial.print(manualOverride ? 1 : 0);
  Serial.print(",\"gpsValid\":"); Serial.print((int)gpsValid);
  Serial.print(",\"gpsSats\":"); Serial.print((int)gpsSats);
  Serial.print(",\"waterTemp\":"); Serial.print(waterTempC, 2);
  Serial.print(",\"waterPH\":"); Serial.print(waterPh, 2);
  Serial.print(",\"waterTDS\":"); Serial.print(waterTdsPpm, 0);
  Serial.print(",\"rssi\":"); Serial.print((long)lastLoraRssi);
  Serial.print(",\"snr\":"); Serial.print(lastLoraSnr, 1);
  Serial.print(",\"rxAge\":"); Serial.print(rxAge);
  Serial.print(",\"txAge\":"); Serial.print(txAge);
  Serial.print(",\"cmdPending\":"); Serial.print(pendingActive ? 1 : 0);
  Serial.print(",\"cmdSeq\":"); Serial.print((unsigned int)pendingSeq);
  Serial.print(",\"cmdAckAge\":"); Serial.print(cmdAckAge);
  Serial.print(",\"lastAckSeq\":"); Serial.print((unsigned int)lastAckSeq);
  Serial.print(",\"lastAckCode\":"); Serial.print((int)lastAckCode);
  Serial.println("}");
}

static void handlePiUart() {
  static char line[256];
  static uint16_t idx = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[idx] = 0;
      if (idx > 0) {
        char cmd[20];
        if (jsonGetString(line, "cmd", cmd, sizeof(cmd))) {
          if (strcmp(cmd, "set_thr") == 0) {
            long l = 0, r = 0;
            if (jsonGetLong(line, "left", &l) && jsonGetLong(line, "right", &r)) {
              l = constrain(l, minThrottleUs, maxThrottleUs);
              r = constrain(r, minThrottleUs, maxThrottleUs);
              setThrottleTarget((uint16_t)l, (uint16_t)r);
            }
          } else if (strcmp(cmd, "start") == 0) {
            queueCommand(CMD_START, 0, 0, 0, 0);
          } else if (strcmp(cmd, "clear") == 0) {
            queueCommand(CMD_CLEAR, 0, 0, 0, 0);
          } else if (strcmp(cmd, "calibrate") == 0) {
            queueCommand(CMD_CALIBRATE, 0, 0, 0, 0);
          } else if (strcmp(cmd, "add_wp") == 0) {
            double lat = 0, lng = 0;
            if (jsonGetDouble(line, "lat", &lat) && jsonGetDouble(line, "lng", &lng)) {
              int32_t latE7 = (int32_t)llround(lat * 10000000.0);
              int32_t lonE7 = (int32_t)llround(lng * 10000000.0);
              queueCommand(CMD_ADD_WP, 0, 0, latE7, lonE7);
            }
          }
        }
      }
      idx = 0;
      continue;
    }
    if (idx + 1 < sizeof(line)) {
      line[idx++] = c;
    }
  }
}
#endif

#if !BRIDGE_MODE
void handleSerialChat() {
  static char line[80];
  static uint8_t idx = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (idx > 0) {
        Serial.print("TX TEXT -> VEHICLE: ");
        Serial.write((uint8_t*)line, idx);
        Serial.println();
        loraSendTextBurst(line, idx);
        idx = 0;
      }
      continue;
    }
    if (idx < sizeof(line) - 1) {
      line[idx++] = c;
    }
  }
}
#endif

void loraPollRadio() {
  int packetSize = LoRa.parsePacket();
  if (packetSize <= 0) return;

  uint8_t type = (uint8_t)LoRa.read();
  lastLoraRxMs = millis();
  lastLoraRssi = LoRa.packetRssi();
  lastLoraSnr = LoRa.packetSnr();

  if (type == PKT_ACK) {
    if (packetSize != (int)sizeof(AckPacket)) {
      while (LoRa.available()) LoRa.read();
      return;
    }
    AckPacket a{};
    a.type = type;
    int n = LoRa.readBytes(((uint8_t*)&a) + 1, sizeof(a) - 1);
    if (n != (int)sizeof(a) - 1) return;
    uint8_t from = a.from;
    uint8_t ackOfType = a.ackType;
    uint16_t seq = a.seq;
    uint8_t code = a.code;

    if (ackOfType == PKT_CMD) {
      lastCmdAckMs = millis();
      lastAckSeq = seq;
      lastAckCode = code;
      if (pendingActive && seq == pendingSeq) {
        pendingActive = false;
      }
    }
#if !BRIDGE_MODE
    Serial.print("LoRa ACK from ");
    Serial.print((int)from);
    Serial.print(" ackType=");
    Serial.print((int)ackOfType);
    Serial.print(" seq=");
    Serial.print((unsigned int)seq);
    Serial.print(" code=");
    Serial.println((int)code);
#endif
    while (LoRa.available()) LoRa.read();
    return;
  }

  if (type == PKT_TELEM) {
    if (packetSize != (int)sizeof(TelemetryPacket)) {
      while (LoRa.available()) LoRa.read();
      return;
    }
    TelemetryPacket p{};
    p.type = type;
    int n = LoRa.readBytes(((uint8_t*)&p) + 1, sizeof(p) - 1);
    if (n != (int)sizeof(p) - 1) return;

    loraRxCount++;
    curLat = (double)p.latE7 / 10000000.0;
    curLon = (double)p.lonE7 / 10000000.0;
    curHead = (float)p.headDeciDeg / 10.0f;
    targetBrng = (float)p.tbrngDeciDeg / 10.0f;
    relBearing = (float)p.relBearingDeciDeg / 10.0f;
    distToWP = (float)p.distDeciM / 10.0f;
    xte = (float)p.xteDeciM / 10.0f;
    currentWP = p.wp;
    throttleLeftUs = p.thrL;
    throttleRightUs = p.thrR;
    manualOverride = p.manual ? true : false;
    gpsValid = p.gpsValid;
    gpsSats = p.gpsSats;
    intensity = p.intensity;
    directionText = String(directionTextFromEnum(p.direction));
    waterTempC = (float)p.tempCentiC / 100.0f;
    waterTdsPpm = (float)p.tdsPpm;
    waterPh = (float)p.phCenti / 100.0f;
#if BRIDGE_MODE
    piSendTelemetryJson();
#endif
    return;
  }

  if (type == PKT_TEXT) {
    if (packetSize < 3) {
      while (LoRa.available()) LoRa.read();
      return;
    }
    uint8_t from = (uint8_t)LoRa.read();
    uint8_t len = (uint8_t)LoRa.read();
    if (len > 60) len = 60;
    char msg[61];
    int n = LoRa.readBytes((uint8_t*)msg, len);
    msg[(n < 0) ? 0 : (n > 60 ? 60 : n)] = 0;
#if !BRIDGE_MODE
    Serial.print("LoRa TEXT from ");
    Serial.print((int)from);
    Serial.print(": ");
    Serial.println(msg);
#endif
    while (LoRa.available()) LoRa.read();
    return;
  }

  while (LoRa.available()) LoRa.read();
}

#if !BRIDGE_MODE
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Water Monitoring Dashboard</title>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    :root {
      --bg0: #f2f6ff;
      --bg1: #e8f7ff;
      --card: #ffffff;
      --text: #0f172a;
      --muted: #64748b;
      --border: rgba(15, 23, 42, 0.10);
      --shadow: 0 12px 32px rgba(15, 23, 42, 0.08);
      --accent: #2563eb;
      --accent2: #06b6d4;
      --danger: #ef4444;
      --warn: #f59e0b;
      --success: #10b981;
      --chip: rgba(37, 99, 235, 0.10);
      --chipText: #1d4ed8;
      --mono: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
      --sans: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial, "Noto Sans", "Liberation Sans", sans-serif;
      --radius: 16px;
    }

    * { box-sizing: border-box; }
    html, body { height: 100%; }
    body {
      margin: 0;
      font-family: var(--sans);
      color: var(--text);
      background: radial-gradient(1200px 700px at 20% 10%, var(--bg1), var(--bg0));
    }

    .app {
      min-height: 100%;
      display: grid;
      grid-template-columns: 1fr;
    }

    .sidebar {
      padding: 20px;
      background: rgba(255, 255, 255, 0.65);
      border-right: 1px solid var(--border);
      backdrop-filter: blur(10px);
      display: flex;
      flex-direction: column;
      gap: 16px;
    }

    .brand {
      display: grid;
      gap: 6px;
      padding: 14px 14px 12px 14px;
      border-radius: var(--radius);
      background: linear-gradient(135deg, rgba(37, 99, 235, 0.12), rgba(6, 182, 212, 0.10));
      border: 1px solid rgba(37, 99, 235, 0.18);
    }
    .brand .kicker { font-size: 12px; color: var(--muted); letter-spacing: 0.08em; text-transform: uppercase; }
    .brand .title { font-size: 16px; font-weight: 800; }

    .nav { display: grid; gap: 8px; }
    .nav-item {
      text-decoration: none;
      color: var(--text);
      padding: 10px 12px;
      border-radius: 12px;
      border: 1px solid transparent;
      display: flex;
      justify-content: space-between;
      align-items: center;
      transition: background 0.12s ease, border-color 0.12s ease;
    }
    .nav-item:hover { background: rgba(15, 23, 42, 0.04); border-color: var(--border); }
    .nav-item.active { background: rgba(37, 99, 235, 0.10); border-color: rgba(37, 99, 235, 0.20); color: #1d4ed8; font-weight: 700; }

    .sidebar-card {
      margin-top: auto;
      padding: 14px;
      border-radius: var(--radius);
      background: var(--card);
      border: 1px solid var(--border);
      box-shadow: var(--shadow);
      display: grid;
      gap: 10px;
    }

    .meta-row { display: flex; align-items: baseline; justify-content: space-between; gap: 12px; }
    .meta-label { font-size: 12px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.08em; }
    .meta-value { font-weight: 800; }

    .note {
      font-size: 12px;
      color: var(--muted);
      line-height: 1.35;
      border-top: 1px solid var(--border);
      padding-top: 10px;
    }

    .main {
      padding: 22px 22px 26px 22px;
      display: grid;
      grid-template-rows: auto 1fr;
      gap: 18px;
    }

    .topbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 18px;
    }

    .topbar-title { display: grid; gap: 6px; }
    .topbar-title .kicker { font-size: 12px; color: var(--muted); letter-spacing: 0.08em; text-transform: uppercase; }
    .topbar-title .heading { font-size: 24px; font-weight: 900; }

    .topbar-right { display: flex; align-items: center; gap: 12px; }

    .gps-status {
      padding: 8px 12px;
      border-radius: 999px;
      border: 1px solid var(--border);
      background: rgba(255, 255, 255, 0.75);
      font-size: 12px;
      font-weight: 700;
    }
    .gps-fixed { border-color: rgba(16, 185, 129, 0.35); background: rgba(16, 185, 129, 0.12); color: #047857; }
    .gps-no-fix { border-color: rgba(239, 68, 68, 0.35); background: rgba(239, 68, 68, 0.10); color: #b91c1c; }
    .pill {
      padding: 8px 12px;
      border-radius: 999px;
      border: 1px solid var(--border);
      background: rgba(255, 255, 255, 0.75);
      display: flex;
      align-items: baseline;
      gap: 8px;
    }
    .pill-label { font-size: 11px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.08em; font-weight: 800; }
    .pill-value { font-family: var(--mono); font-weight: 900; }

    .grid {
      height: 100%;
      display: grid;
      grid-template-columns: 1.15fr 1.4fr;
      grid-template-rows: minmax(0, 1fr) auto;
      gap: 18px;
    }

    .card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: var(--radius);
      box-shadow: var(--shadow);
      overflow: hidden;
    }

    .card-inner {
      padding: 18px;
      height: 100%;
      display: flex;
      flex-direction: column;
      gap: 14px;
    }

    .card-header { display: flex; align-items: baseline; justify-content: space-between; gap: 10px; }
    .card-title { font-size: 14px; letter-spacing: 0.08em; text-transform: uppercase; color: var(--muted); font-weight: 800; }
    .card-subtitle { font-size: 12px; color: var(--muted); }

    .metrics {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }

    .metric {
      padding: 12px;
      border-radius: 14px;
      border: 1px solid var(--border);
      background: rgba(15, 23, 42, 0.02);
      display: grid;
      gap: 6px;
      min-width: 0;
    }

    .metric-label { font-size: 11px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.08em; }
    .val { font-family: var(--mono); font-weight: 900; font-size: 16px; color: var(--accent); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .val-warn { color: #b45309; }

    .intensity-wrap { display: grid; gap: 8px; }
    .intensity-bar { width: 100%; height: 10px; background: rgba(15, 23, 42, 0.10); border-radius: 999px; overflow: hidden; }
    .intensity-fill { height: 100%; background: linear-gradient(90deg, rgba(37, 99, 235, 0.60), rgba(6, 182, 212, 0.85)); transition: width 0.2s ease; }

    .divider { height: 1px; background: var(--border); }

    .bearing-visual {
      padding: 12px;
      border-radius: 14px;
      border: 1px solid var(--border);
      background: linear-gradient(135deg, rgba(37, 99, 235, 0.07), rgba(6, 182, 212, 0.06));
      display: grid;
      gap: 10px;
    }
    .bearing-track {
      position: relative;
      height: 14px;
      border-radius: 999px;
      border: 1px solid var(--border);
      background: rgba(255, 255, 255, 0.75);
      overflow: hidden;
    }
    .bearing-track::before {
      content: "";
      position: absolute;
      inset: 0;
      background: linear-gradient(90deg, rgba(37, 99, 235, 0.10), rgba(6, 182, 212, 0.12));
    }
    .bearing-center {
      position: absolute;
      left: 50%;
      top: 0;
      bottom: 0;
      width: 2px;
      transform: translateX(-50%);
      background: rgba(15, 23, 42, 0.35);
      z-index: 2;
    }
    .bearing-cursor {
      position: absolute;
      top: 50%;
      width: 0;
      height: 0;
      transform: translate(-50%, -50%);
      border-left: 7px solid transparent;
      border-right: 7px solid transparent;
      border-bottom: 10px solid rgba(37, 99, 235, 0.95);
      filter: drop-shadow(0 4px 10px rgba(37, 99, 235, 0.35));
      z-index: 3;
    }
    .bearing-caption {
      display: flex;
      justify-content: space-between;
      align-items: baseline;
      gap: 12px;
      font-family: var(--mono);
      font-weight: 900;
      color: var(--text);
    }
    .bearing-caption .muted {
      font-family: var(--sans);
      font-size: 12px;
      color: var(--muted);
    }

    .map-card { grid-row: 1 / span 2; grid-column: 2; }
    .map { flex: 1; min-height: 320px; border-radius: 14px; border: 1px solid var(--border); }

    .map-actions { display: flex; flex-wrap: wrap; gap: 10px; }
    .chip {
      border: 1px solid rgba(37, 99, 235, 0.20);
      background: var(--chip);
      color: var(--chipText);
      border-radius: 999px;
      padding: 8px 12px;
      font-weight: 800;
      font-size: 12px;
      cursor: pointer;
    }
    .chip:hover { background: rgba(37, 99, 235, 0.14); }

    .btn-row { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 10px; }
    .btn {
      border: 1px solid var(--border);
      background: rgba(15, 23, 42, 0.02);
      color: var(--text);
      border-radius: 14px;
      padding: 12px;
      font-weight: 900;
      cursor: pointer;
    }
    .btn:hover { background: rgba(15, 23, 42, 0.04); }
    .btn-primary { background: rgba(16, 185, 129, 0.14); border-color: rgba(16, 185, 129, 0.30); color: #065f46; }
    .btn-danger { background: rgba(239, 68, 68, 0.12); border-color: rgba(239, 68, 68, 0.28); color: #991b1b; }
    .btn-warning { background: rgba(245, 158, 11, 0.14); border-color: rgba(245, 158, 11, 0.30); color: #92400e; }

    .throttle-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; }

    input[type=range] {
      width: 100%;
      accent-color: var(--accent);
    }

    .throttle-box {
      border-radius: 16px;
      border: 1px solid rgba(15, 23, 42, 0.14);
      background: linear-gradient(180deg, rgba(15, 23, 42, 0.92), rgba(30, 41, 59, 0.90));
      color: rgba(255, 255, 255, 0.92);
      padding: 14px;
      display: grid;
      gap: 12px;
    }
    .throttle-head {
      font-size: 12px;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: rgba(226, 232, 240, 0.90);
      font-weight: 900;
    }
    .throttle-values {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
    }
    .throttle-col { display: grid; gap: 6px; min-width: 0; }
    .throttle-label {
      font-size: 11px;
      color: rgba(226, 232, 240, 0.70);
      text-transform: uppercase;
      letter-spacing: 0.08em;
      font-weight: 800;
    }
    .throttle-value {
      font-family: var(--mono);
      font-weight: 900;
      color: #fbbf24;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    .sync-check {
      display: inline-flex;
      align-items: center;
      gap: 10px;
      font-size: 12px;
      font-weight: 900;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: rgba(226, 232, 240, 0.85);
      user-select: none;
      padding: 10px 12px;
      border-radius: 12px;
      border: 1px solid rgba(168, 85, 247, 0.35);
      background: rgba(168, 85, 247, 0.10);
      width: fit-content;
    }
    .sync-check input { width: 16px; height: 16px; accent-color: #a855f7; }
    .throttle-slider { display: grid; gap: 8px; }
    .throttle-slider-label {
      font-size: 12px;
      font-weight: 900;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: rgba(226, 232, 240, 0.75);
    }
    .throttle-box input[type=range] { accent-color: #a855f7; }

    .leaflet-container { border-radius: 14px; }
    .leaflet-control-container .leaflet-control { box-shadow: none; border: 1px solid var(--border); }
    .leaflet-control-zoom a { color: var(--text); }

    @media (max-width: 980px) {
      .app { grid-template-columns: 1fr; }
      .grid { grid-template-columns: 1fr; grid-template-rows: auto auto auto; }
      .map-card { grid-row: auto; grid-column: auto; }
      .btn-row { grid-template-columns: 1fr; }
      .throttle-grid { grid-template-columns: 1fr; }
      .metrics { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="app">
    <main class="main">
      <header class="topbar">
        <div class="topbar-title">
          <div class="kicker">USV Ground Control</div>
          <div class="heading">Water Monitoring Dashboard</div>
        </div>
        <div class="topbar-right">
          <div id="gpsStatus" class="gps-status gps-no-fix">GPS: No Fix</div>
          <div class="pill"><div class="pill-label">Status</div><div id="stat" class="pill-value">IDLE</div></div>
          <div class="pill" id="loraPill"><div class="pill-label">LoRa</div><div id="loraStat" class="pill-value">---</div></div>
          <div class="pill"><div class="pill-label">Last Comms</div><div id="time" class="pill-value">---</div></div>
        </div>
      </header>

      <section class="grid">
        <div class="card">
          <div class="card-inner">
            <div class="card-header">
              <div class="card-title">Live Telemetry</div>
              <div id="prog" class="card-subtitle">---</div>
            </div>

            <div class="metrics">
              <div class="metric"><div class="metric-label">Compass Heading</div><div id="head" class="val">---</div></div>
              <div class="metric"><div class="metric-label">Target Bearing</div><div id="tbrng" class="val val-warn">---</div></div>
              <div class="metric"><div class="metric-label">Relative Bearing</div><div id="relBearing" class="val val-warn">---</div></div>
              <div class="metric"><div class="metric-label">Steering Error</div><div id="err" class="val">---</div></div>
              <div class="metric"><div class="metric-label">Direction</div><div id="direction" class="val">---</div></div>
              <div class="metric">
                <div class="metric-label">Turn Intensity</div>
                <div id="intensity" class="val">---</div>
                <div class="intensity-wrap">
                  <div class="intensity-bar"><div id="intensityFill" class="intensity-fill" style="width: 0%"></div></div>
                </div>
              </div>
              <div class="metric"><div class="metric-label">Cross-track Error</div><div id="xte" class="val">---</div></div>
              <div class="metric"><div class="metric-label">Dist to Node</div><div id="dist" class="val">---</div></div>
              <div class="metric"><div class="metric-label">Latitude</div><div id="lat" class="val">---</div></div>
              <div class="metric"><div class="metric-label">Longitude</div><div id="lon" class="val">---</div></div>
              <div class="metric"><div class="metric-label">Active Node</div><div id="wp" class="val">---</div></div>
              <div class="metric"><div class="metric-label">Water Temp</div><div id="waterTemp" class="val">---</div></div>
              <div class="metric"><div class="metric-label">Water pH</div><div id="waterPH" class="val">---</div></div>
              <div class="metric"><div class="metric-label">Water TDS</div><div id="waterTDS" class="val">---</div></div>
            </div>

            <div class="divider"></div>
            <div>
              <div class="metric-label">Bearing Indicator</div>
              <div id="bearingVisual" class="bearing-visual">---</div>
            </div>
          </div>
        </div>

        <div class="card map-card">
          <div class="card-inner">
            <div class="card-header">
              <div class="card-title">Track View</div>
              <div class="card-subtitle">Tap map to add waypoints</div>
            </div>
            <div id="map" class="map"></div>
            <div class="map-actions">
              <button class="chip" onclick="centerAndZoom()">Center</button>
              <button class="chip" onclick="zoomToFit()">Zoom to Fit</button>
              <button class="chip" onclick="toggleAutoZoom()" id="autoZoomBtn">Auto-Zoom: OFF</button>
            </div>
          </div>
        </div>

        <div class="card">
          <div class="card-inner">
            <div class="card-header">
              <div class="card-title">Control Center</div>
              <div class="card-subtitle">Manual throttle and mission controls</div>
            </div>

            <div class="btn-row">
              <button class="btn btn-primary" onclick="fetch('/start')">Start Mission</button>
              <button class="btn btn-danger" onclick="stopReset()">Stop / Reset</button>
              <button class="btn btn-warning" onclick="startCalibration()">Calibrate Compass</button>
            </div>

            <div class="divider"></div>
            <div class="throttle-box">
              <div class="throttle-head">Manual Throttle (ESC)</div>
              <div class="throttle-values">
                <div class="throttle-col">
                  <div class="throttle-label">Left (µs)</div>
                  <div id="thrLeftVal" class="throttle-value">---</div>
                </div>
                <div class="throttle-col">
                  <div class="throttle-label">Right (µs)</div>
                  <div id="thrRightVal" class="throttle-value">---</div>
                </div>
                <div class="throttle-col">
                  <div class="throttle-label">Manual Override</div>
                  <div id="manualOverrideVal" class="throttle-value">---</div>
                </div>
              </div>

              <label class="sync-check">
                <input type="checkbox" id="syncEnabled" onchange="onSyncToggle(this.checked)">
                Sync
              </label>
              <input type="range" min="1000" max="2000" value="1000" id="syncThrottle" oninput="updateSyncThrottle(this.value)" style="display:none">

              <div class="throttle-slider">
                <div class="throttle-slider-label">Left</div>
                <input type="range" min="1000" max="2000" value="1000" id="leftThrottle" oninput="updateLeftThrottle(this.value)">
              </div>
              <div class="throttle-slider">
                <div class="throttle-slider-label">Right</div>
                <input type="range" min="1000" max="2000" value="1000" id="rightThrottle" oninput="updateRightThrottle(this.value)">
              </div>
            </div>
          </div>
        </div>
      </section>
    </main>
  </div>

  <script>
    var map = L.map('map').setView([14.5915, 121.0965], 18);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 21 }).addTo(map);
    
    var usvIcon = L.circleMarker([0, 0], {radius: 7, color: '#ff1744', fillOpacity: 1, weight: 3}).addTo(map);
    var route = L.polyline([], {color: '#00e676', weight: 3}).addTo(map);
    var targetLine = L.polyline([], {color: '#d29922', weight: 2, dashArray: '5, 10'}).addTo(map);
    var waypointMarkers = [];
    
    var autoZoomEnabled = false;
    var lastZoomPosition = null;
    var initialZoomDone = false;
    var throttleDragging = false;
    
    function centerAndZoom(zoomLevel) {
      var lat = parseFloat(document.getElementById('lat').innerText);
      var lng = parseFloat(document.getElementById('lon').innerText);
      if (lat && lng && lat != 0 && lng != 0) {
        var zoom = zoomLevel || 19;
        map.setView([lat, lng], zoom);
        return true;
      }
      return false;
    }
    
    function zoomToFit() {
      var bounds = [];
      var hasPoints = false;
      
      var lat = parseFloat(document.getElementById('lat').innerText);
      var lng = parseFloat(document.getElementById('lon').innerText);
      if (lat && lng && lat != 0 && lng != 0) {
        bounds.push([lat, lng]);
        hasPoints = true;
      }
      
      waypointMarkers.forEach(function(marker) {
        var latlng = marker.getLatLng();
        bounds.push([latlng.lat, latlng.lng]);
        hasPoints = true;
      });
      
      if (hasPoints) {
        map.fitBounds(bounds, {padding: [50, 50]});
      }
    }
    
    function toggleAutoZoom() {
      autoZoomEnabled = !autoZoomEnabled;
      var btn = document.getElementById('autoZoomBtn');
      if (autoZoomEnabled) {
        btn.innerHTML = 'Auto-Zoom: ON';
        btn.style.background = 'rgba(16, 185, 129, 0.14)';
        btn.style.borderColor = 'rgba(16, 185, 129, 0.30)';
        btn.style.color = '#065f46';
        centerAndZoom(19);
      } else {
        btn.innerHTML = 'Auto-Zoom: OFF';
        btn.style.background = 'rgba(37, 99, 235, 0.10)';
        btn.style.borderColor = 'rgba(37, 99, 235, 0.20)';
        btn.style.color = '#1d4ed8';
      }
    }
    
    var gpsDetected = false;
    
    function startCalibration() {
      if(confirm('Start compass calibration? You will need to rotate the robot in all directions for 30 seconds.')) {
        fetch('/calibrate').then(() => {
          alert('Calibration started! Please rotate the robot in a figure-8 pattern for 30 seconds. Check Serial Monitor for results.');
        });
      }
    }

    function stopReset() {
      fetch('/clear', { cache: 'no-store' })
        .then(() => {
          route.setLatLngs([]);
          targetLine.setLatLngs([]);
          waypointMarkers.forEach(marker => map.removeLayer(marker));
          waypointMarkers = [];
          document.getElementById('prog').innerHTML = '0 waypoints';
          document.getElementById('stat').innerHTML = 'IDLE';
          updatePath();
          updateWaypointMarkers();
        })
        .catch(() => {});
    }

    function updateWaypointMarkers() {
      waypointMarkers.forEach(marker => map.removeLayer(marker));
      waypointMarkers = [];
      fetch('/path').then(r => r.json()).then(d => {
        d.forEach((wp, idx) => {
          var marker = L.circleMarker([wp.lat, wp.lng], {radius: 5, color: '#00e676', fillOpacity: 0.8});
          marker.bindTooltip(`WP ${idx + 1}`);
          marker.addTo(map);
          waypointMarkers.push(marker);
        });
      });
    }

    map.on('click', function(e) {
      fetch(`/addWP?lat=${e.latlng.lat}&lng=${e.latlng.lng}`).then(() => {
        updateWaypointMarkers();
        updatePath();
      });
    });

    function updatePath() {
      fetch('/path').then(r => r.json()).then(d => { 
        route.setLatLngs(d.map(p => [p.lat, p.lng])); 
        document.getElementById('prog').innerHTML = d.length + " waypoints";
      });
    }

    function createBearingVisual(relBearing) {
      var val = (typeof relBearing === 'number' && isFinite(relBearing)) ? relBearing : 0;
      var clamped = Math.min(Math.max(val, -180), 180);
      var posPct = ((clamped + 180) / 360) * 100;
      var dir = clamped > 4 ? 'RIGHT' : (clamped < -4 ? 'LEFT' : 'STRAIGHT');
      return (
        '<div class="bearing-track">' +
          '<div class="bearing-center"></div>' +
          '<div class="bearing-cursor" style="left:' + posPct.toFixed(2) + '%"></div>' +
        '</div>' +
        '<div class="bearing-caption">' +
          '<div><span class="muted">Rel</span> ' + clamped.toFixed(0) + '°</div>' +
          '<div class="muted">' + dir + '</div>' +
        '</div>'
      );
    }

    function isSyncEnabled() {
      var el = document.getElementById("syncEnabled");
      return !!(el && el.checked);
    }

    function onSyncToggle(checked) {
      if (checked) {
        var left = document.getElementById("leftThrottle");
        if (left) updateSyncThrottle(left.value);
      }
    }

    function updateSyncThrottle(val) {
      var v = val;
      var sync = document.getElementById("syncThrottle");
      if (sync) sync.value = v;
      var left = document.getElementById("leftThrottle");
      var right = document.getElementById("rightThrottle");
      if (left) left.value = v;
      if (right) right.value = v;
      fetch("/set-sync?value=" + v);
    }

    function updateLeftThrottle(val) {
      if (isSyncEnabled()) {
        updateSyncThrottle(val);
        return;
      }
      fetch("/set-left?value=" + val);
    }

    function updateRightThrottle(val) {
      if (isSyncEnabled()) {
        updateSyncThrottle(val);
        return;
      }
      fetch("/set-right?value=" + val);
    }

    function markThrottleDragging(val) {
      throttleDragging = val;
    }

    ["syncThrottle", "leftThrottle", "rightThrottle"].forEach(function(id) {
      var el = document.getElementById(id);
      el.addEventListener('pointerdown', function(){ markThrottleDragging(true); });
      el.addEventListener('pointerup', function(){ markThrottleDragging(false); });
      el.addEventListener('pointercancel', function(){ markThrottleDragging(false); });
      el.addEventListener('touchstart', function(){ markThrottleDragging(true); }, {passive: true});
      el.addEventListener('touchend', function(){ markThrottleDragging(false); }, {passive: true});
    });

    setInterval(() => {
      fetch('/data').then(r => r.json()).then(d => {
        var hasFixForDisplay = (typeof d.gpsValid !== 'undefined') ? (d.gpsValid == 1) : (d.lat != 0 && d.lng != 0);
        if (hasFixForDisplay) {
          document.getElementById('lat').innerText = d.lat.toFixed(6);
          document.getElementById('lon').innerText = d.lng.toFixed(6);
        } else {
          document.getElementById('lat').innerText = '---';
          document.getElementById('lon').innerText = '---';
        }
        document.getElementById('head').innerText = d.head.toFixed(1) + "°";
        document.getElementById('tbrng').innerText = d.tbrng.toFixed(1) + "°";
        document.getElementById('relBearing').innerText = d.relBearing.toFixed(1) + "°";
        document.getElementById('dist').innerText = d.dist.toFixed(1) + "m";
        document.getElementById('wp').innerText = "Node " + (d.wp + 1);
        document.getElementById('direction').innerHTML = d.direction;
        document.getElementById('intensity').innerHTML = d.intensity + "%";
        document.getElementById('intensityFill').style.width = d.intensity + "%";
        document.getElementById('xte').innerHTML = d.xte.toFixed(1) + "m";
        document.getElementById('waterTemp').innerText = (typeof d.waterTemp !== 'undefined') ? (Number(d.waterTemp).toFixed(2) + " °C") : "---";
        document.getElementById('waterPH').innerText = (typeof d.waterPH !== 'undefined') ? Number(d.waterPH).toFixed(2) : "---";
        document.getElementById('waterTDS').innerText = (typeof d.waterTDS !== 'undefined') ? (Math.round(Number(d.waterTDS)) + " ppm") : "---";
        
        let error = d.tbrng - d.head;
        if(error > 180) error -= 360; if(error < -180) error += 360;
        document.getElementById('err').innerHTML = error.toFixed(1) + "°";
        document.getElementById('time').innerHTML = new Date().toLocaleTimeString();

        var loraStat = document.getElementById('loraStat');
        var loraPill = document.getElementById('loraPill');
        var age = (typeof d.rxAge !== 'undefined') ? d.rxAge : 999999;
        var cmdPending = (typeof d.cmdPending !== 'undefined') ? (d.cmdPending == 1) : false;
        var cmdAckAge = (typeof d.cmdAckAge !== 'undefined') ? d.cmdAckAge : 999999;
        if (cmdPending && cmdAckAge >= 1500) {
          loraStat.innerHTML = "CMD LOST (" + cmdAckAge + "ms)";
          loraPill.style.borderColor = "rgba(239, 68, 68, 0.35)";
          loraPill.style.background = "rgba(239, 68, 68, 0.10)";
          loraPill.style.color = "#b91c1c";
        } else if (age < 2000) {
          loraStat.innerHTML = "OK (" + age + "ms)";
          loraPill.style.borderColor = "rgba(16, 185, 129, 0.35)";
          loraPill.style.background = "rgba(16, 185, 129, 0.12)";
          loraPill.style.color = "#047857";
        } else {
          loraStat.innerHTML = "LOST (" + age + "ms)";
          loraPill.style.borderColor = "rgba(239, 68, 68, 0.35)";
          loraPill.style.background = "rgba(239, 68, 68, 0.10)";
          loraPill.style.color = "#b91c1c";
        }
        
        var gpsStatusDiv = document.getElementById('gpsStatus');
        var hasGpsData = (typeof d.gpsByteAge !== 'undefined') && (d.gpsByteAge < 2000);
        var hasFix = hasFixForDisplay;
        if (hasFix) {
          var satsText = (typeof d.gpsSats !== 'undefined') ? (" (sat: " + d.gpsSats + ")") : "";
          gpsStatusDiv.innerHTML = 'GPS: Fixed ✓' + satsText;
          gpsStatusDiv.className = 'gps-status gps-fixed';
          
          if (autoZoomEnabled) {
            var currentPos = d.lat + ',' + d.lng;
            if (lastZoomPosition !== currentPos) {
              centerAndZoom(19);
              lastZoomPosition = currentPos;
            }
          } else if (!initialZoomDone && d.lat != 0 && d.lng != 0) {
            centerAndZoom(18);
            initialZoomDone = true;
          }
        } else {
          if (hasGpsData) {
            var satsText = (typeof d.gpsSats !== 'undefined') ? (" (sat: " + d.gpsSats + ")") : "";
            gpsStatusDiv.innerHTML = 'GPS: No Fix ⚠️' + satsText;
          } else {
            gpsStatusDiv.innerHTML = 'GPS: No Data ⚠️';
          }
          gpsStatusDiv.className = 'gps-status gps-no-fix';
        }
        
        if (d.dist < 2.5 && d.wp >= 0) {
          document.getElementById('stat').innerHTML = "WAYPOINT REACHED";
          document.getElementById('stat').style.color = "#d29922";
        } else if (d.dist > 0) {
          document.getElementById('stat').innerHTML = "NAVIGATING";
          document.getElementById('stat').style.color = "#238636";
        } else {
          document.getElementById('stat').innerHTML = "IDLE";
          document.getElementById('stat').style.color = "white";
        }
        
        document.getElementById('bearingVisual').innerHTML = createBearingVisual(d.relBearing);

        if (typeof d.thrL !== 'undefined') {
          document.getElementById('thrLeftVal').innerText = d.thrL + " µs";
          document.getElementById('thrRightVal').innerText = d.thrR + " µs";
          document.getElementById('manualOverrideVal').innerText = d.manual ? "ON" : "OFF";
          if (!throttleDragging) {
            var avg = Math.round((d.thrL + d.thrR) / 2);
            document.getElementById("syncThrottle").value = avg;
            document.getElementById("leftThrottle").value = d.thrL;
            document.getElementById("rightThrottle").value = d.thrR;
          }
        }
        
        if (hasFix) {
          usvIcon.setLatLng([d.lat, d.lng]);
          if (d.tbrng > 0 && d.wp >= 0) {
            fetch('/path').then(r => r.json()).then(pathData => {
              if (pathData[d.wp]) {
                targetLine.setLatLngs([[d.lat, d.lng], [pathData[d.wp].lat, pathData[d.wp].lng]]);
              }
            });
          }
        }
      });
    }, 500);
    
    updatePath();
    updateWaypointMarkers();
  </script>
</body></html>
)rawliteral";

const char esc_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESC Control</title>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root {
      --bg0: #f2f6ff;
      --bg1: #e8f7ff;
      --card: #ffffff;
      --text: #0f172a;
      --muted: #64748b;
      --border: rgba(15, 23, 42, 0.10);
      --shadow: 0 12px 32px rgba(15, 23, 42, 0.08);
      --accent: #2563eb;
      --accent2: #06b6d4;
      --radius: 16px;
      --mono: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
      --sans: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial, "Noto Sans", "Liberation Sans", sans-serif;
    }
    * { box-sizing: border-box; }
    html, body { height: 100%; }
    body {
      margin: 0;
      font-family: var(--sans);
      color: var(--text);
      background: radial-gradient(1200px 700px at 20% 10%, var(--bg1), var(--bg0));
      padding: 18px;
    }
    .wrap { max-width: 860px; margin: 0 auto; display: grid; gap: 14px; }
    .top {
      display: flex;
      justify-content: space-between;
      align-items: baseline;
      gap: 12px;
      padding: 14px 16px;
      border-radius: var(--radius);
      border: 1px solid var(--border);
      background: rgba(255, 255, 255, 0.75);
      backdrop-filter: blur(10px);
    }
    .kicker { font-size: 12px; color: var(--muted); letter-spacing: 0.08em; text-transform: uppercase; }
    .title { font-size: 20px; font-weight: 900; }
    .link { text-decoration: none; color: #1d4ed8; font-weight: 800; }
    .grid { display: grid; grid-template-columns: 1.1fr 1fr; gap: 14px; }
    .card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: var(--radius);
      box-shadow: var(--shadow);
      padding: 16px;
      display: grid;
      gap: 10px;
    }
    .card-title { font-size: 12px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.08em; font-weight: 800; }
    .speed { font-family: var(--mono); font-size: 54px; font-weight: 900; color: var(--accent2); line-height: 1; }
    .unit { font-size: 12px; color: var(--muted); }
    .row { display: grid; gap: 8px; }
    .label { font-size: 12px; color: var(--muted); font-weight: 800; }
    .val { font-family: var(--mono); font-weight: 900; color: var(--accent); }
    input[type=range] { width: 100%; accent-color: var(--accent); }
    @media (max-width: 820px) { .grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="top">
      <div>
        <div class="kicker">USV</div>
        <div class="title">ESC Control</div>
      </div>
      <a class="link" href="/">Back to Dashboard</a>
    </div>

    <div class="grid">
      <div class="card">
        <div class="card-title">Speed</div>
        <div class="speed"><span id="speed-display">0</span></div>
        <div class="unit">Power % (derived from average throttle)</div>
      </div>

      <div class="card">
        <div class="card-title">Sync</div>
        <div class="row">
          <div class="label">Throttle</div>
          <input type="range" min="1000" max="2000" value="1000" id="sync-slider" oninput="updateSync(this.value)">
        </div>
      </div>

      <div class="card">
        <div class="card-title">Left</div>
        <div class="row">
          <div class="label">Throttle</div>
          <input type="range" min="1000" max="2000" value="1000" id="left-slider" oninput="updateLeft(this.value)">
          <div class="label">Value</div>
          <div id="val-left" class="val">1000</div>
        </div>
      </div>

      <div class="card">
        <div class="card-title">Right</div>
        <div class="row">
          <div class="label">Throttle</div>
          <input type="range" min="1000" max="2000" value="1000" id="right-slider" oninput="updateRight(this.value)">
          <div class="label">Value</div>
          <div id="val-right" class="val">1000</div>
        </div>
      </div>
    </div>
  </div>

  <script>
    function calculateSpeed(val) {
      return Math.round(((val - 1000) / 1000) * 100);
    }

    function updateSpeedDisplay() {
      let l = parseInt(document.getElementById("left-slider").value);
      let r = parseInt(document.getElementById("right-slider").value);
      let avg = (l + r) / 2;
      document.getElementById("speed-display").innerHTML = calculateSpeed(avg);
    }

    function updateSync(val) {
      document.getElementById("left-slider").value = val;
      document.getElementById("right-slider").value = val;
      document.getElementById("val-left").innerHTML = val;
      document.getElementById("val-right").innerHTML = val;
      updateSpeedDisplay();
      fetch("/set-sync?value=" + val);
    }

    function updateLeft(val) {
      document.getElementById("val-left").innerHTML = val;
      updateSpeedDisplay();
      fetch("/set-left?value=" + val);
    }

    function updateRight(val) {
      document.getElementById("val-right").innerHTML = val;
      updateSpeedDisplay();
      fetch("/set-right?value=" + val);
    }
  </script>
</body>
</html>
)rawliteral";

void setupRoutes() {
  server.on("/", [](){ server.send_P(200, "text/html", index_html); });
  server.on("/esc", [](){ server.send_P(200, "text/html", esc_html); });

  server.on("/set-left", []() {
    if (server.hasArg("value")) {
      cmdThrLUi = (uint16_t)constrain(server.arg("value").toInt(), minThrottleUs, maxThrottleUs);
      setThrottleTarget(cmdThrLUi, cmdThrRUi);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/set-right", []() {
    if (server.hasArg("value")) {
      cmdThrRUi = (uint16_t)constrain(server.arg("value").toInt(), minThrottleUs, maxThrottleUs);
      setThrottleTarget(cmdThrLUi, cmdThrRUi);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/set-sync", []() {
    if (server.hasArg("value")) {
      uint16_t v = (uint16_t)constrain(server.arg("value").toInt(), minThrottleUs, maxThrottleUs);
      cmdThrLUi = v;
      cmdThrRUi = v;
      setThrottleTarget(cmdThrLUi, cmdThrRUi);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/start", [](){
    queueCommand(CMD_START, 0, 0, 0, 0);
    server.send(200, "text/plain", "OK");
  });

  server.on("/clear", [](){
    pathLen = 0;
    currentWP = 0;
    queueCommand(CMD_CLEAR, 0, 0, 0, 0);
    server.send(200, "text/plain", "OK");
  });

  server.on("/calibrate", [](){
    queueCommand(CMD_CALIBRATE, 0, 0, 0, 0);
    server.send(200, "text/plain", "OK");
  });

  server.on("/addWP", [](){
    if (pathLen >= 50) {
      server.send(500, "text/plain", "Max waypoints reached");
      return;
    }
    double lat = server.arg("lat").toDouble();
    double lng = server.arg("lng").toDouble();
    path[pathLen].lat = lat;
    path[pathLen].lng = lng;
    pathLen++;
    queueCommand(CMD_ADD_WP, 0, 0, (int32_t)llround(lat * 10000000.0), (int32_t)llround(lng * 10000000.0));
    server.send(200, "text/plain", "OK");
  });

  server.on("/data", [](){
    unsigned long ageMs = (lastLoraRxMs == 0) ? 999999UL : (unsigned long)(millis() - lastLoraRxMs);
    unsigned long txAgeMs = (lastLoraTxMs == 0) ? 999999UL : (unsigned long)(millis() - lastLoraTxMs);
    unsigned long cmdAckAgeMs = (lastCmdAckMs == 0) ? 999999UL : (unsigned long)(millis() - lastCmdAckMs);
    String j = "{\"lat\":"+String(curLat,6)+
               ",\"lng\":"+String(curLon,6)+
               ",\"head\":"+String(curHead,1)+
               ",\"tbrng\":"+String(targetBrng,1)+
               ",\"relBearing\":"+String(relBearing,1)+
               ",\"dist\":"+String(distToWP,1)+
               ",\"xte\":"+String(xte,1)+
               ",\"wp\":"+String(currentWP)+
               ",\"direction\":\""+directionText+"\""+
               ",\"intensity\":"+String((int)intensity)+
               ",\"thrL\":"+String(throttleLeftUs)+
               ",\"thrR\":"+String(throttleRightUs)+
               ",\"manual\":"+String(manualOverride ? 1 : 0)+
               ",\"gpsValid\":"+String((int)gpsValid)+
               ",\"gpsSats\":"+String((int)gpsSats)+
               ",\"waterTemp\":"+String(waterTempC,2)+
               ",\"waterPH\":"+String(waterPh,2)+
               ",\"waterTDS\":"+String(waterTdsPpm,0)+
               ",\"gpsAge\":0"+
               ",\"gpsHdop\":0"+
               ",\"gpsByteAge\":"+String(ageMs)+
               ",\"gpsFixAge\":-1"+
               ",\"gpsChars\":0"+
               ",\"gpsSent\":0"+
               ",\"gpsFail\":0"+
               ",\"rxAge\":"+String(ageMs)+
               ",\"loraOk\":"+String(loraOk ? 1 : 0)+
               ",\"rxCount\":"+String((unsigned long)loraRxCount)+
               ",\"txCount\":"+String((unsigned long)loraTxCount)+
               ",\"txAge\":"+String(txAgeMs)+
               ",\"lastCmd\":"+String((int)lastCmdSent)+
               ",\"cmdPending\":"+String(pendingActive ? 1 : 0)+
               ",\"cmdSeq\":"+String((unsigned int)pendingSeq)+
               ",\"cmdAckAge\":"+String(cmdAckAgeMs)+
               ",\"lastAckSeq\":"+String((unsigned int)lastAckSeq)+
               ",\"lastAckCode\":"+String((int)lastAckCode)+
               ",\"rssi\":"+String((long)lastLoraRssi)+
               ",\"snr\":"+String(lastLoraSnr,1)+
               "}";
    server.send(200, "application/json", j);
  });
}
#endif

void setup() {
  Serial.begin(115200);
  delay(200);

  loraOk = initLoRa();
  if (loraOk) LoRa.receive();
#if !BRIDGE_MODE
  Serial.print("LoRa: ");
  Serial.println(loraOk ? "OK" : "FAILED");

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStartMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStartMs < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("STA IP: ");
  Serial.println(WiFi.localIP());

  const char* apSsid = "USV-LORA-CTRL";
  const char* apPass = "12345678";
  WiFi.softAP(apSsid, apPass);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  setupRoutes();
  server.begin();
  Serial.println("HTTP server started");
#else
  Serial.println("{\"type\":\"status\",\"bridge\":1}");
#endif
}

void loop() {
#if !BRIDGE_MODE
  server.handleClient();
  handleSerialChat();
#else
  handlePiUart();
#endif
  servicePendingCommand();
  serviceThrottleStream();
  if (loraOk) loraPollRadio();
}

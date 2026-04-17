#include <SPI.h>
#include <LoRa.h>
#include <ESP32Servo.h>

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

// ========== SERVO CONFIGURATION ==========
#define SERVO_PIN 4           // GPIO pin for servo
#define SERVO_OFF_ANGLE 0     // 0 degrees = laser OFF
#define SERVO_ON_ANGLE 20     // default ON angle (can be overridden)

Servo laserServo;
int currentLaserAngle = 0;    // Track current servo position
bool laserState = false;      // Track laser on/off state

// ========== LORA CONFIGURATION ==========
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
  CMD_LASER = 6,        // kept for possible future use (remote laser)
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

// ========== SERVO CONTROL FUNCTIONS ==========
void initServo() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  laserServo.attach(SERVO_PIN, 500, 2400); // standard servo range
  laserServo.write(SERVO_OFF_ANGLE);
  currentLaserAngle = SERVO_OFF_ANGLE;
  laserState = false;
  Serial.println("Servo initialized at 0° (LASER OFF)");
  
  // Test servo movement (optional)
  delay(500);
  laserServo.write(10);
  delay(500);
  laserServo.write(SERVO_OFF_ANGLE);
  Serial.println("Servo test complete");
}

void setLaser(bool on, int angle = SERVO_ON_ANGLE) {
  if (on) {
    int targetAngle = constrain(angle, 0, 180);
    laserServo.write(targetAngle);
    currentLaserAngle = targetAngle;
    laserState = true;
    Serial.printf("LASER ON - Servo at %d°\n", targetAngle);
  } else {
    laserServo.write(SERVO_OFF_ANGLE);
    currentLaserAngle = SERVO_OFF_ANGLE;
    laserState = false;
    Serial.println("LASER OFF - Servo at 0°");
  }
}

// ========== LORA FUNCTIONS ==========
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

// ========== LASER COMMAND HANDLER ==========
void handleLaserCommand(bool laserOn, int angle = SERVO_ON_ANGLE) {
  setLaser(laserOn, angle);
  // Send acknowledgment back via serial (for Pi to see)
  Serial.print("{\"type\":\"laser_status\",\"laser_on\":");
  Serial.print(laserOn ? "true" : "false");
  Serial.print(",\"angle\":");
  Serial.print(laserOn ? currentLaserAngle : SERVO_OFF_ANGLE);
  Serial.println("}");
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
  Serial.print(",\"laserAngle\":"); Serial.print(currentLaserAngle);
  Serial.print(",\"laserState\":"); Serial.print(laserState ? 1 : 0);
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
              Serial.println("{\"type\":\"ack\",\"cmd\":\"set_thr\",\"status\":\"ok\"}");
            }
          } else if (strcmp(cmd, "start") == 0) {
            queueCommand(CMD_START, 0, 0, 0, 0);
            Serial.println("{\"type\":\"ack\",\"cmd\":\"start\",\"status\":\"queued\"}");
          } else if (strcmp(cmd, "clear") == 0) {
            queueCommand(CMD_CLEAR, 0, 0, 0, 0);
            Serial.println("{\"type\":\"ack\",\"cmd\":\"clear\",\"status\":\"queued\"}");
          } else if (strcmp(cmd, "calibrate") == 0) {
            queueCommand(CMD_CALIBRATE, 0, 0, 0, 0);
            Serial.println("{\"type\":\"ack\",\"cmd\":\"calibrate\",\"status\":\"queued\"}");
          } else if (strcmp(cmd, "add_wp") == 0) {
            double lat = 0, lng = 0;
            if (jsonGetDouble(line, "lat", &lat) && jsonGetDouble(line, "lng", &lng)) {
              int32_t latE7 = (int32_t)llround(lat * 10000000.0);
              int32_t lonE7 = (int32_t)llround(lng * 10000000.0);
              queueCommand(CMD_ADD_WP, 0, 0, latE7, lonE7);
              Serial.println("{\"type\":\"ack\",\"cmd\":\"add_wp\",\"status\":\"queued\"}");
            }
          } else if (strcmp(cmd, "laser_on") == 0) {
            long angle = SERVO_ON_ANGLE;
            jsonGetLong(line, "angle", &angle);
            handleLaserCommand(true, (int)angle);
          } else if (strcmp(cmd, "laser_off") == 0) {
            handleLaserCommand(false);
          } else if (strcmp(cmd, "laser_angle") == 0) {
            long angle = 0;
            if (jsonGetLong(line, "angle", &angle)) {
              angle = constrain(angle, 0, 180);
              if (laserState) {
                // If laser is on, move to new angle
                setLaser(true, (int)angle);
              } else {
                // Just update the angle for next time
                currentLaserAngle = angle;
                Serial.printf("Laser angle preset to %d° (laser off)\n", (int)angle);
              }
              Serial.print("{\"type\":\"laser_angle\",\"angle\":");
              Serial.print(angle);
              Serial.println(",\"status\":\"ok\"}");
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

void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize servo
  initServo();

  loraOk = initLoRa();
  if (loraOk) LoRa.receive();
  
#if !BRIDGE_MODE
  // (existing WiFi setup code would go here - omitted as BRIDGE_MODE=1)
#else
  Serial.println("{\"type\":\"status\",\"bridge\":1,\"servo\":\"ready\"}");
#endif
}

void loop() {
#if !BRIDGE_MODE
  // (existing loop code for non-bridge mode)
#else
  handlePiUart();
#endif
  servicePendingCommand();
  serviceThrottleStream();
  if (loraOk) loraPollRadio();
}

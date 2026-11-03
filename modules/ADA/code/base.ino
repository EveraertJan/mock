/*
  Pico W OSC node: base sketch for networked physical prototyping
  ----------------------------------------------------------------
  Board:  Raspberry Pi Pico W, with the "Raspberry Pi Pico/RP2040/RP2350"
          core by Earle Philhower (arduino-pico). No extra libraries needed.

  What it does
    - Connects to a fixed WiFi network (non-blocking, auto-reconnects)
    - Sends  /<name>/wake  once after boot (and after a WiFi reconnect)
    - Sends  /<name>/ping  every minute (uptime in s, RSSI)
    - Listens on UDP for  /<name>/...  and  /all/...
        /<name>/led r g b   (0-255)  manual LED colour
        /<name>/led                  back to status display
        /<name>/ping                 replies with /<name>/pong
        /<name>/reboot
    - Accepts firmware updates over WiFi (ArduinoOTA, password protected);
      sends /<name>/ota start | done | error, LED purple while updating
    - Shows status on an RGB LED:
        blue blinking = connecting     dim green = online
        red blinking  = WiFi lost      green flash = ping sent
        cyan flash    = message received

  Adapting it: only touch the "YOUR COMPONENT" section.
    setupComponent()        -> pinMode, sensor init
    loopComponent()         -> read sensors, call OscMsg("name").f(value).send()
    handleCustomMessage()   -> react to incoming messages

  Wiring (RGB LED, common cathode, 220R resistor on each leg):
    GP13 -> R, GP14 -> G, GP15 -> B, common -> GND
    (common anode? set COMMON_ANODE true and wire common to 3V3)
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// ============================ CONFIG ============================
const char* WIFI_SSID   = "YOUR_SSID";
const char* WIFI_PASS   = "YOUR_PASSWORD";

const char* DEVICE_NAME = "pico1";        // unique per node -> OSC prefix /pico1/...

// Where to send. Broadcast means every node and the computer on the
// subnet can hear everything with zero configuration. Set false and fill
// in TARGET_IP to send to one machine only.
const bool      TARGET_BROADCAST = true;
const IPAddress TARGET_IP(192, 168, 1, 100);
const uint16_t  TARGET_PORT = 9000;       // port your computer/Max/TouchDesigner listens on
const uint16_t  LISTEN_PORT = 8000;       // port this node listens on

// WiFi firmware updates (ArduinoOTA). Needs a Flash Size setting with a
// filesystem part (Tools > Flash Size > e.g. "2MB (Sketch: 1MB, FS: 1MB)").
// First flash with OTA must be over USB. Node shows up as a network port
// named after DEVICE_NAME in the Arduino IDE.
const char* OTA_PASSWORD = "change-me";

const uint32_t PING_INTERVAL_MS   = 60UL * 1000UL;
const uint32_t WIFI_RETRY_MS      = 10UL * 1000UL;

const int  PIN_R = 13, PIN_G = 14, PIN_B = 15;
const bool COMMON_ANODE = false;
// ================================================================

WiFiUDP udp;

// ------------------------- LED handling --------------------------
enum NetState { NET_CONNECTING, NET_ONLINE, NET_OFFLINE };
NetState netState = NET_CONNECTING;

bool     manualLed = false;
uint8_t  manualR = 0, manualG = 0, manualB = 0;
uint8_t  flashR = 0, flashG = 0, flashB = 0;
uint32_t flashUntil = 0;

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  if (COMMON_ANODE) { r = 255 - r; g = 255 - g; b = 255 - b; }
  analogWrite(PIN_R, r);
  analogWrite(PIN_G, g);
  analogWrite(PIN_B, b);
}

void ledFlash(uint8_t r, uint8_t g, uint8_t b, uint32_t ms) {
  flashR = r; flashG = g; flashB = b;
  flashUntil = millis() + ms;
}

void ledUpdate() {
  uint32_t now = millis();
  if (manualLed) { setRGB(manualR, manualG, manualB); return; }
  if ((int32_t)(flashUntil - now) > 0) { setRGB(flashR, flashG, flashB); return; }

  switch (netState) {
    case NET_CONNECTING: ((now / 250) % 2) ? setRGB(0, 0, 80) : setRGB(0, 0, 0); break;
    case NET_ONLINE:     setRGB(0, 10, 0); break;
    case NET_OFFLINE:    ((now / 1000) % 2) ? setRGB(80, 0, 0) : setRGB(0, 0, 0); break;
  }
}

// --------------------------- OSC out -----------------------------
// (OTA update handling is further down, after OscMsg is defined)
static size_t pad4(size_t n) { return (n + 3) & ~(size_t)3; }

static size_t writeOscString(uint8_t* dst, const char* s) {
  size_t n = pad4(strlen(s) + 1);
  memset(dst, 0, n);
  memcpy(dst, s, strlen(s));
  return n;
}

IPAddress targetIP() {
  if (!TARGET_BROADCAST) return TARGET_IP;
  return IPAddress((uint32_t)WiFi.localIP() | ~(uint32_t)WiFi.subnetMask());
}

// Usage: OscMsg("temperature").f(21.5).send();   -> /pico1/temperature 21.5
//        OscMsg("button").i(1).send();
class OscMsg {
public:
  OscMsg(const char* suffix) {
    snprintf(addr, sizeof(addr), "/%s/%s", DEVICE_NAME, suffix);
  }
  OscMsg& i(int32_t v) {
    if (nt < 15 && dlen + 4 <= sizeof(data)) {
      tags[nt++] = 'i'; tags[nt] = 0;
      put32((uint32_t)v);
    }
    return *this;
  }
  OscMsg& f(float v) {
    if (nt < 15 && dlen + 4 <= sizeof(data)) {
      uint32_t u; memcpy(&u, &v, 4);
      tags[nt++] = 'f'; tags[nt] = 0;
      put32(u);
    }
    return *this;
  }
  OscMsg& s(const char* v) {
    size_t n = pad4(strlen(v) + 1);
    if (nt < 15 && dlen + n <= sizeof(data)) {
      tags[nt++] = 's'; tags[nt] = 0;
      dlen += writeOscString(data + dlen, v);
    }
    return *this;
  }
  bool send() {
    if (WiFi.status() != WL_CONNECTED) return false;
    uint8_t buf[320];
    size_t p = writeOscString(buf, addr);
    char t[18]; t[0] = ','; strcpy(t + 1, tags);
    p += writeOscString(buf + p, t);
    memcpy(buf + p, data, dlen); p += dlen;
    udp.beginPacket(targetIP(), TARGET_PORT);
    udp.write(buf, p);
    return udp.endPacket();
  }
private:
  void put32(uint32_t u) {
    data[dlen++] = u >> 24; data[dlen++] = u >> 16;
    data[dlen++] = u >> 8;  data[dlen++] = u;
  }
  char    addr[64];
  char    tags[16] = {0};
  uint8_t data[192];
  size_t  dlen = 0;
  uint8_t nt = 0;
};

// ---------------------------- OSC in -----------------------------
struct OscArg { char type; int32_t i; float f; const char* s; };
struct OscIn {
  const char* addr;
  OscArg args[8];
  int n = 0;
};

// number arg as float whether it was sent as int or float
float argF(const OscIn& m, int idx, float def = 0) {
  if (idx >= m.n) return def;
  if (m.args[idx].type == 'i') return (float)m.args[idx].i;
  if (m.args[idx].type == 'f') return m.args[idx].f;
  return def;
}
const char* argS(const OscIn& m, int idx, const char* def = "") {
  return (idx < m.n && m.args[idx].type == 's') ? m.args[idx].s : def;
}

static uint32_t rd32(const uint8_t* b) {
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
}

bool parseOsc(uint8_t* b, int len, OscIn& m) {
  if (len < 4 || b[0] != '/') return false;      // ignores bundles
  m.addr = (const char*)b;
  size_t p = pad4(strlen(m.addr) + 1);
  m.n = 0;
  if ((int)p >= len || b[p] != ',') return true; // no args
  const char* tags = (const char*)&b[p];
  p += pad4(strlen(tags) + 1);
  for (const char* t = tags + 1; *t && m.n < 8; t++) {
    OscArg& a = m.args[m.n];
    a.type = *t; a.i = 0; a.f = 0; a.s = "";
    if (*t == 'i') {
      if ((int)p + 4 > len) break;
      a.i = (int32_t)rd32(b + p); p += 4;
    } else if (*t == 'f') {
      if ((int)p + 4 > len) break;
      uint32_t u = rd32(b + p); memcpy(&a.f, &u, 4); p += 4;
    } else if (*t == 's') {
      if ((int)p >= len) break;
      a.s = (const char*)(b + p); p += pad4(strlen(a.s) + 1);
    } else if (*t == 'T') { a.type = 'i'; a.i = 1; }
    else if (*t == 'F')   { a.type = 'i'; a.i = 0; }
    else break;                                  // unsupported type: stop parsing
    m.n++;
  }
  return true;
}

// Returns pointer to the command part if addr is /<name>/cmd or /all/cmd, else nullptr
const char* matchCommand(const char* addr) {
  char prefix[40];
  snprintf(prefix, sizeof(prefix), "/%s/", DEVICE_NAME);
  if (strncmp(addr, prefix, strlen(prefix)) == 0) return addr + strlen(prefix);
  if (strncmp(addr, "/all/", 5) == 0) return addr + 5;
  return nullptr;
}

// Forward declaration, defined in the component section
void handleCustomMessage(const char* cmd, const OscIn& m);

void handleOsc(uint8_t* buf, int len) {
  OscIn m;
  if (!parseOsc(buf, len, m)) return;
  const char* cmd = matchCommand(m.addr);
  if (!cmd) return;

  ledFlash(0, 60, 60, 80);

  if (strcmp(cmd, "led") == 0) {
    if (m.n >= 3) {
      manualLed = true;
      manualR = constrain((int)argF(m, 0), 0, 255);
      manualG = constrain((int)argF(m, 1), 0, 255);
      manualB = constrain((int)argF(m, 2), 0, 255);
    } else {
      manualLed = false;
    }
  } else if (strcmp(cmd, "ping") == 0) {
    OscMsg("pong").i(millis() / 1000).send();
  } else if (strcmp(cmd, "reboot") == 0) {
    delay(100);
    rp2040.reboot();
  } else {
    handleCustomMessage(cmd, m);
  }
}

// ======================= YOUR COMPONENT ==========================
// Everything specific to the hardware attached to this node goes here.

void setupComponent() {
  // e.g. pinMode(2, INPUT_PULLUP);
}

void loopComponent() {
  // e.g. read a sensor and send it:
  //   static uint32_t last = 0;
  //   if (millis() - last > 50) { last = millis(); OscMsg("light").i(analogRead(26)).send(); }
}

// Called for any incoming /<name>/<cmd> or /all/<cmd> that isn't built in.
void handleCustomMessage(const char* cmd, const OscIn& m) {
  // e.g. if (strcmp(cmd, "motor") == 0) analogWrite(16, (int)argF(m, 0));
}
// ================================================================

// ----------------------- WiFi + main loop ------------------------
uint32_t lastPing = 0, lastWifiAttempt = 0;
bool everConnected = false;

void startWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_NAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiAttempt = millis();
}

// ------------------------------ OTA ------------------------------
bool otaStarted = false;

void setupOta() {
  if (otaStarted) return;               // begin() once, after first WiFi connect
  ArduinoOTA.setHostname(DEVICE_NAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    manualLed = false;
    setRGB(120, 0, 120);                // purple = updating
    OscMsg("ota").s("start").send();
    Serial.println("OTA start");
  });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    // handle() blocks during the upload, so drive the LED from here:
    // purple blinking, one blink per eighth of the upload
    uint8_t v = 40 + (uint8_t)((done * 8 / (total ? total : 1)) % 2) * 80;
    setRGB(v, 0, v);
  });
  ArduinoOTA.onEnd([]() {
    setRGB(0, 120, 0);
    OscMsg("ota").s("done").send();
    Serial.println("OTA done, rebooting");
  });
  ArduinoOTA.onError([](ota_error_t err) {
    setRGB(255, 0, 0);
    OscMsg("ota").s("error").i((int)err).send();
    Serial.printf("OTA error %d\n", (int)err);
  });

  ArduinoOTA.begin();
  otaStarted = true;
}

void sendWake(const char* reason) {
  OscMsg("wake").s(reason).s(WiFi.localIP().toString().c_str()).i(WiFi.RSSI()).send();
}

void sendPing() {
  OscMsg("ping").i(millis() / 1000).i(WiFi.RSSI()).send();
  ledFlash(0, 255, 0, 60);
}

void setup() {
  Serial.begin(115200);
  setRGB(0, 0, 80);                  // solid blue while WiFi starts
  setupComponent();
  startWifi();
}

void loop() {
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && netState != NET_ONLINE) {
    netState = NET_ONLINE;
    udp.stop();
    udp.begin(LISTEN_PORT);
    Serial.print("Connected, IP "); Serial.println(WiFi.localIP());
    sendWake(everConnected ? "reconnect" : "boot");
    everConnected = true;
    setupOta();
    lastPing = millis();
  } else if (!connected && netState == NET_ONLINE) {
    netState = NET_OFFLINE;
    Serial.println("WiFi lost");
  }

  if (!connected && millis() - lastWifiAttempt > WIFI_RETRY_MS) {
    if (everConnected) netState = NET_OFFLINE;
    WiFi.disconnect();
    startWifi();
  }

  if (connected) {
    ArduinoOTA.handle();
    int size = udp.parsePacket();
    if (size > 0) {
      static uint8_t rx[512];
      int len = udp.read(rx, sizeof(rx) - 1);
      if (len > 0) { rx[len] = 0; handleOsc(rx, len); }
    }
    if (millis() - lastPing >= PING_INTERVAL_MS) {
      lastPing += PING_INTERVAL_MS;
      sendPing();
    }
  }

  loopComponent();
  ledUpdate();
}

/*
  Pico W network node: SONAR DISTANCE SENSOR (HC-SR04 type)
  ==========================================================
  Board: Raspberry Pi Pico W, "Raspberry Pi Pico/RP2040/RP2350" core by Earle Philhower.
  No extra libraries needed.

  WHAT THIS DOES
    1. Connects to the WiFi network below (and reconnects if it drops).
    2. Sends  /NAME/wake   once when it comes online.
    3. Sends  /NAME/ping   every minute.
    4. Measures the distance every 250 ms and sends  /NAME/distance <cm>
       (a decimal number) when it changed by at least 1 cm since the last one sent.
       Nothing is sent while nothing is in range.
    5. Listens on port 8000 for messages starting with /NAME/ or /all/ :

         /NAME/identify 10      blink the LED white for 10 seconds (find the box)
         /NAME/ping             answer: /NAME/pong
         /NAME/reboot           restart the board
         /all/who               every node answers /NAME/iam with its IP address

    6. Accepts new firmware over WiFi (in the Arduino IDE: pick the network port).

  STATUS LED COLOURS
    blue blinking : connecting          dim green   : online
    red blinking  : WiFi lost           green flash : ping sent
    cyan flash    : message received    purple      : firmware update running

  WIRING
    Status LED (one pixel of an APA102 / SK9822 strip):
      Pico GP13 -> strip DI,  Pico GP14 -> strip CI,
      strip 5V  -> Pico VBUS (pin 40),  strip GND -> Pico GND
    Sonar sensor (HC-SR04 type, 4 pins):
      VCC  -> Pico VBUS (pin 40, 5 V),  GND -> Pico GND
      TRIG -> Pico GP2
      ECHO -> Pico GP3  THROUGH A VOLTAGE DIVIDER: the sensor's ECHO pin gives 5 V
              but the Pico only tolerates 3.3 V. Use two resistors:
              ECHO --[ 1 kOhm ]--+--[ 2 kOhm ]-- GND,  and connect GP3 to the middle (+).
              (Sensors sold as HC-SR04P, "3.3 V", or RCWL-1601 can be connected directly,
              and can be powered from the Pico's 3V3 pin.)

  TO MAKE A NEW NODE: change NAME below and flash it once over USB.
  After that you can update it over WiFi.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


// =====================================================================
//  SETTINGS
// =====================================================================

const char* NAME          = "regi";            // unique per node: a-z, 0-9 and -
const char* WIFI_SSID     = "MOCK";
const char* WIFI_PASS     = "BlockedAndLoaded";
const char* OTA_PASSWORD  = "change-me";         // asked when updating over WiFi

// Where the node sends its messages to.
// Broadcast reaches every computer on the network. If your router blocks it,
// set SEND_TO_BROADCAST to false and enter the IP address of the computer.
const bool  SEND_TO_BROADCAST = true;
IPAddress   COMPUTER_IP(192, 168, 8, 100);
const int   SEND_PORT   = 9000;                  // the computer listens on this port
const int   LISTEN_PORT = 8000;                  // this node listens on this port

// Pins
const int  PIN_STRIP_DATA  = 15;                 // strip DI
const int  PIN_STRIP_CLOCK = 14;                 // strip CI
const int  STRIP_BRIGHTNESS = 31;                // APA102 / SK9822: 0 (off) to 31 (full)

// Sonar sensor
const int  PIN_SONAR_TRIGGER = 2;                // TRIG
const int  PIN_SONAR_ECHO    = 3;                // ECHO (through the voltage divider!)
const int  SONAR_INTERVAL_MS = 250;              // time between measurements (keep above 50)
const float SONAR_MIN_CHANGE_CM = 1.0;           // only send when it moved at least this much
                                                 // (0 = send every measurement)
const int  SONAR_MAX_CM      = 400;              // further away counts as "nothing in range"



// =====================================================================
//  VARIABLES (the node's memory; you normally don't change these)
// =====================================================================

// network
WiFiUDP       udp;
bool          online = false;                    // true while WiFi is connected
bool          hasBeenOnline = false;             // true once WiFi connected at least once
bool          otaStarted = false;                // true once WiFi updates are switched on
unsigned long lastWifiTry = 0;                   // when we last tried to (re)connect
unsigned long lastPing = 0;                      // when we last sent /ping

// status LED
int           flashRed = 0, flashGreen = 0, flashBlue = 0;
unsigned long flashUntil = 0;                    // short colour flash until this time
unsigned long identifyUntil = 0;                 // white blinking until this time
int           lastRed = 0, lastGreen = 0, lastBlue = 0;   // colour last sent to the strip
unsigned long lastSent = 0;                      // when it was last sent

// sonar
unsigned long lastMeasure = 0;                   // when we last measured
float         distanceCm = -1;                   // last measurement (-1 = nothing in range)
float         lastSentCm = -1;                   // last distance we sent (-1 = none yet)

// OSC message being sent
uint8_t       packet[160];
int           packetLength = 0;

// OSC message just received (filled in by readMessage)
uint8_t       buffer[256];                       // the raw bytes
char          messageName[32];                   // "identify" for /sonar1/identify or /all/identify
float         number[3];                         // the numbers in the message
int           numberCount = 0;                   // how many numbers there were
char          textArgument[16];                  // the text in the message, if any


// =====================================================================
//  SETUP AND LOOP
// =====================================================================

void setup() {
  Serial.begin(115200);

  pinMode(PIN_STRIP_DATA, OUTPUT);
  pinMode(PIN_STRIP_CLOCK, OUTPUT);
  digitalWrite(PIN_STRIP_CLOCK, LOW);
  showColor(0, 0, 80);                           // blue: starting

  pinMode(PIN_SONAR_TRIGGER, OUTPUT);
  digitalWrite(PIN_SONAR_TRIGGER, LOW);
  pinMode(PIN_SONAR_ECHO, INPUT);

  connectWifi();
}

void loop() {
  bool connected = (WiFi.status() == WL_CONNECTED);

  // --- WiFi just came back / connected for the first time ---
  if (connected && !online) {
    online = true;
    hasBeenOnline = true;
    Serial.print(NAME);
    Serial.print(" online, IP ");
    Serial.println(WiFi.localIP());
    udp.stop();
    udp.begin(LISTEN_PORT);
    sendText("wake", WiFi.localIP().toString().c_str());
    startOta();
    lastPing = millis();
  }

  // --- WiFi just went away ---
  if (!connected && online) {
    online = false;
    Serial.println("WiFi lost");
  }

  // --- not connected: try again every 10 seconds ---
  if (!connected && millis() - lastWifiTry > 10000) {
    WiFi.disconnect();
    connectWifi();
  }

  if (connected) {
    ArduinoOTA.handle();

    // --- incoming message? ---
    if (udp.parsePacket() > 0) {
      int length = udp.read(buffer, sizeof(buffer) - 1);
      if (length > 0) {
        buffer[length] = 0;
        if (readMessage(buffer, length)) handleMessage();
      }
    }

    // --- ping every minute ---
    if (millis() - lastPing >= 60000) {
      lastPing = millis();
      sendNumber("ping", millis() / 1000);
      flashColor(0, 255, 0, 60);                 // green flash
    }
  }

  // --- measure the distance, send it if it is new ---
  if (millis() - lastMeasure >= SONAR_INTERVAL_MS) {
    lastMeasure = millis();
    distanceCm = measureDistanceCm();
    if (distanceCm < 0) {
      lastSentCm = -1;                           // nothing there: forget, so the next
                                                 // object is always reported
    } else if (lastSentCm < 0 || fabs(distanceCm - lastSentCm) >= SONAR_MIN_CHANGE_CM) {
      lastSentCm = distanceCm;
      sendDecimal("distance", distanceCm);       // sends /sonar1/distance <cm>
    }
  }

  updateStatusLed();
}


// =====================================================================
//  SONAR
// =====================================================================

// Measure the distance in centimetres. Returns -1 when no echo came back.
// Note: while waiting for an echo that never comes, this blocks for up to
// about 25 ms, which is fine for this node.
float measureDistanceCm() {
  // A 10 microsecond pulse on TRIG starts one measurement.
  digitalWrite(PIN_SONAR_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SONAR_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIGGER, LOW);

  // The sensor then holds ECHO high for as long as the sound needed to go
  // to the object and back. Sound covers 1 cm in about 29 microseconds,
  // so there and back is about 58 microseconds per cm of distance.
  unsigned long timeout = SONAR_MAX_CM * 58UL + 1000;
  unsigned long microseconds = pulseIn(PIN_SONAR_ECHO, HIGH, timeout);
  if (microseconds == 0) return -1;              // no echo: nothing in range

  float cm = microseconds / 58.0;
  if (cm > SONAR_MAX_CM) return -1;
  return cm;
}


// =====================================================================
//  STATUS LED
// =====================================================================

// Send one byte to the strip, one bit at a time (data line + clock line).
void sendByteToStrip(uint8_t value) {
  for (int bit = 7; bit >= 0; bit--) {
    digitalWrite(PIN_STRIP_DATA, (value >> bit) & 1);
    digitalWrite(PIN_STRIP_CLOCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_STRIP_CLOCK, LOW);
    delayMicroseconds(1);
  }
}

// Show a colour on the pixel (0-255 per colour).
// Only talks to the strip when the colour changes, plus once every half second.
void showColor(uint8_t red, uint8_t green, uint8_t blue) {
  bool same = (red == lastRed && green == lastGreen && blue == lastBlue);
  if (same && millis() - lastSent < 500) return;
  lastRed = red;  lastGreen = green;  lastBlue = blue;
  lastSent = millis();

  for (int i = 0; i < 4; i++) sendByteToStrip(0x00);            // start of message
  sendByteToStrip(0xE0 | STRIP_BRIGHTNESS);                     // brightness
  sendByteToStrip(blue);                                        // this chip wants blue first
  sendByteToStrip(green);
  sendByteToStrip(red);
  for (int i = 0; i < 4; i++) sendByteToStrip(0xFF);            // end of message
}

// Short colour flash that overrides the status colour
void flashColor(int red, int green, int blue, int milliseconds) {
  flashRed = red;  flashGreen = green;  flashBlue = blue;
  flashUntil = millis() + milliseconds;
}

// Decide what colour the LED shows right now. Called on every loop.
void updateStatusLed() {
  unsigned long now = millis();
  bool blinkSlow = (now / 1000) % 2;
  bool blinkFast = (now / 250) % 2;

  if ((long)(identifyUntil - now) > 0) {         // /identify: white blinking
    if ((now / 150) % 2) showColor(255, 255, 255); else showColor(0, 0, 0);
  } else if ((long)(flashUntil - now) > 0) {     // short flash (ping, message)
    showColor(flashRed, flashGreen, flashBlue);
  } else if (online) {                           // normal: dim green
    showColor(0, 10, 0);
  } else if (hasBeenOnline) {                    // lost the network: red blinking
    if (blinkSlow) showColor(80, 0, 0); else showColor(0, 0, 0);
  } else {                                       // still connecting: blue blinking
    if (blinkFast) showColor(0, 0, 80); else showColor(0, 0, 0);
  }
}


// =====================================================================
//  SENDING OSC MESSAGES
// =====================================================================
// An OSC message is a list of items: the address ("/sonar1/ping"), the type of
// the argument (",i" = integer, ",s" = text) and the argument itself. Every item
// is padded with zeros to a multiple of 4 bytes. We build it in "packet".

void addText(const char* text) {
  int length = strlen(text);
  if (packetLength + length + 4 >= (int)sizeof(packet)) return;   // too long, skip
  for (int i = 0; i <= length; i++) packet[packetLength++] = text[i];
  while (packetLength % 4 != 0) packet[packetLength++] = 0;
}

void addInteger(int32_t value) {
  packet[packetLength++] = value >> 24;
  packet[packetLength++] = value >> 16;
  packet[packetLength++] = value >> 8;
  packet[packetLength++] = value;
}

void addDecimal(float value) {
  uint32_t raw;
  memcpy(&raw, &value, 4);                       // the 4 bytes of the decimal number
  packet[packetLength++] = raw >> 24;
  packet[packetLength++] = raw >> 16;
  packet[packetLength++] = raw >> 8;
  packet[packetLength++] = raw;
}

IPAddress whereToSend() {
  if (!SEND_TO_BROADCAST) return COMPUTER_IP;
  // broadcast address = our address with all the "free" bits set to 1
  return IPAddress((uint32_t)WiFi.localIP() | ~(uint32_t)WiFi.subnetMask());
}

void sendPacket() {
  if (WiFi.status() != WL_CONNECTED) return;
  udp.beginPacket(whereToSend(), SEND_PORT);
  udp.write(packet, packetLength);
  udp.endPacket();
}

// sendNumber("ping", 12)  sends  /sonar1/ping 12
void sendNumber(const char* what, int32_t value) {
  char address[64];
  snprintf(address, sizeof(address), "/%s/%s", NAME, what);
  packetLength = 0;
  addText(address);
  addText(",i");
  addInteger(value);
  sendPacket();
}

// sendDecimal("distance", 12.5)  sends  /sonar1/distance 12.5
void sendDecimal(const char* what, float value) {
  char address[64];
  snprintf(address, sizeof(address), "/%s/%s", NAME, what);
  packetLength = 0;
  addText(address);
  addText(",f");
  addDecimal(value);
  sendPacket();
}

// sendText("wake", "192.168.8.237")  sends  /sonar1/wake 192.168.8.237
void sendText(const char* what, const char* text) {
  char address[64];
  snprintf(address, sizeof(address), "/%s/%s", NAME, what);
  packetLength = 0;
  addText(address);
  addText(",s");
  addText(text);
  sendPacket();
}


// =====================================================================
//  RECEIVING OSC MESSAGES
// =====================================================================
// readMessage() takes the raw bytes and fills in messageName, number[],
// numberCount and textArgument (listed under VARIABLES at the top).

int roundUpTo4(int n) {
  return (n + 3) & ~3;
}

bool readMessage(uint8_t* data, int length) {
  // 1. Is it for us? The address must start with /sonar1/ or /all/
  const char* address = (const char*)data;
  char ourPrefix[40];
  snprintf(ourPrefix, sizeof(ourPrefix), "/%s/", NAME);

  const char* rest;
  if (strncmp(address, ourPrefix, strlen(ourPrefix)) == 0) {
    rest = address + strlen(ourPrefix);
  } else if (strncmp(address, "/all/", 5) == 0) {
    rest = address + 5;
  } else {
    return false;
  }
  strncpy(messageName, rest, sizeof(messageName) - 1);
  messageName[sizeof(messageName) - 1] = 0;

  // 2. Find the type tags (",ifs" etc.) that follow the address
  numberCount = 0;
  textArgument[0] = 0;
  int position = roundUpTo4(strlen(address) + 1);
  if (position >= length || data[position] != ',') return true;      // no arguments
  const char* types = (const char*)data + position + 1;
  position += roundUpTo4(strlen((const char*)data + position) + 1);

  // 3. Read the arguments one by one
  for (const char* type = types; *type; type++) {
    if (*type == 'i' || *type == 'f') {
      if (position + 4 > length) break;
      uint32_t raw = ((uint32_t)data[position] << 24) | ((uint32_t)data[position + 1] << 16) |
                     ((uint32_t)data[position + 2] << 8) | (uint32_t)data[position + 3];
      position += 4;
      float value;
      if (*type == 'i') value = (int32_t)raw; else memcpy(&value, &raw, 4);
      if (numberCount < 3) number[numberCount++] = value;
    } else if (*type == 's') {
      if (position >= length) break;
      const char* text = (const char*)data + position;
      strncpy(textArgument, text, sizeof(textArgument) - 1);
      textArgument[sizeof(textArgument) - 1] = 0;
      position += roundUpTo4(strlen(text) + 1);
    } else if (*type == 'T' || *type == 'F') {                       // true / false
      if (numberCount < 3) number[numberCount++] = (*type == 'T') ? 1 : 0;
    } else {
      break;                                                         // unknown type: stop
    }
  }
  return true;
}

bool isMessage(const char* name) {
  return strcmp(messageName, name) == 0;
}

// What to do with each message. Add new commands here.
void handleMessage() {
  flashColor(0, 60, 60, 80);                     // cyan flash: message received

  if (isMessage("identify")) {
    float seconds = (numberCount > 0) ? number[0] : 10;
    identifyUntil = millis() + (unsigned long)(seconds * 1000);

  } else if (isMessage("ping")) {
    sendNumber("pong", millis() / 1000);

  } else if (isMessage("who")) {
    sendText("iam", WiFi.localIP().toString().c_str());

  } else if (isMessage("reboot")) {
    delay(100);
    rp2040.reboot();
  }
}


// =====================================================================
//  FIRMWARE UPDATES OVER WIFI
// =====================================================================

void otaStart() {
  showColor(120, 0, 120);                        // purple
  sendText("ota", "start");
}

void otaProgress(unsigned int done, unsigned int total) {
  // the update blocks everything else, so blink the LED from here
  bool bright = ((done * 8 / (total ? total : 1)) % 2) == 1;
  if (bright) showColor(120, 0, 120); else showColor(40, 0, 40);
}

void otaEnd() {
  showColor(0, 120, 0);                          // green: done, board restarts
  sendText("ota", "done");
}

void otaError(ota_error_t error) {
  showColor(255, 0, 0);                          // red: something went wrong
  sendText("ota", "error");
}

void startOta() {
  if (otaStarted) return;
  ArduinoOTA.setHostname(NAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart(otaStart);
  ArduinoOTA.onProgress(otaProgress);
  ArduinoOTA.onEnd(otaEnd);
  ArduinoOTA.onError(otaError);
  ArduinoOTA.begin();
  otaStarted = true;
}


// =====================================================================
//  WIFI
// =====================================================================

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(NAME);
  WiFi.noLowPowerMode();                         // stay awake, so broadcast messages arrive
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiTry = millis();
}

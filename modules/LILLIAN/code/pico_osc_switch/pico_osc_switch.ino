/*
  Pico W network node: SWITCH (sends a message when the switch changes)
  ======================================================================
  Board: Raspberry Pi Pico W, "Raspberry Pi Pico/RP2040/RP2350" core by Earle Philhower.
  No extra libraries needed.

  WHAT THIS DOES
    1. Connects to the WiFi network below (and reconnects if it drops).
    2. Sends  /NAME/wake   once when it comes online.
    3. Sends  /NAME/ping   every minute.
    4. Sends  /NAME/switch 1   when the switch is pressed / closed, and
              /NAME/switch 0   when it is released / opened.
       The current state is also sent right after connecting.
    5. Listens on port 8000 for messages starting with /NAME/ or /all/ :

         /NAME/switch           ask for the state, answer: /NAME/switch 0|1
         /NAME/identify 10     blink the LED white for 10 seconds (find the box)
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
    Switch (or push button):
      one contact -> Pico GP16 (physical pin 21), the other contact -> a Pico GND pin
      (for example pin 23, right next to it). No resistor needed: the Pico's own
      pull-up resistor is switched on.

  TO MAKE A NEW NODE: change NAME below and flash it once over USB.
  After that you can update it over WiFi.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


// =====================================================================
//  SETTINGS
// =====================================================================

const char* NAME          = "lillian";           // unique per node: a-z, 0-9 and -
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
const int  PIN_STRIP_DATA  = 13;                 // strip DI
const int  PIN_STRIP_CLOCK = 14;                 // strip CI
const int  STRIP_BRIGHTNESS = 31;                // APA102 / SK9822: 0 (off) to 31 (full)

// Switch
const int  PIN_SWITCH = 16;                      // GP16 = physical pin 21
const bool SWITCH_ACTIVE_LOW = true;             // true: the switch connects the pin to GND
const int  DEBOUNCE_MS = 20;                     // ignore contact bounce shorter than this



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

// switch
bool          switchOn = false;                  // confirmed state (true = pressed / closed)
bool          lastReading = false;               // what the pin showed the last time we looked
unsigned long lastChange = 0;                    // when the pin last changed

// OSC message being sent
uint8_t       packet[160];
int           packetLength = 0;

// OSC message just received (filled in by readMessage)
uint8_t       buffer[256];                       // the raw bytes
char          messageName[32];                   // "identify" for /switch1/identify or /all/identify
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

  pinMode(PIN_SWITCH, INPUT_PULLUP);             // the Pico pulls the pin HIGH by itself
  switchOn = readSwitch();                       // start from the real state: no false message
  lastReading = switchOn;

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
    sendNumber("switch", switchOn ? 1 : 0);      // tell where the switch is now
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

  checkSwitch();

  updateStatusLed();
}


// =====================================================================
//  SWITCH
// =====================================================================

// true when the switch is pressed / closed
bool readSwitch() {
  return digitalRead(PIN_SWITCH) == (SWITCH_ACTIVE_LOW ? LOW : HIGH);
}

// Contacts bounce: for a few milliseconds the pin flickers on and off.
// So we only believe a change after the pin has stayed the same for DEBOUNCE_MS.
void checkSwitch() {
  bool reading = readSwitch();
  if (reading != lastReading) {                  // it moved: start the timer again
    lastReading = reading;
    lastChange = millis();
  }
  if (reading != switchOn && millis() - lastChange >= DEBOUNCE_MS) {
    switchOn = reading;                          // it stayed: this is a real change
    sendNumber("switch", switchOn ? 1 : 0);      // /switch1/switch 1 or 0
    flashColor(255, 255, 255, 100);              // white flash on the status LED
  }
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
// An OSC message is a list of items: the address ("/switch1/ping"), the type of
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

// sendNumber("ping", 12)  sends  /switch1/ping 12
void sendNumber(const char* what, int32_t value) {
  char address[64];
  snprintf(address, sizeof(address), "/%s/%s", NAME, what);
  packetLength = 0;
  addText(address);
  addText(",i");
  addInteger(value);
  sendPacket();
}

// sendText("wake", "192.168.8.237")  sends  /switch1/wake 192.168.8.237
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
  // 1. Is it for us? The address must start with /switch1/ or /all/
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

  } else if (isMessage("switch")) {
    sendNumber("switch", switchOn ? 1 : 0);      // /switch1/switch 0 or 1
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

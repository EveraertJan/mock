/*
  Pico W network node: RGB LIGHT SPOT (10 APA102 / SK9822 LEDs)
  =============================================================
  Board: Raspberry Pi Pico W, "Raspberry Pi Pico/RP2040/RP2350" core by Earle Philhower.
  No extra libraries needed.

  WHAT THIS DOES
    1. Connects to the WiFi network below (and reconnects if it drops).
    2. Sends  /NAME/wake   once when it comes online.
    3. Sends  /NAME/ping   every minute.
    4. Listens on port 8000 for messages starting with /NAME/ or /all/ :

         /NAME/color 255 80 0   all LEDs to this colour (red, green, blue: 0-255)
         /NAME/color 0 0 255 2  same, but fade to it during 2 seconds
         /NAME/off              all LEDs off (fade time as a number is allowed: /NAME/off 3)
         /NAME/identify 10      blink the status LED white for 10 seconds (find the box)
         /NAME/ping             answer: /NAME/pong
         /NAME/reboot           restart the board
         /all/who               every node answers /NAME/iam with its IP address

    5. Accepts new firmware over WiFi (in the Arduino IDE: pick the network port).

  STATUS LED COLOURS (the separate single LED)
    blue blinking : connecting          dim green   : online
    red blinking  : WiFi lost           green flash : ping sent
    cyan flash    : message received    purple      : firmware update running
    white blinking: /identify
  The 10 LEDs of the spot only show your colour. At power-up they are off
  (see START_RED/GREEN/BLUE to change that).

  WIRING (two strips of APA102 / SK9822 LEDs, each with its own two pins)
    Status LED (1 LED):   Pico GP13 -> DI,  Pico GP14 -> CI,
                          5V -> Pico VBUS (pin 40),  GND -> Pico GND
    Light spot (10 LEDs): Pico GP11 -> DI,  Pico GP12 -> CI,  GND -> Pico GND
                          5V -> a separate 5 V power supply, whose GND also goes to
                          the Pico GND. 10 LEDs on full white take up to about 0.6 A,
                          more than USB or the Pico should supply. For a lower current,
                          lower SPOT_BRIGHTNESS below.

  TO MAKE A NEW NODE: change NAME below and flash it once over USB.
  After that you can update it over WiFi.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


// =====================================================================
//  SETTINGS
// =====================================================================

const char* NAME          = "oleg";             // unique per node: a-z, 0-9 and -
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
const int  PIN_STATUS_DATA  = 16;                // status LED: DI
const int  PIN_STATUS_CLOCK = 17;                // status LED: CI
const int  STATUS_BRIGHTNESS = 31;               // APA102 / SK9822: 0 (off) to 31 (full)

// Light spot
const int  PIN_SPOT_DATA  = 13;                  // spot strip: DI
const int  PIN_SPOT_CLOCK = 12;                  // spot strip: CI
const int  SPOT_BRIGHTNESS = 31;                 // 0 (off) to 31 (full); lower = less current
const int  NUM_LEDS = 15;                        // number of LEDs in the spot
const int  START_RED = 0;                        // colour at power-up (0-255 each)
const int  START_GREEN = 0;
const int  START_BLUE = 0;


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
int           statusLastRed = 0, statusLastGreen = 0, statusLastBlue = 0;   // last sent to the LED
unsigned long statusLastSent = 0;                // when it was last sent

// light spot
int           spotLastRed = 0, spotLastGreen = 0, spotLastBlue = 0;         // last sent to the strip
unsigned long spotLastSent = 0;                  // when it was last sent
// light colour and fade
int           fromRed = START_RED, fromGreen = START_GREEN, fromBlue = START_BLUE;   // fade starts here
int           toRed = START_RED, toGreen = START_GREEN, toBlue = START_BLUE;         // fade ends here
int           shownRed = START_RED, shownGreen = START_GREEN, shownBlue = START_BLUE; // colour now
unsigned long fadeStart = 0;                     // when the fade started
unsigned long fadeLength = 0;                    // how long it lasts, in milliseconds (0 = at once)

// OSC message being sent
uint8_t       packet[160];
int           packetLength = 0;

// OSC message just received (filled in by readMessage)
uint8_t       buffer[256];                       // the raw bytes
char          messageName[32];                   // "color" for /spot1/color or /all/color
float         number[4];                         // the numbers in the message
int           numberCount = 0;                   // how many numbers there were
char          textArgument[16];                  // the text in the message, if any


// =====================================================================
//  SETUP AND LOOP
// =====================================================================

void setup() {
  Serial.begin(115200);

  pinMode(PIN_STATUS_DATA, OUTPUT);
  pinMode(PIN_STATUS_CLOCK, OUTPUT);
  digitalWrite(PIN_STATUS_CLOCK, LOW);
  pinMode(PIN_SPOT_DATA, OUTPUT);
  pinMode(PIN_SPOT_CLOCK, OUTPUT);
  digitalWrite(PIN_SPOT_CLOCK, LOW);
  showStatus(0, 0, 80);                          // blue: starting

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

  updateStatusLed();
  updateSpot();
}


// =====================================================================
//  STATUS LED AND LIGHT SPOT
// =====================================================================
// Both are strips of APA102 / SK9822 LEDs, each with its own data and clock pin.

// Send one byte to a strip, one bit at a time (data line + clock line).
void sendByteToStrip(int dataPin, int clockPin, uint8_t value) {
  for (int bit = 7; bit >= 0; bit--) {
    digitalWrite(dataPin, (value >> bit) & 1);
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(clockPin, LOW);
    delayMicroseconds(1);
  }
}

// Give all LEDs of a strip the same colour (0-255 per colour).
void writeStrip(int dataPin, int clockPin, int ledCount, int brightness,
                uint8_t red, uint8_t green, uint8_t blue) {
  for (int i = 0; i < 4; i++) sendByteToStrip(dataPin, clockPin, 0x00);   // start of message
  for (int led = 0; led < ledCount; led++) {                              // one block per LED
    sendByteToStrip(dataPin, clockPin, 0xE0 | brightness);                // brightness
    sendByteToStrip(dataPin, clockPin, blue);                             // this chip wants blue first
    sendByteToStrip(dataPin, clockPin, green);
    sendByteToStrip(dataPin, clockPin, red);
  }
  for (int i = 0; i < 4; i++) sendByteToStrip(dataPin, clockPin, 0xFF);   // end of message
}

// Show a colour on the status LED.
// Only talks to the strip when the colour changes, plus once every half second.
void showStatus(uint8_t red, uint8_t green, uint8_t blue) {
  bool same = (red == statusLastRed && green == statusLastGreen && blue == statusLastBlue);
  if (same && millis() - statusLastSent < 500) return;
  statusLastRed = red;  statusLastGreen = green;  statusLastBlue = blue;
  statusLastSent = millis();
  writeStrip(PIN_STATUS_DATA, PIN_STATUS_CLOCK, 1, STATUS_BRIGHTNESS, red, green, blue);
}

// Show a colour on the 10 LEDs of the spot. Same idea.
void showSpot(uint8_t red, uint8_t green, uint8_t blue) {
  bool same = (red == spotLastRed && green == spotLastGreen && blue == spotLastBlue);
  if (same && millis() - spotLastSent < 500) return;
  spotLastRed = red;  spotLastGreen = green;  spotLastBlue = blue;
  spotLastSent = millis();
  writeStrip(PIN_SPOT_DATA, PIN_SPOT_CLOCK, NUM_LEDS, SPOT_BRIGHTNESS, red, green, blue);
}

// Short colour flash on the status LED that overrides the status colour
void flashColor(int red, int green, int blue, int milliseconds) {
  flashRed = red;  flashGreen = green;  flashBlue = blue;
  flashUntil = millis() + milliseconds;
}

// Decide what colour the status LED shows right now. Called on every loop.
void updateStatusLed() {
  unsigned long now = millis();
  bool blinkSlow = (now / 1000) % 2;
  bool blinkFast = (now / 250) % 2;

  if ((long)(identifyUntil - now) > 0) {         // /identify: white blinking
    if ((now / 150) % 2) showStatus(255, 255, 255); else showStatus(0, 0, 0);
  } else if ((long)(flashUntil - now) > 0) {     // short flash (ping, message)
    showStatus(flashRed, flashGreen, flashBlue);
  } else if (online) {                           // normal: dim green
    showStatus(0, 10, 0);
  } else if (hasBeenOnline) {                    // lost the network: red blinking
    if (blinkSlow) showStatus(80, 0, 0); else showStatus(0, 0, 0);
  } else {                                       // still connecting: blue blinking
    if (blinkFast) showStatus(0, 0, 80); else showStatus(0, 0, 0);
  }
}

// Go to a new colour. seconds = 0 changes at once, otherwise it fades.
void setLight(int red, int green, int blue, float seconds) {
  fromRed = shownRed;  fromGreen = shownGreen;  fromBlue = shownBlue;   // start from what we see now
  toRed = constrain(red, 0, 255);
  toGreen = constrain(green, 0, 255);
  toBlue = constrain(blue, 0, 255);
  fadeLength = (seconds > 0) ? (unsigned long)(seconds * 1000) : 0;
  fadeStart = millis();
}

// Work out the colour of the light right now (halfway a fade: a mix) and show it.
void updateSpot() {
  float progress = 1;                            // 0 = start colour, 1 = end colour
  if (fadeLength > 0) {
    progress = (millis() - fadeStart) / (float)fadeLength;
    if (progress > 1) progress = 1;
  }
  shownRed   = fromRed   + (toRed   - fromRed)   * progress;
  shownGreen = fromGreen + (toGreen - fromGreen) * progress;
  shownBlue  = fromBlue  + (toBlue  - fromBlue)  * progress;
  showSpot(shownRed, shownGreen, shownBlue);
}


// =====================================================================
//  SENDING OSC MESSAGES
// =====================================================================
// An OSC message is a list of items: the address ("/spot1/ping"), the type of
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

// sendNumber("ping", 12)  sends  /spot1/ping 12
void sendNumber(const char* what, int32_t value) {
  char address[64];
  snprintf(address, sizeof(address), "/%s/%s", NAME, what);
  packetLength = 0;
  addText(address);
  addText(",i");
  addInteger(value);
  sendPacket();
}

// sendText("wake", "192.168.8.237")  sends  /spot1/wake 192.168.8.237
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
  // 1. Is it for us? The address must start with /spot1/ or /all/
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
      if (numberCount < 4) number[numberCount++] = value;
    } else if (*type == 's') {
      if (position >= length) break;
      const char* text = (const char*)data + position;
      strncpy(textArgument, text, sizeof(textArgument) - 1);
      textArgument[sizeof(textArgument) - 1] = 0;
      position += roundUpTo4(strlen(text) + 1);
    } else if (*type == 'T' || *type == 'F') {                       // true / false
      if (numberCount < 4) number[numberCount++] = (*type == 'T') ? 1 : 0;
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

  if (isMessage("color")) {
    // /spot1/color red green blue [fade seconds]
    if (numberCount >= 3) {
      setLight(number[0], number[1], number[2], (numberCount > 3) ? number[3] : 0);
    }

  } else if (isMessage("off")) {
    // /spot1/off [fade seconds]
    setLight(0, 0, 0, (numberCount > 0) ? number[0] : 0);

  } else if (isMessage("identify")) {
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
  showStatus(120, 0, 120);                        // purple
  sendText("ota", "start");
}

void otaProgress(unsigned int done, unsigned int total) {
  // the update blocks everything else, so blink the LED from here
  bool bright = ((done * 8 / (total ? total : 1)) % 2) == 1;
  if (bright) showStatus(120, 0, 120); else showStatus(40, 0, 40);
}

void otaEnd() {
  showStatus(0, 120, 0);                          // green: done, board restarts
  sendText("ota", "done");
}

void otaError(ota_error_t error) {
  showStatus(255, 0, 0);                          // red: something went wrong
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

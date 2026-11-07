/*
  Pico W network node: STEPPER MOTOR (step/direction driver, gear 10:45)
  ======================================================================
  Board: Raspberry Pi Pico W, "Raspberry Pi Pico/RP2040/RP2350" core by Earle Philhower.
  No extra libraries needed.

  WHAT THIS DOES
    1. Connects to the WiFi network below (and reconnects if it drops).
    2. Sends  /NAME/wake   once when it comes online.
    3. Sends  /NAME/ping   every minute.
    4. Listens on port 8000 for messages starting with /NAME/ or /all/ :

         /NAME/goto 90          move to 90 degrees (0 = where the motor was at power-up)
         /NAME/move -45         move 45 degrees backwards from where it is heading
         /NAME/stop             slow down and stop
         /NAME/speed 90         top speed in degrees per second
         /NAME/accel 180        speeding up / slowing down, in degrees per second per second
         /NAME/zero             call the current position 0 (only while standing still)
         /NAME/position         ask where it is, answer: /NAME/position <degrees>
         /NAME/identify 10     blink the LED white for 10 seconds (find the box)
         /NAME/ping             answer: /NAME/pong
         /NAME/reboot           restart the board
         /all/who               every node answers /NAME/iam with its IP address

       All degrees are at the OUTPUT of the gears, not at the motor. Every move
       starts and ends with a ramp (see speed and accel). The node sends
       /NAME/position <degrees> every time a move has finished.
    5. Accepts new firmware over WiFi (in the Arduino IDE: pick the network port).

  STATUS LED COLOURS
    blue blinking : connecting          dim green   : online
    red blinking  : WiFi lost           green flash : ping sent
    cyan flash    : message received    purple      : firmware update running

  WIRING
    Status LED (one pixel of an APA102 / SK9822 strip):
      Pico GP13 -> strip DI,  Pico GP14 -> strip CI,
      strip 5V  -> Pico VBUS (pin 40),  strip GND -> Pico GND
    Stepper driver (A4988 / DRV8825 type) and motor:
      STEP -> Pico GP2,  DIR -> Pico GP3
      driver VDD -> Pico 3V3 (pin 36),  driver GND -> Pico GND
      driver VMOT / GND -> separate motor power supply (8-35 V) with a 100 uF
        capacitor close to the driver. NEVER power the motor from the Pico.
        Connect the supply's GND to the Pico GND as well.
      motor coils -> driver 1A/1B and 2A/2B (one coil on each pair)
      driver RESET and SLEEP: connect them to each other.
      Microstepping (MICROSTEPS below is 16): A4988 MS1+MS2+MS3 to 3V3,
        DRV8825 only MS3 to 3V3 (check your driver's table).
      Set the driver's current limit with its small potentiometer first.
      Never unplug the motor while the supply is on.

  The stepping itself runs on the Pico's second processor core (setup1 / loop1),
  so WiFi activity cannot make the motor stutter.

  TO MAKE A NEW NODE: change NAME below and flash it once over USB.
  After that you can update it over WiFi.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


// =====================================================================
//  SETTINGS
// =====================================================================

const char* NAME          = "stepper1";          // unique per node: a-z, 0-9 and -
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

// Stepper
const int   PIN_STEP = 2;
const int   PIN_DIR  = 3;
const bool  INVERT_DIRECTION = false;            // true = the motor turns the other way

const float MOTOR_STEPS_PER_REV = 200;           // 1.8 degree motor = 200 full steps
const float MICROSTEPS          = 16;            // must match the driver's MS pins
const float GEAR_MOTOR_TEETH    = 10;            // gear on the motor ...
const float GEAR_OUTPUT_TEETH   = 45;            // ... drives this gear on the output

// steps the driver must make for 1 degree at the output (10:45 gear, 16 microsteps = 40)
const float STEPS_PER_DEGREE = MOTOR_STEPS_PER_REV * MICROSTEPS *
                               (GEAR_OUTPUT_TEETH / GEAR_MOTOR_TEETH) / 360.0;

const float DEFAULT_SPEED_DEG = 90;              // degrees per second (top speed)
const float DEFAULT_ACCEL_DEG = 180;             // degrees per second per second
const float MAX_SPEED_DEG     = 360;             // /speed can not ask for more than this
const float START_SPEED_STEPS = 100;             // steps per second at the start / end of a move



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

// stepper (used by both processor cores, that is why they are "volatile")
volatile long  positionSteps = 0;                // where the motor is, in steps (0 = power-up)
volatile long  targetSteps = 0;                  // where it has to go, in steps
volatile float stepSpeed = 0;                    // speed now, steps per second (negative = back)
volatile float maxSpeedSteps = DEFAULT_SPEED_DEG * STEPS_PER_DEGREE;
volatile float accelSteps = DEFAULT_ACCEL_DEG * STEPS_PER_DEGREE;
volatile bool  justArrived = false;              // set when a move has finished
unsigned long  lastStepMicros = 0;               // when the last step was made
unsigned long  lastUpdateMicros = 0;             // when the speed was last updated

// OSC message being sent
uint8_t       packet[160];
int           packetLength = 0;

// OSC message just received (filled in by readMessage)
uint8_t       buffer[256];                       // the raw bytes
char          messageName[32];                   // "goto" for /stepper1/goto or /all/goto
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

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_STEP, LOW);

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

  // --- a move has finished: tell where we are ---
  if (justArrived) {
    justArrived = false;
    sendDecimal("position", positionSteps / STEPS_PER_DEGREE);   // /stepper1/position <degrees>
  }

  updateStatusLed();
}


// =====================================================================
//  STEPPER (runs on the second processor core)
// =====================================================================
// setup1() and loop1() are the "setup" and "loop" of core 1. This core does
// nothing but make the motor steps on time, whatever WiFi is doing on core 0.
//
// How a move works: every round we look at how far the target still is.
//   - far away and below top speed : speed up a little
//   - the target is close enough that we must start braking: slow down a little
//   - target on the other side (it was changed during a move): brake to a stop,
//     then go the other way
// Then we make a step whenever the time between two steps has passed.

void setup1() {
  lastUpdateMicros = micros();
}

void loop1() {
  unsigned long now = micros();
  float seconds = (now - lastUpdateMicros) / 1000000.0;
  lastUpdateMicros = now;
  if (seconds > 0.05) seconds = 0.05;            // after a long pause: don't make a jump

  // Up to this speed the motor can stop within one step, so it may stop instantly.
  float stopSpeed = sqrt(2 * accelSteps);
  if (stopSpeed < START_SPEED_STEPS) stopSpeed = START_SPEED_STEPS;

  long remaining = targetSteps - positionSteps;  // steps still to go (negative = backwards)
  if (remaining == 0 && fabs(stepSpeed) <= stopSpeed) {   // on the target and slow: done
    if (stepSpeed != 0) justArrived = true;      // it was still moving: a move just finished
    stepSpeed = 0;
    return;
  }

  float speed = stepSpeed;                       // steps per second, negative = backwards
  float wantedDirection;
  if (remaining > 0) wantedDirection = 1;
  else if (remaining < 0) wantedDirection = -1;
  else wantedDirection = (speed > 0) ? -1 : 1;   // on the target but too fast: brake
  bool movingTowardTarget = (speed == 0) || (speed * wantedDirection > 0);
  float stopDistance = speed * speed / (2 * accelSteps);   // steps needed to stop from here
  float magnitude = fabs(speed);

  if (speed == 0) {
    // start moving
    speed = wantedDirection * START_SPEED_STEPS;
  } else if (!movingTowardTarget || stopDistance >= fabs((float)remaining)) {
    // slow down: we are going the wrong way, or we would overshoot
    magnitude = magnitude - accelSteps * seconds;
    if (magnitude < START_SPEED_STEPS) {
      // wrong way: stop completely. Right way: creep the last steps.
      magnitude = movingTowardTarget ? START_SPEED_STEPS : 0;
    }
    speed = ((speed > 0) ? 1 : -1) * magnitude;
  } else {
    // speed up (or slow down if we are above the top speed)
    if (magnitude < maxSpeedSteps) {
      magnitude = magnitude + accelSteps * seconds;
      if (magnitude > maxSpeedSteps) magnitude = maxSpeedSteps;
    } else {
      magnitude = magnitude - accelSteps * seconds;
      if (magnitude < maxSpeedSteps) magnitude = maxSpeedSteps;
    }
    speed = wantedDirection * magnitude;
  }

  stepSpeed = speed;
  if (speed == 0) return;

  // time between two steps, in microseconds
  unsigned long interval = (unsigned long)(1000000.0 / fabs(speed));
  if (now - lastStepMicros >= interval) {
    lastStepMicros = now;
    takeStep((speed > 0) ? 1 : -1);
  }
}

// One step forwards (+1) or backwards (-1)
void takeStep(int direction) {
  digitalWrite(PIN_DIR, ((direction > 0) != INVERT_DIRECTION) ? HIGH : LOW);
  delayMicroseconds(2);                          // the driver needs a moment to see the direction
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(3);                          // pulse must be at least ~2 microseconds
  digitalWrite(PIN_STEP, LOW);
  positionSteps = positionSteps + direction;
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
// An OSC message is a list of items: the address ("/stepper1/ping"), the type of
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

// sendNumber("ping", 12)  sends  /stepper1/ping 12
void sendNumber(const char* what, int32_t value) {
  char address[64];
  snprintf(address, sizeof(address), "/%s/%s", NAME, what);
  packetLength = 0;
  addText(address);
  addText(",i");
  addInteger(value);
  sendPacket();
}

// sendDecimal("position", 12.5)  sends  /stepper1/position 12.5
void sendDecimal(const char* what, float value) {
  char address[64];
  snprintf(address, sizeof(address), "/%s/%s", NAME, what);
  packetLength = 0;
  addText(address);
  addText(",f");
  addDecimal(value);
  sendPacket();
}

// sendText("wake", "192.168.8.237")  sends  /stepper1/wake 192.168.8.237
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
  // 1. Is it for us? The address must start with /stepper1/ or /all/
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

  } else if (isMessage("goto")) {
    // go to a position in degrees, counted from the power-up position
    if (numberCount > 0) targetSteps = lroundf(number[0] * STEPS_PER_DEGREE);

  } else if (isMessage("move")) {
    // go this many degrees further than the position we are heading for
    if (numberCount > 0) targetSteps = targetSteps + lroundf(number[0] * STEPS_PER_DEGREE);

  } else if (isMessage("stop")) {
    // the target becomes the spot where the motor can still stop smoothly
    float speed = stepSpeed;
    long stopDistance = (long)ceil(speed * speed / (2 * accelSteps));
    targetSteps = positionSteps + ((speed > 0) ? stopDistance : -stopDistance);

  } else if (isMessage("speed")) {
    if (numberCount > 0) {
      float degrees = number[0];
      if (degrees > MAX_SPEED_DEG) degrees = MAX_SPEED_DEG;
      if (degrees < 1) degrees = 1;
      maxSpeedSteps = degrees * STEPS_PER_DEGREE;
    }

  } else if (isMessage("accel")) {
    if (numberCount > 0) {
      float degrees = number[0];
      if (degrees < 1) degrees = 1;
      accelSteps = degrees * STEPS_PER_DEGREE;
    }

  } else if (isMessage("zero")) {
    // only when standing still, otherwise the motor would jump
    if (stepSpeed == 0 && targetSteps == positionSteps) {
      positionSteps = 0;
      targetSteps = 0;
    }

  } else if (isMessage("position")) {
    sendDecimal("position", positionSteps / STEPS_PER_DEGREE);
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

// Arctos gripper controller — Arduino Nano + MCP2515 (TJA1050) + servo.
//
// Wiring (kept identical to the user's original setup):
//   MCP2515 INT  -> D2
//   MCP2515 CS   -> D10
//   MCP2515 SCK  -> D13   (SPI SCK, hardware-fixed on Nano)
//   MCP2515 SI   -> D11   (SPI MOSI)
//   MCP2515 SO   -> D12   (SPI MISO)
//   MCP2515 VCC  -> 5V
//   MCP2515 GND  -> GND
//   Servo signal -> D9
//   Servo VCC    -> EXTERNAL 5V supply (do not power servo from Nano 5V pin)
//   Servo GND    -> common GND with Nano + external 5V
//   CAN bus      -> CANH/CANL on the MCP2515 module (TJA1050 transceiver),
//                   120 ohm termination at each end of the bus
//
// Protocol (matches arctos-pi backend/gripper.py):
//   Standard CAN ID 0x07, 1-byte payload, payload[0] = position 0..255
//
// Build-time options (override via -D from arduino-cli, see ../flash.sh):
//   MCP_CRYSTAL_8MHZ   set if the MCP2515 module has an 8 MHz crystal
//                      (default = 16 MHz, which is what most off-the-shelf
//                      MCP2515+TJA1050 modules ship with — check the silver
//                      oscillator can next to the chip; "16.000" = 16 MHz)
//   DEBUG_SERIAL       chatty serial output for bring-up diagnostics

#include <SPI.h>
#include <mcp_can.h>
#include <Servo.h>

static const uint8_t PIN_CAN_INT = 2;
static const uint8_t PIN_CAN_CS  = 10;
static const uint8_t PIN_SERVO   = 9;
static const uint8_t PIN_LED     = LED_BUILTIN;

static const long GRIPPER_CAN_ID = 0x07;

// 0..255 (CAN payload) maps linearly to SERVO_MIN_DEG..SERVO_MAX_DEG.
// Calibrated for the actual gripper linkage by walking the slider until
// the linkage hit each hard stop:
//   slider 40  → 35° = fully closed (anything below binds the mechanism)
//   slider 110 → 79° = fully open   (anything above binds the mechanism)
// So the real mechanical travel is just 35°..79°. Re-mapping the full
// 0..255 byte range to that 44° window means the slider (and the open/
// close buttons) now exercise the entire gripper travel without any
// wasted/binding region.
//
// Slider 0   → 35° (Close button — close_position=0   in config.yaml)
// Slider 255 → 79° (Open  button — open_position=255 in config.yaml)
static const int SERVO_MIN_DEG   = 35;
static const int SERVO_MAX_DEG   = 79;
static const int SERVO_BOOT_BYTE = 255;  // boot to the "open" end, matching
                                         // arctos-pi default_position=255

#ifdef MCP_CRYSTAL_8MHZ
  static const uint8_t MCP_CLOCK = MCP_8MHZ;
#else
  static const uint8_t MCP_CLOCK = MCP_16MHZ;
#endif

MCP_CAN CAN(PIN_CAN_CS);
Servo gripperServo;

// Shared between ISR and loop. The ISR only flips the flag — all real work
// (Servo.write, Serial.print) happens in loop() to avoid the well-known
// hazard of calling those from inside an ISR.
volatile bool canFrameAvailable = false;
volatile uint32_t framesSeen   = 0;
volatile uint32_t framesForUs  = 0;

#ifdef DEBUG_SERIAL
  #define DBG(x)   Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

static void writeServoFromByte(uint8_t value) {
    int angle = map(value, 0, 255, SERVO_MIN_DEG, SERVO_MAX_DEG);
    angle = constrain(angle, SERVO_MIN_DEG, SERVO_MAX_DEG);
    gripperServo.write(angle);
}

static void blinkLed(uint8_t count, uint16_t period_ms) {
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(PIN_LED, HIGH); delay(period_ms);
        digitalWrite(PIN_LED, LOW);  delay(period_ms);
    }
}

static void onCanInterrupt() {
    canFrameAvailable = true;
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_CAN_INT, INPUT);

    delay(100); // let MCP2515 settle after power-up
    Serial.println(F("\n[arctos-gripper] booting"));
    Serial.print(F("  crystal: "));
    Serial.println(MCP_CLOCK == MCP_16MHZ ? F("16 MHz") : F("8 MHz"));

    // Bring the servo up first, including the self-test sweep, so the user
    // gets visible confirmation the servo wiring works even if the CAN bus
    // never comes up. Without this, a stuck CAN init means the servo never
    // moves and you can't tell whether the servo or the bus is broken.
    gripperServo.attach(PIN_SERVO);
    writeServoFromByte(SERVO_BOOT_BYTE);
    Serial.println(F("  servo self-test sweep"));
    for (int v = 0;   v <= 255; v += 32) { writeServoFromByte(v); delay(40); }
    for (int v = 255; v >= 0;   v -= 32) { writeServoFromByte(v); delay(40); }
    writeServoFromByte(SERVO_BOOT_BYTE);

    // Retry CAN init forever rather than locking up: during bring-up the
    // MCP2515's 5V/GND may not be connected yet, and we want the firmware
    // to start working the moment power arrives — no re-flash needed.
    // The blink pattern (3 short flashes per attempt) doubles as a visual
    // "MCP2515 not responding" indicator.
    uint16_t attempt = 0;
    while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_CLOCK) != CAN_OK) {
        ++attempt;
        Serial.print(F("  CAN init failed (attempt "));
        Serial.print(attempt);
        Serial.println(F(") — check MCP2515 power/wiring/crystal"));
        blinkLed(3, 200);
        delay(500);
    }
    // begin() already puts the chip in NORMAL mode; setting it explicitly
    // here makes the intent obvious and corrects the original sketch's
    // setMode(MCP_STDEXT) which is a filter-mode constant, not an
    // operating-mode constant.
    CAN.setMode(MCP_NORMAL);
    Serial.println(F("  CAN ok @ 500 kbps, mode=NORMAL, filter=ANY"));
    Serial.print(F("  listening for ID 0x"));
    Serial.println(GRIPPER_CAN_ID, HEX);

    attachInterrupt(digitalPinToInterrupt(PIN_CAN_INT), onCanInterrupt, FALLING);

    blinkLed(2, 100);
    Serial.println(F("  ready"));
}

static void drainCan() {
    // Pull every queued frame out of the MCP2515 RX buffers. Multiple
    // frames may have arrived while loop() was busy; clearing the flag
    // first and then draining matches the chip's two-RX-buffer hardware.
    while (CAN.checkReceive() == CAN_MSGAVAIL) {
        long unsigned int rxId = 0;
        unsigned char len = 0;
        unsigned char buf[8];
        if (CAN.readMsgBuf(&rxId, &len, buf) != CAN_OK) continue;
        framesSeen++;

        // Mask off the extended-frame flag bit so spurious extended IDs
        // don't accidentally match the standard ID we care about.
        long unsigned int id = rxId & 0x1FFFFFFFUL;
        if (id != (long unsigned int)GRIPPER_CAN_ID) continue;
        if (len < 1) continue;

        framesForUs++;
        uint8_t pos = buf[0];
        writeServoFromByte(pos);
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));

        DBG(F("  pos="));
        DBGLN(pos);
    }
}

void loop() {
    if (canFrameAvailable) {
        canFrameAvailable = false;
        drainCan();
    }
    // Polling fallback: if the INT line ever misses an edge (noise,
    // ground bounce, missed FALLING during a long ISR somewhere), the
    // poll picks frames up within a few ms.
    static uint32_t lastPoll = 0;
    if (millis() - lastPoll >= 5) {
        lastPoll = millis();
        if (CAN.checkReceive() == CAN_MSGAVAIL) drainCan();
    }
    // Heartbeat to serial every 5 s during bring-up. Counters tell you
    // whether ANY frames are reaching the Nano vs. only frames addressed
    // to the gripper — useful for narrowing down bus-vs-config issues.
    static uint32_t lastBeat = 0;
    if (millis() - lastBeat >= 5000) {
        lastBeat = millis();
        Serial.print(F("[arctos-gripper] alive, frames seen="));
        Serial.print(framesSeen);
        Serial.print(F(", for-us="));
        Serial.println(framesForUs);
    }
}

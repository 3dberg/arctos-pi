// Minimal servo-on-D9 isolation test. No MCP2515, no CAN, no SPI.
// If the servo doesn't move when this sketch is running, the fault is
// power/wiring/signal-pin — not in the gripper firmware.
//
// Behavior on boot:
//   1) D13 (LED_BUILTIN) goes ON for 1s (visual: AVR is alive).
//   2) Servo on D9 goes to 90° (mid).
//   3) Forever: sweep 0° → 180° → 0° in 1° steps, ~2s per direction.
//      The on-board LED toggles at each direction reversal so you can
//      verify the loop is running even if the servo isn't moving.
//
// Wiring (must match):
//   Servo signal (orange) -> Nano D9
//   Servo VCC    (red)    -> external 5V supply (≥2 A for MG945)
//   Servo GND    (brown)  -> external supply GND, also tied to Nano GND
//
// Build/flash:
//   ./flash.sh --sketch servo_test --port /dev/ttyUSB0
// (no --crystal flag needed — we don't touch CAN here)

#include <Servo.h>

static const uint8_t PIN_SERVO = 9;
static const uint8_t PIN_LED   = LED_BUILTIN;

Servo s;

void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    delay(1000);
    digitalWrite(PIN_LED, LOW);

    s.attach(PIN_SERVO);
    s.write(90);
    Serial.println(F("[servo-test] D9 sweeping 0..180 forever"));
    Serial.println(F("[servo-test] LED toggles at each end of sweep"));
}

void loop() {
    for (int a = 0; a <= 180; a++) { s.write(a); delay(11); }
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    Serial.println(F("[servo-test] reached 180°"));
    for (int a = 180; a >= 0; a--) { s.write(a); delay(11); }
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    Serial.println(F("[servo-test] reached 0°"));
}

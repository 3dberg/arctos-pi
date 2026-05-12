// MCP2515 SPI smoke test for the arctos-gripper Nano.
//
// Bypasses the mcp_can library and talks raw SPI to the MCP2515. Tells you
// whether SPI itself is working, before any bus-timing / CAN-config
// concerns enter the picture.
//
// What it does on every boot:
//   1. Drives CS high, sends MCP2515 RESET (0xC0)
//   2. Waits for the chip to settle in CONFIG mode (~128 osc cycles)
//   3. Reads CANSTAT (0x0E), CANCTRL (0x0F), and CNF1/2/3 registers
//   4. Prints the raw bytes plus an interpretation (floating MISO / stuck
//      MISO / SPI healthy / partial)
//
// Wiring is identical to gripper.ino:
//   MCP2515 CS  -> D10
//   MCP2515 SI  -> D11   (Nano MOSI)
//   MCP2515 SO  -> D12   (Nano MISO)
//   MCP2515 SCK -> D13
//   MCP2515 VCC -> 5V, GND -> GND

#include <SPI.h>

static const uint8_t CAN_CS = 10;

static const uint8_t CMD_RESET   = 0xC0;
static const uint8_t CMD_READ    = 0x03;
static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;
static const uint8_t REG_CNF1    = 0x2A;
static const uint8_t REG_CNF2    = 0x29;
static const uint8_t REG_CNF3    = 0x28;

static inline void cs(bool low) { digitalWrite(CAN_CS, low ? LOW : HIGH); }

static uint8_t readReg(uint8_t addr) {
    cs(true);
    SPI.transfer(CMD_READ);
    SPI.transfer(addr);
    uint8_t v = SPI.transfer(0x00);
    cs(false);
    return v;
}

static void mcpReset() {
    cs(true);
    SPI.transfer(CMD_RESET);
    cs(false);
    delay(20); // generous wait; datasheet calls for ~128 osc cycles
}

static void hex2(uint8_t v) {
    if (v < 0x10) Serial.print('0');
    Serial.print(v, HEX);
}

static void runOnce() {
    Serial.println(F("\n[mcp2515 smoke test] issuing RESET..."));
    mcpReset();

    uint8_t canstat = readReg(REG_CANSTAT);
    uint8_t canctrl = readReg(REG_CANCTRL);
    uint8_t cnf1    = readReg(REG_CNF1);
    uint8_t cnf2    = readReg(REG_CNF2);
    uint8_t cnf3    = readReg(REG_CNF3);

    Serial.print(F("  CANSTAT  = 0x")); hex2(canstat);
    Serial.println(F("   (expect 0x80: chip in CONFIG mode after reset)"));
    Serial.print(F("  CANCTRL  = 0x")); hex2(canctrl);
    Serial.println(F("   (expect 0x87: REQOP=CONFIG, CLKEN=1, CLKPRE=11)"));
    Serial.print(F("  CNF1/2/3 = 0x"));
    hex2(cnf1); Serial.print(' ');
    hex2(cnf2); Serial.print(' ');
    hex2(cnf3); Serial.println();

    Serial.println();
    if (canstat == 0xFF && canctrl == 0xFF && cnf1 == 0xFF) {
        Serial.println(F("verdict: MISO is floating (reads 0xFF for everything)."));
        Serial.println(F("  -> check MCP2515 SO is connected to Nano D12 (not D11)"));
        Serial.println(F("  -> check MCP2515 VCC=5V and GND are good"));
    } else if (canstat == 0x00 && canctrl == 0x00 && cnf1 == 0x00) {
        Serial.println(F("verdict: MISO is stuck low (reads 0x00 for everything)."));
        Serial.println(F("  -> SO shorted to GND, wrong pin, or chip not powered"));
    } else if (canstat == 0x80) {
        Serial.println(F("verdict: SPI is healthy. Chip entered CONFIG mode."));
        Serial.println(F("  If mcp_can begin() still fails, suspect crystal mismatch"));
        Serial.println(F("  (try the other --crystal value when reflashing gripper)."));
    } else {
        Serial.println(F("verdict: partial / noisy SPI."));
        Serial.println(F("  -> shorten / re-seat SPI jumpers"));
        Serial.println(F("  -> verify CS=D10 and SCK=D13 are not crossed"));
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    delay(200);

    pinMode(CAN_CS, OUTPUT);
    cs(false);
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    runOnce();
    SPI.endTransaction();
    Serial.println(F("\n(test runs once at boot — press the Nano reset button to repeat)"));
}

void loop() {}

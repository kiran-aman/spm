#include <Arduino.h>
#include <Wire.h>

// ─── I2C Scanner — diagnostic only ─────────────────────────
// Purpose: determine if the BNO085 chip is alive at all, independent
// of the SPI wiring/init path used in the main firmware.
//
// BEFORE FLASHING THIS:
//   1. Rewire PS0 and PS1 on the BNO085 to GND (was 3V3 for SPI mode)
//   2. Wire SDA -> D10 (or any free GPIO), SCL -> D8
//   3. Leave VCC, GND, INT, RST as they were
//
// Expected BNO08x I2C address: 0x4A (sometimes 0x4B if ADR pin is high)

#define SDA_PIN D10
#define SCL_PIN D8

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { /* wait briefly for host serial */ }

    Serial.println("\n=== I2C Scanner — BNO085 diagnostic ===\n");

    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("Scanning...");
}

void loop() {
    int found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("  Device found at address 0x%02X", addr);
            if (addr == 0x4A || addr == 0x4B) {
                Serial.print("  <-- BNO08x expected address!");
            }
            Serial.println();
            found++;
        }
    }

    if (found == 0) {
        Serial.println("No I2C devices found.");
    } else {
        Serial.printf("Scan complete. %d device(s) found.\n", found);
    }

    Serial.println("---");
    delay(3000);
}
#include <Arduino.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_BNO08x.h>
#include "platform_config.h"

// 1 -> print to serial port (debugging)
// 0 -> disable print to serial port (not debugging)
#define DEBUG_SERIAL_PRINT 1

// ─── BNO085 instance (SPI mode) ────────────────────────────
Adafruit_BNO08x bno08x(PIN_BNO_RST);
sh2_SensorValue_t sensorValue;

// ─── ESP-NOW packet ─────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    float    quat_i;
    float    quat_j;
    float    quat_k;
    float    quat_real;
    float    quat_accuracy_rad;
    float    battery_voltage;
} platform_packet_t;

static platform_packet_t pkt;

// Host dongle's MAC address — fill in once you've read it off the ESP32C6 dongle
static uint8_t HOST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // TODO: replace

// ─── Interrupt flag ─────────────────────────────────────────
// ISR only sets a flag — actual SPI read happens in loop(), never inside an ISR.
static volatile bool bno_data_ready = false;

void IRAM_ATTR bno_isr() {
    bno_data_ready = true;
}

// ─── Battery voltage tracking ───────────────────────────────
static float last_battery_voltage = 0.0f;
static uint32_t last_batt_sample_ms = 0;

static void sample_battery() {
    uint32_t sum_mv = 0;
    for (int i = 0; i < BATT_SAMPLE_COUNT; i++) {
        sum_mv += analogReadMilliVolts(PIN_BATT_ADC);
    }
    float avg_mv = (float)sum_mv / BATT_SAMPLE_COUNT;
    // XIAO ESP32C6 onboard divider is 1:2 -> multiply by 2 to recover true battery voltage
    last_battery_voltage = (avg_mv / 1000.0f) * 2.0f;
}

// ─── BNO085 setup ────────────────────────────────────────────
static bool setup_bno_reports() {
    // ARVR-stabilized rotation vector: gyro+accel fused, mag influence minimized
    // to avoid yaw jumps.
    if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, BNO_REPORT_INTERVAL_US)) {
        Serial.println("[ERR] Failed to enable ARVR-stabilized rotation vector");
        return false;
    }
    return true;
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    // while (!Serial && millis() - t0 < 3000) { /* wait briefly for host serial */ }
    delay(3000);

    Serial.println("\n=== Platform ESP32C6 — BNO085 + ESP-NOW ===\n");

    pinMode(PIN_BATT_ADC, INPUT);

    // ── SPI / BNO085 init ──
    pinMode(PIN_BNO_INT, INPUT_PULLUP);
    delay(3000);

    // ESP32 SPI pins are configurable, not fixed — must explicitly map the bus
    // to D8/D9/D10 or it may default to different physical pins than wired.
    SPI.begin(PIN_BNO_SCK, PIN_BNO_MISO, PIN_BNO_MOSI, PIN_BNO_CS);

    if (!bno08x.begin_SPI(PIN_BNO_CS, PIN_BNO_INT)) {
        Serial.println("[ERR] BNO085 not detected over SPI — check wiring/PS0/PS1");
        while (1) delay(10);
    }
    Serial.println("[OK] BNO085 detected");

    if (!setup_bno_reports()) {
        while (1) delay(10);
    }

    // Attach interrupt AFTER begin_SPI/enableReport so we don't race init traffic
    attachInterrupt(digitalPinToInterrupt(PIN_BNO_INT), bno_isr, FALLING);

    // ── ESP-NOW init ──
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERR] ESP-NOW init failed");
        while (1) delay(10);
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, HOST_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[ERR] Failed to add ESP-NOW peer — set HOST_MAC first");
        while (1) delay(10);
    }

    sample_battery();
    last_batt_sample_ms = millis();

    Serial.println("[OK] Ready — streaming quaternions over ESP-NOW\n");
}

void loop() {
    // ── Battery sampling (slow, non-blocking) ──
    uint32_t now_ms = millis();
    if (now_ms - last_batt_sample_ms >= BATT_REPORT_MS) {
        sample_battery();
        last_batt_sample_ms = now_ms;
    }

    // ── BNO085 read on interrupt ──
    if (bno_data_ready) {
        bno_data_ready = false;

        // getSensorEvent() internally handles the SPI transaction and
        // clears the interrupt condition on the chip.
        if (bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
                pkt.timestamp_us      = micros();
                pkt.quat_i            = sensorValue.un.arvrStabilizedRV.i;
                pkt.quat_j            = sensorValue.un.arvrStabilizedRV.j;
                pkt.quat_k            = sensorValue.un.arvrStabilizedRV.k;
                pkt.quat_real         = sensorValue.un.arvrStabilizedRV.real;
                pkt.quat_accuracy_rad = sensorValue.un.arvrStabilizedRV.accuracy;
                pkt.battery_voltage   = last_battery_voltage;

                #if DEBUG_SERIAL_PRINT
                Serial.printf("t=%lu  i=%.4f j=%.4f k=%.4f real=%.4f  acc=%.4f  batt=%.2fV\n",
                    pkt.timestamp_us, pkt.quat_i, pkt.quat_j, pkt.quat_k,
                    pkt.quat_real, pkt.quat_accuracy_rad, pkt.battery_voltage);
                #endif

                esp_now_send(HOST_MAC, (uint8_t*)&pkt, sizeof(pkt));
            }
        }
    }
}
#include <Arduino.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_BNO08x.h>
#include "pins.h"

// bno085 object using hardware spi
Adafruit_BNO08x bno08x(PIN_BNO_RST);
sh2_SensorValue_t sensorValue;

// esp-now packet
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

// base station esp32c6 mac address
static uint8_t HOST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // TODO: replace

// interrupt flag for bno085 data ready
static volatile bool bno_data_ready = false;

void IRAM_ATTR bno_isr() {
    bno_data_ready = true;
}

// battery voltage tracking
static float last_battery_voltage = 0.0f;
static uint32_t last_batt_sample_ms = 0;

static void sample_battery() {
    uint32_t sum_mv = 0;
    for (int i = 0; i < BATT_SAMPLE_COUNT; i++) {
        sum_mv += analogReadMilliVolts(PIN_BATT_ADC);
    }
    float avg_mv = (float)sum_mv / BATT_SAMPLE_COUNT;
    last_battery_voltage = (avg_mv / 1000.0f) * 2.0f;
}

// bno085 setup
static bool setup_bno_reports() {
    // arvr-stabilized rotation vector: gyro+accel fused, mag influence minimized
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
    while (!Serial && millis() - t0 < 3000) { /* wait briefly for host serial */ }

    Serial.println("\n=== Platform ESP32C6 — BNO085 + ESP-NOW ===\n");

    pinMode(PIN_BATT_ADC, INPUT);

    // spi/bno085 init
    pinMode(PIN_BNO_INT, INPUT_PULLUP);

    if (!bno08x.begin_SPI(PIN_BNO_CS, PIN_BNO_INT)) {
        Serial.println("[ERR] BNO085 not detected over SPI — check wiring/PS0/PS1");
        while (1) delay(10);
    }
    Serial.println("[OK] BNO085 detected");

    if (!setup_bno_reports()) {
        while (1) delay(10);
    }

    attachInterrupt(digitalPinToInterrupt(PIN_BNO_INT), bno_isr, FALLING);

    // esp-now init
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
    // battery sampling
    uint32_t now_ms = millis();
    if (now_ms - last_batt_sample_ms >= BATT_REPORT_MS) {
        sample_battery();
        last_batt_sample_ms = now_ms;
    }

    // bno085 data ready interrupt handling
    if (bno_data_ready) {
        bno_data_ready = false;

        if (bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
                pkt.timestamp_us      = micros();
                pkt.quat_i            = sensorValue.un.arvrStabilizedRV.i;
                pkt.quat_j            = sensorValue.un.arvrStabilizedRV.j;
                pkt.quat_k            = sensorValue.un.arvrStabilizedRV.k;
                pkt.quat_real         = sensorValue.un.arvrStabilizedRV.real;
                pkt.quat_accuracy_rad = sensorValue.un.arvrStabilizedRV.accuracy;
                pkt.battery_voltage   = last_battery_voltage;

                esp_now_send(HOST_MAC, (uint8_t*)&pkt, sizeof(pkt));
            }
        }
    }
}
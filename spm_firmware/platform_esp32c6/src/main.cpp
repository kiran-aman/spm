#include <Arduino.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_BNO08x.h>
#include <cmath>
#include "platform_config.h"

// 1 -> print to serial port (debugging)
// 0 -> disable print to serial port (not debugging)
#define DEBUG_SERIAL_PRINT 1

Adafruit_BNO08x bno08x(PIN_BNO_RST);
sh2_SensorValue_t sensorValue;

// data packet esp now
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

// USE ACTUAL MAC ADDRESS
static uint8_t HOST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static volatile bool bno_data_ready = false;

void IRAM_ATTR bno_isr() {
    bno_data_ready = true;
}

static float last_battery_voltage = 0.0f;
static uint32_t last_batt_sample_ms = 0;

struct Quaternion {
    double w, x, y, z;
};

struct Vector3 {
    double roll, pitch, yaw;
};
static bool isFirstRead = true;
static Quaternion q_tare = {1.0, 0.0, 0.0, 0.0};

Quaternion inverseQuaternion(const Quaternion& q) {
    return Quaternion{ q.w, -q.x, -q.y, -q.z };
}


Vector3 quaternionToRPY(const Quaternion& q) {
    Vector3 rpy;
    
    // roll
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    rpy.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        rpy.pitch = std::copysign(M_PI / 2, sinp); // Gimbal lock
    else
        rpy.pitch = std::asin(sinp);

    // yaw
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    rpy.yaw = std::atan2(siny_cosp, cosy_cosp);

    return rpy;
}

Quaternion multiplyQuaternions(const Quaternion& q1, const Quaternion& q2) {
    return Quaternion{
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    };
}

Quaternion rpyToQuaternion(double roll, double pitch, double yaw) {
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);

    return Quaternion{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };
}

static void sample_battery() {
    uint32_t sum_mv = 0;
    for (int i = 0; i < BATT_SAMPLE_COUNT; i++) {
        sum_mv += analogReadMilliVolts(PIN_BATT_ADC);
    }
    float avg_mv = (float)sum_mv / BATT_SAMPLE_COUNT;
    last_battery_voltage = (avg_mv / 1000.0f) * 2.0f;
}

static bool setup_bno_reports() {
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US)) {
        Serial.println("[ERR] Failed to enable gamerotation-stabilized rotation vector");
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
            if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
                pkt.timestamp_us      = micros();
                pkt.quat_i            = sensorValue.un.gameRotationVector.i;
                pkt.quat_j            = sensorValue.un.gameRotationVector.j;
                pkt.quat_k            = sensorValue.un.gameRotationVector.k;
                pkt.quat_real         = sensorValue.un.gameRotationVector.real;
                pkt.battery_voltage   = last_battery_voltage;

                #if DEBUG_SERIAL_PRINT
                
                

                // Serial.printf("%.4f,%.4f,%.4f,%.4f\n",
                //     pkt.quat_i, pkt.quat_j, pkt.quat_k, pkt.quat_real);
                // Serial.printf("t=%lu  i=%.4f j=%.4f k=%.4f real=%.4f  acc=%.4f  batt=%.2fV\n",
                //     pkt.timestamp_us, pkt.quat_i, pkt.quat_j, pkt.quat_k,
                //     pkt.quat_real, pkt.quat_accuracy_rad, pkt.battery_voltage);
                #endif
                
                // MATCH ORIENTATION OF INVERSE KINEMATICS
                Quaternion q_raw = { pkt.quat_real, pkt.quat_i, pkt.quat_j, pkt.quat_k };
                Quaternion z90Clockwise = {0.70710678118, 0.0, 0.0, 0.70710678118};
                Quaternion q_aligned = multiplyQuaternions(q_raw, z90Clockwise);

                if (isFirstRead) {
                    q_tare = q_aligned; 
                    isFirstRead = false;
                    Serial.println("IMU Tared to current heading!");
                }
                Quaternion q_inverse_tare = inverseQuaternion(q_tare);
                Quaternion q_final = multiplyQuaternions(q_inverse_tare, q_aligned);

                Serial.printf("%.4f,%.4f,%.4f,%.4f\n",
                    q_final.x, q_final.y, q_final.z, q_final.w);
                
                // Vector3 rpy = quaternionToRPY(q_final);
                // Serial.printf("t=%lu  roll=%.4f pitch=%.4f yaw=%.4f\n",
                //     pkt.timestamp_us, rpy.roll, rpy.pitch, rpy.yaw);


                esp_now_send(HOST_MAC, (uint8_t*)&pkt, sizeof(pkt));
            }
        }
    }
}
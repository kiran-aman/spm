#include <Arduino.h>
#include "tmc2209.h"
#include "encoder.h"
#include "stepperdriver.h"
#include "ik.h"
#include "trajectory.h"

// ─── Control loop timing ───────────────────────────────────
static constexpr float    CTRL_HZ   = 100.0f;
static constexpr uint32_t CTRL_US   = 1000000 / CTRL_HZ;
static constexpr float    CTRL_DT   = 1.0f / CTRL_HZ;

static uint32_t _last_ctrl_us = 0;

// ─── Trajectory ────────────────────────────────────────────
static TrajectoryInterpolator traj;
static bool traj_running = false;
static RPY  current_rpy  = {0.0f, 0.0f, 0.0f};

// ─── Setup ────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("\n=== 3-RRR SPM Firmware ===\n");

    tmc_init_all();
    encoder_init_all();
    stepper_init_all();
    ik_reset_home();

    tmc_enable(true);

    Serial.println("Ready.");
    Serial.println("Commands:");
    Serial.println("  t — TMC status");
    Serial.println("  e — encoder readings");
    Serial.println("  h — go to home");
    Serial.println("  1 — test: 10 deg pitch over 2s");
    Serial.println("  2 — test: 20 deg roll over 2s");
    Serial.println("  s — e-stop");
    Serial.println("  x — re-enable motors");
}

// ─── Control loop ─────────────────────────────────────────
static void control_loop() {
    if (!traj_running) return;

    RPY target = traj.update(CTRL_DT);
    current_rpy = target;

    IKResult result = ik(target.roll, target.pitch, target.yaw);

    if (!result.valid) {
        Serial.println("[WARN] IK failed — stopping");
        traj_running = false;
        stepper_stop_all();
        return;
    }

    stepper_move_to_deg(1, degrees(result.theta[0]));
    stepper_move_to_deg(2, degrees(result.theta[1]));
    stepper_move_to_deg(3, degrees(result.theta[2]));

    if (traj.is_done()) {
        traj_running = false;
        Serial.printf("[TRAJ] Done — roll=%.2f pitch=%.2f yaw=%.2f deg\n",
            degrees(target.roll), degrees(target.pitch), degrees(target.yaw));
    }
}

// ─── Loop ─────────────────────────────────────────────────
void loop() {
    uint32_t now = micros();
    if (now - _last_ctrl_us >= CTRL_US) {
        _last_ctrl_us = now;
        control_loop();

        for (uint8_t i = 1; i <= 3; i++) {
            if (stepper_is_running(i) && tmc_stall_detected(i)) {
                Serial.printf("[WARN] Motor %d stall — e-stop\n", i);
                stepper_stop_all();
                tmc_enable(false);
                traj_running = false;
            }
        }
    }

    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 't':
                tmc_print_status(1);
                tmc_print_status(2);
                tmc_print_status(3);
                break;

            case 'e':
                for (uint8_t i = 1; i <= 3; i++)
                    Serial.printf("  Motor %d: counts=%d deg=%.2f\n",
                        i, encoder_read(i), encoder_degrees(i));
                break;

            case 'h': {
                RPY home = {0.0f, 0.0f, 0.0f};
                traj.set_target(current_rpy, home, 2.0f);
                traj_running = true;
                Serial.println("[TRAJ] Going home...");
                break;
            }

            case '1': {
                RPY end = {0.0f, radians(10.0f), 0.0f};
                traj.set_target(current_rpy, end, 2.0f);
                traj_running = true;
                Serial.println("[TRAJ] 10 deg pitch over 2s...");
                break;
            }

            case '2': {
                RPY end = {radians(20.0f), 0.0f, 0.0f};
                traj.set_target(current_rpy, end, 2.0f);
                traj_running = true;
                Serial.println("[TRAJ] 20 deg roll over 2s...");
                break;
            }

            case 's':
                stepper_stop_all();
                tmc_enable(false);
                traj_running = false;
                Serial.println("[ESTOP] Motors disabled");
                break;

            case 'x':
                tmc_enable(true);
                Serial.println("[OK] Motors re-enabled");
                break;
        }
    }
}
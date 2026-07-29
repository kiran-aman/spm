#include <Arduino.h>
#include "tmc2209.h"
#include "encoder.h"
#include "stepper_driver.h"
#include "kinematics/ik.h"
#include "kinematics/trajectory.h"

// control loop timing
static constexpr float    CTRL_HZ   = 100.0f;
static constexpr uint32_t CTRL_US   = 1000000 / CTRL_HZ;
static constexpr float    CTRL_DT   = 1.0f / CTRL_HZ;
static uint32_t _last_ctrl_us = 0;

static StepperGroup stepper_group = {s1, s2, s3};

// trajectory
static TrajectoryInterpolator traj;
static bool traj_running = false;
static RPY  current_rpy  = {0.0f, 0.0f, 0.0f};
static float _prev_target_deg[4] = {90.0f, 90.0f, 90.0f, 90.0f};

static int timer = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    delay(1000);

    Serial.println("\n=== 3-RRR SPM Firmware ===\n");

    tmc_init_all();
    encoder_init_all();
    stepper_init_all();
    
    delay(100);

    encoder_reset(1);
    encoder_reset(2);
    encoder_reset(3);
    ik_reset_home();

    tmc_enable(true);
    Serial.println("ready");
}

// ─── Control loop ─────────────────────────────────────────
static void control_loop() {
    if (!traj_running) return;

    RPY target = traj.update(CTRL_DT);
    current_rpy = target;
    IKResult result = ik(target.roll, target.pitch, target.yaw);

    // Serial.printf("[CTRL] t=%.2f t1=%.2f t2=%.2f t3=%.2f\n",
    //     traj.elapsed(),
    //     degrees(result.theta[0]),
    //     degrees(result.theta[1]),
    //     degrees(result.theta[2]));
 
    if (!result.valid) {
        Serial.println("[WARN] IK failed — stopping");
        traj_running = false;
        stepper_stop_all();
        return;
    }

    for (uint8_t i = 1; i <= 3; i++) {
        float target_deg  = degrees(result.theta[i-1]);
        float delta_deg   = target_deg - _prev_target_deg[i];
        float speed_hz = delta_deg * STEPS_PER_DEG * CTRL_HZ;

        constexpr float MIN_SPEED_HZ = 5.0f; // floor instead of 0.1f threshold-stop
        float clamped_speed_hz = speed_hz;
        if (fabsf(clamped_speed_hz) < MIN_SPEED_HZ) {
            clamped_speed_hz = (clamped_speed_hz < 0.0f) ? -MIN_SPEED_HZ : MIN_SPEED_HZ;
        }
        stepper_set_speed(i, clamped_speed_hz / MAX_SPEED_HZ);

        _prev_target_deg[i] = target_deg;
    }

    if (traj.is_done()) {
        traj_running = false;
        steppers_end_rotation();
        Serial.printf("[TRAJ] Done — roll=%.2f pitch=%.2f yaw=%.2f deg\n",
            degrees(target.roll), degrees(target.pitch), degrees(target.yaw));
        Serial.printf("final time: %.3f sec\n", (millis() - timer) * 1e-3f);
        timer = millis();
    }
}


void loop() {
    // run control loop at fixed frequency
    uint32_t now = micros();
    if (now - _last_ctrl_us >= CTRL_US) {
        _last_ctrl_us = now;
        control_loop();
        float angles[3];
        for (uint8_t i = 1; i <= 3; i++) {
            angles[i-1] = encoder_degrees(i);
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

            case '1': {
                Serial.println("SPINNING MOTOR 1...");
                s1.rotateAsync(0.5);
                break;
            }

            case '2': {
                Serial.println("SPINNING MOTOR 2...");
                s3.rotateAsync(0.1);
                break;
            }

            case '3': {
                steppers_start_rotation();
                RPY end = {radians(90.0f), 0.0f, 0.0f};  // 180 deg yaw spin over 2s
                timer = millis();
                traj.set_target(current_rpy, end, 2.0f);
                traj_running = true;
                Serial.println("[TRAJ] 180 deg yaw spin over 2s...");
                break;
            }

            case '4': {
                Serial.printf("[DEBUG] current_rpy at start: roll=%.2f pitch=%.2f yaw=%.2f\n",
                    degrees(current_rpy.roll),
                    degrees(current_rpy.pitch),
                    degrees(current_rpy.yaw));
                steppers_start_rotation();
                RPY end = {radians(360.0f), 0.0f, radians(20.0f)};  // 180 deg roll spin over 2s
                timer = millis();
                traj.set_target(current_rpy, end, 10.0f);
                traj_running = true;
                Serial.println("[TRAJ] 180 deg roll spin over 2s...");
                break;
            }

            case 'e': {
                // read encoders
                float angles[3];
                for (uint8_t i = 1; i <= 3; i++) {
                    angles[i-1] = encoder_degrees(i);
                }
                Serial.printf("[ENCODER] angles: %.2f, %.2f, %.2f\n",
                    angles[0],
                    angles[1],
                    angles[2]);
                break;
            }

            case 'h': {
                timer = millis();
                steppers_start_rotation();
                RPY end = {0.0f, 0.0f, 0.0f};
                traj.set_target(current_rpy, end, 1.0f);
                traj_running = true;
                Serial.println("[TRAJ] Home over 0.5s...");
                break;
            }

            case 'v': {
                // IK validation — print joint angles along pitch trajectory
                // compare against Python output
                ik_reset_home();
                float t_f = 2.0f;
                int N = 200;
                float prev_thetas[3] = {radians(90.0f), radians(90.0f), radians(90.0f)};
                
                Serial.println("t,pitch_deg,theta1,theta2,theta3");
                for (int k = 0; k < N; k++) {
                    float t = k * (t_f / N);
                    float s = 3*pow(t/t_f,2) - 2*pow(t/t_f,3);
                    float yaw = s * radians(20.0f);
                    float roll = s * radians(360.0f);
                    
                    IKResult result = ik(roll, 0.0f, yaw);
                    if (!result.valid) {
                        Serial.printf("%.3f,%.4f,FAIL,FAIL,FAIL\n", t, degrees(roll));
                        continue;
                    }
                    Serial.printf("%.3f,%.4f,%.4f,%.4f\n",
                        t,
                        degrees(result.theta[0]),
                        degrees(result.theta[1]),
                        degrees(result.theta[2]));
                }
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
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
    ik_reset_home();

    tmc_enable(true);
}

// ─── Control loop ─────────────────────────────────────────
static void control_loop() {
    if (!traj_running) return;
    uint32_t t0, t1, t2, t3;    

    t0 = micros();
    RPY target = traj.update(CTRL_DT);
    current_rpy = target;

    t1 = micros();
    IKResult result = ik(target.roll, target.pitch, target.yaw);
    // Serial.printf("[TRAJ] target: roll=%.2f pitch=%.2f yaw=%.2f deg\n",
    //     degrees(target.roll), degrees(target.pitch), degrees(target.yaw));
    // Serial.printf("%.4f,%.4f,%.4f\n",
    //                 degrees(result.theta[0]),
    //                 degrees(result.theta[1]),
    //                 degrees(result.theta[2]));
    // Serial.println(result.theta[0] == result.theta[1] && result.theta[1] == result.theta[2]);
 
    t2 = micros();
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

        stepper_set_speed(i, fabs(speed_hz) / MAX_SPEED_HZ); // set speed as fraction of max speed

        _prev_target_deg[i] = target_deg;
        // delay(50);
    }

    // stepper_move_to_deg(1, degrees(result.theta[0]) - 90.0f); // offset for home position
    // stepper_move_to_deg(2, degrees(result.theta[1]) - 90.0f);
    // stepper_move_to_deg(3, degrees(result.theta[2]) - 90.0f);
    // stepper_group.move();  // non-blocking run for all motors

    t3 = micros();

    // // print every 100 iterations
    // static uint32_t _dbg_count = 0;
    // if (++_dbg_count % 100 == 0) {
    //     Serial.printf("[TIME] traj=%luus ik=%luus stepper=%luus\n",
    //         t1-t0, t2-t1, t3-t2);
    // }
 
    if (traj.is_done()) {
        traj_running = false;
        Serial.printf("[TRAJ] Done — roll=%.2f pitch=%.2f yaw=%.2f deg\n",
            degrees(target.roll), degrees(target.pitch), degrees(target.yaw));
        Serial.printf("final time: %.3f sec\n", (millis() - timer) * 1e-3f);
        timer = millis();
    }
    // delay(1);
}


void loop() {
    uint32_t now = micros();
    if (now - _last_ctrl_us >= CTRL_US) {
        // static uint32_t last_debug = 0;
        // static uint32_t loop_count = 0;

        // loop_count++;
        // if (millis() - last_debug >= 1000) {
        //     Serial.printf("[TIMING] loops/sec=%d CTRL_US=%d\n", loop_count, CTRL_US);
        //     loop_count = 0;
        //     last_debug = millis();
        // }        
        _last_ctrl_us = now;
        control_loop();
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
                RPY end = {0.0f, 0.0f, radians(180.0f)};  // 180 deg yaw spin over 2s
                traj.set_target(current_rpy, end, 2.0f);
                traj_running = true;
                Serial.println("[TRAJ] 180 deg yaw spin over 2s...");
                break;
            }

            case '4': {
                timer = millis();
                RPY end = {0.0f, 0.0f, radians(180.0f)};
                traj.set_target(current_rpy, end, 2.0f);
                traj_running = true;
                Serial.println("[TRAJ] 10 deg pitch over 2s...");
                break;
            }
            case '5': {
                ik_reset_home();
                int N = 20;  // fewer points, larger steps
                float t_f = 2.0f;

                for (int k = 0; k < N; k++) {
                    float t = (k + 1) * (t_f / N);
                    float s = 3*pow(t/t_f, 2) - 2*pow(t/t_f, 3);
                    float pitch = s * radians(30.0f);

                    IKResult result = ik(0.0f, pitch, 0.0f);
                    if (!result.valid) break;

                    for (uint8_t i = 1; i <= 3; i++) {
                        int32_t steps = (int32_t)((degrees(result.theta[i-1]) - 60.0f) * STEPS_PER_DEG);
                        stepper_set_speed(i, MAX_SPEED_HZ);
                        stepper_move_to(i, steps);
                    }

                    // wait until all motors reach target
                    while (stepper_is_running(1) || stepper_is_running(2) || stepper_is_running(3)) {
                        delay(1);
                    }
                }
                break;
            }

            case 'h': {
                timer = millis();
                RPY end = {0.0f, 0.0f, 0.0f};
                traj.set_target(current_rpy, end, 0.5f);
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
                float prev_thetas[3] = {radians(60.0f), radians(60.0f), radians(60.0f)};
                
                Serial.println("t,pitch_deg,theta1,theta2,theta3");
                for (int k = 0; k < N; k++) {
                    float t = k * (t_f / N);
                    float s = 3*pow(t/t_f,2) - 2*pow(t/t_f,3);
                    float pitch = s * radians(180.0f);
                    
                    IKResult result = ik(0.0f, 0.0f, pitch);
                    if (!result.valid) {
                        Serial.printf("%.3f,%.4f,FAIL,FAIL,FAIL\n", t, degrees(pitch));
                        continue;
                    }
                    Serial.printf("%.3f,%.4f,%.4f,%.4f,%.4f\n",
                        t,
                        degrees(pitch),
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
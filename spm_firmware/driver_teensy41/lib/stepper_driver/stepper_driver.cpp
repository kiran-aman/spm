#include "stepper_driver.h"

// Stepper instances — STEP pin, DIR pin
Stepper s1(MOT1_STEP_PIN, MOT1_DIR_PIN);
Stepper s2(MOT2_STEP_PIN, MOT2_DIR_PIN);
Stepper s3(MOT3_STEP_PIN, MOT3_DIR_PIN);

bool steppers_running = false;
bool stepper1_running = false;
bool stepper2_running = false;
bool stepper3_running = false;

static int32_t target[4] = {0, 0, 0};

void stepper_init_all() {
    TS4::begin();

    for (Stepper* s : {&s1, &s2, &s3}) {
        s->setMaxSpeed(MAX_SPEED_HZ);
        s->setAcceleration(MAX_ACCEL_HZ);
    }

    Serial.println("teensystep4 initialized successfully");
}

void stepper_move_rel(uint8_t motor, int32_t steps) {
    switch (motor) {
        case 1: s1.moveRel(steps); break;
        case 2: s2.moveRel(steps); break;
        case 3: s3.moveRel(steps); break;
    }
}

void stepper_move_to(uint8_t motor, int32_t steps) {
    // write target position to target
    target[motor-1] = steps;

    switch (motor) {
        case 1: s1.setTargetAbs(steps); break;
        case 2: s2.setTargetAbs(steps); break;
        case 3: s3.setTargetAbs(steps); break;
    }
}

void stepper_move_to_deg(uint8_t motor, float degrees) {
    int32_t steps = (int32_t)(degrees * STEPS_PER_DEG);
    stepper_move_to(motor, steps);
}

void stepper_restart_rotation(uint8_t motor) {
    switch (motor) {
        case 1: 
            if(!stepper1_running) {
                Serial.println("[CTRL] restarting motor 1 rotation...");
                s1.rotateAsync();
                stepper1_running = true;
            }
            break;
        case 2: 
            if(!stepper2_running) {
                Serial.println("[CTRL] restarting motor 2 rotation...");
                s2.rotateAsync();
                stepper2_running = true;
            }
            break;
        case 3: 
            if(!stepper3_running) {
                Serial.println("[Ctril] restarting motor 3");
                s3.rotateAsync();
                stepper3_running = true;
            }
            break;
    }
}

void stepper_start_rotation(uint8_t motor) {
    switch (motor) {
        case 1: s1.rotateAsync(); break;
        case 2: s2.rotateAsync(); break;
        case 3: s3.rotateAsync(); break;
    }
}

void stepper_stop_rotation(uint8_t motor) {
    switch (motor) {
        case 1: 
            s1.stopAsync();
            stepper1_running = false;
            // target[0] = s1.getPosition();
            break;
        case 2: 
            s2.stopAsync();
            stepper2_running = false;
            // target[1] = s2.getPosition();
            break;
        case 3: 
            s3.stopAsync();
            stepper3_running = false;
            // target[2] = s3.getPosition();
            break;
    }
}

void steppers_start_rotation() {
    Serial.println("[CTRL] starting steppers rotation...");
    steppers_running = true;
    
    s1.overrideSpeed(0.1f);
    s2.overrideSpeed(0.1f);
    s3.overrideSpeed(0.1f);

    stepper_restart_rotation(1);
    stepper_restart_rotation(2);
    stepper_restart_rotation(3);
}

void steppers_end_rotation() {
    Serial.println("[CTRL] stopping steppers rotation...");
    steppers_running = false;
    stepper1_running = false;
    stepper2_running = false;
    stepper3_running = false;
    // s1.overrideSpeed(0.1f);
    // s2.overrideSpeed(0.1f);
    // s3.overrideSpeed(0.1f);


    s1.stopAsync();
    s2.stopAsync();
    s3.stopAsync();
}

void stepper_stop(uint8_t motor) {
    switch (motor) {
        case 1: 
            s1.stop();
            target[0] = s1.getPosition();
            break;
        case 2: 
            s2.stop();
            target[1] = s2.getPosition();
            break;
        case 3: 
            s3.stop();
            target[2] = s3.getPosition();
            break;
    }
}

void stepper_set_speed(uint8_t motor, float prop) {
    switch (motor) {
        case 1: s1.overrideSpeed(prop); break;
        case 2: s2.overrideSpeed(prop); break;
        case 3: s3.overrideSpeed(prop); break;
    }
}

void stepper_stop_all() {
    s1.stop();
    s2.stop();
    s3.stop();
}

bool stepper_is_running(uint8_t motor) {
    switch (motor) {
        case 1: return stepper1_running;
        case 2: return stepper2_running;
        case 3: return stepper3_running;
        default: return false;
    }
}

int32_t stepper_position(uint8_t motor) {
    switch (motor) {
        case 1: return s1.getPosition();
        case 2: return s2.getPosition();
        case 3: return s3.getPosition();
        default: return 0;
    }
}

void stepper_print_status(uint8_t motor) {
    switch (motor) {
        case 1: 
            Serial.printf("[MOTOR 1] pos=%d, target=%d, running=%d\n", s1.getPosition(), target[0], stepper1_running);
            break;
        case 2: 
            Serial.printf("[MOTOR 2] pos=%d, target=%d, running=%d\n", s2.getPosition(), target[1], stepper2_running);
            break;
        case 3: 
            Serial.printf("[MOTOR 3] pos=%d, target=%d, running=%d\n", s3.getPosition(), target[2], stepper3_running);
            break;
    }
}
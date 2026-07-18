#include "stepper_driver.h"

// Steps per degree at output shaft (microsteps * gear ratio / 360)
static constexpr float STEPS_PER_DEG =
    (STEPS_PER_REV_MICRO * GEAR_RATIO) / 360.0f;

// Stepper instances — STEP pin, DIR pin
Stepper s1(MOT1_STEP_PIN, MOT1_DIR_PIN);
Stepper s2(MOT2_STEP_PIN, MOT2_DIR_PIN);
Stepper s3(MOT3_STEP_PIN, MOT3_DIR_PIN);

static int32_t target[4] = {0, 0, 0};

void stepper_init_all() {
    TS4::begin();

    for (Stepper* s : {&s1, &s2, &s3}) {
        s->setMaxSpeed(MAX_SPEED_HZ);
        s->setAcceleration(MAX_ACCEL_HZ);
    }

    Serial.println("teensystep4 initialized successfully");
}

void stepper_spin(uint8_t motor) {
    switch (motor) {
        case 1: s1.rotateAsync(0.25); break;
        case 2: s2.rotateAsync(0.25); break;
        case 3: s3.rotateAsync(0.25); break;
    }
}

void stepper_move_to(uint8_t motor, int32_t steps) {
    // write target position to target
    target[motor-1] = steps;

    switch (motor) {
        case 1: s1.moveAbs(steps); break;
        case 2: s2.moveAbs(steps); break;
        case 3: s3.moveAbs(steps); break;
    }
}

void stepper_move_to_deg(uint8_t motor, float degrees) {
    int32_t steps = (int32_t)(degrees * STEPS_PER_DEG);
    stepper_move_to(motor, steps);
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

void stepper_stop_all() {
    s1.stop();
    s2.stop();
    s3.stop();
}

bool stepper_is_running(uint8_t motor) {
    return stepper_position(motor) != target[motor-1];
}

int32_t stepper_position(uint8_t motor) {
    switch (motor) {
        case 1: return s1.getPosition();
        case 2: return s2.getPosition();
        case 3: return s3.getPosition();
        default: return 0;
    }
}
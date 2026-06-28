#include "tmc2209.h"

// tmc2209 motor drivers
TMC2209Stepper tmc1(&TMC_SERIAL, TMC_RSENSE, TMC1_ADDR);
TMC2209Stepper tmc2(&TMC_SERIAL, TMC_RSENSE, TMC2_ADDR);
TMC2209Stepper tmc3(&TMC_SERIAL, TMC_RSENSE, TMC3_ADDR);

static void _init_driver(TMC2209Stepper &drv) {
    // tmc2209 + motor initialization
    drv.begin();
    drv.toff(5);
    drv.rms_current(RMS_CURRENT_MA);
    drv.microsteps(MICROSTEPS);
    drv.en_spreadCycle(false);
    drv.pwm_autoscale(true);
    drv.pwm_autograd(true);
    drv.SGTHRS(100);
}

void tmc_init_all() {
    // start tmc uart comm (serial1)
    TMC_SERIAL.begin(115200);

    // initialize shared enable pin active low
    pinMode(MOT_ENABLE_PIN, OUTPUT);
    digitalWrite(MOT_ENABLE_PIN, HIGH); // disable motors during initialization

    // initialize tmc diag pins
    pinMode(TMC1_DIAG_PIN, INPUT);
    pinMode(TMC2_DIAG_PIN, INPUT);
    pinMode(TMC3_DIAG_PIN, INPUT);

    // initialize each driver
    _init_driver(tmc1);
    _init_driver(tmc2);
    _init_driver(tmc3);

    // confirmation
    Serial.println("tmc initialization success");
    Serial.printf("[TMC] drv1 microsteps: %d\n", tmc1.microsteps());
    Serial.printf("[TMC] drv2 microsteps: %d\n", tmc2.microsteps());
    Serial.printf("[TMC] drv3 microsteps: %d\n", tmc3.microsteps());
}

void tmc_enable(bool en) {
    digitalWrite(MOT_ENABLE_PIN, en ? LOW : HIGH);
}

bool tmc_detect_stall(uint8_t motor) {
    switch(motor) {
        case 1: return digitalRead(TMC1_DIAG_PIN);
        case 2: return digitalRead(TMC2_DIAG_PIN);
        case 3: return digitalRead(TMC3_DIAG_PIN);
        default: return false;
    }
}

void tmc_print_status(uint8_t motor) {
    TMC2209Stepper *drv = nullptr;
    switch (motor) {
        case 1: drv = &tmc1; break;
        case 2: drv = &tmc2; break;
        case 3: drv = &tmc3; break;
        default: return;
    }
    Serial.printf("[TMC%d] current_mA=%d microsteps=%d stall=%d ot=%d otpw=%d\n",
        motor,
        RMS_CURRENT_MA,
        drv->microsteps(),
        tmc_detect_stall(motor),
        drv->ot(),    // overtemperature
        drv->otpw()   // overtemperature warning
    );
}

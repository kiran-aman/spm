#include "tmc2209.h"

// tmc2209 motor drivers
TMC2209Stepper tmc1(&Serial1, TMC_RSENSE, 0b00);
TMC2209Stepper tmc2(&Serial2, TMC_RSENSE, 0b00);
TMC2209Stepper tmc3(&Serial3, TMC_RSENSE, 0b00);

static void _init_driver(TMC2209Stepper &drv) {
    // tmc2209 + motor initialization
    drv.begin();
    drv.toff(4);
    drv.rms_current(RMS_CURRENT_MA);
    drv.microsteps(MICROSTEPS);
    drv.en_spreadCycle(false);
    drv.pwm_autoscale(true);
    drv.pwm_autograd(true);
    drv.GCONF();
    drv.SGTHRS(100);
}

void tmc_init_all() {
    // start tmc uart comms
    Serial1.begin(57600);
    Serial2.begin(57600);
    Serial3.begin(57600);
    delay(1500);

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

    while(Serial1.available() > 0) { Serial1.read(); } // clear serial
    while(Serial2.available() > 0) { Serial2.read(); }
    while(Serial3.available() > 0) { Serial3.read(); }

    // confirmation (0 = success)
    Serial.println("tmc initialization success");
    Serial.printf("TMC1 Link: %d\n", tmc1.test_connection());
    Serial.printf("TMC2 Link: %d\n", tmc2.test_connection());
    Serial.printf("TMC3 Link: %d\n", tmc3.test_connection());

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

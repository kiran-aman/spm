#include <Arduino.h>
#include "tmc2209.h"
#include "encoder.h"
#include "stepper_driver.h"

// state machine for testing
enum class BringupState {
    IDLE, TMC_CHECK, ENCODER_CHECK, MOTION_TEST, DONE
};
static BringupState state = BringupState::IDLE;

void setup() {
    // initialize serial; wait 5s
    Serial.begin(115200);
    while(!Serial && millis() < 5000);

    Serial.println("unit testing spm");

    // initialize subsystems
    tmc_init_all();
    encoder_init_all();
    stepper_init_all();

    // enable tmc2209 and motors
    tmc_enable(true);

    Serial.println("t:tmc status, e:encoder read, m:motion test, s:stop all");

    // starting state: IDLE
    state = BringupState::IDLE;
}

void loop() {
    // serial command interface
    if(Serial.available()) {
        char cmd = Serial.read();

        switch(cmd) {
            case 't':
                // print tmc status
                Serial.println("tmc2209 status");
                tmc_print_status(1);
                tmc_print_status(2);
                tmc_print_status(3);
                break;
            
            case 'e':
                // print encoder counts/degrees
                Serial.println("encoder stuff");
                for(uint8_t i = 1; i <= 3; i++) {
                    Serial.printf("Motor %d: counts=%d    deg=%.2f\n", i, encoder_read(i), encoder_degrees(i));
                }
                break;

            case 'm':
                // spin each motor 360 deg
                Serial.println("motion test");
                stepper_move_to_deg(1, 360.0f);
                stepper_move_to_deg(2, 360.0f);
                stepper_move_to_deg(3, 360.0f);
                break;

            case 'r':
                // return to 0 deg
                Serial.println("return to 0");
                stepper_move_to_deg(1, 0.0f);
                stepper_move_to_deg(2, 0.0f);
                stepper_move_to_deg(3, 0.0f);
                break;

            case 's':
                // stop all motors immediately
                Serial.println("stop");
                stepper_stop_all();
                tmc_enable(false);
                break;
            
            case 'x':
                // re-enable tmcs
                Serial.println("re-enable tmcs");
                tmc_enable(true);
                break;

            default:
                break;
        }
    }
}
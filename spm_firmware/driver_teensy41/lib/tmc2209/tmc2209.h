#pragma once

#include <Arduino.h>
#include <TMCStepper.h>
#include "teensy41_config.h"
#include "machine_params.h"

extern TMC2209Stepper tmc1;
extern TMC2209Stepper tmc2;
extern TMC2209Stepper tmc3;

void tmc_init_all();
void tmc_enable(bool en);               // toggle enable pin (e-stop)
bool tmc_detect_stall(uint8_t motor); // stall guard (homing/crash detection)
void tmc_print_status(uint8_t motor);   // debugging
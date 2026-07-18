#pragma once

#include <Arduino.h>
#include "teensystep4.h"
#include "teensy41_config.h"
#include "machine_params.h"

using namespace TS4;

// stepper instances
extern Stepper s1;
extern Stepper s2;
extern Stepper s3;

void stepper_init_all();

// move to absolute step position
void stepper_move_to(uint8_t motor, int32_t steps);

void stepper_spin(uint8_t motor);

// move to absolute angle in degrees (accounts for microsteps + gear ratio)
void stepper_move_to_deg(uint8_t motor, float degrees);

// stop with deceleration
void stepper_stop(uint8_t motor);
void stepper_stop_all();

// status
bool stepper_is_running(uint8_t motor);
int32_t stepper_position(uint8_t motor);
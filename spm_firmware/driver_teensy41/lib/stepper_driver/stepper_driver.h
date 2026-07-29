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
extern bool steppers_running;
extern bool stepper1_running;
extern bool stepper2_running;
extern bool stepper3_running;

void stepper_init_all();

// move to absolute step position
void stepper_move_to(uint8_t motor, int32_t steps);

// set stepper speed
void stepper_set_speed(uint8_t motor, float speed_hz);

// deactivate stopped motors
// void stepper_stop_turnoff(uint8_t motor);

void stepper_restart_rotation(uint8_t motor);

void stepper_start_rotation(uint8_t motor);

void stepper_stop_rotation(uint8_t motor);

void steppers_start_rotation();

void steppers_end_rotation();

void stepper_move_rel(uint8_t motor, int32_t steps);

// move to absolute angle in degrees (accounts for microsteps + gear ratio)
void stepper_move_to_deg(uint8_t motor, float degrees);

// stop with deceleration
void stepper_stop(uint8_t motor);
void stepper_stop_all();

// status
bool stepper_is_running(uint8_t motor);
int32_t stepper_position(uint8_t motor);
void stepper_print_status(uint8_t motor);
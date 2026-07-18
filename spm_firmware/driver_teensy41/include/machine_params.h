#pragma once

// PLACEHOLDERS
// kinematics constants
#define ALPHA1  60.0f
#define ALPHA2  51.315f
#define BETA    59.99f

// motion transmission constants
#define TEETH_INPUT     40
#define TEETH_OUTPUT    60
#define GEAR_RATIO      (60.0f/40.0f)

// stepper parameters
#define STEPS_PER_REV       200
#define MICROSTEPS          16
#define STEPS_PER_REV_MICRO (STEPS_PER_REV * MICROSTEPS)
#define ENCODER_CPR         4000

// tmc2209 parameters
#define RMS_CURRENT_MA     500 // 500 FOR NEMA 11 V1 TEST
#define TMC_RSENSE         0.11f

// motion limits
#define MAX_SPEED_HZ    10000   // steps/sec
#define MAX_ACCEL_HZ    5000    // steps/sec^2
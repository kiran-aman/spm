#pragma once

// kinematics constants
// USING V1 FOR TESTING
#define ALPHA1  0.785398f // 45 deg -> rad
#define ALPHA2  1.20942591f // 69.295 deg -> rad
#define BETA    1.0472f // 60 deg -> rad

// motion transmission constants
#define TEETH_INPUT     9
#define TEETH_OUTPUT    25
#define GEAR_RATIO      (25.0f/9.0f)

// stepper parameters
#define STEPS_PER_REV       200
#define MICROSTEPS          16
#define STEPS_PER_REV_MICRO (STEPS_PER_REV * MICROSTEPS)
#define ENCODER_CPR         4000
#define STEPS_PER_DEG   ((STEPS_PER_REV_MICRO * GEAR_RATIO) / 360.0f)

// tmc2209 parameters
#define RMS_CURRENT_MA     500 // 500 FOR NEMA 11 V1 TEST
#define TMC_RSENSE         0.11f

// motion limits
#define MAX_SPEED_HZ    10000   // steps/sec
#define MAX_ACCEL_HZ    10000    // steps/sec^2
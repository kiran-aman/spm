#pragma once

// kinematics constants
#define ALPHA1  0.872665f // 50 deg -> rad
#define ALPHA2  0.95235636f // 54.566 deg -> rad
#define BETA    1.22173f // 70 deg -> rad
#define HOME_ANGLE 60.0f // home position angle in degrees

// motion transmission constants
#define TEETH_INPUT     48
#define TEETH_OUTPUT    60
#define GEAR_RATIO      (60.0f/48.0f)

// stepper parameters
#define STEPS_PER_REV       200
#define MICROSTEPS          16
#define STEPS_PER_REV_MICRO (STEPS_PER_REV * MICROSTEPS)
#define ENCODER_CPR         4000
#define STEPS_PER_DEG   ((STEPS_PER_REV_MICRO * GEAR_RATIO) / 360.0f)

// tmc2209 parameters
#define RMS_CURRENT_MA     1000 // 500 FOR NEMA 11 V1 TEST
#define TMC_RSENSE         0.11f

// motion limits
#define MAX_SPEED_HZ    20000   // steps/sec
#define MAX_ACCEL_HZ    100000    // steps/sec^2
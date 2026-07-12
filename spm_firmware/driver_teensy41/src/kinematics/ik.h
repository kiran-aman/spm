#pragma once

#include <Arduino.h>
#include "machine_params.h"

struct IKResult {
    float theta[3]; // joint angles (theta_i)
    bool valid; // false for invalid position
}

// ik solver; weird convention -> x (yaw), y (pitch), z (roll)
// uses branch tracking to select correct configuration
IKResult ik(float roll, float pitch, float yaw);

// reset branch tracking to home position (beta)
void ik_reset_home();
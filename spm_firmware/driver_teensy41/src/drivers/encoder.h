#pragma once

#include <Arduino.h>
#include <QuadEncoder.h>
#include "../teensy41_config.h"
#include "../machine_params.h"

void encoder_init_all();
 
// raw encoder output
int32_t encoder_read(uint8_t motor);    // motor 1-3
void encoder_reset(uint8_t motor);
 
// converted to deg encoder output
float encoder_degrees(uint8_t motor);
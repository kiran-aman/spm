#include "encoder.h"

static QuadEncoder enc1(1, ENC1_A, ENC1_B, 0);
static QuadEncoder enc2(2, ENC2_A, ENC2_B, 0);
static QuadEncoder enc3(3, ENC3_A, ENC3_B, 0);

void encoder_init_all() {
    enc1.setInitConfig();
    enc1.EncConfig.filterSamplePeriod = 0x1F; // Sample period clock cycles (0-255)
    enc1.EncConfig.filterCount = 5;            // Consecutive samples required
    enc1.init();
 
    enc2.setInitConfig();
    enc2.EncConfig.filterSamplePeriod = 0x1F; // Sample period clock cycles (0-255)
    enc2.EncConfig.filterCount = 5;            // Consecutive samples required
    enc2.init();
    enc2.init();
 
    enc3.setInitConfig();
    enc3.EncConfig.filterSamplePeriod = 0x1F; // Sample period clock cycles (0-255)
    enc3.EncConfig.filterCount = 5;            // Consecutive samples required
    enc3.init();
 
    Serial.println("stepper quadrature encoders initialization success");
}
 
int32_t encoder_read(uint8_t motor) {
    switch (motor) {
        case 1: return enc1.read();
        case 2: return enc2.read();
        case 3: return enc3.read();
        default: return 0;
    }
}
 
void encoder_reset(uint8_t motor) {
    switch (motor) {
        case 1: enc1.write(0); break;
        case 2: enc2.write(0); break;
        case 3: enc3.write(0); break;
    }
}
 
float encoder_degrees(uint8_t motor) {
    return (float)encoder_read(motor) / ENCODER_CPR * 360.0f;
}

float encoder_joint_degrees(uint8_t motor) {
    return encoder_degrees(motor) / GEAR_RATIO;
}
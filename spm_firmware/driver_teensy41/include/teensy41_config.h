#pragma once

// tmc2209 uarts
#define TMC1_RX      0 // motor 1 on serial 1
#define TMC1_TX      1

#define TMC2_RX      7 // motor 2 on serial 2
#define TMC2_TX      8

#define TMC3_RX      15 // motor 3 on serial 3
#define TMC3_TX      14

// quadrature encoder pins
#define ENC1_A          2       // HW encoder ch1 PhaseA
#define ENC1_B          3       // HW encoder ch1 PhaseB

#define ENC2_A          4       // HW encoder ch2 PhaseA
#define ENC2_B          5       // HW encoder ch2 PhaseB

#define ENC3_A          30      // HW encoder ch3 PhaseA
#define ENC3_B          31      // HW encoder ch3 PhaseB

// step/dir pins for tmc2209
#define MOT1_STEP_PIN   34
#define MOT1_DIR_PIN    35

#define MOT2_STEP_PIN   22
#define MOT2_DIR_PIN    23

#define MOT3_STEP_PIN   24
#define MOT3_DIR_PIN    25

// common tmc2209 enable
#define MOT_ENABLE_PIN  27

// tmc2209 diag
#define TMC1_DIAG_PIN   28
#define TMC2_DIAG_PIN   29
#define TMC3_DIAG_PIN   30

// receiver esp32 uart
#define ESP32_SERIAL    Serial3
#define ESP32_RX_PIN    14
#define ESP32_TX_PIN    15
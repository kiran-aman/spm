#pragma once

// xiao esp32c6

#define PIN_BNO_CS    D3   // spi chip select
#define PIN_BNO_INT   D2   // data-ready interrupt (active-low)
#define PIN_BNO_RST   D1   // reset (active-low)
#define PIN_BNO_SCK   D8   // spi clock  -> bno085 scl
#define PIN_BNO_MISO  D9   // spi MISO   -> bno085 sda
#define PIN_BNO_MOSI  D10  // spi MOSI   -> bno085 ado
// ps0, ps1 on bno085 tied directly to 3v3

#define PIN_BATT_ADC  A0 

#define BATT_SAMPLE_COUNT   16
#define BATT_REPORT_MS      2000   // how often to sample/send battery voltage

#define BNO_REPORT_INTERVAL_US  10000   // 10ms -> 100Hz, matches Teensy control loop rate
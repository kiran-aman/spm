#pragma once

// xiao esp32c6

#define PIN_BNO_CS    D3   // spi chip select
#define PIN_BNO_INT   D2   // interrupt
#define PIN_BNO_RST   D1   // reset
// sck  -> d8  (bno085 scl)
// miso -> d9  (bno085 ado)
// mosi -> d10 (bno085 sda)
// ps0, ps1 on bno085 tied directly to 3v3 to select spi mode

#define PIN_BATT_ADC  A0 

#define BATT_SAMPLE_COUNT   16
#define BATT_REPORT_MS      2000   // how often to sample/send battery voltage

#define BNO_REPORT_INTERVAL_US  10000   // 10ms -> 100Hz, matches Teensy control loop rate
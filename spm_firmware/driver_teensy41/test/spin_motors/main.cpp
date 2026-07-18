#include <Arduino.h>
#include <TMCStepper.h>
#include <teensystep4.h>

using namespace TS4;

// 3 drivers using serial 1, 2, 3
TMC2209Stepper tmc1(&Serial1, 0.11f, 0b00);
TMC2209Stepper tmc2(&Serial2, 0.11f, 0b00);
TMC2209Stepper tmc3(&Serial3, 0.11f, 0b00);

Stepper s1(34, 35); 
Stepper s2(22, 23);
Stepper s3(24, 25);

void setup() {
  Serial.begin(115200);   // usb Monitor
  delay(3000);

  Serial1.begin(57600);  // tmc uart Rails
  Serial2.begin(57600);
  Serial3.begin(57600);
  delay(1500);

  tmc1.GCONF();
  tmc2.GCONF();
  tmc3.GCONF();
  delay(50);

  while(Serial1.available() > 0) { Serial1.read(); } // clear serial
  while(Serial2.available() > 0) { Serial2.read(); }
  while(Serial3.available() > 0) { Serial3.read(); }

  TS4::begin();
  s1.setMaxSpeed(10000); s1.setAcceleration(20000);
  s2.setMaxSpeed(10000); s2.setAcceleration(20000);
  s3.setMaxSpeed(10000); s3.setAcceleration(20000);

  // enable pin active low
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH); // lock out power output during config

  tmc1.begin();
  tmc1.toff(4);
  tmc1.rms_current(600); // 500mA
  tmc1.microsteps(8);   // 32 Microsteps

  tmc2.begin();
  tmc2.toff(4);
  tmc2.rms_current(600);
  tmc2.microsteps(16);

  tmc3.begin();
  tmc3.toff(4);
  tmc3.rms_current(600);
  tmc3.microsteps(32);

  delay(1000);

  while(Serial1.available() > 0) { Serial1.read(); } // clear serial 1 more time
  while(Serial2.available() > 0) { Serial2.read(); }
  while(Serial3.available() > 0) { Serial3.read(); }

  // print raw live hardware checks (0 = good)
  Serial.printf("TMC1 Link: %d\n", tmc1.test_connection());
  Serial.printf("TMC2 Link: %d\n", tmc2.test_connection());
  Serial.printf("TMC3 Link: %d\n", tmc3.test_connection());

  // drop enable pin to gnd to throw full current to the coils
  digitalWrite(27, LOW); 
}

void loop() {
  s1.rotateAsync(0.5); // spin at 50% max speed
  s2.rotateAsync(0.5);
  s3.rotateAsync(0.5);
  delay(4000);

  s1.stopAsync();
  s2.stopAsync();
  s3.stopAsync();
  delay(2000);
}

#include <Arduino.h>
#include ".\feature\EepromCAT24M01.h"

EepromCAT24M01 eeprom1(0x00, false);

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  eeprom1.setTimeoutInMillis(1000);
}
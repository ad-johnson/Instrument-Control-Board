#include <Arduino.h>
#include ".\feature\AdcMcp3428.h"
#include ".\feature\EepromCAT24M01.h"
#include ".\feature\RtccMcp79410.h"

EepromCAT24M01 eeprom1(0x00);
AdcMcp3428 adc(0x06);
RtccMcp79410 rtcc;

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  eeprom1.setTimeoutInMillis(1000);
  adc.getAddress();
  rtcc.isRunning();
}
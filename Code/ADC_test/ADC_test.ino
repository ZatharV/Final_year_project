#include "ADS1X15.h"

ADS1115 ADS(0x48);
float PT1000, HCPV, batteryLevel;

void setup() {
  Serial.begin(115200);
  ADS.begin();
  ADS.setGain(2);
  delay(1000);
}

void loop() {
  PT1000 = ADS.toVoltage(ADS.readADC(0));
  Serial.printf("PT1000 ADC voltage \t= %f and temperature \t= %f*C \n", PT1000, (329.537 * PT1000 - 207.935) / 385 * 100);

  HCPV = ADS.toVoltage(ADS.readADC(1));
  Serial.printf("HCPV ADC voltage \t= %f and Humidity \t= %f%% \n", HCPV, HCPV * (2.7 / 1.5 - 0.330) * 23.1729 + 10);

  batteryLevel = ADS.toVoltage(ADS.readADC(2));
  Serial.printf("Battery ADC voltage \t= %f and Battery Voltage = %f%% \n", batteryLevel, (((133.21 * batteryLevel) + 1491.05) / 32830.21) * 241);

  Serial.println();
  delay(2000);
}

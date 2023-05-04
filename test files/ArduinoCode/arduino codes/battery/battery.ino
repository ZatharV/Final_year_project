#include "ADS1X15.h"

ADS1115 ADS(0x48);


void setup(){
  Serial.begin(115200);
  Serial.print("ADS1X15_LIB_VERSION: ");
  Serial.println(ADS1X15_LIB_VERSION);
  ADS.begin();
}

void loop(){ 
  ADS.setGain(2);
  int16_t val_0 = ADS.readADC(0);  float f = ADS.toVoltage(val_0);
  Serial.print("\tAnalog0: "); Serial.print(val_0); Serial.print('\t'); Serial.print("\tvoltage: "); Serial.print(f); Serial.print('\n');
 
  delay(1000);
}

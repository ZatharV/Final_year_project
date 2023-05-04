const int LDRPin = 34; // LDR pin connected to ESP32 pin 34
int LDRValue = 0; // variable to store LDR value

void setup() {
  Serial.begin(115200); // Start serial communication
}

void loop() {
  LDRValue = analogRead(LDRPin); // Read LDR value from pin 34
  Serial.print("LDR Value: ");
  Serial.println(LDRValue); // Print LDR value to serial monitor
  delay(1000); // Delay for 1 second
}

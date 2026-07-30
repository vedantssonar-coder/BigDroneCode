#include <SoftwareSerial.h>
SoftwareSerial debugIn(2, 3); // 2 = RX (from first Arduino's debug TX), 3 = unused

void setup() {
  Serial.begin(38400);      // to your PC via USB
  debugIn.begin(38400);     // from the flight controller
}

void loop() {
  while (debugIn.available()) {
    Serial.write(debugIn.read());
  }
}
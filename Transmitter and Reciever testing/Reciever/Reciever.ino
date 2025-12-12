#include <RH_ASK.h>
#include <SPI.h>  // Required by RadioHead

// Use default: RX on digital pin 11 for Arduino Uno
RH_ASK driver;

void setup() {
  Serial.begin(9600);
  driver.init();
}

void loop() {
  uint8_t buf[32];
  uint8_t buflen = sizeof(buf);

  if (driver.recv(buf, &buflen)) {
    // Add terminator to treat as C-string
    buf[buflen] = '\0';
    Serial.print("Received: ");
    Serial.println((char*)buf);
  }
  else
      Serial.println("not got yet");
  delay(200);
}

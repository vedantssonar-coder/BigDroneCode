#include <RH_ASK.h>
#include <SPI.h>

RH_ASK driver;

void setup() {
  Serial.begin(9600);
  driver.init();
}

void loop() {
  uint8_t buf[32];
  uint8_t buflen = sizeof(buf);

  if (driver.recv(buf, &buflen)) {
    buf[buflen] = '\0';
    Serial.print("Received: ");
    Serial.println((char*)buf);
  }
  // Remove or reduce "not got yet" printing to avoid flooding
  // Serial.println("not got yet");
  delay(200);
}

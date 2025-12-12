#include <RH_ASK.h>
#include <SPI.h>

RH_ASK driver;

void setup() {
  Serial.begin(9600); // Add Serial for debugging
  driver.init();
}

void loop() {
  const char *msg = "Hello";
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  Serial.println("Sent: Hello"); // Confirm transmission
  delay(1000);
}

#include <RH_ASK.h>
#include <SPI.h>  // Required by RadioHead

// Use default: TX on digital pin 12 for Arduino Uno
RH_ASK driver;

void setup() {
  driver.init();
}

void loop() {
  const char *msg = "Hello";
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  delay(1000);
}

#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h>  // Not actually used but needed to compile
#endif
int slider1 = 0;
int slider2 = 0;
int slider3 = 0;
int slider4 = 0;
uint8_t x = 0;
uint8_t y = 0;
uint8_t button = 0;
RH_ASK driver;
// RH_ASK driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2
// RH_ASK driver(2000, 3, 4, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85),
// RH_ASK driver(2000, PD14, PD13, 0); STM32F4 Discovery: see tx and rx on Orange and Red LEDS
uint8_t data = 255;
uint8_t data1 = 230;
void setup() {

  Serial.begin(9600);  // Debugging only
  driver.init();
}

void loop() {
  slider2 = 0;
  slider3 = 0;
  slider4 = 0;
  slider1 = analogRead(A0);
  if (slider1 > 255) {
    slider2 = slider1 - 255;
    if (slider2 > 255) {
      slider3 = slider2 - 255;
      if (slider3 > 255) {
        slider4 = slider3 - 255;
        if (slider4 > 255)
          slider4 = 255;
        slider3 = 255;
      }
      slider2 = 255;
    }
    slider1 = 255;
  }
  x = analogRead(A1) / 4;
  y = analogRead(A2) / 4;
  button = analogRead(A3) / 4;
  Serial.print((uint8_t)slider1);
  Serial.print(" / ");
  Serial.print((uint8_t)slider2);
  Serial.print(" / ");
  Serial.print((uint8_t)slider3);
  Serial.print(" / ");
  Serial.print((uint8_t)slider4);
  Serial.print(" / ");
  Serial.print(x);
  Serial.print(" / ");
  Serial.print(y);
  Serial.print(" / ");
  Serial.println(button);
  send();
  delay(20);
}

void send() {
  //const char *msg = "abcb";
  uint8_t dataarray[] = { (uint8_t)slider1, (uint8_t)slider2, (uint8_t)slider3, (uint8_t)slider4, x, y, button };

  driver.send((uint8_t *)dataarray, 7);
  driver.waitPacketSent();
}

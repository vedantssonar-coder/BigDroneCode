#include <Wire.h>
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h>  // Needed to compile RH_ASK
#endif

unsigned long lastSignalTime = 0;

// Remote
RH_ASK driver;
int slider = 0, x = 0, y = 0;
bool button = 1;


// ------------------ SETUP ------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Started serial monitor output");
  driver.init();
  Serial.println("Finished Testing remote");
  Serial.println("System ready");
  delay(1000);
}

// ------------------ LOOP ------------------
void loop() {
  
  recv();
  debug_output();
  delayMicroseconds(500);
}

// ------------------ RECV ------------------
void recv() {
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (driver.recv(buf, &len)) {
    slider = buf[0] + buf[1] + buf[2] + buf[3];
    x = constrain(map(buf[4], 0, 255, -5, 7), -5, 5);
    y = constrain(map(buf[5], 0, 255, -5, 7), -5, 5);
    button = buf[6];
    lastSignalTime = millis();  // Signal received
  }
}

// ------------------ DEBUG ------------------
void debug_output() {
  Serial.print(" | ");
  Serial.print(slider);
  Serial.print("/");
  Serial.print(x);
  Serial.print("/");
  Serial.print(y);
  Serial.print("/");
  Serial.println(button);
}


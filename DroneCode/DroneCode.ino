#include <Wire.h>
#include <RH_ASK.h>
#include <ServoTimer2.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h>  // Needed to compile RH_ASK
#endif

#define RAD2DEG (180.0 / 3.14159265)
#define MPU_ADDR 0x68
#define MAX_THROTTLE 1950  // Set to 2000 for full range
#define TEST_MODE false    // Set to false for actual flight
const int OFFSET[4] = { 500, 0, 0, 0 };
const int test = 3;

// Timer globals
unsigned long currentTime = 0, previousTime = 0;
float elapsedTime = 0;

unsigned long lastSignalTime = 0;
const unsigned long SIGNAL_TIMEOUT = 1000;  // in milliseconds
bool failsafeLanding = false;
bool landingInProgress = false;
unsigned long landingStartTime = 0;
unsigned long lastLandingCommandTime = 0;
const unsigned long LANDING_COMMAND_COOLDOWN = 1000;  // ms


// Remote
RH_ASK driver;
int slider = 0, x = 0, y = 0;
bool button = 1;

ServoTimer2 esc[4];
void update() {
  int us = (int)Power;
  esc[test].write(us);  // non-blocking servo pulse
}



float throttle = 1000;
// order: +x +y -x -y on pins 3,5,6,9

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Started serial monitor output");
  // Attach ESCs
  esc[0].attach(3);
  esc[1].attach(5);
  esc[2].attach(6);
  esc[3].attach(9);

  // Send minimum throttle to all ESCs for arming
  for (int i = 0; i < 4; i++) {
    m[i].Power = m[i].Initial = m[i].Final = 850;
    m[i].update();
  }
  for (int i = 0; i < 100000; i++) {
    m[test].Power = 1000 + OFFSET[test];
  }

  Serial.println("Finished Motor Calibration");
  driver.init();
  Serial.println("Remote COntrol driver intialized");
  delay(20);
  Serial.println("Testing remote...");
  for (int i = 0; i < 20; i++) {
    recv();
    debug_output();
  }
  Serial.println("Finished Testing remote");
  Serial.println("System ready");
  delay(1000);
}

// ------------------ LOOP ------------------
void loop() {
  recv();
  update();
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
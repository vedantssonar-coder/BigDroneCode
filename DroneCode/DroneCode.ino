#include <ServoTimer2.h>

ServoTimer2 esc[4];
/*
void setup() {
  Serial.begin(9600);
  Serial.println("ESC Calibration Mode - STEP 1: MAX THROTTLE");
  Serial.println("Make sure battery is DISCONNECTED");
  delay(2000);
  
  // Attach ESCs
  esc[0].attach(3);
  esc[1].attach(5);
  esc[2].attach(6);
  esc[3].attach(9);
  
  // Send MAX throttle to all ESCs
  for (int i = 0; i < 4; i++) {
    esc[i].write(2000);  // Maximum pulse for your ESCs
  }
  
  Serial.println("Sending MAX throttle (2000 µs) to all ESCs");
  Serial.println("Now connect the battery - ESCs should beep");
  delay(3000);
}
*/


void setup() {
  Serial.begin(9600);
  Serial.println("ESC Calibration Mode - STEP 2: MIN THROTTLE");
  
  // Attach ESCs
  esc[0].attach(3);
  esc[1].attach(5);
  esc[2].attach(6);
  esc[3].attach(9);
  
  // Send MIN throttle to all ESCs
  for (int i = 0; i < 4; i++) {
    esc[i].write(1000);  // Minimum pulse for your ESCs
  }
  
  Serial.println("Sending MIN throttle (1000 µs) to all ESCs");
  Serial.println("You should hear a confirmation beep");
  delay(3000);
}

void loop() {
  delay(1000);
}
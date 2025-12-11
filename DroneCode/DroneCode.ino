#include <ServoTimer2.h>

ServoTimer2 esc[4];

void setup() {
  Serial.begin(9600);
  Serial.println("=== ESC THROTTLE RANGE CALIBRATION ===");
  Serial.println("DISCONNECT BATTERY NOW!");
  Serial.println("Waiting 3 seconds...");
  delay(3000);
  
  esc[0].attach(3);
  esc[1].attach(5);
  esc[2].attach(6);
  esc[3].attach(9);
  
  // Step 1: Send MAX throttle to all ESCs
  Serial.println("\nStep 1: Sending MAX throttle (2000 µs)");
  for (int i = 0; i < 4; i++) {
    esc[i].write(2000);
  }
  
  Serial.println("CONNECT BATTERY NOW - Listen for beeping");
  Serial.println("You will hear: beep-beep-beep, beep-beep");
  Serial.println("Wait 5 seconds for beeping to complete...");
  delay(5000);
  
  // Step 2: Send MIN throttle to all ESCs
  Serial.println("\nStep 2: Sending MIN throttle (1000 µs)");
  for (int i = 0; i < 4; i++) {
    esc[i].write(1000);
  }
  
  Serial.println("Listen for final confirmation beeps (should be 3-4 quick beeps)");
  Serial.println("Calibration complete!");
  Serial.println("DISCONNECT BATTERY");
  delay(5000);
}

void loop() {
  // Hold MIN throttle
  for (int i = 0; i < 4; i++) {
    esc[i].write(1000);
  }
  delay(100);
}

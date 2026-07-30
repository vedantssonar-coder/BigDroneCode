#include <ServoTimer2.h>

ServoTimer2 esc[4];

// ------------------ ESC CALIBRATION ------------------
void esc_calibration() {
  Serial.println("=== ESC THROTTLE RANGE CALIBRATION ===");
  Serial.println("DISCONNECT BATTERY NOW!");
  Serial.println("Waiting 3 seconds...");
  delay(3000);

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

  // Hold at minimum throttle
  while (true) {
    for (int i = 0; i < 4; i++) {
      esc[i].write(1000);
    }
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);

  esc[0].attach(8);
  esc[1].attach(9);
  esc[2].attach(10);
  esc[3].attach(11);
  delay(2000);
  esc_calibration();
}

void loop() {
  // nothing — esc_calibration() never returns
}
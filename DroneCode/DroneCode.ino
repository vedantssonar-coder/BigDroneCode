#include <ServoTimer2.h>

ServoTimer2 esc[4];

void setup() {
  Serial.begin(9600);
  Serial.println("Testing ESC calibration...");
  Serial.println("Battery must be connected, props OFF");
  delay(2000);
  
  esc[0].attach(3);
  esc[1].attach(5);
  esc[2].attach(6);
  esc[3].attach(9);
  
  // Arm at 1000
  for (int i = 0; i < 4; i++) {
    esc[i].write(1000);
  }
  delay(2000);
}

void loop() {
  // Slowly increase throttle from 1000 to 2000
  for (int throttle = 1000; throttle <= 2000; throttle += 10) {
    for (int i = 0; i < 4; i++) {
      esc[i].write(throttle);
    }
    
    Serial.print("Throttle: ");
    Serial.print(throttle);
    Serial.println(" µs - All motors should start together");
    delay(500);
  }
  
  // Return to min
  for (int i = 0; i < 4; i++) {
    esc[i].write(1000);
  }
  delay(2000);
}

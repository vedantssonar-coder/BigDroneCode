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

const int OFFSET[4] = {-122, -50, -258, 73 };
const int test = 3; //motor number

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

// Declare Power as a global variable
int Power = 1000;

void update() {
  esc[test].write(Power);  // non-blocking servo pulse
}

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
    esc[i].write(850);
  }
  delay(2000); // Wait for ESCs to arm

  Serial.println("Finished Motor Calibration");
  driver.init();
  Serial.println("Remote Control driver initialized");
  delay(20);
  Serial.println("Testing remote...");
  for (int i = 0; i < 20; i++) {
    recv();
    // Comment out or implement debug_output if needed
  }
  Serial.println("Finished Testing remote");
  Serial.println("System ready");
  delay(1000);
}

// ------------------ LOOP ------------------
void loop() {
  recv();
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read the full line
    input.trim();                                 // Remove whitespace
    if (input.length() > 0) {
      int pwm = input.toInt();                    // Convert to integer
      if (pwm >= 500 && pwm <= 3000) {    // Ensure within valid ESC range
        Serial.print("Setting ESC to: ");
        Serial.println(pwm);
        Power = pwm;
        update();
      } else {
        Serial.println("Invalid PWM value. Must be between 850 and 1950.");
      }
    }
  }
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

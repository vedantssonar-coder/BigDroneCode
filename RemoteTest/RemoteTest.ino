#include <ServoTimer2.h>

// Define pins for FlySky CT6B receiver
#define CH1_PIN 2  // Throttle (interrupt)
#define CH2_PIN 3  // Roll (interrupt)
#define CH3_PIN 4  // Pitch (polling)
#define CH4_PIN 5  // Yaw (polling)
#define CH5_PIN 6  // Arm / mode switch (polling)
#define CH6_PIN 7  // Aux (polling)

// PWM value ranges
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_MID 1500
#define PWM_DEADZONE 50

// Structure to store channel data
struct ChannelData {
  volatile unsigned long risingEdge;
  volatile int pulseWidth;
};

ChannelData ch[6];

// Interrupt Service Routines for pins 2 and 3
void ch1_ISR() {
  if (digitalRead(CH1_PIN)) ch[0].risingEdge = micros();
  else ch[0].pulseWidth = micros() - ch[0].risingEdge;
}
void ch2_ISR() {
  if (digitalRead(CH2_PIN)) ch[1].risingEdge = micros();
  else ch[1].pulseWidth = micros() - ch[1].risingEdge;
}

// Function to map PWM value to output range
int mapPWM(int v, int outMin, int outMax) {
  v = constrain(v, PWM_MIN, PWM_MAX);
  if (abs(v - PWM_MID) < PWM_DEADZONE) v = PWM_MID;
  return map(v, PWM_MIN, PWM_MAX, outMin, outMax);
}

// Function to read a channel and map it
int readChannel(int idx, int outMin, int outMax) {
  int pw = ch[idx].pulseWidth;
  if (pw < PWM_MIN || pw > PWM_MAX) return outMin;
  return mapPWM(pw, outMin, outMax);
}

// Function to read a switch channel
bool readSwitch(int idx) {
  int pw = ch[idx].pulseWidth;
  if (pw < PWM_MIN || pw > PWM_MAX) return false;
  return (pw > PWM_MID);
}

// Initialize receiver pins and interrupts
void flyskyInit() {
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  pinMode(CH6_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(CH1_PIN), ch1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2_PIN), ch2_ISR, CHANGE);
}

// Polling function for pins 4–7 using a different logic
void pollChannels() {
  static unsigned long lastRising[4] = {0};
  static bool lastState[4] = {LOW};
  int pins[4] = {CH3_PIN, CH4_PIN, CH5_PIN, CH6_PIN};
  int idxs[4] = {4, 5, 6, 7}; // ch[2] for CH3_PIN, etc.

  for (int i = 0; i < 4; i++) {
    int pin = pins[i];
    int idx = idxs[i];
    int currentState = digitalRead(pin);

    if (currentState == HIGH && lastState[i] == LOW) {
      lastRising[i] = micros();
    } else if (currentState == LOW && lastState[i] == HIGH) {
      ch[idx].pulseWidth = micros() - lastRising[i];
    }
    lastState[i] = currentState;
  }
}

void setup() {
  Serial.begin(9600);
  flyskyInit();
  Serial.println("FlySky CT6B Receiver Standalone Test");
}

void loop() {
  // Poll pins 4–7
  pollChannels();

  // Read and print all channel values
  int throttle = readChannel(0, 0, 1000);
  int roll = readChannel(1, 0, 1000);
  int pitch = readChannel(2, 0, 1000);
  int yaw = readChannel(3, 0, 1000);
  int button = readSwitch(4);
  int aux = readChannel(5, 0, 1000);
  //int aux = digitalRead(4);

  Serial.print("Throttle: ");
  Serial.print(throttle);
  Serial.print(" | Roll: ");
  Serial.print(roll);
  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Yaw: ");
  Serial.print(yaw);
  Serial.print(" | Button: ");
  Serial.print(button);
  Serial.print(" | Aux: ");
  Serial.println(aux);

  delay(100); // Adjust delay as needed for update rate
}

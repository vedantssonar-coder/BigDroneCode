#include <ServoTimer2.h>

// Define pins for FlySky CT6B receiver
#define CH1_PIN 4  // Throttle
#define CH2_PIN 3  // Roll
#define CH3_PIN 2  // Pitch
#define CH4_PIN 5  // Yaw
#define CH5_PIN 6  // Arm / mode switch
#define CH6_PIN 7  // Aux

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

// Interrupt Service Routines for each channel
void ch1_ISR() {
  if (digitalRead(CH1_PIN)) ch[0].risingEdge = micros();
  else ch[0].pulseWidth = micros() - ch[0].risingEdge;
}
void ch2_ISR() {
  if (digitalRead(CH2_PIN)) ch[1].risingEdge = micros();
  else ch[1].pulseWidth = micros() - ch[1].risingEdge;
}
void ch3_ISR() {
  if (digitalRead(CH3_PIN)) ch[2].risingEdge = micros();
  else ch[2].pulseWidth = micros() - ch[2].risingEdge;
}
void ch4_ISR() {
  if (digitalRead(CH4_PIN)) ch[3].risingEdge = micros();
  else ch[3].pulseWidth = micros() - ch[3].risingEdge;
}
void ch5_ISR() {
  if (digitalRead(CH5_PIN)) ch[4].risingEdge = micros();
  else ch[4].pulseWidth = micros() - ch[4].risingEdge;
}
void ch6_ISR() {
  if (digitalRead(CH6_PIN)) ch[5].risingEdge = micros();
  else ch[5].pulseWidth = micros() - ch[5].risingEdge;
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
  attachInterrupt(digitalPinToInterrupt(CH3_PIN), ch3_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH4_PIN), ch4_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH5_PIN), ch5_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH6_PIN), ch6_ISR, CHANGE);
}

void setup() {
  Serial.begin(9600);
  flyskyInit();
  Serial.println("FlySky CT6B Receiver Standalone Test");
}

void loop() {
  // Read and print all channel values
  int throttle = readChannel(0, 0, 1000);
  int roll = readChannel(1, -20, 20);
  int pitch = readChannel(2, -20, 20);
  int yaw = readChannel(3, -20, 20);
  bool button = readSwitch(4);
  int aux = readChannel(5, 0, 100);

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

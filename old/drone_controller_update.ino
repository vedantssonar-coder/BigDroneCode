#include <Wire.h>
#include <PinChangeInterrupt.h>
#include <ServoTimer2.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h>  // Needed to compile RH_ASK
#endif


#define RAD2DEG (180.0 / 3.14159265)
#define MPU_ADDR 0x68
#define MAX_THROTTLE 1950       // Set to 2000 for full range
#define TEST_MODE true          // Set to false for actual flight
#define CALIBRATION_MODE false  // Set to false after calibration is done


const int OFFSET[4] = { 0, 0, 0, 0 };  //{ -122, -50, -258, 73 }
#define MIN_POWER 1000
#define MAX_POWER 2000


#define LOOP_FREQUENCY 100                         // Hz (changed from 100 to match MATLAB dt=0.002)
#define LOOP_PERIOD_US (1000000 / LOOP_FREQUENCY)  // 2000us for 500Hz


const int led = 12;   //+ve
const int led1 = 13;  //-ve


bool armed = true;


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


// IMU
float accRaw[3], accAngle[3], accError[3];
float gyrRaw[3], gyrAngle[3], gyrError[3];
float roll = 0, pitch = 0, yaw = 0;
float prev_roll = 0, prev_pitch = 0, prev_yaw = 0;


// Kalman filter variables
float kalmanAngleX = 0, kalmanAngleY = 0;
float biasX = 0, biasY = 0;
float P00_X = 1, P01_X = 0, P10_X = 0, P11_X = 1;
float P00_Y = 1, P01_Y = 0, P10_Y = 0, P11_Y = 1;
const float Q_angle = 0.0001;
const float Q_bias = 0.001;
const float R_measure = 0.05;


// ===== CASCADED CONTROLLER (from MATLAB) =====
// Outer loop (angle -> rate command)
const float OPIDP_ROLL = 3.0;
const float OPIDP_PITCH = 3.0;
const float OPIDP_YAW = 2.0;
const float ANGLE_DB_DEG = 0.2;
const float RATECMD_LIM_DPS = 60.0;


// Inner loop (rate PID with anti-windup)
float KPIDP = 0.10;    // P gain for rate control
float KPIDI = 0.010;   // I gain for rate control
float KPIDD = 0.006;   // D gain for rate control (gyro-based)
const float PID_LIM = 350;  // us output limit
const float IRATE_LIM = 80;  // integral saturation


// Rate integral states
float iRateRoll = 0;
float iRatePitch = 0;
float iRateYaw = 0;


// Motor outputs from PID
float PID_x = 0, PID_y = 0, PID_z = 0;


// Gyro rates (deg/s)
float gyrRateX = 0, gyrRateY = 0, gyrRateZ = 0;


ServoTimer2 esc[4];


// Motor Class
class Motor {
public:
  const int index;
  float Power = 1000, Initial = 1000, Final = 1000, Diff = 0;
  Motor(int i)
    : index(i) {}
  void update() {
    int us = constrain((int)(Power + OFFSET[index]), 1000, 2000);
    esc[index].write(us);
  }
};


float throttle = 1000;
Motor m[] = { Motor(0), Motor(1), Motor(2), Motor(3) };


// Remote
int slider = 0, x = 0, y = 0;
bool button = 1;


// ===================== FlySky CT6B RECEIVER (PWM channels) =====================
#define CH1_PIN 2  // Throttle
#define CH2_PIN 3  // Roll (x)
#define CH3_PIN 4  // Pitch (y)
#define CH4_PIN 5  // Yaw (unused here)
#define CH5_PIN 6  // Arm / mode switch (button)
#define CH6_PIN 7  // Aux (optional)


#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_MID 1500
#define PWM_DEADZONE 50


struct ChannelData {
  volatile unsigned long risingEdge;
  volatile int pulseWidth;
};


ChannelData ch[6];


// ISRs
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


int mapPWM(int v, int outMin, int outMax) {
  v = constrain(v, PWM_MIN, PWM_MAX);
  if (abs(v - PWM_MID) < PWM_DEADZONE) v = PWM_MID;
  return map(v, PWM_MIN, PWM_MAX, outMin, outMax);
}


int readChannel(int idx, int outMin, int outMax) {
  int pw = ch[idx].pulseWidth;
  if (pw < PWM_MIN || pw > PWM_MAX) return outMin;
  return mapPWM(pw, outMin, outMax);
}


bool readSwitch(int idx) {
  int pw = ch[idx].pulseWidth;
  if (pw < PWM_MIN || pw > PWM_MAX) return false;
  return (pw > PWM_MID);
}


void flyskyInit() {
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  pinMode(CH6_PIN, INPUT);


  attachInterrupt(digitalPinToInterrupt(CH1_PIN), ch1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2_PIN), ch2_ISR, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH3_PIN), ch3_ISR, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH4_PIN), ch4_ISR, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH5_PIN), ch5_ISR, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH6_PIN), ch6_ISR, CHANGE);
}


void esc_calibration() {
  Serial.println("=== ESC THROTTLE RANGE CALIBRATION ===");
  Serial.println("DISCONNECT BATTERY NOW!");
  Serial.println("Waiting 3 seconds...");
  delay(3000);

  Serial.println("\nStep 1: Sending MAX throttle (2000 µs)");
  for (int i = 0; i < 4; i++) {
    esc[i].write(2000);
  }

  Serial.println("CONNECT BATTERY NOW - Listen for beeping");
  Serial.println("You will hear: beep-beep-beep, beep-beep");
  Serial.println("Wait 5 seconds for beeping to complete...");
  delay(5000);

  Serial.println("\nStep 2: Sending MIN throttle (1000 µs)");
  for (int i = 0; i < 4; i++) {
    esc[i].write(1000);
  }

  Serial.println("Listen for final confirmation beeps (should be 3-4 quick beeps)");
  Serial.println("Calibration complete!");
  Serial.println("DISCONNECT BATTERY");
  delay(5000);

  while (true) {
    for (int i = 0; i < 4; i++) {
      m[i].Power = 1000;
      m[i].update();
    }
    delay(100);
  }
}


// ===================== SETUP =====================
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Started serial monitor output");
  pinMode(led, OUTPUT);
  pinMode(led1, OUTPUT);
  Serial.println("Set LED");
  
  Serial.println("Starting IMU...");
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  
  Serial.println("Started IMU  /  Calibrating...");
  digitalWrite(led, HIGH);
  delay(2000);
  digitalWrite(led, LOW);
  calculate_IMU_error();
  Serial.println("Finished Calibrating IMU");
  delay(500);
  Serial.println("Starting Motor Calibration...");

  esc[0].attach(8);
  esc[1].attach(9);
  esc[2].attach(10);
  esc[3].attach(11);

  if (CALIBRATION_MODE) {
    esc_calibration();
  }

  for (int i = 0; i < 4; i++) {
    m[i].Power = 2000;
    m[i].update();
  }
  for (int j = 0; j < 10; j++)
    for (int i = 0; i < 4; i++) {
      m[i].Power = 1000;
      m[i].update();
    }

  Serial.println("Finished Motor Calibration");
  flyskyInit();
  Serial.println("Remote Control driver initialized");
  delay(20);
  Serial.println("Testing remote...");
  for (int i = 0; i < 20; i++) {
    recv();
    debug_output();
  }
  Serial.println("Finished Testing remote");
  Serial.println("System ready");
  delay(5000);
}


const int BUFFER_SIZE = 20;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;


// ===================== LOOP =====================
void loop() {
  static unsigned long loop_timer = micros();
  unsigned long now = micros();

  while (now - loop_timer < LOOP_PERIOD_US) {
    now = micros();
  }

  loop_timer = now;

  LedBlinker();

  // Calculate elapsed time in seconds
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0f;

  if (!TEST_MODE && !landingInProgress) {
    throttle = constrain(1000 + slider, 1000, MAX_THROTTLE);
  }
  
  if (TEST_MODE) {
    static unsigned long testStart = millis();
    unsigned long elapsed = millis() - testStart;
    SerialReader();
    throttle = constrain(1000 + slider, 1000, MAX_THROTTLE);
    armed = true;
    lastSignalTime = millis();
    button = 1;
  } else {
    recv();
    static bool lastButton = 1;
    if (lastButton == 1 && button == 0) {
      if (!armed) {
        if (throttle <= 1050) {
          armed = true;
          Serial.println("Drone ARMED");
          prev_roll = prev_pitch = prev_yaw = 0;
          roll = pitch = yaw = 0;
          kalmanAngleX = 0;
          kalmanAngleY = 0;
          biasX = biasY = 0;
          iRateRoll = 0;
          iRatePitch = 0;
          iRateYaw = 0;
        } else {
          Serial.println("Throttle too high! Set throttle below 1050 to arm.");
          digitalWrite(led, HIGH);
          delay(500);
          digitalWrite(led, LOW);
          delay(500);
          digitalWrite(led, HIGH);
          delay(500);
          digitalWrite(led, LOW);
        }
      } else if (!landingInProgress && millis() - lastLandingCommandTime > LANDING_COMMAND_COOLDOWN) {
        landingInProgress = true;
        landingStartTime = millis();
        lastLandingCommandTime = millis();
        Serial.println("Landing triggered by user.");
      }
    }
    lastButton = button;
  }

  if (armed) {
    IMU();

    if (!TEST_MODE && millis() - lastSignalTime > SIGNAL_TIMEOUT && !failsafeLanding) {
      Serial.println("Signal lost — initiating emergency landing.");
      failsafeLanding = true;
      landingInProgress = true;
      landingStartTime = millis();
    }

    if (landingInProgress) {
      land();
    } else {
      // Cascaded attitude controller
      PID_cascaded_X();
      PID_cascaded_Y();
      PID_cascaded_Z();

      if (throttle <= 1050 && !landingInProgress) {
        PID_x = PID_y = PID_z = 0;
        iRateRoll = 0;
        iRatePitch = 0;
        iRateYaw = 0;
        for (int i = 0; i < 4; i++) m[i].Final = throttle;
      }

      motorchangetest(false);
    }

    printLoopHz();
    Serial.print(PID_x);
    Serial.print(",");
    Serial.print(PID_y);
    Serial.print(",");
    Serial.print(PID_z);
    Serial.print(" | ");
    debug_output();
  }
}


void SerialReader() {
  while (Serial.available() > 0 && bufferIndex < BUFFER_SIZE - 1) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      buffer[bufferIndex] = '\0';

      char* p = buffer;

      char* token = strtok(p, ",");
      if (token != NULL) {
        float tempThrottleOffset = atof(token);
        slider = (int)tempThrottleOffset;
      }

      token = strtok(NULL, ",");
      if (token != NULL) {
        float tempKPIDP = atof(token);
        KPIDP = tempKPIDP;
      }

      token = strtok(NULL, ",");
      if (token != NULL) {
        float tempKPIDD = atof(token);
        KPIDD = tempKPIDD;
      }

      throttle = constrain(1000 + slider, 1000, MAX_THROTTLE);

      Serial.print("Slider: ");
      Serial.print(slider);
      Serial.print("  KPIDP: ");
      Serial.print(KPIDP, 4);
      Serial.print("  KPIDD: ");
      Serial.println(KPIDD, 4);

      bufferIndex = 0;
    } else {
      buffer[bufferIndex++] = c;
    }
  }
}


void printLoopHz() {
  static unsigned long lastPrint = 0;
  static unsigned long count = 0;
  count++;
  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    Serial.print("Loop Hz: ");
    Serial.println(count);
    count = 0;
    lastPrint = now;
  }
}


// ===================== IMU =====================
void IMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  float ax = (Wire.read() << 8 | Wire.read()) / 4096.0;
  float ay = (Wire.read() << 8 | Wire.read()) / 4096.0;
  float az = (Wire.read() << 8 | Wire.read()) / 4096.0;

  float accAngleX = atan2(ay, sqrt(ax * ax + az * az)) * RAD2DEG - accError[0];
  float accAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * RAD2DEG - accError[1];

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  float gx = (Wire.read() << 8 | Wire.read()) / 32.8 - gyrError[0];
  float gy = (Wire.read() << 8 | Wire.read()) / 32.8 - gyrError[1];
  float gz = (Wire.read() << 8 | Wire.read()) / 32.8 - gyrError[2];

  gyrRateX = gx;
  gyrRateY = gy;
  gyrRateZ = gz;

  kalman_predict(&kalmanAngleX, &biasX, &P00_X, &P01_X, &P10_X, &P11_X, gx, elapsedTime);
  kalman_update(&kalmanAngleX, &biasX, &P00_X, &P01_X, &P10_X, &P11_X, accAngleX);

  kalman_predict(&kalmanAngleY, &biasY, &P00_Y, &P01_Y, &P10_Y, &P11_Y, gy, elapsedTime);
  kalman_update(&kalmanAngleY, &biasY, &P00_Y, &P01_Y, &P10_Y, &P11_Y, accAngleY);

  roll = kalmanAngleX;
  pitch = kalmanAngleY;
  yaw += gz * elapsedTime;

  if (abs(roll - prev_roll) > 10)
    roll = prev_roll;
  if (abs(pitch - prev_pitch) > 10)
    pitch = prev_pitch;
  if (abs(yaw - prev_yaw) > 10)
    yaw = prev_yaw;

  prev_roll = roll;
  prev_pitch = pitch;
  prev_yaw = yaw;
}


void kalman_predict(float* angle, float* bias,
                    float* P00, float* P01, float* P10, float* P11,
                    float newRate, float dt) {
  *angle += dt * (newRate - *bias);
  *P00 += dt * (dt * *P11 - *P01 - *P10 + Q_angle);
  *P01 -= dt * *P11;
  *P10 -= dt * *P11;
  *P11 += Q_bias * dt;
}


void kalman_update(float* angle, float* bias,
                   float* P00, float* P01, float* P10, float* P11,
                   float newAngle) {
  float y = newAngle - *angle;
  float S = *P00 + R_measure;
  float K0 = *P00 / S;
  float K1 = *P10 / S;

  *angle += K0 * y;
  *bias += K1 * y;

  float P00_temp = *P00;
  float P01_temp = *P01;

  *P00 -= K0 * P00_temp;
  *P01 -= K0 * P01_temp;
  *P10 -= K1 * P00_temp;
  *P11 -= K1 * P01_temp;
}


void calculate_IMU_error() {
  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    accRaw[0] = (Wire.read() << 8 | Wire.read()) / 4096.0;
    accRaw[1] = (Wire.read() << 8 | Wire.read()) / 4096.0;
    accRaw[2] = (Wire.read() << 8 | Wire.read()) / 4096.0;
    accError[0] += atan2(accRaw[1], sqrt(accRaw[0] * accRaw[0] + accRaw[2] * accRaw[2])) * RAD2DEG;
    accError[1] += atan2(-accRaw[0], sqrt(accRaw[1] * accRaw[1] + accRaw[2] * accRaw[2])) * RAD2DEG;
  }
  accError[0] /= 2000.0;
  accError[1] /= 2000.0;

  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    gyrError[0] += (Wire.read() << 8 | Wire.read()) / 32.8;
    gyrError[1] += (Wire.read() << 8 | Wire.read()) / 32.8;
    gyrError[2] += (Wire.read() << 8 | Wire.read()) / 32.8;
  }
  for (int i = 0; i < 3; i++) gyrError[i] /= 2000.0;
}


// ===================== CASCADED CONTROLLER (MATLAB implementation) =====================

/**
 * Outer loop: angle command -> rate command
 * Implements: rateCmd = OPIDP * (cmdDeg - measDeg) with deadband and limit
 */
float outerAngleToRate(float cmdDeg, float measDeg, float OPIDP) {
  float err = cmdDeg - measDeg;
  if (abs(err) < ANGLE_DB_DEG) err = 0;

  float rateCmd = OPIDP * err;
  rateCmd = constrain(rateCmd, -RATECMD_LIM_DPS, RATECMD_LIM_DPS);
  return rateCmd;
}


/**
 * Inner loop: rate PID with anti-windup (gyro-based D term)
 * Implements:
 *   pTerm = KPIDP * rateErr
 *   dTerm = KPIDD * (-gyroRate)       [gyro-based D]
 *   iTerm = KPIDI * iRate with conditional integration
 * 
 * Anti-windup: only integrate if output is not saturated
 *   AND error is in the correct direction
 */
float innerRatePID(float rateCmd, float gyroRate, float& iRate, float dt) {
  float rateErr = rateCmd - gyroRate;

  float pTerm = KPIDP * rateErr;
  float dTerm = KPIDD * (-gyroRate);  // Gyro-based D (important!)

  float uNoI = pTerm + dTerm;
  float uPred = uNoI + KPIDI * iRate;

  // Conditional integration anti-windup
  bool satHigh = (uPred >= PID_LIM) && (rateErr > 0);
  bool satLow = (uPred <= -PID_LIM) && (rateErr < 0);

  if (!(satHigh || satLow)) {
    iRate += rateErr * dt;
    iRate = constrain(iRate, -IRATE_LIM, IRATE_LIM);
  }

  float u = uNoI + KPIDI * iRate;
  u = constrain(u, -PID_LIM, PID_LIM);
  return u;
}


/**
 * PLUS Mixer: converts axis commands to motor outputs
 * Motor order: [Front Right Back Left]
 * 
 * Pitch control: Front/Back
 * Roll control: Left/Right
 * Yaw control: Diagonal pairs
 */
void mixPlus(float base, float PIDx, float PIDy, float PIDz) {
  float mF = base + (-PIDy) + (0)     + (+PIDz);
  float mR = base + (0)     + (-PIDx) + (-PIDz);
  float mB = base + (+PIDy) + (0)     + (+PIDz);
  float mL = base + (0)     + (+PIDx) + (-PIDz);

  m[0].Final = constrain(mF, 1000.0f, 2000.0f);
  m[1].Final = constrain(mR, 1000.0f, 2000.0f);
  m[2].Final = constrain(mB, 1000.0f, 2000.0f);
  m[3].Final = constrain(mL, 1000.0f, 2000.0f);
}


/**
 * X-axis (Roll) cascaded controller
 */
void PID_cascaded_X() {
  // Outer: angle -> rate command
  float rateCmd = outerAngleToRate(x, roll, OPIDP_ROLL);

  // Inner: rate -> axis control output
  PID_x = innerRatePID(rateCmd, gyrRateX, iRateRoll, elapsedTime);
}


/**
 * Y-axis (Pitch) cascaded controller
 */
void PID_cascaded_Y() {
  // Outer: angle -> rate command
  float rateCmd = outerAngleToRate(y, pitch, OPIDP_PITCH);

  // Inner: rate -> axis control output
  PID_y = innerRatePID(rateCmd, gyrRateY, iRatePitch, elapsedTime);
}


/**
 * Z-axis (Yaw) cascaded controller
 */
void PID_cascaded_Z() {
  // For now, simple rate control (outer loop can be added if needed)
  // Set desired yaw rate based on stick or desired angle
  float yawRateCmd = 0;  // TODO: add outer loop if angle-based yaw control desired

  // Inner: rate -> axis control output
  PID_z = innerRatePID(yawRateCmd, gyrRateZ, iRateYaw, elapsedTime);
}


// ===================== MOTOR UPDATE =====================
void motorchangetest(bool fast = false) {
  float factor = fast ? 1.0f : 0.2f;

  for (int i = 0; i < 4; i++) {
    float diff = m[i].Final - m[i].Initial;
    m[i].Power = m[i].Initial + factor * diff;
    m[i].Initial = m[i].Power;
    m[i].update();
  }
}


// ===================== RECEIVER =====================
void recv() {
  // Throttle: CH1 → slider (0–1000)
  slider = readChannel(0, 0, 1000);

  // Roll: CH2 → x (in degrees, typically -30 to +30)
  x = map(readChannel(1, 0, 1000), 0, 1000, -30, 30);

  // Pitch: CH3 → y (in degrees, typically -30 to +30)
  y = map(readChannel(2, 0, 1000), 0, 1000, -30, 30);

  // Switch: CH5 → button (1/0)
  button = readSwitch(4);

  lastSignalTime = millis();
}


// ===================== LANDING =====================
void land() {
  static bool firstRun = true;
  static unsigned long lastStepTime = 0;
  static float descentRate = 2.0;

  if (firstRun) {
    Serial.println("Landing started...");
    firstRun = false;
    descentRate = 2.0;
  }

  if (!failsafeLanding && millis() - landingStartTime > 2000 && !button) {
    Serial.println("Landing aborted by user.");
    landingInProgress = false;
    firstRun = true;
    lastLandingCommandTime = millis();
    return;
  }

  if (failsafeLanding && millis() - lastSignalTime < SIGNAL_TIMEOUT) {
    Serial.println("Signal recovered — resuming flight.");
    failsafeLanding = false;
    landingInProgress = false;
    firstRun = true;
    return;
  }

  descentRate += 0.05;
  descentRate = constrain(descentRate, 2.0, 10.0);

  if (millis() - lastStepTime > 200) {
    lastStepTime = millis();
    throttle = max(1000, throttle - descentRate);
  }

  x = 0;
  y = 0;
  IMU();
  PID_cascaded_X();
  PID_cascaded_Y();
  PID_cascaded_Z();

  roll = pitch = kalmanAngleX = kalmanAngleY = 0;
  biasX = biasY = 0;

  for (int i = 0; i < 4; i++) {
    m[i].Final = throttle;
  }
  motorchangetest(true);

  if (m[0].Power <= 1030 && m[1].Power <= 1030 && m[2].Power <= 1030 && m[3].Power <= 1030) {
    Serial.println("Landing complete. Drone disarmed.");
    for (int i = 0; i < 4; i++) {
      m[i].Power = 1000;
      m[i].update();
    }
    armed = false;
    failsafeLanding = false;
    landingInProgress = false;
    firstRun = true;
  }
}


// ===================== DEBUG =====================
void debug_output() {
  Serial.print(m[0].Power);
  Serial.print("/");
  Serial.print(m[1].Power);
  Serial.print("/");
  Serial.print(m[2].Power);
  Serial.print("/");
  Serial.print(m[3].Power);
  Serial.print(" | ");
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.print(yaw);
  Serial.print(" | ");
  Serial.print(slider);
  Serial.print("/");
  Serial.print(x);
  Serial.print("/");
  Serial.print(y);
  Serial.print("/");
  Serial.println(button);
}


void LedBlinker() {
  static unsigned long lastBlinkTime = 0;
  static int blinkPhase = 0;
  static const int failsafePatternCount = 6;
  static const int landingPatternCount = 2;
  static const int failsafeTimes[failsafePatternCount] = { 300, 100, 100, 50, 50, 1000 };
  static const bool failsafeStates[failsafePatternCount] = { HIGH, LOW, HIGH, LOW, HIGH, LOW };
  static const int landingTimes[landingPatternCount] = { 300, 300 };
  static const bool landingStates[landingPatternCount] = { HIGH, LOW };

  if (failsafeLanding || landingInProgress) {
    unsigned long now = millis();
    const int* blinkTimes;
    const bool* states;
    int patternCount;

    if (failsafeLanding) {
      blinkTimes = failsafeTimes;
      states = failsafeStates;
      patternCount = failsafePatternCount;
    } else {
      blinkTimes = landingTimes;
      states = landingStates;
      patternCount = landingPatternCount;
    }

    if (now - lastBlinkTime >= blinkTimes[blinkPhase]) {
      lastBlinkTime = now;
      digitalWrite(led, states[blinkPhase]);
      blinkPhase = (blinkPhase + 1) % patternCount;
    }
  } else if (armed) {
    digitalWrite(led, HIGH);
    blinkPhase = 0;
  } else {
    digitalWrite(led, LOW);
    blinkPhase = 0;
  }
}

#include <Wire.h>
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h>
#endif

#define MPU_ADDR 0x68
#define RAD2DEG (180.0 / 3.14159265)
#define MAX_THROTTLE 1950
#define TEST_MODE false

RH_ASK driver;
float acc[3], gyro[3];
float angle[2], kalAngle[2];
float pidError[3] = {0};
float pidI[3] = {0};
float pidOutput[3] = {0};
float setpoint[3] = {0};  // pitch, roll, yaw

float Kp[3] = {2.0, 2.0, 3.0};
float Ki[3] = {0.05, 0.05, 0.0};
float Kd[3] = {1.0, 1.0, 0.0};

unsigned long prevTime = 0;
int16_t throttle = 0, input[4] = {0};
int motor[4];
float pitchInput, rollInput, yawInput;
bool signalReceived = false, failsafeTriggered = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0); Wire.endTransmission(true); // ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(true); // ±500°/s

  if (!driver.init()) {
    Serial.println(F("Radio init failed"));
    while (1);
  }

  prevTime = micros();
  digitalWrite(13, HIGH); delay(300); digitalWrite(13, LOW);
}

void loop() {
  readIMU();
  kalmanFilter();
  if (driver.recv((uint8_t*)input, NULL)) {
    throttle = input[0];
    pitchInput = input[1];
    rollInput = input[2];
    yawInput = input[3];
    signalReceived = true;
  } else {
    if (signalReceived) {
      triggerFailsafe();
    }
  }

  computePID();
  updateMotors();
  if (TEST_MODE) debugOutput();
}

void readIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  acc[0] = (Wire.read() << 8 | Wire.read()) / 16384.0;
  acc[1] = (Wire.read() << 8 | Wire.read()) / 16384.0;
  acc[2] = (Wire.read() << 8 | Wire.read()) / 16384.0;
  Wire.read(); Wire.read(); // skip temp
  gyro[0] = (Wire.read() << 8 | Wire.read()) / 65.5;
  gyro[1] = (Wire.read() << 8 | Wire.read()) / 65.5;
  gyro[2] = (Wire.read() << 8 | Wire.read()) / 65.5;

  angle[0] = atan2(acc[1], acc[2]) * RAD2DEG;
  angle[1] = atan2(acc[0], acc[2]) * RAD2DEG;
}

void kalmanFilter() {
  float dt = (micros() - prevTime) / 1000000.0;
  prevTime = micros();

  for (uint8_t i = 0; i < 2; i++) {
    float rate = gyro[i] - 0;  // assume 0 bias
    kalAngle[i] += dt * rate;
    float err = angle[i] - kalAngle[i];
    kalAngle[i] += 0.01 * err;
  }
}

void computePID() {
  float dt = (micros() - prevTime) / 1000000.0;
  setpoint[0] = pitchInput;
  setpoint[1] = rollInput;
  setpoint[2] = yawInput;

  for (uint8_t i = 0; i < 3; i++) {
    float current = (i < 2) ? kalAngle[i] : gyro[2];  // yaw from gyro
    pidError[i] = setpoint[i] - current;
    pidI[i] += pidError[i] * Ki[i] * dt;
    float D = (pidError[i] - pidOutput[i]) / dt;
    pidOutput[i] = Kp[i] * pidError[i] + pidI[i] + Kd[i] * D;
  }
}

void updateMotors() {
  if (failsafeTriggered || throttle < 1050) {
    for (uint8_t i = 0; i < 4; i++) motor[i] = 0;
    return;
  }

  // Mixing pitch, roll, yaw
  motor[0] = throttle - pidOutput[0] - pidOutput[1] + pidOutput[2]; // FL
  motor[1] = throttle - pidOutput[0] + pidOutput[1] - pidOutput[2]; // FR
  motor[2] = throttle + pidOutput[0] + pidOutput[1] + pidOutput[2]; // RR
  motor[3] = throttle + pidOutput[0] - pidOutput[1] - pidOutput[2]; // RL

  for (uint8_t i = 0; i < 4; i++) {
    motor[i] = constrain(motor[i], 1000, MAX_THROTTLE);
    analogWrite(i + 3, map(motor[i], 1000, MAX_THROTTLE, 0, 255));  // Pins 3–6
  }
}

void triggerFailsafe() {
  failsafeTriggered = true;
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  for (uint8_t i = 0; i < 4; i++) analogWrite(i + 3, 0);
}

void debugOutput() {
  Serial.print(F("P:")); Serial.print(kalAngle[0]);
  Serial.print(F(" R:")); Serial.print(kalAngle[1]);
  Serial.print(F(" Y:")); Serial.print(gyro[2]);
  Serial.print(F(" T:")); Serial.println(throttle);
}

#include <Wire.h>
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h>  // Needed to compile RH_ASK
#endif

#define RAD2DEG (180.0 / 3.14159265)
#define MPU_ADDR 0x68
#define MAX_THROTTLE 1950  // Set to 2000 for full range
#define TEST_MODE false    // Set to false for actual flight

bool armed = false;
float previous_error_x = 0, previous_error_y = 0;
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

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Started serial monitor output");

  // IMU Init
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
  Wire.write(0x1B);// dmp
  Wire.write(0x10);
  Wire.endTransmission();
  Serial.println("Started IMU  /  Calibrating...");
  calculate_IMU_error();
  Serial.println("Finished Calibrating IMU");
  delay(500);
  Serial.println("System ready");
  delay(1000);
}

// ------------------ LOOP ------------------
void loop() {

      if (!armed) {
        armed = true;
        Serial.println("Drone ARMED");
        roll = pitch = yaw = 0;
        kalmanAngleX = 0;
        kalmanAngleY = 0;
        biasX = biasY = 0;
       
        previous_error_x = 0;
        previous_error_y = 0;
      } 
    

 

    IMU();
    


    debug_output();
  
  delayMicroseconds(500);
}

// ------------------ IMU ------------------
void IMU() {
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0f;

  // Get raw data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Start reading from Accel X
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  float ax = (Wire.read() << 8 | Wire.read()) / 4096.0;
  float ay = (Wire.read() << 8 | Wire.read()) / 4096.0;
  float az = (Wire.read() << 8 | Wire.read()) / 4096.0;

  float accAngleX = atan2(ay, sqrt(ax * ax + az * az)) * RAD2DEG - accError[0];
  float accAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * RAD2DEG - accError[1];


  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);  // Start reading from Gyro X
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  float gx = (Wire.read() << 8 | Wire.read()) / 32.8 - gyrError[0];
  float gy = (Wire.read() << 8 | Wire.read()) / 32.8 - gyrError[1];
  float gz = (Wire.read() << 8 | Wire.read()) / 32.8 - gyrError[2];

  // --- Kalman Filter for X axis ---
  kalman_predict(&kalmanAngleX, &biasX, &P00_X, &P01_X, &P10_X, &P11_X, gx, elapsedTime);
  kalman_update(&kalmanAngleX, &biasX, &P00_X, &P01_X, &P10_X, &P11_X, accAngleX);

  // --- Kalman Filter for Y axis ---
  kalman_predict(&kalmanAngleY, &biasY, &P00_Y, &P01_Y, &P10_Y, &P11_Y, gy, elapsedTime);
  kalman_update(&kalmanAngleY, &biasY, &P00_Y, &P01_Y, &P10_Y, &P11_Y, accAngleY);

  roll = kalmanAngleX;
  pitch = kalmanAngleY;
  yaw += gz * elapsedTime;

  if (abs(roll - prev_roll) > 10)
    roll = prev_roll;
  if (abs(roll - prev_pitch) > 10)
    pitch = prev_pitch;
  if (abs(roll - prev_yaw) > 10)
    yaw = prev_yaw;
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

// ------------------ CALIBRATION ------------------
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

// ------------------ DEBUG ------------------
void debug_output() {
  Serial.print(" | ");
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.print(yaw);
  Serial.println(" | ");

}


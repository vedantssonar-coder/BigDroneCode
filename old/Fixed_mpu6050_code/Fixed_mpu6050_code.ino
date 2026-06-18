#include <Wire.h>

#define RAD2DEG (180.0 / 3.14159265)
#define DEG2RAD (3.14159265 / 180.0)
#define MPU_ADDR 0x68

const float MOUNT_OFFSET_ROLL = -2;
const float MOUNT_OFFSET_PITCH = 0.35;

float gyrError[3];
float roll = 0, pitch = 0, yaw = 0;
float prev_roll = 0, prev_pitch = 0, prev_yaw = 0;
float gyrRateX = 0, gyrRateY = 0, gyrRateZ = 0;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
const float Kp = 2.0;
const float Ki = 0.005;
float eIntX = 0, eIntY = 0, eIntZ = 0;

float axf = 0, ayf = 0, azf = 0;
const float ACC_LPF = 0.2f;

unsigned long currentTime = 0, previousTime = 0;
float elapsedTime = 0;

float invSqrt(float x) {
  if (x < 1e-10f) return 0;
  return 1.0f / sqrt(x);
}

void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt) {
  float norm = invSqrt(ax * ax + ay * ay + az * az);
  if (!isfinite(norm) || norm == 0) return;
  ax *= norm;
  ay *= norm;
  az *= norm;

  float vx = 2 * (q1 * q3 - q0 * q2);
  float vy = 2 * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  float ex = ay * vz - az * vy;
  float ey = az * vx - ax * vz;
  float ez = ax * vy - ay * vx;

  eIntX += Ki * ex * dt;
  eIntY += Ki * ey * dt;
  eIntZ += Ki * ez * dt;

  gx += Kp * ex + eIntX;
  gy += Kp * ey + eIntY;
  gz += Kp * ez + eIntZ;

  float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
  float dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
  float dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
  float dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;

  q0 += dq0;
  q1 += dq1;
  q2 += dq2;
  q3 += dq3;

  float qnorm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  if (!isfinite(qnorm)) {
    // should never happen now but just in case log it
    Serial.println("ERR: quaternion NaN");
    return;  // do NOT reset — hold last good value
  }
  if (qnorm < 0.9f || qnorm > 1.1f) {
    // drifting, use sqrt for accurate renormalization
    float s = sqrt(qnorm);
    q0 /= s;
    q1 /= s;
    q2 /= s;
    q3 /= s;
  } else {
    norm = invSqrt(qnorm);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
  }
}

void quaternionToEuler(float &r, float &p, float &y) {
  r = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * RAD2DEG;
  float sinp = constrain(2 * (q0 * q2 - q3 * q1), -1.0f, 1.0f);
  p = asin(sinp) * RAD2DEG;
  y = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * RAD2DEG;
}

void IMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  // Check we got all 14 bytes
  int received = Wire.requestFrom(MPU_ADDR, 14);
  if (received != 14) return;  // skip cycle, don't touch quaternion

  float ax_raw = (Wire.read() << 8 | Wire.read()) / 4096.0;
  float ay_raw = (Wire.read() << 8 | Wire.read()) / 4096.0;
  float az_raw = (Wire.read() << 8 | Wire.read()) / 4096.0;
  Wire.read();
  Wire.read();
  float gx = ((Wire.read() << 8 | Wire.read()) / 32.8 - gyrError[0]) * DEG2RAD;
  float gy = ((Wire.read() << 8 | Wire.read()) / 32.8 - gyrError[1]) * DEG2RAD;
  float gz = ((Wire.read() << 8 | Wire.read()) / 32.8 - gyrError[2]) * DEG2RAD;

  // Sanity check — if accel reads all zero I2C silently failed
  if (ax_raw == 0 && ay_raw == 0 && az_raw == 0) return;

  axf += ACC_LPF * (ax_raw - axf);
  ayf += ACC_LPF * (ay_raw - ayf);
  azf += ACC_LPF * (az_raw - azf);

  gyrRateX = gx * RAD2DEG;
  gyrRateY = gy * RAD2DEG;
  gyrRateZ = gz * RAD2DEG;

  mahonyUpdate(gx, gy, gz, axf, ayf, azf, elapsedTime);
  quaternionToEuler(roll, pitch, yaw);

  roll -= MOUNT_OFFSET_ROLL;
  pitch -= MOUNT_OFFSET_PITCH;
}

void calculate_IMU_error() {
  Serial.println("Calibrating... keep still");

  for (int i = 0; i < 5000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    gyrError[0] += (Wire.read() << 8 | Wire.read()) / 32.8;
    gyrError[1] += (Wire.read() << 8 | Wire.read()) / 32.8;
    gyrError[2] += (Wire.read() << 8 | Wire.read()) / 32.8;
  }
  for (int i = 0; i < 3; i++) gyrError[i] /= 5000.0;

  Serial.print("Gyro offsets: ");
  Serial.print(gyrError[0]);
  Serial.print(", ");
  Serial.print(gyrError[1]);
  Serial.print(", ");
  Serial.println(gyrError[2]);

  float axSum = 0, aySum = 0, azSum = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    axSum += (Wire.read() << 8 | Wire.read()) / 4096.0;
    aySum += (Wire.read() << 8 | Wire.read()) / 4096.0;
    azSum += (Wire.read() << 8 | Wire.read()) / 4096.0;
  }
  float ax = axSum / 500.0;
  float ay = aySum / 500.0;
  float az = azSum / 500.0;

  axf = ax;
  ayf = ay;
  azf = az;

  float initRoll = atan2(ay, sqrt(ax * ax + az * az));
  float initPitch = atan2(-ax, sqrt(ay * ay + az * az));

  q0 = cos(initRoll / 2) * cos(initPitch / 2);
  q1 = sin(initRoll / 2) * cos(initPitch / 2);
  q2 = cos(initRoll / 2) * sin(initPitch / 2);
  q3 = -sin(initRoll / 2) * sin(initPitch / 2);

  float norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= norm;
  q1 *= norm;
  q2 *= norm;
  q3 *= norm;

  quaternionToEuler(roll, pitch, yaw);

  Serial.print("Measured mount offsets — Roll: ");
  Serial.print(roll);
  Serial.print(", Pitch: ");
  Serial.println(pitch);

  roll -= MOUNT_OFFSET_ROLL;
  pitch -= MOUNT_OFFSET_PITCH;
  prev_roll = roll;
  prev_pitch = pitch;
  prev_yaw = yaw;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

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

  calculate_IMU_error();
  Serial.println("Ready.");
}

void loop() {
  static unsigned long loopTimer = micros();
  while (micros() - loopTimer < 10000)
    ;
  loopTimer = micros();

  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0f;
  if (elapsedTime <= 0 || elapsedTime > 0.05f) elapsedTime = 0.01f;

  IMU();
  roll = -roll;

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" | Pitch: ");
    Serial.print(pitch);
    Serial.print(" | Yaw: ");
    Serial.println(yaw);
  }

  roll = -roll;
}
#include <Wire.h>
#include <ServoTimer2.h>

// ============================================================
// DEFINES
// ============================================================
#define RAD2DEG      (180.0f / 3.14159265f)
#define DEG2RAD      (3.14159265f / 180.0f)
#define MPU_ADDR     0x68
#define MAX_THROTTLE 1950
#define TEST_MODE    false
#define CALIBRATION_MODE false

#define LOOP_FREQUENCY 200
#define LOOP_PERIOD_US (1000000 / LOOP_FREQUENCY)  // 5000 µs @ 200 Hz

#define PID_THROTTLE_MIN 1100  // PIDs active only above this throttle value

// ============================================================
// PINS
// ============================================================
const int led  = 12;
const int led1 = 13;

// ============================================================
// TIMING
// ============================================================
unsigned long currentTime = 0, previousTime = 0;
float elapsedTime = 0;

// ============================================================
// FAILSAFE / LANDING
// ============================================================
unsigned long lastSignalTime = 0;
const unsigned long SIGNAL_TIMEOUT = 1000;  // ms without a valid iBUS frame
bool failsafeLanding   = false;
bool landingInProgress = false;
unsigned long landingStartTime = 0;

// ============================================================
// IMU — Mahony quaternion filter
// ============================================================
const float MOUNT_OFFSET_ROLL  = -2.0f;
const float MOUNT_OFFSET_PITCH =  0.35f;

float gyrError[3] = {0, 0, 0};
float roll = 0, pitch = 0, yaw = 0;
float prev_roll = 0, prev_pitch = 0, prev_yaw = 0;
float gyrRateX = 0, gyrRateY = 0, gyrRateZ = 0;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
const float Kp_mah = 2.0f;
const float Ki_mah = 0.005f;
float eIntX = 0, eIntY = 0, eIntZ = 0;

float axf = 0, ayf = 0, azf = 0;
const float ACC_LPF = 0.2f;

// ============================================================
// PID
// ============================================================
float PID_x = 0, PID_y = 0, PID_z = 0;

const float OPIDP_ROLL  = 3.0f;
const float OPIDP_PITCH = 3.0f;
const float OPIDP_YAW   = 2.0f;
const float ANGLE_DB_DEG    = 0.2f;
const float RATECMD_LIM_DPS = 60.0f;

float KPIDP = 2.0f;
float KPIDI = 0.0f;
float KPIDD = 0.5f;

const float PID_LIM   = 350.0f;
const float IRATE_LIM =  80.0f;

float iRateRoll  = 0;
float iRatePitch = 0;
float iRateYaw   = 0;

// ============================================================
// iBUS RECEIVER
// iBUS always encodes channel values as 1000–2000 (PWM µs range).
// CH1 = Pitch (y) | CH2 = Roll (x) | CH3 = Throttle
// CH4 = Yaw       | CH5 = mirror of CH2, ignored | CH6 = spare
// ============================================================
#define IBUS_LENGTH 32
uint8_t  ibusBuf[IBUS_LENGTH];
uint8_t  ibusIdx = 0;
// Safe defaults: sticks centred, throttle at minimum
uint16_t ibusChannels[6] = {1500, 1500, 1000, 1500, 1500, 1500};

#define PWM_MIN      1000
#define PWM_MAX      2000
#define PWM_MID      1500
#define PWM_DEADZONE   50

int slider = 0, x = 0, y = 0;
float yawRateCmd = 0;

// ============================================================
// ESC / MOTORS  (pins 8–11: Front, Right, Back, Left)
// ============================================================
ServoTimer2 esc[4];

class Motor {
public:
  const int index;
  float Power = 1000, Initial = 1000, Final = 1000;
  Motor(int i) : index(i) {}
  void update() {
    esc[index].write(constrain((int)Power, 1000, 2000));
  }
};

float throttle = 1000;
Motor m[] = { Motor(0), Motor(1), Motor(2), Motor(3) };

// ============================================================
// MAHONY HELPERS
// ============================================================
float invSqrt(float x) {
  if (x < 1e-10f) return 0;
  return 1.0f / sqrtf(x);
}

void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt) {
  float norm = invSqrt(ax*ax + ay*ay + az*az);
  if (!isfinite(norm) || norm == 0) return;
  ax *= norm; ay *= norm; az *= norm;

  float vx = 2.0f*(q1*q3 - q0*q2);
  float vy = 2.0f*(q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  float ex = ay*vz - az*vy;
  float ey = az*vx - ax*vz;
  float ez = ax*vy - ay*vx;

  eIntX += Ki_mah * ex * dt;
  eIntY += Ki_mah * ey * dt;
  eIntZ += Ki_mah * ez * dt;

  gx += Kp_mah*ex + eIntX;
  gy += Kp_mah*ey + eIntY;
  gz += Kp_mah*ez + eIntZ;

  float dq0 = 0.5f*(-q1*gx - q2*gy - q3*gz)*dt;
  float dq1 = 0.5f*( q0*gx + q2*gz - q3*gy)*dt;
  float dq2 = 0.5f*( q0*gy - q1*gz + q3*gx)*dt;
  float dq3 = 0.5f*( q0*gz + q1*gy - q2*gx)*dt;

  q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

  float qnorm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
  if (!isfinite(qnorm)) {
    Serial.println("ERR: quaternion NaN");
    return;
  }
  if (qnorm < 0.9f || qnorm > 1.1f) {
    float s = sqrtf(qnorm);
    q0 /= s; q1 /= s; q2 /= s; q3 /= s;
  } else {
    norm = invSqrt(qnorm);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
  }
}

void quaternionToEuler(float &r, float &p, float &y_out) {
  r     = atan2f(2.0f*(q0*q1 + q2*q3),
                 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD2DEG;
  float sinp = constrain(2.0f*(q0*q2 - q3*q1), -1.0f, 1.0f);
  p     = asinf(sinp) * RAD2DEG;
  y_out = atan2f(2.0f*(q0*q3 + q1*q2),
                 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD2DEG;
}

// ============================================================
// IMU
// ============================================================
void IMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  // Burst read: accel(6) + temp(2, discarded) + gyro(6) = 14 bytes
  if (Wire.requestFrom(MPU_ADDR, 14) != 14) return;

  float ax_raw = (Wire.read() << 8 | Wire.read()) / 4096.0f;
  float ay_raw = (Wire.read() << 8 | Wire.read()) / 4096.0f;
  float az_raw = (Wire.read() << 8 | Wire.read()) / 4096.0f;
  Wire.read(); Wire.read();  // temperature — discard
  float gx = ((Wire.read() << 8 | Wire.read()) / 32.8f - gyrError[0]) * DEG2RAD;
  float gy = ((Wire.read() << 8 | Wire.read()) / 32.8f - gyrError[1]) * DEG2RAD;
  float gz = ((Wire.read() << 8 | Wire.read()) / 32.8f - gyrError[2]) * DEG2RAD;

  if (ax_raw == 0 && ay_raw == 0 && az_raw == 0) return;

  axf += ACC_LPF * (ax_raw - axf);
  ayf += ACC_LPF * (ay_raw - ayf);
  azf += ACC_LPF * (az_raw - azf);

  gyrRateX = gx * RAD2DEG;
  gyrRateY = gy * RAD2DEG;
  gyrRateZ = gz * RAD2DEG;

  mahonyUpdate(gx, gy, gz, axf, ayf, azf, elapsedTime);
  quaternionToEuler(roll, pitch, yaw);

  roll  -= MOUNT_OFFSET_ROLL;
  pitch -= MOUNT_OFFSET_PITCH;

  if (fabsf(roll  - prev_roll)  > 10.0f) roll  = prev_roll;
  if (fabsf(pitch - prev_pitch) > 10.0f) pitch = prev_pitch;
  if (fabsf(yaw   - prev_yaw)   > 10.0f) yaw   = prev_yaw;

  prev_roll  = roll;
  prev_pitch = pitch;
  prev_yaw   = yaw;
}

// ============================================================
// IMU CALIBRATION
// ============================================================
void calculate_IMU_error() {
  Serial.println("Calibrating IMU — keep still...");

  for (int i = 0; i < 5000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    gyrError[0] += (Wire.read() << 8 | Wire.read()) / 32.8f;
    gyrError[1] += (Wire.read() << 8 | Wire.read()) / 32.8f;
    gyrError[2] += (Wire.read() << 8 | Wire.read()) / 32.8f;
  }
  for (int i = 0; i < 3; i++) gyrError[i] /= 5000.0f;

  Serial.print("Gyro offsets: ");
  Serial.print(gyrError[0]); Serial.print(", ");
  Serial.print(gyrError[1]); Serial.print(", ");
  Serial.println(gyrError[2]);

  // Average accel over 500 samples to seed the quaternion at actual tilt
  float axSum = 0, aySum = 0, azSum = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6);
    axSum += (Wire.read() << 8 | Wire.read()) / 4096.0f;
    aySum += (Wire.read() << 8 | Wire.read()) / 4096.0f;
    azSum += (Wire.read() << 8 | Wire.read()) / 4096.0f;
  }
  float ax = axSum / 500.0f;
  float ay = aySum / 500.0f;
  float az = azSum / 500.0f;

  axf = ax; ayf = ay; azf = az;

  float initRoll  = atan2f(ay, sqrtf(ax*ax + az*az));
  float initPitch = atan2f(-ax, sqrtf(ay*ay + az*az));

  q0 =  cosf(initRoll/2)*cosf(initPitch/2);
  q1 =  sinf(initRoll/2)*cosf(initPitch/2);
  q2 =  cosf(initRoll/2)*sinf(initPitch/2);
  q3 = -sinf(initRoll/2)*sinf(initPitch/2);

  float norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;

  quaternionToEuler(roll, pitch, yaw);
  Serial.print("Mount tilt measured — Roll: "); Serial.print(roll);
  Serial.print(", Pitch: ");                    Serial.println(pitch);

  roll  -= MOUNT_OFFSET_ROLL;
  pitch -= MOUNT_OFFSET_PITCH;
  prev_roll = roll; prev_pitch = pitch; prev_yaw = yaw;

  Serial.println("IMU calibration done.");
}

// ============================================================
// iBUS RECEIVER
// ============================================================
bool ibusValidChecksum(uint8_t *frame) {
  uint16_t csum = 0xFFFF;
  for (uint8_t i = 0; i < 30; i++) csum -= frame[i];
  return csum == (uint16_t)(frame[30] | (frame[31] << 8));
}

// Call every loop — drains RX buffer, updates ibusChannels[] on valid frame
void ibusRead() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    if (ibusIdx == 0 && b != 0x20) continue;
    if (ibusIdx == 1 && b != 0x40) { ibusIdx = 0; continue; }

    ibusBuf[ibusIdx++] = b;

    if (ibusIdx >= IBUS_LENGTH) {
      ibusIdx = 0;
      if (ibusValidChecksum(ibusBuf)) {
        for (uint8_t i = 0; i < 6; i++) {
          ibusChannels[i] = ibusBuf[2 + i*2] | (ibusBuf[3 + i*2] << 8);
        }
        lastSignalTime = millis();
      }
      // Invalid frames dropped silently — printing here would corrupt RX timing
    }
  }
}

void recv() {
  ibusRead();

  // All iBUS channel values are in the range 1000–2000 µs (standard PWM).
  // If your transmitter outputs a different range, adjust PWM_MIN/MAX above.

  // CH1 → Pitch (y): 1000–2000 → -15 to +15 deg, deadzone around centre
  int rawY = constrain((int)ibusChannels[0], PWM_MIN, PWM_MAX);
  if (abs(rawY - PWM_MID) < PWM_DEADZONE) rawY = PWM_MID;
  y = (int)(((rawY - PWM_MID) / 500.0f) * 15.0f);

  // CH2 → Roll (x): 1000–2000 → -15 to +15 deg
  int rawX = constrain((int)ibusChannels[1], PWM_MIN, PWM_MAX);
  if (abs(rawX - PWM_MID) < PWM_DEADZONE) rawX = PWM_MID;
  x = (int)(((rawX - PWM_MID) / 500.0f) * 15.0f);

  // CH3 → Throttle: 1000–2000 → 0–1000
  slider = constrain((int)ibusChannels[2] - PWM_MIN, 0, 1000);

  // CH4 → Yaw rate: 1000–2000 → -60 to +60 deg/s
  int rawYaw = constrain((int)ibusChannels[3], PWM_MIN, PWM_MAX);
  if (abs(rawYaw - PWM_MID) < PWM_DEADZONE) rawYaw = PWM_MID;
  yawRateCmd = ((rawYaw - PWM_MID) / 500.0f) * 60.0f;

  // CH5 = hardware mirror of CH2 — ignored
  // CH6 = spare — ignored
}

// ============================================================
// ESC CALIBRATION
// ============================================================
void esc_calibration() {
  Serial.println("=== ESC CALIBRATION ===");
  Serial.println("DISCONNECT BATTERY. Waiting 3s...");
  delay(3000);
  for (int i = 0; i < 4; i++) esc[i].write(2000);
  Serial.println("CONNECT BATTERY — wait for beeps (~5s)");
  delay(5000);
  for (int i = 0; i < 4; i++) esc[i].write(1000);
  Serial.println("Done. DISCONNECT BATTERY.");
  delay(5000);
  while (true) {
    for (int i = 0; i < 4; i++) { m[i].Power = 1000; m[i].update(); }
    delay(100);
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  // Serial is shared: iBUS arrives on RX (pin 0), debug goes out on TX (pin 1).
  // They are separate physical pins so they coexist. Never call Serial.read()
  // outside of ibusRead() or it will steal bytes from iBUS frames.
  Serial.begin(115200);

  Wire.begin();
  // 400 kHz I2C — lower to Wire.setClock(100000) if IMU reads look noisy
  Wire.setClock(400000);

  pinMode(led,  OUTPUT);
  pinMode(led1, OUTPUT);

  // MPU-6050 init
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);  // wake, internal clock
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x10);  // accel ±8 g
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x10);  // gyro ±1000 °/s
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x05);  // DLPF ~10 Hz
  Wire.endTransmission();

  digitalWrite(led, HIGH);
  delay(2000);
  digitalWrite(led, LOW);

  calculate_IMU_error();

  esc[0].attach(8);
  esc[1].attach(9);
  esc[2].attach(10);
  esc[3].attach(11);

  if (CALIBRATION_MODE) esc_calibration();

  // Standard ESC arming pulse
  for (int i = 0; i < 4; i++) { m[i].Power = 2000; m[i].update(); }
  delay(100);
  for (int i = 0; i < 4; i++) { m[i].Power = 1000; m[i].update(); }

  Serial.println("System ready.");
  delay(2000);  // let iBUS stream stabilise before first recv()
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  static unsigned long loop_timer = micros();
  unsigned long now = micros();
  while (now - loop_timer < LOOP_PERIOD_US) now = micros();
  loop_timer = now;

  LedBlinker();

  previousTime = currentTime;
  currentTime  = millis();
  elapsedTime  = (currentTime - previousTime) / 1000.0f;
  if (elapsedTime <= 0 || elapsedTime > 0.05f) elapsedTime = 0.01f;

  recv();  // drain iBUS buffer — also updates lastSignalTime on valid frame

  // Update throttle from stick unless failsafe landing is overriding it
  if (!landingInProgress) {
    throttle = constrain(1000.0f + slider, 1000.0f, (float)MAX_THROTTLE);
  }

  // IMU runs every loop regardless of throttle — keeps filter converged
  IMU();

  // ---- Failsafe: trigger slow descent on sustained signal loss ----
  if (millis() - lastSignalTime > SIGNAL_TIMEOUT && !failsafeLanding) {
    Serial.println("Signal lost — failsafe landing.");
    failsafeLanding   = true;
    landingInProgress = true;
    landingStartTime  = millis();
  }

  // ---- State machine ----
  if (landingInProgress) {
    // Failsafe slow descent — land() handles its own PID and motor writes
    land();

  } else if (throttle >= PID_THROTTLE_MIN) {
    // Normal flight — PIDs active
    PID_cascaded_X();
    PID_cascaded_Y();
    // PID_cascaded_Z();  // yaw not wired into mixer yet — enable when ready
    mixPlus(throttle, PID_x, PID_y, PID_z);
    motorchangetest(false);

  } else {
    // Throttle below minimum — PIDs off, motors get raw throttle only
    PID_x = PID_y = PID_z = 0;
    iRateRoll = iRatePitch = iRateYaw = 0;
    for (int i = 0; i < 4; i++) {
      m[i].Final = throttle;
      m[i].update();
    }
  }

  printLoopHz();

  // Debug output throttled to every 100 ms — printing at 200 Hz would stall timing
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 100) {
    lastDebug = millis();
    Serial.print(millis());
    Serial.print(" | M:");
    Serial.print(m[0].Power); Serial.print("/");
    Serial.print(m[1].Power); Serial.print("/");
    Serial.print(m[2].Power); Serial.print("/");
    Serial.print(m[3].Power);
    Serial.print(" | PID:");
    Serial.print(PID_x, 1); Serial.print(",");
    Serial.print(PID_y, 1);
    Serial.print(" | R:");  Serial.print(roll, 1);
    Serial.print(" P:");    Serial.println(pitch, 1);
  }
}

// ============================================================
// PID — cascaded angle → rate → output
// ============================================================
float outerAngleToRate(float cmdDeg, float measDeg, float OPIDP) {
  float err = cmdDeg - measDeg;
  if (fabsf(err) < ANGLE_DB_DEG) err = 0;
  return constrain(OPIDP * err, -RATECMD_LIM_DPS, RATECMD_LIM_DPS);
}

float innerRatePID(float rateCmd, float gyroRate, float &iRate, float dt) {
  float rateErr = rateCmd - gyroRate;
  float pTerm   = KPIDP * rateErr;
  float dTerm   = KPIDD * (-gyroRate);  // gyro-based D — no derivative kick on setpoint change
  float uNoI    = pTerm + dTerm;
  float uPred   = uNoI + KPIDI * iRate;

  bool satHigh = (uPred >=  PID_LIM) && (rateErr > 0);
  bool satLow  = (uPred <= -PID_LIM) && (rateErr < 0);
  if (!(satHigh || satLow)) {
    iRate += rateErr * dt;
    iRate = constrain(iRate, -IRATE_LIM, IRATE_LIM);
  }

  return constrain(uNoI + KPIDI * iRate, -PID_LIM, PID_LIM);
}

void mixPlus(float base, float PIDx, float PIDy, float PIDz) {
  // Motor layout: Front(0) Right(1) Back(2) Left(3)
  m[0].Final = constrain(base - PIDy, 1000.0f, 2000.0f);  // Front
  m[1].Final = constrain(base - PIDx, 1000.0f, 2000.0f);  // Right
  m[2].Final = constrain(base + PIDy, 1000.0f, 2000.0f);  // Back
  m[3].Final = constrain(base + PIDx, 1000.0f, 2000.0f);  // Left
}

void PID_cascaded_X() {
  float rateCmd = outerAngleToRate(x, roll, OPIDP_ROLL);
  PID_x = innerRatePID(rateCmd, gyrRateX, iRateRoll, elapsedTime);
}

void PID_cascaded_Y() {
  float rateCmd = outerAngleToRate(y, pitch, OPIDP_PITCH);
  PID_y = innerRatePID(rateCmd, gyrRateY, iRatePitch, elapsedTime);
}

void PID_cascaded_Z() {
  // yawRateCmd comes directly from CH4 stick — no outer angle loop needed for yaw
  PID_z = innerRatePID(yawRateCmd, gyrRateZ, iRateYaw, elapsedTime);
}

// ============================================================
// MOTOR UPDATE
// ============================================================
void motorchangetest(bool fast) {
  float factor = fast ? 1.0f : 0.2f;
  for (int i = 0; i < 4; i++) {
    m[i].Power   = m[i].Initial + factor * (m[i].Final - m[i].Initial);
    m[i].Initial = m[i].Power;
    m[i].update();
  }
}

// ============================================================
// FAILSAFE LANDING
// Manual trigger removed — this only runs on signal loss.
// If signal recovers mid-descent, normal flight resumes immediately.
// ============================================================
void land() {
  static bool  firstRun     = true;
  static unsigned long lastStepTime = 0;
  static float descentRate  = 2.0f;

  if (firstRun) {
    Serial.println("Failsafe landing started...");
    firstRun    = false;
    descentRate = 2.0f;
  }

  // Signal recovered — hand back control immediately
  if (millis() - lastSignalTime < SIGNAL_TIMEOUT) {
    Serial.println("Signal recovered — resuming normal flight.");
    failsafeLanding   = false;
    landingInProgress = false;
    firstRun          = true;
    // Restore throttle from stick so there is no jump
    throttle = constrain(1000.0f + slider, 1000.0f, (float)MAX_THROTTLE);
    return;
  }

  // Gradually increase descent speed over time
  descentRate = constrain(descentRate + 0.05f, 2.0f, 10.0f);

  if (millis() - lastStepTime > 200) {
    lastStepTime = millis();
    throttle = max(1000.0f, throttle - descentRate);
  }

  // Level-hold during descent — PIDs stay active with zero stick input
  x = 0; y = 0;
  PID_cascaded_X();
  PID_cascaded_Y();
  PID_cascaded_Z();

  for (int i = 0; i < 4; i++) m[i].Final = throttle;
  motorchangetest(true);

  // Once all motors are at minimum, cut power
  if (m[0].Power <= 1030 && m[1].Power <= 1030 &&
      m[2].Power <= 1030 && m[3].Power <= 1030) {
    Serial.println("Landing complete.");
    for (int i = 0; i < 4; i++) { m[i].Power = 1000; m[i].update(); }
    failsafeLanding   = false;
    landingInProgress = false;
    firstRun          = true;
  }
}

// ============================================================
// DEBUG / STATUS
// ============================================================
void debug_output() {
  Serial.print(m[0].Power); Serial.print("/");
  Serial.print(m[1].Power); Serial.print("/");
  Serial.print(m[2].Power); Serial.print("/");
  Serial.print(m[3].Power);
  Serial.print(" | R:"); Serial.print(roll);
  Serial.print(" P:");   Serial.print(pitch);
  Serial.print(" Y:");   Serial.print(yaw);
  Serial.print(" | thr:"); Serial.print(slider);
  Serial.print(" x:");     Serial.print(x);
  Serial.print(" y:");     Serial.println(y);
}

void printLoopHz() {
  static unsigned long lastPrint = 0;
  static unsigned long count = 0;
  count++;
  if (millis() - lastPrint >= 1000) {
    Serial.print("Loop Hz: ");
    Serial.println(count);
    count     = 0;
    lastPrint = millis();
  }
}

void LedBlinker() {
  static unsigned long lastBlinkTime = 0;
  static int blinkPhase = 0;

  static const int  failsafeTimes[6]  = {300, 100, 100, 50, 50, 1000};
  static const bool failsafeStates[6] = {HIGH, LOW, HIGH, LOW, HIGH, LOW};
  static const int  landingTimes[2]   = {300, 300};
  static const bool landingStates[2]  = {HIGH, LOW};

  if (failsafeLanding) {
    if (millis() - lastBlinkTime >= (unsigned long)failsafeTimes[blinkPhase]) {
      lastBlinkTime = millis();
      digitalWrite(led, failsafeStates[blinkPhase]);
      blinkPhase = (blinkPhase + 1) % 6;
    }
  } else if (landingInProgress) {
    if (millis() - lastBlinkTime >= (unsigned long)landingTimes[blinkPhase]) {
      lastBlinkTime = millis();
      digitalWrite(led, landingStates[blinkPhase]);
      blinkPhase = (blinkPhase + 1) % 2;
    }
  } else if (throttle >= PID_THROTTLE_MIN) {
    digitalWrite(led, HIGH);  // solid on = PIDs active
    blinkPhase = 0;
  } else {
    digitalWrite(led, LOW);   // off = throttle idle
    blinkPhase = 0;
  }
}

/*
 * NOTE — Roll sign convention:
 * Mahony may produce roll with the opposite sign to the old Kalman code.
 * On first bench test: tilt the frame right — if roll goes negative instead
 * of positive, add  roll = -roll;  after the MOUNT_OFFSET line in IMU().
 *
 * LED guide:
 *   Solid ON        throttle >= 1100, PIDs active
 *   Solid OFF       throttle < 1100, PIDs off
 *   Slow blink      failsafe landing in progress
 *   Long-short-short pattern  signal lost
 */
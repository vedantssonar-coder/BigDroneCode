/* Arduino Drone PID Auto-Tuner

This document provides a **practical PID tuning system** for Arduino + IMU drone. two approaches:
1. **Manual Sequential Tuner** (safer, more controlled)
2. **Relay-Based Auto Tuner** (Ziegler-Nichols method)

---

  OPTION 1: Manual Sequential Tuner (RECOMMENDED for your setup)

This is the safest approach. Test one axis at a time and abort immediately if something goes wrong.

Code Addition: add this to drone sketch:

*/
// ==================== PID TUNING MODE ====================

#define TUNING_MODE false          // Set to true to enable tuning
#define TUNE_AXIS 0                // 0=Roll(X), 1=Pitch(Y), 2=Yaw(Z)
#define TUNE_TYPE 0                // 0=P_gain, 1=I_gain, 2=D_gain
const int TUNING_THROTTLE = 1150;  // Hover throttle (start low!)

// Tuning state machine
enum TuningState {
  TUNING_IDLE,
  TUNING_ACTIVE,
  TUNING_COMPLETE
};

TuningState tuningState = TUNING_IDLE;
float tuningValue = 0;
float tuningStep = 0.1;
unsigned long tuningStartTime = 0;
const unsigned long TUNING_DURATION = 3000;  // 3 seconds per test
unsigned int oscillationCount = 0;
float maxDeviation = 0;
float tuningResult[3][3];  // Store best Kp, Ki, Kd for each axis


// ==================== TUNING FUNCTIONS ====================

void startManualPIDTuning() {
  Serial.println("\n========== PID MANUAL TUNING MODE ==========");
  Serial.print("Tuning Axis: ");
  
  switch(TUNE_AXIS) {
    case 0: Serial.println("ROLL (X)"); break;
    case 1: Serial.println("PITCH (Y)"); break;
    case 2: Serial.println("YAW (Z)"); break;
  }
  
  Serial.print("Tuning Parameter: ");
  switch(TUNE_TYPE) {
    case 0: Serial.println("P (Proportional)"); break;
    case 1: Serial.println("I (Integral)"); break;
    case 2: Serial.println("D (Derivative)"); break;
  }
  
  Serial.println("\nStarting throttle to find hover point...");
  Serial.println("Adjust throttle until drone hovers at ~1150µs");
  
  throttle = TUNING_THROTTLE;
  armed = true;
  tuningState = TUNING_ACTIVE;
  tuningStartTime = millis();
}


void manualPIDTuningLoop() {
  // Get user input from remote for gain adjustment
  // x-axis on remote can control the gain value
  static float baseKp = 2.5, baseKi = 0.005, baseKd = 1.2;
  
  // Adjust gains based on slider movement
  float gainMultiplier = 1.0 + (slider / 1000.0);  // Scale by throttle input
  
  switch(TUNE_AXIS) {
    case 0:  // Roll
      if(TUNE_TYPE == 0) kp_x = baseKp * gainMultiplier;
      if(TUNE_TYPE == 1) ki_x = baseKi * gainMultiplier;
      if(TUNE_TYPE == 2) kd_x = baseKd * gainMultiplier;
      break;
      
    case 1:  // Pitch
      if(TUNE_TYPE == 0) kp_y = baseKp * gainMultiplier;
      if(TUNE_TYPE == 1) ki_y = baseKi * gainMultiplier;
      if(TUNE_TYPE == 2) kd_y = baseKd * gainMultiplier;
      break;
      
    case 2:  // Yaw
      if(TUNE_TYPE == 0) kp_z = baseKp * gainMultiplier;
      if(TUNE_TYPE == 1) ki_z = baseKi * gainMultiplier;
      if(TUNE_TYPE == 2) kd_z = baseKd * gainMultiplier;
      break;
  }
  
  // Monitor oscillations
  measureOscillations();
  
  // Print tuning metrics every 500ms
  static unsigned long lastPrint = 0;
  if(millis() - lastPrint > 500) {
    Serial.print("Gain: ");
    Serial.print(gainMultiplier, 3);
    Serial.print(" | Osc: ");
    Serial.print(oscillationCount);
    Serial.print(" | Dev: ");
    Serial.println(maxDeviation, 2);
    lastPrint = millis();
  }
}


void measureOscillations() {
  // Simple oscillation detection based on angle reversals
  static float lastAngle = 0;
  static int direction = 0;
  
  float currentAngle = 0;
  if(TUNE_AXIS == 0) currentAngle = roll;
  if(TUNE_AXIS == 1) currentAngle = pitch;
  if(TUNE_AXIS == 2) currentAngle = yaw;
  
  // Track direction changes (oscillations)
  if((currentAngle - lastAngle) * direction < 0) {
    oscillationCount++;
  }
  direction = (currentAngle > lastAngle) ? 1 : -1;
  
  // Track maximum deviation from zero
  maxDeviation = max(maxDeviation, abs(currentAngle));
  lastAngle = currentAngle;
}


// ==================== OPTION 2: RELAY-BASED AUTO TUNER ====================
// More sophisticated: measures system response and calculates optimal gains

struct AutoTunerState {
  bool active = false;
  float targetSetpoint = 0;
  float outputHigh = 500;    // Max motor power differential
  float outputLow = -500;    // Min motor power differential
  bool outputState = true;   // true = high, false = low
  
  unsigned long t1 = 0, t2 = 0;
  float tHigh = 0, tLow = 0;
  float maxValue = -999, minValue = 999;
  
  float Ku = 0;  // Ultimate gain
  float Tu = 0;  // Period of oscillation
  
  int cycleCount = 0;
  float pAverage = 0, iAverage = 0, dAverage = 0;
};

AutoTunerState autoTuner;

void startRelayAutoTuning(int axis) {
  Serial.println("\n========== RELAY AUTO-TUNING (Ziegler-Nichols) ==========");
  Serial.print("Starting auto-tune for axis: ");
  
  switch(axis) {
    case 0: Serial.println("ROLL"); break;
    case 1: Serial.println("PITCH"); break;
    case 2: Serial.println("YAW"); break;
  }
  
  Serial.println("This will oscillate the drone for ~20-30 seconds.");
  Serial.println("Keep hands ready to disarm!");
  
  autoTuner.active = true;
  autoTuner.cycleCount = 0;
  autoTuner.maxValue = -999;
  autoTuner.minValue = 999;
  throttle = TUNING_THROTTLE;
  armed = true;
}


void relayAutoTunerLoop(int axis) {
  if(!autoTuner.active) return;
  
  float currentAngle = 0;
  
  if(axis == 0) currentAngle = roll;
  else if(axis == 1) currentAngle = pitch;
  else if(axis == 2) currentAngle = yaw;
  
  // Track max/min values
  autoTuner.maxValue = max(autoTuner.maxValue, currentAngle);
  autoTuner.minValue = min(autoTuner.minValue, currentAngle);
  
  // Relay switching: toggle when crossing setpoint
  if(autoTuner.outputState && currentAngle >= autoTuner.targetSetpoint) {
    // Switch to LOW
    autoTuner.outputState = false;
    autoTuner.t1 = micros();
    autoTuner.tHigh = autoTuner.t1 - autoTuner.t2;
    
    Serial.print("Cycle ");
    Serial.print(autoTuner.cycleCount);
    Serial.print(": tHigh=");
    Serial.print(autoTuner.tHigh / 1000.0);
    Serial.println("ms");
    
    autoTuner.maxValue = autoTuner.targetSetpoint;
  } 
  else if(!autoTuner.outputState && currentAngle <= autoTuner.targetSetpoint) {
    // Switch to HIGH
    autoTuner.outputState = true;
    autoTuner.t2 = micros();
    autoTuner.tLow = autoTuner.t2 - autoTuner.t1;
    
    Serial.print("  tLow=");
    Serial.print(autoTuner.tLow / 1000.0);
    Serial.println("ms");
    
    // Calculate Ziegler-Nichols coefficients
    calculateZNGains();
    
    autoTuner.cycleCount++;
    autoTuner.minValue = autoTuner.targetSetpoint;
    
    // Stop after 5-6 cycles
    if(autoTuner.cycleCount >= 5) {
      finishAutoTuning();
    }
  }
  
  // Apply relay output to motors
  if(autoTuner.outputState) {
    m[0].Final += autoTuner.outputHigh;  // Increase certain motors
    m[2].Final -= autoTuner.outputHigh;
  } else {
    m[0].Final -= autoTuner.outputHigh;  // Decrease
    m[2].Final += autoTuner.outputHigh;
  }
}


void calculateZNGains() {
  // Ku = 4*d / (π*a)
  // d = output amplitude, a = input amplitude
  float d = (autoTuner.outputHigh - autoTuner.outputLow) / 2.0;
  float a = (autoTuner.maxValue - autoTuner.minValue) / 2.0;
  
  if(a < 0.1) return;  // Avoid division errors
  
  float Ku = (4.0 * d) / (M_PI * a);
  float Tu = (autoTuner.tHigh + autoTuner.tLow) / 1000000.0;  // Convert to seconds
  
  Serial.print("Ku=");
  Serial.print(Ku, 3);
  Serial.print(", Tu=");
  Serial.println(Tu, 3);
  
  // Ziegler-Nichols coefficients (normal PID - quarter overshoot)
  float Kp = 0.6 * Ku;
  float Ki = (1.2 * Ku) / Tu;
  float Kd = (0.075 * Ku) * Tu;
  
  Serial.print("  → Kp=");
  Serial.print(Kp, 4);
  Serial.print(", Ki=");
  Serial.print(Ki, 6);
  Serial.print(", Kd=");
  Serial.println(Kd, 4);
  
  // Average with previous cycles
  if(autoTuner.cycleCount > 1) {
    autoTuner.pAverage += Kp;
    autoTuner.iAverage += Ki;
    autoTuner.dAverage += Kd;
  }
}


void finishAutoTuning() {
  autoTuner.active = false;
  
  // Calculate averages
  float finalKp = autoTuner.pAverage / (autoTuner.cycleCount - 1);
  float finalKi = autoTuner.iAverage / (autoTuner.cycleCount - 1);
  float finalKd = autoTuner.dAverage / (autoTuner.cycleCount - 1);
  
  Serial.println("\n========== TUNING COMPLETE ==========");
  Serial.print("Final Kp: ");
  Serial.println(finalKp, 4);
  Serial.print("Final Ki: ");
  Serial.println(finalKi, 6);
  Serial.print("Final Kd: ");
  Serial.println(finalKd, 4);
  
  // Copy to appropriate axis
  // (Match your tuning axis here)
  
  landingInProgress = true;  // Land safely
}


// ==================== INTEGRATION INTO MAIN LOOP ====================
// Add this to your loop() function:

/*
if (TUNING_MODE) {
  if (tuningState == TUNING_IDLE && armed) {
    startManualPIDTuning();
  }
  
  if (tuningState == TUNING_ACTIVE && armed) {
    manualPIDTuningLoop();
  }
  
  // Standard flight control during tuning
  IMU();
  PID_X();
  PID_Y();
  PID_Z();
  motorchangetest(false);
  debug_output();
}
*/

/*
---

## Manual Tuning Procedure (Step-by-Step)

### 1. **Initial P-Gain Tuning (Roll)**

```
1. Set TUNING_MODE = true
2. Set TUNE_AXIS = 0 (Roll)
3. Set TUNE_TYPE = 0 (P gain)
4. Upload code
5. Arm drone with low throttle
6. Gradually increase slider to raise Kp_x
7. Watch serial output for oscillations
8. Sweet spot: slight oscillation that dampens quickly
9. Disarm and note the value
```

**What you're looking for:**
- **Too low Kp**: Drone drifts, slow response
- **Optimal Kp**: Small oscillations that settle in 1-2 seconds
- **Too high Kp**: Continuous rapid oscillations

### 2. **Add D-Gain (Damping)**

```
1. Keep Kp from step 1
2. Set TUNE_TYPE = 2 (D gain)
3. Slowly increase Kd_x slider
4. D gain dampens oscillations
5. Continue until oscillations stop
```

### 3. **Add I-Gain (Error Recovery)**

```
1. Keep Kp and Kd from previous steps
2. Set TUNE_TYPE = 1 (I gain)
3. Increase Ki_x slowly
4. I gain recovers from steady-state drift
5. Don't set too high (causes oscillations)
```

### 4. **Repeat for Pitch and Yaw**

Same process for Y-axis (TUNE_AXIS=1) and Z-axis (TUNE_AXIS=2).

---

## Auto-Tuning with Relay Method

**Better for production, requires careful safety monitoring:**

```cpp
// In setup() or after arming:
startRelayAutoTuning(0);  // Auto-tune roll

// This will:
// 1. Apply alternating motor pulses
// 2. Measure oscillation response
// 3. Calculate optimal Kp, Ki, Kd automatically
// 4. Complete in 20-30 seconds
```

**Safety Considerations:**
- Keep drone on ground during relay tuning (tethered)
- Have kill switch ready
- Only use one axis at a time
- Let it complete all cycles before interrupting

---

## Expected Values for Your Drone

Based on your current gains:

```
ROLL/PITCH:
  Kp: 1.5 - 4.0    (you have 2.5)
  Ki: 0.002 - 0.01 (you have 0.005)
  Kd: 0.5 - 2.0    (you have 1.2)

YAW:
  Kp: 1.5 - 3.0    (you have 2.0)
  Ki: 0.002 - 0.008
  Kd: 0.3 - 1.5
```

Your current values are reasonable starting points!

---

## Tuning Tips

1. **Always start with low gains** - increase gradually
2. **Tune one axis completely** before moving to next
3. **Test each change** for 5-10 seconds minimum
4. **Use serial output** to monitor oscillation patterns
5. **Video record flights** for post-analysis
6. **Log data** to SD card if available for detailed analysis
7. **Environmental factors matter**: wind, temperature, weight distribution

---

## Quick Tuning Checklist

- [ ] ESCs calibrated and verified
- [ ] IMU calibrated on level surface
- [ ] Motor response linear and consistent
- [ ] Throttle hovers around 1150µs (test first)
- [ ] Start with Kp tuning only
- [ ] Then add D (damping)
- [ ] Finally add I (recovery)
- [ ] Test in confined space
- [ ] Record video for analysis
- [ ] Log oscillation data

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Drone oscillates continuously | Reduce Kp or increase Kd |
| Drone doesn't respond to input | Increase Kp |
| Slow oscillation recovery | Increase Ki gradually |
| Servo buzzing/overheating | Reduce Kd too much |
| Yaw spins uncontrollably | Reduce Kp_z first |
| Drift in one direction | Increase Ki slightly |
| Jerky movements | Smooth motor updates with motorchangetest() |

---

## Advanced: Frequency Analysis

Once working, you can analyze PID performance by:

1. Recording roll/pitch/yaw to EEPROM during flight
2. Downloading data to PC
3. Performing FFT (Fast Fourier Transform)
4. Looking for resonant frequencies
5. Adjusting gains to avoid those frequencies

For Arduino, consider: [ArduinoFFT library](https://github.com/kosme/arduinoFFT)

---

## References

- Ziegler-Nichols method: https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
- Brett Beauregard's PID Autotune: http://brettbeauregard.com/blog/2012/01/arduino-pid-autotune-library/
- Betaflight PID tuning: https://betaflight.com/docs/wiki/guides/current/PID-Tuning-Guide
- ArduPilot tuning: https://ardupilot.org/plane/docs/new-roll-and-pitch-tuning.html
*/
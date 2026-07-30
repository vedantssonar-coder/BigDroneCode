// PWM to IBUS - Pro Mini
// CH2 fixed on receiver - direct 1:1 channel mapping, no mirroring
// Arduino Pro Mini 5V 16MHz

#define IBUS_FRAME_LENGTH 0x20
#define IBUS_COMMAND40    0x40
#define IBUS_MAXCHANNELS  14
#define IBUS_DEFAULT      1500
#define NUM_CHANNELS      6

// Physical pins, fixed order matching ISR bit positions - do not reorder
const uint8_t PWM_PINS[NUM_CHANNELS] = {2, 3, 4, 5, 6, 7};
// pwmRaw index:                          0  1  2  3  4  5
// Receiver channel:                     CH1 CH2 CH3 CH4 CH5 CH6

volatile uint32_t pwmStart[NUM_CHANNELS];
volatile uint16_t pwmRaw[NUM_CHANNELS];
uint16_t pwmValue[NUM_CHANNELS];

void isrCH1() {
  if (PIND & (1 << 2)) pwmStart[0] = micros();
  else pwmRaw[0] = micros() - pwmStart[0];
}

void isrCH2() {
  if (PIND & (1 << 3)) pwmStart[1] = micros();
  else pwmRaw[1] = micros() - pwmStart[1];
}

ISR(PCINT2_vect) {
  uint8_t cur = PIND;
  static uint8_t prev = 0;
  uint8_t changed = cur ^ prev;
  prev = cur;

  for (uint8_t i = 0; i < 4; i++) {
    uint8_t mask = 1 << (i + 4);
    if (changed & mask) {
      uint32_t now = micros();
      if (cur & mask) pwmStart[i + 2] = now;
      else            pwmRaw[i + 2]   = now - pwmStart[i + 2];
    }
  }
}

static inline uint16_t atomicRead16(volatile uint16_t *p) {
  uint8_t sreg = SREG;
  cli();
  uint16_t v = *p;
  SREG = sreg;
  return v;
}

byte ibuf[IBUS_FRAME_LENGTH];

void buildAndSendIBUS() {
  // Read all physical pins atomically
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    uint16_t v = atomicRead16(&pwmRaw[i]);
    pwmValue[i] = constrain(v, 1000, 2000);
  }

  ibuf[0] = IBUS_FRAME_LENGTH;
  ibuf[1] = IBUS_COMMAND40;

  for (uint8_t i = 0; i < IBUS_MAXCHANNELS; i++) {
    uint16_t v;
    switch (i) {
      case 0: v = pwmValue[0]; break;       // IBUS CH1 <- pin2 (CH1)
      case 1: v = pwmValue[1]; break;       // IBUS CH2 <- pin3 (CH2)
      case 2: v = pwmValue[2]; break;       // IBUS CH3 <- pin4 (CH3)
      case 3: v = pwmValue[3]; break;       // IBUS CH4 <- pin5 (CH4)
      case 4: v = pwmValue[4]; break;       // IBUS CH5 <- pin6 (CH5)
      case 5: v = pwmValue[5]; break;       // IBUS CH6 <- pin7 (CH6)
      default: v = IBUS_DEFAULT; break;
    }
    ibuf[2 + i * 2]     =  v & 0xFF;
    ibuf[2 + i * 2 + 1] = (v >> 8) & 0xFF;
  }

  uint16_t csum = 0xFFFF;
  for (uint8_t i = 0; i < 30; i++) csum -= ibuf[i];
  ibuf[30] = csum & 0xFF;
  ibuf[31] = (csum >> 8) & 0xFF;

  Serial.write(ibuf, IBUS_FRAME_LENGTH);
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    pinMode(PWM_PINS[i], INPUT);
    pwmRaw[i] = IBUS_DEFAULT;
  }
  attachInterrupt(digitalPinToInterrupt(2), isrCH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), isrCH2, CHANGE);
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20) | (1 << PCINT21) | (1 << PCINT22) | (1 << PCINT23);
}

void loop() {
  buildAndSendIBUS();
  delay(7);
}
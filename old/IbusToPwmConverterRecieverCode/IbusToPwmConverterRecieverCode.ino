// UNO - Receive real IBUS (115200) on Pin0/RX, decode and print to Serial Monitor
// NOTE: Pro Mini TX -> UNO Pin 0. Disconnect this wire before re-uploading new code to UNO.

#define IBUS_LENGTH 32
uint8_t buf[IBUS_LENGTH];
uint8_t idx = 0;

bool validateChecksum(uint8_t *frame) {
  uint16_t csum = 0xFFFF;
  for (uint8_t i = 0; i < 30; i++) csum -= frame[i];
  uint16_t received = frame[30] | (frame[31] << 8);
  return csum == received;
}

void setup() {
  Serial.begin(115200); // shared: IBUS input AND debug output
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    if (idx == 0 && b != 0x20) continue;
    if (idx == 1 && b != 0x40) { idx = 0; continue; }

    buf[idx++] = b;

    if (idx >= IBUS_LENGTH) {
      idx = 0;
      if (validateChecksum(buf)) {
        for (uint8_t i = 0; i < 6; i++) {
          uint16_t v = buf[2 + i*2] | (buf[3 + i*2] << 8);
          Serial.print("CH");
          Serial.print(i + 1);
          Serial.print(":");
          Serial.print(v);
          Serial.print(" ");
        }
        Serial.println();
      }
      // silently drop invalid frames - printing here would itself
      // inject bytes into the same buffer timing and risk re-corruption
    }
  }
}
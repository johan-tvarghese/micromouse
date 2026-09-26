// =====================================================================
// TOOL - I2C scanner: what's actually connected?
// =====================================================================
// WIRING
//   Any I2C devices: SDA -> GPIO6, SCL -> GPIO7, VCC -> 3V3, GND -> GND
//   ToF XSHUT pins (if connected): GPIO18, GPIO19, GPIO20
//
// WHAT YOU'LL SEE
//   Serial Monitor with the IMU and ToF sensors all connected:
//       idle SDA=1 SCL=1 (both should be 1)
//
//       Scanning...
//         found 0x29
//         found 0x68
//       2 device(s)
//   (repeats every 3 seconds)
//   0x68 = the IMU (0x69 if its AD0 pin is HIGH)
//   0x29 = the ToF sensors. All three wake up at the same address in this
//          tool, so they show as ONE device. That's expected - steps 3-6
//          move them to separate addresses.
//
//   Problems:
//       idle SDA=0 SCL=0                -> a wire not connected, something
//                                          shorted to GND, or a board with
//                                          no power
//       idle 1/1 but "0 device(s)"      -> try swapping SDA and SCL
//       one device missing              -> check that board's 4 wires
// =====================================================================

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t I2C_SDA = 6;
constexpr uint8_t I2C_SCL = 7;
constexpr uint8_t XSHUT_PINS[] = {18, 19, 20};

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}

  for (uint8_t p : XSHUT_PINS) {
    pinMode(p, OUTPUT);
    digitalWrite(p, HIGH);
  }
  delay(20);

  // Idle bus lines should sit HIGH. LOW means a missing pull-up or a short.
  pinMode(I2C_SDA, INPUT);
  pinMode(I2C_SCL, INPUT);
  Serial.printf("idle SDA=%d SCL=%d (both should be 1)\n",
                digitalRead(I2C_SDA), digitalRead(I2C_SCL));

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // slow and forgiving while debugging wiring
}

void loop() {
  Serial.println("\nScanning...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found 0x%02X\n", addr);
      found++;
    }
  }
  Serial.printf("%u device(s)\n", found);
  delay(3000);
}

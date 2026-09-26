// =====================================================================
// MICROMOUSE - full firmware (ESP32-C6)
// 16x16 maze, 180mm x 180mm cells. Flood-fill search -> return-to-start
// -> speed run. Built up from your individual test sketches (IMU, 3x
// VL53L0X, 2x motor+encoder).
// =====================================================================
//
// ---------------------------------------------------------------------
// FIX LOG (this revision)
// ---------------------------------------------------------------------
//   1. COMPILE ERROR from your screenshot ("Tof was not declared in
//      this scope", cascading into "redeclared as different kind of
//      entity"): the Arduino IDE auto-generates function prototypes and
//      inserts them immediately after your last #include line, BEFORE
//      any of your own struct/enum definitions, no matter where they
//      appear later in the file. Any function signature that used a
//      custom type (Tof&, Coord*, the old Dir enum) broke because of
//      this. Fix: every function signature below now uses only
//      primitive types (uint8_t/int8_t/etc). Custom types are still
//      used freely INSIDE function bodies - that's fine, it's only
//      signatures that trip this bug. This keeps everything in one
//      file rather than splitting into a header to dodge it.
//   2. The min<int8_t>/max<int8_t> template calls flagged earlier are
//      already replaced with plain if-checks (see driveOneCell's
//      position update).
//
// ---------------------------------------------------------------------
// CONFIRMED WIRING (from your test sketches - not re-derived, just reused)
// ---------------------------------------------------------------------
//   I2C bus (IMU + all 3 ToF share this):
//     SDA -> GPIO6   SCL -> GPIO7
//   IMU (MPU-6050): VCC->3V3 GND->GND SDA/SCL as above, AD0->GND (addr 0x68)
//   ToF sensors (VIN->3V3, GND->GND, SDA/SCL shared):
//     LEFT   XSHUT -> GPIO18   -> reassigned to 0x30
//     FRONT  XSHUT -> GPIO19   -> reassigned to 0x31
//     RIGHT  XSHUT -> GPIO20   -> stays at default 0x29 (woken last)
//   Motors:
//     LEFT  (Motor A): DIR -> GPIO0   PWM -> GPIO2
//     RIGHT (Motor B): DIR -> GPIO3   PWM -> GPIO10
//   Encoders:
//     LEFT  (Motor A): C1 -> GPIO21   C2 -> GPIO22        <- confirmed
//     RIGHT (Motor B): C1 -> GPIO15   C2 -> GPIO16        <- *** GUESS ***
//
// !!! RIGHT ENCODER PINS WERE NEVER TESTED IN WHAT YOU SENT ME !!!
// You only gave me a Motor A (left) encoder test. GPIO15/16 are a
// placeholder guess at free pins on the C6 - run something like your
// STEP-6-style encoder test on the RIGHT motor first and fix
// ENC_R_C1_PIN / ENC_R_C2_PIN below before trusting anything that uses
// the right wheel's distance or the drive-straight logic.
//
// ---------------------------------------------------------------------
// CALIBRATION CHECKLIST - search for "TUNE" in this file for all of these
// ---------------------------------------------------------------------
//   1. ENC_R_C1_PIN / ENC_R_C2_PIN   - confirm real right-encoder wiring
//   2. TICKS_PER_MM                  - measure: roll the mouse exactly
//                                      1000mm by hand, print raw encoder
//                                      counts, divide count/1000
//   3. SIDE_WALL_MM / FRONT_WALL_MM  - place the mouse centered in a
//                                      walled cell, read raw L/F/R mm
//                                      over serial, set thresholds from
//                                      what you actually see
//   4. KP_STRAIGHT / KP_TURN         - start small, increase until the
//                                      mouse holds heading / stops
//                                      turning exactly 90 without
//                                      overshoot or oscillation
//   5. SPEED_SEARCH / SPEED_FAST     - both must stay <= SPEED_MAX (170,
//                                      the motor's 6V/9V hardware limit
//                                      from your test notes - do not
//                                      raise SPEED_MAX itself)
//   6. Encoder sign                  - if "distance traveled" comes out
//                                      negative while driving forward,
//                                      flip ENC_L_INVERT / ENC_R_INVERT
//
// This is real closed-loop control logic, not a stub - but every
// physical robot's PID gains and thresholds are different, so treat
// the numbers below as a reasonable starting point, not a guarantee.
// This revision fixes a real compiler error; it has still only been
// checked by hand-tracing the logic, not by building it myself.
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <string.h>

// ============================= PINS ==================================
constexpr uint8_t PIN_SDA = 6;
constexpr uint8_t PIN_SCL = 7;

constexpr uint8_t XSHUT_L = 18, ADDR_L = 0x30;
constexpr uint8_t XSHUT_F = 19, ADDR_F = 0x31;
constexpr uint8_t XSHUT_R = 20, ADDR_R = 0x29;  // last one up, keeps default

constexpr uint8_t DIR_L_PIN = 0,  PWM_L_PIN = 2;   // left  = Motor A
constexpr uint8_t DIR_R_PIN = 3,  PWM_R_PIN = 10;  // right = Motor B

constexpr uint8_t ENC_L_C1_PIN = 21, ENC_L_C2_PIN = 22;  // confirmed
constexpr uint8_t ENC_R_C1_PIN = 15, ENC_R_C2_PIN = 16;  // *** GUESS - CONFIRM ***

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 8
#endif

// ===================== ROBOT / MAZE CONSTANTS ========================
constexpr float CELL_MM      = 180.0f;   // given: 18cm cells
constexpr uint8_t MAZE_SIZE  = 16;       // given: 16x16

// --- TUNE #2: measure this on your robot, see checklist above ---
constexpr float TICKS_PER_MM = 12.0f;    // >>> TUNE <<< placeholder guess

// --- TUNE #6: flip either of these to +1/-1 if distance reads backwards ---
constexpr int8_t ENC_L_INVERT = +1;      // >>> TUNE <<<
constexpr int8_t ENC_R_INVERT = +1;      // >>> TUNE <<<

// --- TUNE #3: measure raw mm readings against a real wall, see checklist ---
constexpr uint16_t SIDE_WALL_MM  = 100;  // >>> TUNE <<< wall-present cutoff, sides
constexpr uint16_t FRONT_WALL_MM = 120;  // >>> TUNE <<< wall-present cutoff, front
constexpr uint16_t MAX_VALID_MM  = 2000; // from your ToF test code - "no wall" cutoff

// PWM / motor hardware limit - DO NOT raise SPEED_MAX (your test notes:
// 170/255 ~= 6V average from a 9V rail, the N20 motors' rating)
constexpr uint32_t PWM_FREQ  = 20000;
constexpr uint8_t  PWM_RES   = 8;
constexpr int16_t  SPEED_MAX = 170;

// --- TUNE #5: keep both <= SPEED_MAX ---
constexpr int16_t SPEED_SEARCH = 90;    // >>> TUNE <<< slow + reliable, exploring
constexpr int16_t SPEED_FAST   = 150;   // >>> TUNE <<< once the maze is known

// --- TUNE #4: closed-loop gains, start conservative and raise slowly ---
constexpr float KP_STRAIGHT = 3.0f;     // >>> TUNE <<< deg-of-drift -> speed correction
constexpr float KP_TURN     = 4.0f;     // >>> TUNE <<< deg-remaining -> turn speed
constexpr int16_t MIN_TURN_SPEED = 60;  // >>> TUNE <<< enough to overcome friction
constexpr float TURN_TOLERANCE_DEG = 2.0f;

// ============================ MAZE DATA ===============================
// wall bit layout per cell: N=1 E=2 S=4 W=8 (1 = wall present / known-closed)
uint8_t walls[MAZE_SIZE][MAZE_SIZE];
uint8_t floodDist[MAZE_SIZE][MAZE_SIZE];

// Plain uint8_t "directions" instead of an enum type - this specifically
// avoids the auto-prototype bug described in the fix log above, since a
// custom enum type in any function signature triggers it just as badly
// as a struct does.
constexpr uint8_t NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3;
constexpr uint8_t WALL_BIT[4] = {1, 2, 4, 8};

int8_t cellX = 0, cellY = 0;
uint8_t heading = NORTH;

enum RunPhase { SEARCH_TO_CENTER, RETURN_TO_START, SPEED_TO_CENTER, DONE };
RunPhase phase = SEARCH_TO_CENTER;  // never passed to a function, so safe as enum

// ============================== IMU ===================================
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t REG_CONFIG      = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;
constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
constexpr uint8_t REG_WHO_AM_I    = 0x75;
constexpr float   GYRO_LSB_PER_DPS = 65.5f;  // +/-500 dps range

float    gyroZBias  = 0.0f;
float    gyroHeadingDeg = 0.0f;  // continuous, NOT wrapped - target math uses this directly
uint32_t lastImuMicros = 0;

void imuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int imuReadReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

float imuReadGyroZdps() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_ZOUT_H);
  // BUGFIX: a shared I2C bus next to PWM motor drivers can occasionally
  // drop or corrupt a transfer. The old code trusted Wire.read() no
  // matter what, so a glitch here used to turn into a huge spurious
  // gyro spike (a dropped byte read as -1 -> 0xFF, folded into a raw
  // int16). Now a failed transfer just reports 0 dps for that one
  // sample (heading briefly stops integrating) instead of injecting
  // garbage that could snap the heading mid-turn.
  if (Wire.endTransmission(false) != 0) return 0.0f;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2) != 2) return 0.0f;
  const uint8_t hi = Wire.read();
  const uint8_t lo = Wire.read();
  const int16_t raw = (int16_t)((hi << 8) | lo);
  return raw / GYRO_LSB_PER_DPS;
}

bool imuBegin() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("FATAL: IMU not found at 0x68");
    return false;
  }
  const int id = imuReadReg(REG_WHO_AM_I);
  Serial.printf("IMU: chip ID 0x%02X\n", id);

  imuWriteReg(REG_PWR_MGMT_1, 0x01);
  delay(100);
  imuWriteReg(REG_GYRO_CONFIG, 0x08);  // +/-500 dps
  imuWriteReg(REG_CONFIG, 0x04);       // ~21 Hz low-pass
  return true;
}

void imuCalibrate() {
  Serial.println("IMU: calibrating - keep the mouse STILL...");
  double sum = 0;
  constexpr uint16_t N = 500;
  for (uint16_t i = 0; i < N; i++) {
    sum += imuReadGyroZdps();
    delay(2);
  }
  gyroZBias = sum / N;
  gyroHeadingDeg = 0.0f;
  lastImuMicros = micros();
  Serial.printf("IMU: bias %.2f dps\n", gyroZBias);
}

// Call every loop iteration - integrates gyro into gyroHeadingDeg.
void imuUpdate() {
  const float gz = imuReadGyroZdps() - gyroZBias;
  const uint32_t now = micros();
  gyroHeadingDeg += gz * (now - lastImuMicros) * 1e-6f;
  lastImuMicros = now;
}

// ============================== TOF ===================================
struct Tof {
  const char* name;
  uint8_t     xshut;
  uint8_t     addr;
  VL53L0X     sensor;
  bool        ok;
  uint16_t    lastMm;
};

Tof tofs[3] = {
  {"L", XSHUT_L, ADDR_L, VL53L0X(), false, 0},
  {"F", XSHUT_F, ADDR_F, VL53L0X(), false, 0},
  {"R", XSHUT_R, ADDR_R, VL53L0X(), false, 0},
};

// Signature is plain uint8_t (an index), not "Tof&" - this is the fix
// for the compile error: no custom type appears in this signature.
bool tofStartOne(uint8_t idx) {
  Tof& t = tofs[idx];  // fine as a LOCAL variable - only signatures trip the bug
  digitalWrite(t.xshut, HIGH);
  delay(10);
  t.sensor.setTimeout(100);
  if (!t.sensor.init()) {
    digitalWrite(t.xshut, LOW);
    return false;
  }
  if (t.addr != 0x29) t.sensor.setAddress(t.addr);
  t.sensor.startContinuous();
  return true;
}

bool tofBeginAll() {
  for (auto& t : tofs) {
    pinMode(t.xshut, OUTPUT);
    digitalWrite(t.xshut, LOW);
  }
  delay(10);

  bool allOk = true;
  for (uint8_t i = 0; i < 3; i++) {  // order matters: L, F, R (R keeps default address)
    tofs[i].ok = tofStartOne(i);
    Serial.printf("ToF %s  XSHUT=GPIO%u  addr=0x%02X  %s\n",
                  tofs[i].name, tofs[i].xshut, tofs[i].addr, tofs[i].ok ? "PASS" : "FAIL");
    if (!tofs[i].ok) allOk = false;
  }
  return allOk;
}

// Blocking (waits for each sensor's reading) - only call while stopped
// at a cell, never mid-drive, or it'll stall the control loop.
void tofReadAll() {
  for (auto& t : tofs) {
    if (!t.ok) { t.lastMm = MAX_VALID_MM; continue; }
    const uint16_t mm = t.sensor.readRangeContinuousMillimeters();
    t.lastMm = (t.sensor.timeoutOccurred() || mm >= MAX_VALID_MM) ? MAX_VALID_MM : mm;
  }
}

// ============================ MOTORS ==================================
void setMotor(uint8_t dirPin, uint8_t pwmPin, int16_t speed) {
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  ledcWrite(pwmPin, abs(speed));
}

void setMotors(int16_t left, int16_t right) {
  setMotor(DIR_L_PIN, PWM_L_PIN, left);
  setMotor(DIR_R_PIN, PWM_R_PIN, right);
}

void stopMotors() { setMotors(0, 0); }

// ============================ ENCODERS ================================
volatile int32_t encL = 0, encR = 0;

void IRAM_ATTR encoderLISR() {
  encL += digitalRead(ENC_L_C2_PIN) ? 1 : -1;
}
void IRAM_ATTR encoderRISR() {
  encR += digitalRead(ENC_R_C2_PIN) ? 1 : -1;
}

// Signed distance since the counters were last zeroed, in mm.
float leftDistMm()  { return (encL * ENC_L_INVERT) / TICKS_PER_MM; }
float rightDistMm() { return (encR * ENC_R_INVERT) / TICKS_PER_MM; }

void zeroEncoders() {
  noInterrupts();
  encL = 0;
  encR = 0;
  interrupts();
}

// ======================= LOW-LEVEL MOTION =============================

// Turn in place by a multiple of 90 degrees (+1 = right/clockwise,
// -1 = left/counter-clockwise, +2/-2 = 180). Closed-loop on gyro heading.
void turnInPlace(int8_t quarterTurns) {
  const float target = gyroHeadingDeg + 90.0f * quarterTurns;
  uint8_t settledCount = 0;

  while (settledCount < 5) {
    imuUpdate();
    const float remaining = target - gyroHeadingDeg;

    if (fabsf(remaining) <= TURN_TOLERANCE_DEG) {
      settledCount++;
      stopMotors();
    } else {
      settledCount = 0;
      int16_t s = (int16_t)(KP_TURN * remaining);
      if (s > 0) s = max(s, MIN_TURN_SPEED);
      if (s < 0) s = min(s, (int16_t)-MIN_TURN_SPEED);
      s = constrain(s, (int16_t)-SPEED_MAX, (int16_t)SPEED_MAX);
      setMotors(s, -s);  // right turn (remaining>0): left+, right-
    }
    delay(2);
  }
  stopMotors();

  // Snap to the nearest 90 to stop tiny errors from accumulating turn
  // after turn over a long run.
  gyroHeadingDeg = roundf(gyroHeadingDeg / 90.0f) * 90.0f;
  heading = (uint8_t)(((int)heading + (quarterTurns % 4) + 4) % 4);
}

// Drive forward exactly one cell (CELL_MM), holding heading with the
// gyro and matching wheel speeds with the encoders. Trapezoidal-ish
// speed profile: ramp up, cruise, ramp down.
void driveOneCell(int16_t speedCap) {
  zeroEncoders();
  const float targetHeading = gyroHeadingDeg;
  const float targetMm = CELL_MM;
  constexpr float RAMP_MM = 40.0f;  // >>> TUNE <<< distance to ramp up/down over
  constexpr int16_t MIN_DRIVE_SPEED = 50;  // >>> TUNE <<<

  while (true) {
    imuUpdate();
    const float traveled = (leftDistMm() + rightDistMm()) * 0.5f;
    const float remaining = targetMm - traveled;
    if (remaining <= 0) break;

    // Trapezoidal speed: ramp up from start, ramp down approaching target.
    float rampFactor = 1.0f;
    if (traveled < RAMP_MM) rampFactor = traveled / RAMP_MM;
    else if (remaining < RAMP_MM) rampFactor = remaining / RAMP_MM;
    rampFactor = constrain(rampFactor, 0.15f, 1.0f);
    const int16_t base = constrain((int16_t)(speedCap * rampFactor), MIN_DRIVE_SPEED, speedCap);

    const float error = targetHeading - gyroHeadingDeg;
    const int16_t steer = (int16_t)(KP_STRAIGHT * error);

    setMotors(base + steer, base - steer);
    delay(2);
  }
  stopMotors();

  // Advance our believed grid position in the direction we were facing.
  switch (heading) {
    case NORTH: if (cellY < MAZE_SIZE - 1) cellY++; break;
    case EAST:  if (cellX < MAZE_SIZE - 1) cellX++; break;
    case SOUTH: if (cellY > 0) cellY--;             break;
    case WEST:  if (cellX > 0) cellX--;             break;
  }
}

// ========================= WALL SENSING ===============================
void markWall(int8_t x, int8_t y, uint8_t d) {
  if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) return;
  walls[x][y] |= WALL_BIT[d];
}

// Opposite direction, so the wall gets marked on both cells that share it.
uint8_t opposite(uint8_t d) { return (d + 2) % 4; }

void neighborCoord(int8_t x, int8_t y, uint8_t d, int8_t& nx, int8_t& ny) {
  nx = x; ny = y;
  switch (d) {
    case NORTH: ny++; break;
    case EAST:  nx++; break;
    case SOUTH: ny--; break;
    case WEST:  nx--; break;
  }
}

// Reads L/F/R, converts to absolute compass directions based on current
// heading, and updates the wall map for the current cell (and its
// neighbor across that wall).
void senseWallsAtCurrentCell() {
  tofReadAll();
  const bool wallFront = tofs[1].lastMm < FRONT_WALL_MM;
  const bool wallLeft  = tofs[0].lastMm < SIDE_WALL_MM;
  const bool wallRight = tofs[2].lastMm < SIDE_WALL_MM;

  const uint8_t frontDir = heading;
  const uint8_t leftDir  = (heading + 3) % 4;
  const uint8_t rightDir = (heading + 1) % 4;

  auto applyWall = [](int8_t x, int8_t y, uint8_t d, bool present) {
    if (!present) return;
    markWall(x, y, d);
    int8_t nx, ny;
    neighborCoord(x, y, d, nx, ny);
    markWall(nx, ny, opposite(d));
  };

  applyWall(cellX, cellY, frontDir, wallFront);
  applyWall(cellX, cellY, leftDir,  wallLeft);
  applyWall(cellX, cellY, rightDir, wallRight);

  Serial.printf("cell(%d,%d) heading=%u  L:%u%s F:%u%s R:%u%s\n",
                cellX, cellY, heading,
                tofs[0].lastMm, wallLeft  ? "(wall)" : "",
                tofs[1].lastMm, wallFront ? "(wall)" : "",
                tofs[2].lastMm, wallRight ? "(wall)" : "");
}

// ============================ FLOOD FILL ==============================
// BFS outward from the goal cells. Unknown (unmarked) walls are treated
// as open - this is the classic flood-fill assumption that makes the
// mouse explore instead of refusing to move on unexplored walls.
// Goal lists are passed as parallel primitive arrays (not a Coord*)
// for the same reason described in the fix log at the top of this file.
struct Coord { int8_t x, y; };  // only ever used as a LOCAL variable type below

void computeFlood(const uint8_t* goalX, const uint8_t* goalY, uint8_t numGoals) {
  for (uint8_t x = 0; x < MAZE_SIZE; x++)
    for (uint8_t y = 0; y < MAZE_SIZE; y++)
      floodDist[x][y] = 255;

  static Coord queue[MAZE_SIZE * MAZE_SIZE];
  uint16_t head = 0, tail = 0;

  for (uint8_t i = 0; i < numGoals; i++) {
    floodDist[goalX[i]][goalY[i]] = 0;
    queue[tail++] = {(int8_t)goalX[i], (int8_t)goalY[i]};
  }

  while (head < tail) {
    const Coord c = queue[head++];
    const uint8_t d = floodDist[c.x][c.y];
    for (uint8_t dir = 0; dir < 4; dir++) {
      if (walls[c.x][c.y] & WALL_BIT[dir]) continue;  // known wall - can't pass
      int8_t nx, ny;
      neighborCoord(c.x, c.y, dir, nx, ny);
      if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) continue;
      if (floodDist[nx][ny] <= d + 1) continue;
      floodDist[nx][ny] = d + 1;
      queue[tail++] = {nx, ny};
    }
  }
}

// ======================== NAVIGATION STEP =============================
// Senses, recomputes flood fill toward the given goals, then moves one
// cell toward the lowest-flood open neighbor. Returns true once the
// mouse is standing in one of the goal cells.
bool stepToward(const uint8_t* goalX, const uint8_t* goalY, uint8_t numGoals, int16_t speed) {
  senseWallsAtCurrentCell();
  computeFlood(goalX, goalY, numGoals);

  for (uint8_t i = 0; i < numGoals; i++) {
    if (goalX[i] == cellX && goalY[i] == cellY) return true;
  }

  const uint8_t here = floodDist[cellX][cellY];
  uint8_t best = heading;
  uint8_t bestDist = 255;

  // Prefer continuing straight on ties, to cut down on unnecessary turns.
  const uint8_t order[4] = {heading, (uint8_t)((heading + 1) % 4),
                            (uint8_t)((heading + 3) % 4), opposite(heading)};
  for (uint8_t d : order) {
    if (walls[cellX][cellY] & WALL_BIT[d]) continue;
    int8_t nx, ny;
    neighborCoord(cellX, cellY, d, nx, ny);
    if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) continue;
    if (floodDist[nx][ny] < bestDist) {
      bestDist = floodDist[nx][ny];
      best = d;
    }
  }

  if (bestDist >= here) {
    // This is NOT a normal dead end - BFS guarantees a real dead end's
    // single entrance is always found as strictly better by exactly 1
    // (floodDist[here] = 1 + min(open neighbors) is a hard BFS
    // property), so the mouse can always back out of a genuine pocket.
    // Reaching this branch means every side reads as a known wall right
    // now, which is realistically a sensor misread (bad SIDE_WALL_MM /
    // FRONT_WALL_MM threshold), not a maze layout the mouse can't solve.
    // Blink clearly so this is visible on the maze table without a
    // laptop tethered, then retry - if it was a transient glitch the
    // next sense will clear it.
    Serial.println("STUCK: every side reads as a wall - check wall thresholds");
    stopMotors();
    rgbLedWrite(RGB_BUILTIN, 32, 0, 0);
    delay(150);
    rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
    delay(150);
    return false;
  }

  const int8_t delta = ((int8_t)best - (int8_t)heading + 4) % 4;
  if (delta == 1) turnInPlace(+1);
  else if (delta == 3) turnInPlace(-1);
  else if (delta == 2) turnInPlace(+2);
  // delta == 0: already facing the right way

  driveOneCell(speed);
  return false;
}

// ============================ SETUP / LOOP ============================
const uint8_t CENTER_GOALS_X[4] = {7, 7, 8, 8};
const uint8_t CENTER_GOALS_Y[4] = {7, 8, 7, 8};
const uint8_t START_GOAL_X[1]   = {0};
const uint8_t START_GOAL_Y[1]   = {0};

void initMazeBorders() {
  for (uint8_t x = 0; x < MAZE_SIZE; x++) {
    markWall(x, 0, SOUTH);
    markWall(x, MAZE_SIZE - 1, NORTH);
  }
  for (uint8_t y = 0; y < MAZE_SIZE; y++) {
    markWall(0, y, WEST);
    markWall(MAZE_SIZE - 1, y, EAST);
  }
}

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  Serial.println("\nMICROMOUSE starting");

  memset(walls, 0, sizeof(walls));
  initMazeBorders();

  pinMode(DIR_L_PIN, OUTPUT);
  pinMode(DIR_R_PIN, OUTPUT);
  digitalWrite(DIR_L_PIN, LOW);
  digitalWrite(DIR_R_PIN, LOW);
  ledcAttach(PWM_L_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(PWM_R_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_L_PIN, 0);
  ledcWrite(PWM_R_PIN, 0);

  pinMode(ENC_L_C1_PIN, INPUT_PULLUP);
  pinMode(ENC_L_C2_PIN, INPUT_PULLUP);
  pinMode(ENC_R_C1_PIN, INPUT_PULLUP);
  pinMode(ENC_R_C2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C1_PIN), encoderLISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1_PIN), encoderRISR, RISING);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  if (!imuBegin()) { rgbLedWrite(RGB_BUILTIN, 32, 0, 0); while (true) delay(1000); }
  imuCalibrate();

  if (!tofBeginAll()) {
    Serial.println("WARNING: not all ToF sensors came up - fix wiring before a real run.");
  }

  Serial.println("Ready. Hands clear...");
  rgbLedWrite(RGB_BUILTIN, 32, 32, 0);
  delay(2000);
  rgbLedWrite(RGB_BUILTIN, 0, 32, 0);
}

void loop() {
  switch (phase) {
    case SEARCH_TO_CENTER:
      if (stepToward(CENTER_GOALS_X, CENTER_GOALS_Y, 4, SPEED_SEARCH)) {
        Serial.println("Reached center. Returning to start...");
        phase = RETURN_TO_START;
        rgbLedWrite(RGB_BUILTIN, 0, 0, 32);
      }
      break;

    case RETURN_TO_START:
      if (stepToward(START_GOAL_X, START_GOAL_Y, 1, SPEED_SEARCH)) {
        Serial.println("Back at start. Speed run to center...");
        phase = SPEED_TO_CENTER;
        rgbLedWrite(RGB_BUILTIN, 32, 0, 32);
      }
      break;

    case SPEED_TO_CENTER:
      if (stepToward(CENTER_GOALS_X, CENTER_GOALS_Y, 4, SPEED_FAST)) {
        Serial.println("DONE - speed run complete.");
        phase = DONE;
        rgbLedWrite(RGB_BUILTIN, 0, 32, 0);
      }
      break;

    case DONE:
      stopMotors();
      delay(500);
      break;
  }
}

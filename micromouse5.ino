// ============================================================================
// MICROMOUSE v5 - ESP32-C6
// 16x16 maze, 180 mm cells
//
// THREE-RUN COMPETITION STRUCTURE
// --------------------------------
// ROUND 1 = EXPLORE
//   Start at (0,0), discover the maze while travelling to the 4-cell centre.
//   The discovered wall map is saved to ESP32 flash when the centre is reached.
//
// ROUND 2 = SLOW REPLAY
//   Start at (0,0), load the saved maze, calculate a route to the centre,
//   and drive it slowly without exploration logic.
//
// ROUND 3 = SPEED RUN
//   Start at (0,0), load the saved maze, calculate the same route, and drive
//   it with the faster motion controller.
//
// IMPORTANT
// ---------
// Set ACTIVE_ROUND below before each competition run:
//     1 = map the maze
//     2 = slow replay
//     3 = final speed run
//
// The maze is saved using ESP32 Preferences/NVS, so it survives a reset.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Preferences.h>
#include <string.h>
#include <math.h>

// ========================= USER CONFIGURATION ===============================

// >>>>>>>>>>>>>>>>>>>>>>>>>>> CHANGE THESE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
constexpr uint8_t ACTIVE_ROUND = 1;   // 1=EXPLORE, 2=SLOW REPLAY, 3=SPEED RUN

// Confirm these with your real hardware before driving.
constexpr uint8_t ENC_L_C1_PIN = 11;
constexpr uint8_t ENC_L_C2_PIN = 23;
constexpr uint8_t ENC_R_C1_PIN = 21;   // *** previously marked GUESS ***
constexpr uint8_t ENC_R_C2_PIN = 22;   // *** previously marked GUESS ***

// Motor direction/PWM pins from micromouse3.
constexpr uint8_t DIR_L_PIN = 0;
constexpr uint8_t PWM_L_PIN = 2;
constexpr uint8_t DIR_R_PIN = 3;
constexpr uint8_t PWM_R_PIN = 10;

// I2C pins.
constexpr uint8_t PIN_SDA = 6;
constexpr uint8_t PIN_SCL = 7;

// ToF XSHUT pins and addresses.
constexpr uint8_t XSHUT_L = 18, ADDR_L = 0x30;
constexpr uint8_t XSHUT_F = 19, ADDR_F = 0x31;
constexpr uint8_t XSHUT_R = 20, ADDR_R = 0x29;

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 8
#endif


// ============================= MAZE =========================================
constexpr uint8_t MAZE_SIZE = 16;
constexpr float CELL_MM = 180.0f;

constexpr uint8_t START_X = 0;
constexpr uint8_t START_Y = 0;

// Centre of a standard 16x16 maze.
constexpr uint8_t CENTER_GOAL_X[4] = {7, 7, 8, 8};
constexpr uint8_t CENTER_GOAL_Y[4] = {7, 8, 7, 8};

constexpr uint8_t NORTH = 0;
constexpr uint8_t EAST  = 1;
constexpr uint8_t SOUTH = 2;
constexpr uint8_t WEST  = 3;
constexpr uint8_t WALL_BIT[4] = {1, 2, 4, 8};

// =========================== ENCODERS =======================================
// Measure each wheel separately. Do NOT assume left and right are identical.
// Example: if 1000 mm gives 12000 counts, use 12.0 ticks/mm.
constexpr float LEFT_TICKS_PER_MM  = 12.0f;   // >>> TUNE <<<
constexpr float RIGHT_TICKS_PER_MM = 12.0f;   // >>> TUNE <<<

constexpr int8_t ENC_L_INVERT = +1;           // >>> TUNE <<<
constexpr int8_t ENC_R_INVERT = +1;           // >>> TUNE <<<

// ========================== MOTOR LIMITS ====================================
constexpr uint32_t PWM_FREQ = 20000;
constexpr uint8_t PWM_RES = 8;
constexpr int16_t PWM_MAX = 170;              // keep within your motor limit

// These are PWM commands, not mm/s.
// Start conservative and increase only after the controller is tuned.
constexpr int16_t PWM_SEARCH = 90;
constexpr int16_t PWM_SLOW   = 105;
constexpr int16_t PWM_FAST   = 150;

// Minimum PWM that actually moves your robot.
constexpr int16_t PWM_MIN_DRIVE = 45;
constexpr int16_t PWM_MIN_TURN  = 55;

// ====================== MOTION / PID TUNING ================================
// Control loop target. The ESP32 may execute slightly slower depending on I2C.
constexpr uint32_t CONTROL_PERIOD_US = 2000;  // 500 Hz

// Wheel velocity PID. These are STARTING VALUES and MUST be tuned.
constexpr float LEFT_KP  = 0.30f;
constexpr float LEFT_KI  = 0.80f;
constexpr float LEFT_KD  = 0.00f;
constexpr float RIGHT_KP = 0.30f;
constexpr float RIGHT_KI = 0.80f;
constexpr float RIGHT_KD = 0.00f;

// Feed-forward: PWM ~= FF * requested encoder velocity + PID correction.
// Leave low until velocity calibration is done.
constexpr float LEFT_FF  = 0.00f;
constexpr float RIGHT_FF = 0.00f;

// Straight heading PD.
constexpr float HEADING_KP = 2.0f;             // PWM correction per degree
constexpr float HEADING_KD = 0.08f;            // PWM correction per deg/s

// Wall centering. Disabled unless both side walls are currently detected.
constexpr float WALL_KP = 1.0f;
constexpr float WALL_KD = 0.05f;
constexpr float WALL_TARGET_MM = 45.0f;         // >>> TUNE <<<

// Turn controller.
constexpr float TURN_KP = 3.0f;
constexpr float TURN_KD = 0.10f;
constexpr float TURN_TOLERANCE_DEG = 1.5f;
constexpr float TURN_SETTLE_RATE_DPS = 12.0f;
constexpr int16_t TURN_MAX_PWM = 150;

// Motion distances. The final few mm are deliberately slow.
constexpr float ACCEL_DISTANCE_MM = 50.0f;
constexpr float DECEL_DISTANCE_MM = 60.0f;
constexpr float CELL_END_SLOW_MM = 25.0f;

// ============================ TOF ===========================================
constexpr uint16_t SIDE_WALL_MM = 100;          // >>> TUNE <<<
constexpr uint16_t FRONT_WALL_MM = 120;         // >>> TUNE <<<
constexpr uint16_t MAX_VALID_MM = 2000;

// ============================= IMU ==========================================
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr float GYRO_LSB_PER_DPS = 65.5f;       // +/-500 dps

// ============================ PERSISTENCE ===================================
Preferences prefs;
constexpr char NVS_NAMESPACE[] = "micromouse";
constexpr char NVS_KEY[] = "maze_v5";
constexpr uint16_t MAX_ROUTE = MAZE_SIZE * MAZE_SIZE;
constexpr uint32_t MAP_MAGIC = 0x4D4D5635UL;     // "MMV5"

struct SavedMap {
  uint32_t magic;
  uint8_t walls[MAZE_SIZE][MAZE_SIZE];
  uint16_t routeLength;
  uint8_t route[MAX_ROUTE];
};

// ============================ GLOBAL STATE ==================================
uint8_t walls[MAZE_SIZE][MAZE_SIZE];
uint8_t floodDist[MAZE_SIZE][MAZE_SIZE];

int8_t cellX = START_X;
int8_t cellY = START_Y;
uint8_t heading = NORTH;

float gyroZBias = 0.0f;
float gyroHeadingDeg = 0.0f;
float gyroRateDps = 0.0f;
uint32_t lastImuMicros = 0;

volatile int32_t encL = 0;
volatile int32_t encR = 0;

float leftVelocityMmS = 0.0f;
float rightVelocityMmS = 0.0f;
int32_t previousVelocityEncL = 0;
int32_t previousVelocityEncR = 0;
float leftTargetMmS = 0.0f;
float rightTargetMmS = 0.0f;

uint32_t lastControlMicros = 0;
int16_t commandedLeftPwm = 0;
int16_t commandedRightPwm = 0;

// ============================= STRUCTS ======================================
struct Tof {
  const char* name;
  uint8_t xshut;
  uint8_t addr;
  VL53L0X sensor;
  bool ok;
  uint16_t lastMm;
};

Tof tofs[3] = {
  {"L", XSHUT_L, ADDR_L, VL53L0X(), false, MAX_VALID_MM},
  {"F", XSHUT_F, ADDR_F, VL53L0X(), false, MAX_VALID_MM},
  {"R", XSHUT_R, ADDR_R, VL53L0X(), false, MAX_VALID_MM}
};

struct PID {
  float kp;
  float ki;
  float kd;
  float integral = 0.0f;
  float previousError = 0.0f;
  float integralLimit = 100.0f;

  void reset() {
    integral = 0.0f;
    previousError = 0.0f;
  }

  float update(float target, float actual, float dt, float outputMin, float outputMax) {
    if (dt <= 0.0f || dt > 0.1f) dt = 0.002f;

    const float error = target - actual;
    integral += error * dt;
    integral = constrain(integral, -integralLimit, integralLimit);

    const float derivative = (error - previousError) / dt;
    previousError = error;

    float output = kp * error + ki * integral + kd * derivative;
    output = constrain(output, outputMin, outputMax);
    return output;
  }
};

PID leftPID  {LEFT_KP, LEFT_KI, LEFT_KD, 0, 0, 150};
PID rightPID {RIGHT_KP, RIGHT_KI, RIGHT_KD, 0, 0, 150};

// A route is stored as compass directions, one direction per cell travelled.
uint8_t route[MAX_ROUTE];
uint16_t routeLength = 0;
uint16_t routeIndex = 0;

// =========================== DEBUG CONFIG ===================================
// Serial Monitor: 115200 baud. Set DEBUG_ENABLED=false for competition.
// DEBUG_LEVEL: 1=major events, 2=sensors/navigation, 3=motion diagnostics.
constexpr bool DEBUG_ENABLED = true;
constexpr uint8_t DEBUG_LEVEL = 2;
constexpr uint32_t DEBUG_BAUD = 115200;

#define DBG1(x) do { if (DEBUG_ENABLED && DEBUG_LEVEL >= 1) Serial.println(x); } while (0)
#define DBG2(x) do { if (DEBUG_ENABLED && DEBUG_LEVEL >= 2) Serial.println(x); } while (0)
#define DBG3(x) do { if (DEBUG_ENABLED && DEBUG_LEVEL >= 3) Serial.println(x); } while (0)
#define DBGF(level, fmt, ...) do { if (DEBUG_ENABLED && DEBUG_LEVEL >= (level)) Serial.printf((fmt), ##__VA_ARGS__); } while (0)

uint32_t debugMoveCount = 0;
uint32_t debugTurnCount = 0;
uint32_t debugLastSensorPrint = 0;

static const char* dirName(uint8_t d) {
  switch (d) {
    case NORTH: return "NORTH";
    case EAST:  return "EAST";
    case SOUTH: return "SOUTH";
    case WEST:  return "WEST";
    default:    return "?";
  }
}

static void debugSensors() {
  if (!DEBUG_ENABLED || DEBUG_LEVEL < 2) return;
  DBGF(2, "[SENS] L=%u mm  F=%u mm  R=%u mm | wall=%d/%d/%d | gyro=%.2f dps  yaw=%.2f deg\n",
       (unsigned)tofs[0].lastMm, (unsigned)tofs[1].lastMm, (unsigned)tofs[2].lastMm,
       tofs[0].lastMm < SIDE_WALL_MM, tofs[1].lastMm < FRONT_WALL_MM,
       tofs[2].lastMm < SIDE_WALL_MM, gyroRateDps, gyroHeadingDeg);
}

static void debugCell(const char* tag) {
  if (!DEBUG_ENABLED || DEBUG_LEVEL < 2) return;
  const uint8_t w = walls[cellX][cellY];
  DBGF(2, "[%s] cell=(%d,%d) heading=%s walls=%c%c%c%c route=%u\n", tag,
       cellX, cellY, dirName(heading),
       (w & WALL_BIT[NORTH]) ? 'N' : '-', (w & WALL_BIT[EAST]) ? 'E' : '-',
       (w & WALL_BIT[SOUTH]) ? 'S' : '-', (w & WALL_BIT[WEST]) ? 'W' : '-',
       routeLength);
}

// ============================== UTILITIES ===================================
uint8_t opposite(uint8_t d) { return (d + 2) % 4; }

void neighborCoord(int8_t x, int8_t y, uint8_t d, int8_t& nx, int8_t& ny) {
  nx = x;
  ny = y;
  switch (d) {
    case NORTH: ny++; break;
    case EAST:  nx++; break;
    case SOUTH: ny--; break;
    case WEST:  nx--; break;
  }
}

bool isCenterCell(int8_t x, int8_t y) {
  for (uint8_t i = 0; i < 4; i++) {
    if (CENTER_GOAL_X[i] == x && CENTER_GOAL_Y[i] == y) return true;
  }
  return false;
}

// ================================ IMU =======================================
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
    Serial.println("FATAL: MPU6050 not found");
    return false;
  }

  Serial.printf("MPU6050 WHO_AM_I = 0x%02X\n", imuReadReg(REG_WHO_AM_I));
  imuWriteReg(REG_PWR_MGMT_1, 0x01);
  delay(100);
  imuWriteReg(REG_GYRO_CONFIG, 0x08); // +/-500 dps
  imuWriteReg(REG_CONFIG, 0x04);      // digital low-pass
  return true;
}

void imuCalibrate() {
  Serial.println("IMU calibration: KEEP ROBOT COMPLETELY STILL");
  double sum = 0.0;
  constexpr uint16_t samples = 1000;

  for (uint16_t i = 0; i < samples; i++) {
    sum += imuReadGyroZdps();
    delay(2);
  }

  gyroZBias = sum / samples;
  gyroHeadingDeg = 0.0f;
  gyroRateDps = 0.0f;
  lastImuMicros = micros();
  Serial.printf("Gyro bias = %.4f dps\n", gyroZBias);
}

void imuUpdate() {
  const uint32_t now = micros();
  const uint32_t elapsed = now - lastImuMicros;
  lastImuMicros = now;

  float gz = imuReadGyroZdps() - gyroZBias;

  // Reject a clearly impossible one-sample spike rather than integrating it.
  if (fabsf(gz) > 500.0f) gz = 0.0f;

  gyroRateDps = gz;
  gyroHeadingDeg += gz * elapsed * 1e-6f;
}

void zeroHeadingReference() {
  gyroHeadingDeg = 0.0f;
  lastImuMicros = micros();
}

// ================================ TOF =======================================
bool tofStartOne(uint8_t idx) {
  Tof& t = tofs[idx];
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
  bool allOk = true;

  for (auto& t : tofs) {
    pinMode(t.xshut, OUTPUT);
    digitalWrite(t.xshut, LOW);
  }
  delay(20);

  for (uint8_t i = 0; i < 3; i++) {
    tofs[i].ok = tofStartOne(i);
    Serial.printf("ToF %s address=0x%02X -> %s\n",
                  tofs[i].name, tofs[i].addr,
                  tofs[i].ok ? "OK" : "FAIL");
    if (!tofs[i].ok) allOk = false;
  }

  return allOk;
}

void tofReadAll() {
  for (auto& t : tofs) {
    if (!t.ok) {
      t.lastMm = MAX_VALID_MM;
      continue;
    }

    uint16_t mm = t.sensor.readRangeContinuousMillimeters();
    if (t.sensor.timeoutOccurred() || mm >= MAX_VALID_MM) mm = MAX_VALID_MM;
    t.lastMm = mm;
  }
}

// ================================ MOTORS ====================================
void setMotor(uint8_t dirPin, uint8_t pwmPin, int16_t pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  digitalWrite(dirPin, pwm >= 0 ? HIGH : LOW);
  ledcWrite(pwmPin, abs(pwm));
}

void setMotors(int16_t left, int16_t right) {
  commandedLeftPwm = constrain(left, -PWM_MAX, PWM_MAX);
  commandedRightPwm = constrain(right, -PWM_MAX, PWM_MAX);
  setMotor(DIR_L_PIN, PWM_L_PIN, commandedLeftPwm);
  setMotor(DIR_R_PIN, PWM_R_PIN, commandedRightPwm);
}

void stopMotors() {
  commandedLeftPwm = 0;
  commandedRightPwm = 0;
  ledcWrite(PWM_L_PIN, 0);
  ledcWrite(PWM_R_PIN, 0);
}

// ============================== ENCODERS ====================================
void IRAM_ATTR encoderLISR() {
  encL += digitalRead(ENC_L_C2_PIN) ? 1 : -1;
}

void IRAM_ATTR encoderRISR() {
  encR += digitalRead(ENC_R_C2_PIN) ? 1 : -1;
}

void zeroEncoders() {
  noInterrupts();
  encL = 0;
  encR = 0;
  interrupts();
}

void readEncoderCounts(int32_t& left, int32_t& right) {
  noInterrupts();
  left = encL;
  right = encR;
  interrupts();
}

float leftDistMm() {
  int32_t l, r;
  readEncoderCounts(l, r);
  return (l * ENC_L_INVERT) / LEFT_TICKS_PER_MM;
}

float rightDistMm() {
  int32_t l, r;
  readEncoderCounts(l, r);
  return (r * ENC_R_INVERT) / RIGHT_TICKS_PER_MM;
}

// ========================= WHEEL VELOCITY ==================================
// Encoder-based velocity estimate. The PID uses this to make each wheel
// actually track its requested speed instead of merely applying a PWM value.
void updateWheelVelocity(float dt) {
  int32_t nowL, nowR;
  readEncoderCounts(nowL, nowR);

  const float rawLeft = ((nowL - previousVelocityEncL) * ENC_L_INVERT) / LEFT_TICKS_PER_MM / dt;
  const float rawRight = ((nowR - previousVelocityEncR) * ENC_R_INVERT) / RIGHT_TICKS_PER_MM / dt;

  previousVelocityEncL = nowL;
  previousVelocityEncR = nowR;

  // Simple low-pass filter. This makes the encoder velocity less noisy.
  constexpr float ALPHA = 0.35f;
  leftVelocityMmS = ALPHA * rawLeft + (1.0f - ALPHA) * leftVelocityMmS;
  rightVelocityMmS = ALPHA * rawRight + (1.0f - ALPHA) * rightVelocityMmS;
}

void resetVelocityController() {
  leftPID.reset();
  rightPID.reset();
  leftVelocityMmS = 0;
  rightVelocityMmS = 0;
  leftTargetMmS = 0;
  rightTargetMmS = 0;

  int32_t l, r;
  readEncoderCounts(l, r);
  previousVelocityEncL = l;
  previousVelocityEncR = r;
}

// Convert a requested mm/s to a feed-forward PWM term. These constants are
// deliberately zero until the robot's PWM-vs-speed relationship is measured.
float feedForwardLeft(float velocity) {
  return LEFT_FF * velocity;
}

float feedForwardRight(float velocity) {
  return RIGHT_FF * velocity;
}

void motorVelocityControl(float dt) {
  updateWheelVelocity(dt);

  if (fabsf(leftTargetMmS) < 0.01f && fabsf(rightTargetMmS) < 0.01f) {
    stopMotors();
    leftPID.reset();
    rightPID.reset();
    return;
  }

  float leftOut = feedForwardLeft(leftTargetMmS) +
                  leftPID.update(leftTargetMmS, leftVelocityMmS, dt, -PWM_MAX, PWM_MAX);
  float rightOut = feedForwardRight(rightTargetMmS) +
                   rightPID.update(rightTargetMmS, rightVelocityMmS, dt, -PWM_MAX, PWM_MAX);

  // Prevent the PID from commanding a tiny value that cannot move the motor.
  if (fabsf(leftOut) > 0.1f && fabsf(leftOut) < PWM_MIN_DRIVE)
    leftOut = copysignf(PWM_MIN_DRIVE, leftOut);
  if (fabsf(rightOut) > 0.1f && fabsf(rightOut) < PWM_MIN_DRIVE)
    rightOut = copysignf(PWM_MIN_DRIVE, rightOut);

  setMotors((int16_t)leftOut, (int16_t)rightOut);
}

// ============================ MAZE MAP =====================================
void clearMaze() {
  memset(walls, 0, sizeof(walls));

  // Outer boundary is known from the beginning.
  for (uint8_t x = 0; x < MAZE_SIZE; x++) {
    walls[x][0] |= WALL_BIT[SOUTH];
    walls[x][MAZE_SIZE - 1] |= WALL_BIT[NORTH];
  }
  for (uint8_t y = 0; y < MAZE_SIZE; y++) {
    walls[0][y] |= WALL_BIT[WEST];
    walls[MAZE_SIZE - 1][y] |= WALL_BIT[EAST];
  }
}

void markWall(int8_t x, int8_t y, uint8_t d) {
  if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) return;
  walls[x][y] |= WALL_BIT[d];

  int8_t nx, ny;
  neighborCoord(x, y, d, nx, ny);
  if (nx >= 0 && ny >= 0 && nx < MAZE_SIZE && ny < MAZE_SIZE) {
    walls[nx][ny] |= WALL_BIT[opposite(d)];
  }
}

void senseWallsAtCurrentCell() {
  tofReadAll();
  debugSensors();

  const bool frontWall = tofs[1].lastMm < FRONT_WALL_MM;
  const bool leftWall  = tofs[0].lastMm < SIDE_WALL_MM;
  const bool rightWall = tofs[2].lastMm < SIDE_WALL_MM;

  const uint8_t frontDir = heading;
  const uint8_t leftDir = (heading + 3) % 4;
  const uint8_t rightDir = (heading + 1) % 4;

  if (frontWall) markWall(cellX, cellY, frontDir);
  if (leftWall)  markWall(cellX, cellY, leftDir);
  if (rightWall) markWall(cellX, cellY, rightDir);

  Serial.printf("CELL (%d,%d) H=%u | L=%u %s | F=%u %s | R=%u %s\n",
                cellX, cellY, heading,
                tofs[0].lastMm, leftWall ? "WALL" : "OPEN",
                tofs[1].lastMm, frontWall ? "WALL" : "OPEN",
                tofs[2].lastMm, rightWall ? "WALL" : "OPEN");
  debugCell("MAP");
}

// ============================ FLOOD FILL ====================================
struct Coord {
  int8_t x;
  int8_t y;
};

void computeFloodToCenter() {
  for (uint8_t x = 0; x < MAZE_SIZE; x++)
    for (uint8_t y = 0; y < MAZE_SIZE; y++)
      floodDist[x][y] = 255;

  static Coord queue[MAZE_SIZE * MAZE_SIZE];
  uint16_t head = 0;
  uint16_t tail = 0;

  for (uint8_t i = 0; i < 4; i++) {
    const uint8_t x = CENTER_GOAL_X[i];
    const uint8_t y = CENTER_GOAL_Y[i];
    floodDist[x][y] = 0;
    queue[tail++] = {(int8_t)x, (int8_t)y};
  }

  while (head < tail) {
    const Coord c = queue[head++];
    const uint8_t d = floodDist[c.x][c.y];

    for (uint8_t dir = 0; dir < 4; dir++) {
      if (walls[c.x][c.y] & WALL_BIT[dir]) continue;

      int8_t nx, ny;
      neighborCoord(c.x, c.y, dir, nx, ny);
      if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) continue;

      if (floodDist[nx][ny] <= d + 1) continue;
      floodDist[nx][ny] = d + 1;
      queue[tail++] = {nx, ny};
    }
  }
}

// Choose the best direction during exploration. Unknown walls are treated as
// open by flood fill, which is what allows exploration of the unknown maze.
uint8_t chooseExplorationDirection() {
  const uint8_t order[4] = {
    heading,
    (uint8_t)((heading + 1) % 4),
    (uint8_t)((heading + 3) % 4),
    opposite(heading)
  };

  uint8_t bestDir = heading;
  uint8_t bestDist = 255;

  for (uint8_t d : order) {
    if (walls[cellX][cellY] & WALL_BIT[d]) continue;

    int8_t nx, ny;
    neighborCoord(cellX, cellY, d, nx, ny);
    if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) continue;

    if (floodDist[nx][ny] < bestDist) {
      bestDist = floodDist[nx][ny];
      bestDir = d;
    }
  }

  return bestDir;
}

// ========================== ROUTE GENERATION ================================
// After the maze is known, calculate a deterministic route from START to the
// centre. This route is used by both Round 2 and Round 3.
bool buildRouteToCenter() {
  computeFloodToCenter();

  int8_t x = START_X;
  int8_t y = START_Y;
  routeLength = 0;

  while (!isCenterCell(x, y)) {
    if (routeLength >= MAX_ROUTE) {
      Serial.println("ERROR: route buffer full");
      return false;
    }

    uint8_t bestDir = NORTH;
    uint8_t bestDist = 255;

    // For replay, prefer straight, then right, left, back on equal distance.
    const uint8_t order[4] = {
      heading,
      (uint8_t)((heading + 1) % 4),
      (uint8_t)((heading + 3) % 4),
      opposite(heading)
    };

    for (uint8_t d : order) {
      if (walls[x][y] & WALL_BIT[d]) continue;

      int8_t nx, ny;
      neighborCoord(x, y, d, nx, ny);
      if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) continue;

      if (floodDist[nx][ny] < bestDist) {
        bestDist = floodDist[nx][ny];
        bestDir = d;
      }
    }

    if (bestDist == 255) {
      Serial.printf("ERROR: no route from (%d,%d)\n", x, y);
      return false;
    }

    route[routeLength++] = bestDir;
    neighborCoord(x, y, bestDir, x, y);
    heading = bestDir;
  }

  // Restore physical navigation heading. Route generation must not leave
  // the actual robot's heading variable changed.
  heading = NORTH;
  Serial.printf("Route generated: %u cells\n", routeLength);
  return true;
}

void printRoute() {
  DBGF(1, "[ROUTE] length=%u\n", routeLength);
  Serial.println("ROUTE:");
  for (uint16_t i = 0; i < routeLength; i++) {
    char c = '?';
    if (route[i] == NORTH) c = 'N';
    if (route[i] == EAST)  c = 'E';
    if (route[i] == SOUTH) c = 'S';
    if (route[i] == WEST)  c = 'W';
    Serial.print(c);
    Serial.print(' ');
  }
  Serial.println();
}

// ========================== MAP SAVE/LOAD ===================================
bool saveMazeToFlash() {
  SavedMap saved;
  saved.magic = MAP_MAGIC;
  memcpy(saved.walls, walls, sizeof(walls));
  saved.routeLength = routeLength;
  memset(saved.route, 0, sizeof(saved.route));
  memcpy(saved.route, route, sizeof(route));

  prefs.begin(NVS_NAMESPACE, false);
  const size_t written = prefs.putBytes(NVS_KEY, &saved, sizeof(saved));
  prefs.end();

  const bool ok = written == sizeof(saved);
  DBGF(1, "[FLASH] Save: %s (%u bytes), route=%u\n", ok ? "OK" : "FAIL", (unsigned)written, routeLength);
  return ok;
}

bool loadMazeFromFlash() {
  SavedMap saved;

  prefs.begin(NVS_NAMESPACE, true);
  const size_t got = prefs.getBytes(NVS_KEY, &saved, sizeof(saved));
  prefs.end();

  if (got != sizeof(saved) || saved.magic != MAP_MAGIC) {
    Serial.println("No valid saved maze found.");
    return false;
  }

  memcpy(walls, saved.walls, sizeof(walls));
  if (saved.routeLength == 0 || saved.routeLength > MAX_ROUTE) {
    Serial.println("Saved maze has no valid discovered route.");
    return false;
  }
  routeLength = saved.routeLength;
  memcpy(route, saved.route, sizeof(route));
  DBGF(1, "[FLASH] Load OK: route=%u cells\n", routeLength);
  return true;
}

void clearSavedMaze() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.clear();
  prefs.end();
  Serial.println("Saved maze erased.");
}

// ========================== MOTION CONTROL ==================================
void setWheelTargets(float leftMmS, float rightMmS) {
  leftTargetMmS = leftMmS;
  rightTargetMmS = rightMmS;
}

void runControlStep() {
  const uint32_t now = micros();
  const uint32_t elapsedUs = now - lastControlMicros;
  if (elapsedUs < CONTROL_PERIOD_US) return;
  lastControlMicros = now;

  const float dt = elapsedUs * 1e-6f;
  imuUpdate();
  motorVelocityControl(dt);

  if (DEBUG_ENABLED && DEBUG_LEVEL >= 3) {
    static uint32_t lastCtrlPrint = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastCtrlPrint >= 250) {
      lastCtrlPrint = nowMs;
      DBGF(3, "[CTRL] target L/R=%6.1f/%6.1f mm/s | actual L/R=%6.1f/%6.1f | PWM L/R=%d/%d | yaw=%6.2f\n",
           leftTargetMmS, rightTargetMmS, leftVelocityMmS, rightVelocityMmS,
           commandedLeftPwm, commandedRightPwm, gyroHeadingDeg);
    }
  }
}

void resetMotionControllers() {
  setWheelTargets(0, 0);
  stopMotors();
  leftPID.reset();
  rightPID.reset();
  leftVelocityMmS = 0;
  rightVelocityMmS = 0;
  lastControlMicros = micros();
}

// We need a controlled loop for a blocking manoeuvre, but the low-level wheel
// PID still runs at its fixed period. The navigation layer waits for completion.
void serviceMotion() {
  runControlStep();
}

void turnInPlace(int8_t quarterTurns) {
  debugTurnCount++;
  DBGF(1, "[TURN] #%lu START quarterTurns=%d from heading=%s yaw=%.2f\n", (unsigned long)debugTurnCount, quarterTurns, dirName(heading), gyroHeadingDeg);
  if (quarterTurns == 0) return;

  // Convert the current physical gyro angle into the desired absolute angle.
  const float start = gyroHeadingDeg;
  const float target = start + 90.0f * quarterTurns;
  uint16_t timeoutMs = (abs(quarterTurns) == 2) ? 1800 : 1200;
  const uint32_t startMs = millis();
  uint8_t stable = 0;

  resetMotionControllers();

  while (millis() - startMs < timeoutMs) {
    imuUpdate();
    const float error = target - gyroHeadingDeg;

    if (fabsf(error) <= TURN_TOLERANCE_DEG && fabsf(gyroRateDps) <= TURN_SETTLE_RATE_DPS) {
      stopMotors();
      if (++stable >= 8) break;
    } else {
      stable = 0;
      float output = TURN_KP * error - TURN_KD * gyroRateDps;
      output = constrain(output, -(float)TURN_MAX_PWM, (float)TURN_MAX_PWM);

      if (fabsf(error) > TURN_TOLERANCE_DEG && fabsf(output) < PWM_MIN_TURN)
        output = copysignf(PWM_MIN_TURN, output);

      // Differential wheel target: positive output = right/clockwise turn.
      setWheelTargets(0, 0);
      setMotors((int16_t)output, (int16_t)-output);
    }

    delayMicroseconds(1000);
  }

  stopMotors();
  DBGF(1, "[TURN] #%lu END heading=%s yaw=%.2f rate=%.2f\n", (unsigned long)debugTurnCount, dirName(heading), gyroHeadingDeg, gyroRateDps);

  if (millis() - startMs >= timeoutMs) {
    Serial.println("WARNING: turn timeout");
  }

  // Discrete maze heading is authoritative for navigation. The gyro remains
  // a physical measurement and is NOT artificially snapped here.
  int h = (int)heading + quarterTurns;
  while (h < 0) h += 4;
  while (h >= 4) h -= 4;
  heading = (uint8_t)h;

  // Reset the gyro reference to zero at the end of every turn. This avoids
  // accumulating long-term gyro drift across an entire maze.
  gyroHeadingDeg = 0.0f;
  lastImuMicros = micros();
}

float calculateWallCorrection() {
  const bool leftWall = tofs[0].lastMm < SIDE_WALL_MM;
  const bool rightWall = tofs[2].lastMm < SIDE_WALL_MM;

  // Only centre from walls when BOTH are present. With one wall, use gyro
  // heading only; otherwise the robot can steer toward the open side.
  if (!leftWall || !rightWall) return 0.0f;

  const float error = tofs[0].lastMm - tofs[2].lastMm;
  static float previousError = 0.0f;
  const float derivative = error - previousError;
  previousError = error;

  return -(WALL_KP * error + WALL_KD * derivative);
}

// Drive one 180 mm cell. This is still cell-based, but now the wheel outputs
// are controlled by encoder velocity PID and corrected by gyro/wall feedback.
void driveOneCell(uint8_t speedMode) {
  debugMoveCount++;
  DBGF(1, "[MOVE] #%lu START cell=(%d,%d) heading=%s mode=%u\n", (unsigned long)debugMoveCount, cellX, cellY, dirName(heading), speedMode);
  // IMPORTANT: these are controller target speeds, not PWM values.
  // These are conservative starting speeds for a 180 mm cell.
  const float targetSpeedMmS = (speedMode == 1) ? 180.0f :
                               (speedMode == 2) ? 230.0f : 450.0f;

  zeroEncoders();
  resetMotionControllers();
  zeroHeadingReference();

  tofReadAll();
  const bool useWalls = speedMode != 3;

  const uint32_t startMs = millis();
  const uint32_t timeoutMs = (speedMode == 3) ? 2500 : 4000;
  float travelled = 0.0f;

  while (true) {
    serviceMotion();

    const float left = leftDistMm();
    const float right = rightDistMm();
    travelled = 0.5f * (left + right);
    const float remaining = CELL_MM - travelled;

    if (remaining <= 0.0f) break;
    if (millis() - startMs > timeoutMs) {
      Serial.println("[ERROR] Cell drive timeout - motors stopped");
      stopMotors();
      return;
    }

    float speed = targetSpeedMmS;

    // Trapezoidal-ish velocity profile.
    if (travelled < ACCEL_DISTANCE_MM) {
      const float f = constrain(travelled / ACCEL_DISTANCE_MM, 0.25f, 1.0f);
      speed *= f;
    }
    if (remaining < DECEL_DISTANCE_MM) {
      const float f = constrain(remaining / DECEL_DISTANCE_MM, 0.20f, 1.0f);
      speed *= f;
    }

    float headingCorrection = HEADING_KP * (-gyroHeadingDeg) - HEADING_KD * gyroRateDps;
    float wallCorrection = 0.0f;

    if (useWalls && travelled > 20.0f && remaining > CELL_END_SLOW_MM) {
      // Refresh ToF less frequently than the control loop.
      static uint32_t lastWallReadMs = 0;
      if (millis() - lastWallReadMs >= 20) {
        tofReadAll();
        lastWallReadMs = millis();
      }
      wallCorrection = calculateWallCorrection();
    }

    float leftTarget = speed + headingCorrection + wallCorrection;
    float rightTarget = speed - headingCorrection - wallCorrection;

    // Convert target velocity to the current controller. There is no direct
    // relationship between target mm/s and PWM until FF/PID is calibrated.
    // Limit the requested velocity to a conservative range.
    leftTargetMmS = constrain(leftTarget, -500.0f, 500.0f);
    rightTargetMmS = constrain(rightTarget, -500.0f, 500.0f);

  }

  setWheelTargets(0, 0);
  stopMotors();

  DBGF(1, "[MOVE] #%lu END travelled=%.1f mm encL=%ld encR=%ld\n", (unsigned long)debugMoveCount, travelled, (long)encL, (long)encR);

  // Only update the logical grid position after the physical movement is
  // complete. The actual position is still subject to encoder calibration.
  switch (heading) {
    case NORTH: if (cellY < MAZE_SIZE - 1) cellY++; break;
    case EAST:  if (cellX < MAZE_SIZE - 1) cellX++; break;
    case SOUTH: if (cellY > 0) cellY--; break;
    case WEST:  if (cellX > 0) cellX--; break;
  }
}

// ======================= EXPLORATION ========================================
uint8_t chooseExplorationMove() {
  computeFloodToCenter();

  const uint8_t order[4] = {
    heading,
    (uint8_t)((heading + 1) % 4),
    (uint8_t)((heading + 3) % 4),
    opposite(heading)
  };

  uint8_t bestDir = heading;
  uint8_t bestDist = 255;

  for (uint8_t d : order) {
    if (walls[cellX][cellY] & WALL_BIT[d]) continue;

    int8_t nx, ny;
    neighborCoord(cellX, cellY, d, nx, ny);
    if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) continue;

    if (floodDist[nx][ny] < bestDist) {
      bestDist = floodDist[nx][ny];
      bestDir = d;
    }
  }

  DBGF(2, "[NAV] Chose %s from (%d,%d), flood=%u\n", dirName(bestDir), cellX, cellY, bestDist);
  return bestDir;
}

bool exploreOneStep() {
  senseWallsAtCurrentCell();

  if (isCenterCell(cellX, cellY)) {
    stopMotors();
    return true;
  }

  const uint8_t best = chooseExplorationMove();
  const int8_t delta = ((int8_t)best - (int8_t)heading + 4) % 4;

  if (delta == 1) turnInPlace(+1);
  else if (delta == 3) turnInPlace(-1);
  else if (delta == 2) turnInPlace(+2);

  // Record the actual direction travelled. This is important because the
  // exploration map may still contain unknown cells when the centre is found.
  if (routeLength >= MAX_ROUTE) {
    Serial.println("ERROR: discovered route buffer full");
    stopMotors();
    return false;
  }
  route[routeLength++] = best;

  driveOneCell(1); // search mode
  return false;
}

// ========================= REPLAY ===========================================
bool prepareReplay() {
  if (!loadMazeFromFlash()) {
    Serial.println("ERROR: Round 2/3 requires a maze + route saved by Round 1.");
    return false;
  }

  // The replay follows the route that was physically proven during Round 1.
  // This is safer than recomputing a route through still-unknown cells.
  printRoute();

  cellX = START_X;
  cellY = START_Y;
  heading = NORTH;
  routeIndex = 0;
  zeroHeadingReference();
  zeroEncoders();
  return true;
}

bool executeReplayStep(uint8_t speedMode) {
  if (routeIndex >= routeLength) return true;
  DBGF(2, "[REPLAY] step=%u/%u target=%s current=%s cell=(%d,%d)\n", routeIndex + 1, routeLength, dirName(route[routeIndex]), dirName(heading), cellX, cellY);

  const uint8_t desiredHeading = route[routeIndex];
  const int8_t delta = ((int8_t)desiredHeading - (int8_t)heading + 4) % 4;

  if (delta == 1) turnInPlace(+1);
  else if (delta == 3) turnInPlace(-1);
  else if (delta == 2) turnInPlace(+2);

  driveOneCell(speedMode);
  routeIndex++;

  return routeIndex >= routeLength;
}

// ============================== SETUP =======================================
void setup() {
  Serial.begin(DEBUG_BAUD);
  const uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 2000) {}

  Serial.println();
  Serial.println("========================================");
  Serial.println("        MICROMOUSE v5 STARTING");
  Serial.println("========================================");
  Serial.printf("ACTIVE ROUND = %u | DEBUG=%s LEVEL=%u\n", ACTIVE_ROUND, DEBUG_ENABLED ? "ON" : "OFF", DEBUG_LEVEL);
  Serial.printf("Geometry: maze=%ux%u cell=%.1f mm start=(%u,%u)\n", MAZE_SIZE, MAZE_SIZE, CELL_MM, START_X, START_Y);

  if (ACTIVE_ROUND < 1 || ACTIVE_ROUND > 3) {
    Serial.println("FATAL: ACTIVE_ROUND must be 1, 2 or 3");
    while (true) delay(1000);
  }

  pinMode(DIR_L_PIN, OUTPUT);
  pinMode(DIR_R_PIN, OUTPUT);
  digitalWrite(DIR_L_PIN, LOW);
  digitalWrite(DIR_R_PIN, LOW);

  ledcAttach(PWM_L_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(PWM_R_PIN, PWM_FREQ, PWM_RES);
  stopMotors();
  DBG1("[INIT] Motors/PWM ready");

  pinMode(ENC_L_C1_PIN, INPUT_PULLUP);
  pinMode(ENC_L_C2_PIN, INPUT_PULLUP);
  pinMode(ENC_R_C1_PIN, INPUT_PULLUP);
  pinMode(ENC_R_C2_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_C1_PIN), encoderLISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1_PIN), encoderRISR, RISING);
  DBG1("[INIT] Encoders/interrupts ready");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  DBG1("[INIT] Starting IMU...");
  if (!imuBegin()) {
    while (true) {
      stopMotors();
      delay(1000);
    }
  }

  imuCalibrate();
  DBGF(1, "[INIT] Gyro calibrated, bias=%.4f dps\n", gyroZBias);

  DBG1("[INIT] Starting ToF sensors...");
  if (!tofBeginAll()) {
    Serial.println("WARNING: one or more ToF sensors failed. DO NOT race until fixed.");
  }
  DBGF(1, "[INIT] ToF status: L=%s F=%s R=%s\n", tofs[0].ok ? "OK" : "FAIL", tofs[1].ok ? "OK" : "FAIL", tofs[2].ok ? "OK" : "FAIL");

  // Round 1 starts with a blank map. Rounds 2/3 load the saved map.
  if (ACTIVE_ROUND == 1) {
    clearMaze();
    routeLength = 0;
    routeIndex = 0;
    cellX = START_X;
    cellY = START_Y;
    heading = NORTH;
    DBG1("[ROUND 1] EXPLORE: start -> centre");
    rgbLedWrite(RGB_BUILTIN, 0, 0, 32);
  } else {
    if (!prepareReplay()) {
      rgbLedWrite(RGB_BUILTIN, 32, 0, 0);
      while (true) {
        stopMotors();
        delay(1000);
      }
    }

    if (ACTIVE_ROUND == 2) {
      DBG1("[ROUND 2] SLOW REPLAY: start -> centre");
      rgbLedWrite(RGB_BUILTIN, 32, 32, 0);
    } else {
      DBG1("[ROUND 3] SPEED RUN: start -> centre");
      rgbLedWrite(RGB_BUILTIN, 32, 0, 32);
    }
  }

  Serial.println("READY - hands clear");
  delay(2000);
  rgbLedWrite(RGB_BUILTIN, 0, 32, 0);
}

// =============================== LOOP =======================================
void loop() {
  if (ACTIVE_ROUND == 1) {
    if (exploreOneStep()) {
      Serial.println("========================================");
      DBG1("[ROUND 1 COMPLETE] Centre found");
      Serial.println("Saving discovered maze + proven route...");
      printRoute();
      saveMazeToFlash();
      Serial.println("STOP. Reset and set ACTIVE_ROUND = 2.");
      stopMotors();
      rgbLedWrite(RGB_BUILTIN, 0, 32, 0);
      while (true) delay(1000);
    }
    return;
  }

  if (ACTIVE_ROUND == 2) {
    if (executeReplayStep(2)) {
      Serial.println("========================================");
      DBG1("[ROUND 2 COMPLETE] Slow replay finished");
      Serial.println("STOP. Reset and set ACTIVE_ROUND = 3.");
      stopMotors();
      rgbLedWrite(RGB_BUILTIN, 0, 32, 0);
      while (true) delay(1000);
    }
    return;
  }

  if (ACTIVE_ROUND == 3) {
    if (executeReplayStep(3)) {
      Serial.println("========================================");
      DBG1("[ROUND 3 COMPLETE] Speed run finished");
      stopMotors();
      rgbLedWrite(RGB_BUILTIN, 0, 32, 0);
      while (true) delay(1000);
    }
    return;
  }
}

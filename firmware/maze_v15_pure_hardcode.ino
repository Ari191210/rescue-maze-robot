// ==================================================================
//  RESCUE MAZE — COMPETITION BUILD v15 (100% PURE HARDCODE)
//  No sensors used for movement. No PID. No wall-following. The bot does
//  ONLY what PATH[] says, using encoder-count dead reckoning. This means
//  nothing corrects drift, so these THREE numbers MUST be right:
//     1) COUNTS_PER_CELL  -> one F = 300mm  (measure, then scale)
//     2) STOP_MARGIN_TURN -> a real 90 deg  (TEST_MODE=2 x3, average)
//     3) PATH[]           -> your route from START (bot's own view)
//  ---- history below ----
//  RESCUE MAZE — COMPETITION BUILD v14 (HARDCODE + clean stop)
//  v14: brake BOTH wheels together (kills the 45-deg swing at each stop)
//       + speed taper: crawl the last SLOW_ZONE counts so it eases into
//       the stop instead of slamming (kills overshoot + straightens stop).
//  ---- original v13 header below ----
//  RESCUE MAZE — COMPETITION BUILD v13 (HARDCODE MODE)
//  Nano #5 master: I2C sensors + RMCS-2303 x2 shared bus
//
//  v13: adds TEST_MODE=8 HARDCODED PATH. You type the route as a string
//  in PATH[] below; the bot runs those moves in order, still using the
//  wall-following PID drive so it self-corrects against walls instead of
//  drifting like raw timed moves would. All prior fixes kept:
//   - hardened encoder reads (rejects 0-sentinel + wild glitches)
//   - brownout detector at boot
//   - corrected sensor address->side mapping
//   - motion timeouts so a bad read can't run the bot forever
//
//  DIRECTION IS CONFIRMED CORRECT (Option A). Do NOT flip it.
// ==================================================================
#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <RMCS2303drive.h>
#include <avr/wdt.h>

// ---- Reset-cause capture (runs before main(); AVR canonical method) ----
uint8_t mcusr_mirror __attribute__((section(".noinit")));
void grabResetCause(void) __attribute__((naked, used, section(".init3")));
void grabResetCause(void) {
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

RMCS2303 rmcs;
SoftwareSerial rmcsSerial(2, 3);     // RX=D2, TX=D3

const byte LEFT_ID  = 7;   // verified: slave 7 = physical left
const byte RIGHT_ID = 3;   // verified: slave 3 = physical right

// ===== DIRECTION — CONFIRMED CORRECT (Option A). Do NOT change. =====
const byte LEFT_FWD  = 0;
const byte LEFT_REV  = 1;
const byte RIGHT_FWD = 1;
const byte RIGHT_REV = 0;

// ==================================================================
//  *** YOUR MAZE ROUTE — THIS IS THE MAIN THING YOU EDIT ***
//  Each letter = ONE action, run in order:
//     F = drive forward one cell
//     R = turn right 90 (in place, does NOT move forward)
//     L = turn left  90 (in place, does NOT move forward)
//     U = u-turn 180   (in place)
//  To round a right corner you write "RF": turn right, THEN go forward.
//  Example below = fwd, fwd, turn right, fwd, turn left, fwd:
const char PATH[] = "FFRFLF";     // <-- REPLACE with your actual route
// ==================================================================

const int INP_CONTROL_MODE = 257;
const int PP_gain = 10, PI_gain = 8, VF_gain = 32;
const int LPR = 334;
const int acceleration = 500;
const byte CONFIG_VERSION = 1;
const int  EEPROM_ADDR    = 0;

const int  BASE_SPEED      = 3000;
const int  TURN_SPEED      = 2000;
const int  DIAG_SPEED      = 1500;
const int  CRAWL_SPEED     = 1200;   // ease-in speed near the stop (taper)
const long SLOW_ZONE       = 1200;   // start crawling this many counts before brake point
const long COUNTS_PER_CELL = 4500;
const long COUNTS_PER_90   = 1695;

long STOP_MARGIN_CELL = 300;
long STOP_MARGIN_TURN = 400;   // PLACEHOLDER — CALIBRATE via TEST_MODE=2 (see notes)

const float KP = 3.0;
const float KD = 1.0;
const int   PID_CLAMP  = 300;
const int   ERR_CLAMP  = 60;
const int   WALL_NEAR  = 25;
const int   FRONT_STOP = 8;
const long  FB_MAX_JUMP = 15000;

// Address -> physical side, per verified cover-test:
//   0x09 = FRONT, 0x0A = RIGHT, 0x08 = BACK, 0x0B = LEFT
const uint8_t ADDR_F = 0x09, ADDR_R = 0x0A, ADDR_B = 0x08, ADDR_L = 0x0B;

// ---------------- Reset cause / brownout report ----------------
void printResetCause() {
  Serial.print(F("Reset cause: "));
  if (mcusr_mirror & (1 << PORF))  Serial.print(F("POWER-ON "));
  if (mcusr_mirror & (1 << EXTRF)) Serial.print(F("EXTERNAL(reset btn/upload) "));
  if (mcusr_mirror & (1 << BORF))  Serial.print(F("*** BROWNOUT *** "));
  if (mcusr_mirror & (1 << WDRF))  Serial.print(F("WATCHDOG "));
  if (mcusr_mirror == 0)           Serial.print(F("(cleared/unknown)"));
  Serial.println();
  if (mcusr_mirror & (1 << BORF)) {
    Serial.println(F(">> BROWNOUT: supply sagged. Fix power before trusting a run."));
  }
}

// ---------------- Sensors ----------------
int16_t readSensorOnce(uint8_t addr) {
  if (Wire.requestFrom(addr, (uint8_t)2) == 2) {
    int16_t v = (int16_t)((Wire.read() << 8) | Wire.read());  // HIGH byte first
    if (v > 0 && v < 500) return v;
  }
  return -1;
}

int16_t readDistance(uint8_t addr) {  // median of 3
  int16_t a = readSensorOnce(addr), b = readSensorOnce(addr), c = readSensorOnce(addr);
  if (a > b) { int16_t t = a; a = b; b = t; }
  if (b > c) { int16_t t = b; b = c; c = t; }
  if (a > b) { int16_t t = a; a = b; b = t; }
  return b;
}

bool wallNear(int16_t d) { return d > 0 && d < WALL_NEAR; }

// ---------------- Encoder feedback (with corrupt-read guard) ----------------
long lastGoodFB = 0;

long seedFB(byte id) {
  for (byte i = 0; i < 3; i++) {
    long fb = rmcs.Position_Feedback(id);
    if (fb != 0) { lastGoodFB = fb; return fb; }
    delay(5);
  }
  lastGoodFB = 0;
  return 0;
}

bool fbLooksBad(long fb) {
  if (fb == 0 && labs(lastGoodFB) > 3000) return true;      // sentinel
  if (labs(fb - lastGoodFB) > FB_MAX_JUMP) return true;      // wild glitch
  return false;
}

long safePosition(byte id) {
  long fb = rmcs.Position_Feedback(id);
  if (fbLooksBad(fb)) {
    long fb2 = rmcs.Position_Feedback(id);                   // one retry
    if (fbLooksBad(fb2)) return lastGoodFB;                  // hold last good
    fb = fb2;
  }
  lastGoodFB = fb;
  return fb;
}

// ---------------- Motor helpers ----------------
void stopMotors() {
  rmcs.Disable_Digital_Mode(LEFT_ID,  0);
  rmcs.Disable_Digital_Mode(RIGHT_ID, 0);
  delay(150);
}

void brakeStop(byte lDir, byte rDir) {
  rmcs.Brake_Motor(LEFT_ID,  lDir);
  rmcs.Brake_Motor(RIGHT_ID, rDir);   // BOTH together — no stagger -> no sideways kick
  delay(250);
  stopMotors();
}

void setSpeeds(int l, int r) {
  rmcs.Speed(LEFT_ID,  l);
  rmcs.Speed(RIGHT_ID, r);
}

void reportOvershoot(const __FlashStringHelper* tag, long startPos, long target) {
  delay(200);
  long endPos = safePosition(LEFT_ID);
  long final_d = labs(endPos - startPos);
  Serial.print(tag);
  Serial.print(F(" target=")); Serial.print(target);
  Serial.print(F(" final="));  Serial.print(final_d);
  Serial.print(F(" overshoot=")); Serial.println(final_d - target);
}

// ---------------- Drive one cell (wall-follow PID) ----------------
void driveOneCellPID() {
  long startPos = seedFB(LEFT_ID);
  long stopAt = COUNTS_PER_CELL - STOP_MARGIN_CELL;
  float lastError = 0;
  int lastL = BASE_SPEED, lastR = BASE_SPEED;
  uint8_t tick = 0;

  int16_t r0L = readDistance(ADDR_L);
  int16_t r0R = readDistance(ADDR_R);
  int16_t refL = wallNear(r0L) ? r0L : -1;
  int16_t refR = wallNear(r0R) ? r0R : -1;
  Serial.print(F("[CELL start] refL=")); Serial.print(refL);
  Serial.print(F(" refR="));             Serial.println(refR);

  setSpeeds(BASE_SPEED, BASE_SPEED);
  rmcs.Enable_Digital_Mode(LEFT_ID,  LEFT_FWD);
  delay(5);
  rmcs.Enable_Digital_Mode(RIGHT_ID, RIGHT_FWD);

  unsigned long cellStart = millis();
  while (true) {
    long delta = labs(safePosition(LEFT_ID) - startPos);
    if (delta >= stopAt) break;
    if (millis() - cellStart > 5000) { Serial.println(F("[CELL] TIMEOUT")); break; }

    int16_t dF = readDistance(ADDR_F);
    int16_t dL = readDistance(ADDR_L);
    int16_t dR = readDistance(ADDR_R);
    if (dF > 0 && dF < FRONT_STOP) { Serial.println(F("[CELL] front backstop")); break; }

    bool nearL = wallNear(dL);
    bool nearR = wallNear(dR);
    if (nearL && refL < 0) refL = dL;
    if (nearR && refR < 0) refR = dR;

    float error = 0;
    if (nearL && nearR)      error = (float)dL - (float)dR;
    else if (nearL)          error = (float)dL - (float)refL;
    else if (nearR)          error = (float)refR - (float)dR;

    error = constrain(error, -ERR_CLAMP, ERR_CLAMP);

    float pid = KP * error + KD * (error - lastError);
    lastError = error;
    pid = constrain(pid, -PID_CLAMP, PID_CLAMP);

    int cap = BASE_SPEED;
    if ((stopAt - delta) < SLOW_ZONE) cap = CRAWL_SPEED;   // ease into the stop
    int l = constrain(BASE_SPEED - (int)pid, 500, cap);
    int r = constrain(BASE_SPEED + (int)pid, 500, cap);

    if (abs(l - lastL) > 50) { rmcs.Speed(LEFT_ID,  l); lastL = l; }
    if (abs(r - lastR) > 50) { rmcs.Speed(RIGHT_ID, r); lastR = r; }

    if (++tick >= 5) {
      tick = 0;
      Serial.print(F("d="));   Serial.print(delta);
      Serial.print(F(" F="));  Serial.print(dF);
      Serial.print(F(" L="));  Serial.print(dL);
      Serial.print(F(" R="));  Serial.print(dR);
      Serial.print(F(" e="));  Serial.print(error, 0);
      Serial.print(F(" | "));  Serial.print(lastL);
      Serial.print(F("/"));    Serial.println(lastR);
    }
  }
  brakeStop(LEFT_FWD, RIGHT_FWD);
  reportOvershoot(F("[CELL]"), startPos, COUNTS_PER_CELL);
}

// ---------------- Turn ----------------
void turn90(int dir) {   // +1 right, -1 left
  long startPos = seedFB(LEFT_ID);
  long stopAt = COUNTS_PER_90 - STOP_MARGIN_TURN;
  byte lDir = (dir > 0) ? LEFT_FWD : LEFT_REV;
  byte rDir = (dir > 0) ? RIGHT_REV : RIGHT_FWD;

  Serial.print(F("[TURN] dir=")); Serial.println(dir > 0 ? F("RIGHT") : F("LEFT"));
  setSpeeds(TURN_SPEED, TURN_SPEED);
  rmcs.Enable_Digital_Mode(LEFT_ID,  lDir);
  delay(5);
  rmcs.Enable_Digital_Mode(RIGHT_ID, rDir);

  unsigned long turnStart = millis();
  while (labs(safePosition(LEFT_ID) - startPos) < stopAt) {
    if (millis() - turnStart > 3000) { Serial.println(F("[TURN] TIMEOUT")); break; }
  }
  brakeStop(lDir, rDir);
  reportOvershoot(F("[TURN]"), startPos, COUNTS_PER_90);
}

void uTurn() { turn90(1); turn90(1); }

// ---- PURE dead-reckoning forward: NO sensors, NO PID, encoder count only ----
void driveForwardBlind() {
  long startPos = seedFB(LEFT_ID);
  long stopAt = COUNTS_PER_CELL - STOP_MARGIN_CELL;
  bool crawling = false;

  setSpeeds(BASE_SPEED, BASE_SPEED);
  rmcs.Enable_Digital_Mode(LEFT_ID,  LEFT_FWD);
  delay(5);
  rmcs.Enable_Digital_Mode(RIGHT_ID, RIGHT_FWD);

  unsigned long t0 = millis();
  while (true) {
    long delta = labs(safePosition(LEFT_ID) - startPos);
    if (delta >= stopAt) break;
    if (!crawling && (stopAt - delta) < SLOW_ZONE) {
      setSpeeds(CRAWL_SPEED, CRAWL_SPEED);   // ease into the stop, once
      crawling = true;
    }
    if (millis() - t0 > 5000) { Serial.println(F("[FWD] TIMEOUT")); break; }
  }
  brakeStop(LEFT_FWD, RIGHT_FWD);
  reportOvershoot(F("[FWD]"), startPos, COUNTS_PER_CELL);
}

// ---------------- HARDCODED PATH RUNNER (TEST_MODE=8) ----------------
void runHardcodedPath() {
  Serial.print(F("Hardcoded route: ")); Serial.println(PATH);
  Serial.println(F("Place bot at START. Running in 3s..."));
  delay(3000);

  for (byte i = 0; PATH[i] != '\0'; i++) {
    char cmd = PATH[i];
    Serial.print(F(">> step ")); Serial.print(i);
    Serial.print(F(" = "));      Serial.println(cmd);

    switch (cmd) {
      case 'F': driveForwardBlind(); break;   // PURE dead reckoning, no sensors
      case 'R': turn90(1);           break;   // right 90 in place
      case 'L': turn90(-1);          break;   // left 90 in place
      case 'U': uTurn();             break;   // u-turn 180
      default:
        Serial.print(F("   unknown cmd '")); Serial.print(cmd);
        Serial.println(F("' - skipped"));
        break;
    }
    delay(200);   // brief settle between actions
  }
  Serial.println(F("=== PATH COMPLETE ==="));
}

// ---------------- Auto maze step (kept as fallback, TEST_MODE=0) ----------------
void navigateStep() {
  int16_t dF = readDistance(ADDR_F);
  int16_t dR = readDistance(ADDR_R);
  int16_t dL = readDistance(ADDR_L);

  Serial.print(F("NAV F:")); Serial.print(dF);
  Serial.print(F(" R:"));    Serial.print(dR);
  Serial.print(F(" L:"));    Serial.println(dL);

  if      (!wallNear(dF)) { Serial.println(F("-> fwd"));   driveOneCellPID(); }
  else if (!wallNear(dR)) { Serial.println(F("-> right")); turn90(1);  driveOneCellPID(); }
  else if (!wallNear(dL)) { Serial.println(F("-> left"));  turn90(-1); driveOneCellPID(); }
  else                    { Serial.println(F("-> uturn")); uTurn(); }
}

// ---------------- Diagnostics ----------------
void spinOne(byte id, const __FlashStringHelper* label) {
  Serial.print(F("\n[SOLO] ")); Serial.println(label);
  long s = rmcs.Position_Feedback(id);
  rmcs.Speed(id, DIAG_SPEED);
  rmcs.Enable_Digital_Mode(id, 1);
  delay(2000);
  rmcs.Disable_Digital_Mode(id, 1);
  delay(300);
  long d = rmcs.Position_Feedback(id) - s;
  Serial.print(F("  delta=")); Serial.print(d);
  if (labs(d) < 200) Serial.println(F("   <-- WEAK/DEAD even alone"));
  else               Serial.println(F("   (ok, motor turns alone)"));
  delay(800);
}

void runSoloDiag() {
  Serial.println(F("\n===== SOLO DIAGNOSTIC (bot on blocks) ====="));
  spinOne(LEFT_ID,  F("[1] LEFT_ID(7) - expect PHYSICAL LEFT wheel"));
  spinOne(RIGHT_ID, F("[2] RIGHT_ID(3) - expect PHYSICAL RIGHT wheel"));
  Serial.println(F("===== SOLO DIAG DONE ====="));
}

void runSensorTest() {   // TEST_MODE=7, motors OFF
  int16_t f = readDistance(ADDR_F);
  int16_t r = readDistance(ADDR_R);
  int16_t b = readDistance(ADDR_B);
  int16_t l = readDistance(ADDR_L);
  Serial.print(F("F(0x09)=")); Serial.print(f);
  Serial.print(F("   R(0x0A)=")); Serial.print(r);
  Serial.print(F("   B(0x08)=")); Serial.print(b);
  Serial.print(F("   L(0x0B)=")); Serial.print(l);
  Serial.print(F("   [F")); Serial.print(wallNear(f) ? '#' : '.');
  Serial.print(F(" R")); Serial.print(wallNear(r) ? '#' : '.');
  Serial.print(F(" L")); Serial.print(wallNear(l) ? '#' : '.');
  Serial.println(F("]"));
  delay(300);
}

void configureDrivesIfNeeded() {
  if (EEPROM.read(EEPROM_ADDR) == CONFIG_VERSION) {
    Serial.println(F("Drive config: already written (skipping)."));
    return;
  }
  Serial.println(F("First boot: writing drive params..."));
  rmcs.WRITE_PARAMETER(LEFT_ID,  INP_CONTROL_MODE, PP_gain, PI_gain, VF_gain, LPR, acceleration, BASE_SPEED);
  rmcs.SAVE(LEFT_ID);
  delay(500);
  rmcs.WRITE_PARAMETER(RIGHT_ID, INP_CONTROL_MODE, PP_gain, PI_gain, VF_gain, LPR, acceleration, BASE_SPEED);
  rmcs.SAVE(RIGHT_ID);
  delay(500);
  EEPROM.write(EEPROM_ADDR, CONFIG_VERSION);
  Serial.println(F("Drive config saved."));
}

// 0 = auto maze | 1 = one cell | 2 = right turn | 3 = left turn
// 4 = square | 5 = solo diag | 7 = sensor test | 8 = HARDCODED PATH
const byte TEST_MODE = 8;   // <-- 8 runs your PATH[] route
bool done = false;

void setup() {
  rmcs.Serial_selection(1);
  rmcs.Serial0(9600);
  rmcs.begin(&rmcsSerial, 9600);

  stopMotors();

  Serial.begin(9600);
  delay(50);
  Serial.println(F("\n\n=== MAZE v13 boot ==="));
  printResetCause();
  configureDrivesIfNeeded();

  Wire.begin();
  Serial.print(F("TEST_MODE=")); Serial.print(TEST_MODE);
  Serial.print(F("  BASE=")); Serial.print(BASE_SPEED);
  Serial.print(F("  TURN_MARGIN=")); Serial.println(STOP_MARGIN_TURN);
  Serial.println(F("Ready.\n"));
  delay(1500);
}

void loop() {
  if (done) return;
  switch (TEST_MODE) {
    case 0: navigateStep(); delay(300); return;
    case 1: driveOneCellPID(); break;
    case 2: turn90(1); break;
    case 3: turn90(-1); break;
    case 4: for (byte i = 0; i < 4; i++) { driveOneCellPID(); turn90(1); } break;
    case 5: runSoloDiag(); break;
    case 7: runSensorTest(); return;
    case 8: runHardcodedPath(); break;   // <-- your hardcoded route
  }
  done = true;
}

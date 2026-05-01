/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║            TABLE CLEANING ROBOT — FULL FIRMWARE v2.3            ║
 * ║            Platform : ESP32 DevModule (Arduino Core)            ║
 * ║            Display  : 1.3" SH1106 OLED (128x64)                 ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  Required Libraries (install via Library Manager):              ║
 * ║    • U8g2               • ESP32Servo                            ║
 * ║    • NewPing              • Preferences (built-in)              ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  CHANGES IN v2.3  (on top of v2.2)                              ║
 * ║    • TCRT5000 now uses MAXIMA-style per-sensor threshold array   ║
 * ║      threshold_ir[2] stored in flash via Preferences            ║
 * ║    • IR Calibration added: robot reads surface & edge values    ║
 * ║      and auto-calculates midpoint threshold (same as MAXIMA)    ║
 * ║    • IR Monitor shows raw ADC, threshold, and 0/1 state         ║
 * ║    • Main menu extended to 7 items (item 7 = IR Calibration)    ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  PIN MAP                                                         ║
 * ║  ──────────────────────────────────────────────────────────────  ║
 * ║  OLED SDA        : 21     OLED SCL         : 22                 ║
 * ║  Servo (sweep)   : 13     ESC (BLDC)        : 16                ║
 * ║  Motor-L IN1     : 25     Motor-L IN2       : 26                ║
 * ║  Motor-L PWM     : 27     Motor-R IN1       : 17                ║
 * ║  Motor-R IN2     : 23     Motor-R PWM       : 33                ║
 * ║  Encoder Left A  : 34     Encoder Right A   : 35  (input-only)  ║
 * ║  TCRT5000 Left   : 36     TCRT5000 Right    : 39  (input-only)  ║
 * ║  Sonar TRIG      : 4      Sonar ECHO        : 14                ║
 * ║  Button OK       : 18     Button BACK       : 19                ║
 * ║  Buzzer          : 32                                            ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include <Wire.h>
#include <U8g2lib.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <NewPing.h>

/* ═══════════════════════════════════════════════════
   BUTTON STRUCT
════════════════════════════════════════════════════ */
#define LONG_PRESS_MS   800
#define DEBOUNCE_MS     40

struct Btn {
  uint8_t  pin;
  bool     lastRaw;
  bool     shortFired;
  bool     longFired;
  bool     longEmitted;
  uint32_t downAt;
};

/* ═══════════════════════════════════════════════════
   PIN DEFINITIONS
════════════════════════════════════════════════════ */
#define SERVO_PIN     13
#define ESC_PIN       16

#define ML_IN1        25
#define ML_IN2        26
#define ML_PWM_PIN    27

#define MR_IN1        17
#define MR_IN2        23
#define MR_PWM_PIN    33

#define ENC_L_PIN     34
#define ENC_R_PIN     35

#define TCRT_L_PIN    36
#define TCRT_R_PIN    39

#define SONAR_TRIG    4
#define SONAR_ECHO    14

#define BTN_OK_PIN    18
#define BTN_BK_PIN    19

#define BUZZER_PIN    32

/* ═══════════════════════════════════════════════════
   TCRT5000 THRESHOLD ARRAY  — MAXIMA style
   ─────────────────────────────────────────────────
   Index 0 = Left  sensor (GPIO 36)
   Index 1 = Right sensor (GPIO 39)

   Raw ADC  > threshold_ir[i]  →  EDGE  (1)
   Raw ADC  ≤ threshold_ir[i]  →  TABLE (0)

   Defaults to 2048 (mid-scale) until IR Calibration
   is run.  Values saved to flash under key "irThrL"
   and "irThrR".
════════════════════════════════════════════════════ */
int threshold_ir[2]  = { 2048, 2048 };
int maximum_ir[2]    = { 4095, 4095 };
int minimum_ir[2]    = {    0,    0 };

/* Sensor index helpers — same naming style as MAXIMA */
#define IR_LEFT   0
#define IR_RIGHT  1

/* ═══════════════════════════════════════════════════
   ROBOT PARAMETERS
════════════════════════════════════════════════════ */
#define ENC_CPR           1050
#define TRACK_W_MM        120.0f

#define BASE_DRIVE_SPEED  180
#define TURN_SPEED        145

#define OBSTACLE_MM       200.0f
#define SPIRAL_STEP_MM    50.0f
#define ZIGZAG_STEP_MM    50.0f

#define ARC_90_MM    ((float)(M_PI * TRACK_W_MM) / 4.0f)
#define ARC_180_MM   ((float)(M_PI * TRACK_W_MM) / 2.0f)

/* ── Wheel diameter — runtime, saved to flash ─────
   Range 10–200 mm, step 0.5 mm, default 46 mm      */
float wheelDia = 46.0f;

inline float mmPerCount()         { return (M_PI * wheelDia) / (float)ENC_CPR; }
inline float countToMM(long  c)   { return (float)c * mmPerCount(); }
inline long  mmToCount(float mm)  { return (long)(mm / mmPerCount()); }

/* ── Per-wheel speed trim (1–25, default 10) ─────── */
int spl = 10;
int spr = 10;

inline int speedL() { return constrain(BASE_DRIVE_SPEED * spl / 10, 0, 255); }
inline int speedR() { return constrain(BASE_DRIVE_SPEED * spr / 10, 0, 255); }

/* ═══════════════════════════════════════════════════
   SONAR / NEWPING
════════════════════════════════════════════════════ */
#define SONAR_MAX_CM  400

/* ═══════════════════════════════════════════════════
   SERVO SWEEP PARAMETERS
════════════════════════════════════════════════════ */
#define SERVO_SWEEP_MIN    0
#define SERVO_SWEEP_MAX    180
#define SERVO_SWEEP_STEP   2
#define SERVO_SWEEP_DELAY  15

/* ═══════════════════════════════════════════════════
   OLED
════════════════════════════════════════════════════ */
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

/* ═══════════════════════════════════════════════════
   NEWPING
════════════════════════════════════════════════════ */
NewPing sonar(SONAR_TRIG, SONAR_ECHO, SONAR_MAX_CM);

/* ═══════════════════════════════════════════════════
   PERIPHERAL OBJECTS
════════════════════════════════════════════════════ */
Servo       myServo;
Servo       myESC;
Preferences prefs;

/* ═══════════════════════════════════════════════════
   SERVO SWEEP STATE
════════════════════════════════════════════════════ */
int  servoPos = 0;
int  servoDir = 1;
bool servoSweepActive = false;
unsigned long lastServoUpdate = 0;

/* ═══════════════════════════════════════════════════
   ENCODERS
════════════════════════════════════════════════════ */
volatile long encL = 0;
volatile long encR = 0;

void IRAM_ATTR isrEncL() { encL++; }
void IRAM_ATTR isrEncR() { encR++; }

void resetEncoders() {
  noInterrupts(); encL = encR = 0; interrupts();
}

long safeAvgCounts() {
  noInterrupts(); long l = encL, r = encR; interrupts();
  return (l + r) >> 1;
}

/* ═══════════════════════════════════════════════════
   MOTOR DRIVER
════════════════════════════════════════════════════ */
#define LEDC_L_CH   2
#define LEDC_R_CH   3
#define LEDC_FREQ   5000
#define LEDC_RES    8

void motorsSetup() {
  ledcSetup(LEDC_L_CH, LEDC_FREQ, LEDC_RES); ledcAttachPin(ML_PWM_PIN, LEDC_L_CH);
  ledcSetup(LEDC_R_CH, LEDC_FREQ, LEDC_RES); ledcAttachPin(MR_PWM_PIN, LEDC_R_CH);
  pinMode(ML_IN1, OUTPUT); pinMode(ML_IN2, OUTPUT);
  pinMode(MR_IN1, OUTPUT); pinMode(MR_IN2, OUTPUT);
}

void motorLeft(int spd) {
  if      (spd > 0) { digitalWrite(ML_IN1, HIGH); digitalWrite(ML_IN2, LOW);  ledcWrite(LEDC_L_CH,  spd); }
  else if (spd < 0) { digitalWrite(ML_IN1, LOW);  digitalWrite(ML_IN2, HIGH); ledcWrite(LEDC_L_CH, -spd); }
  else              { digitalWrite(ML_IN1, LOW);  digitalWrite(ML_IN2, LOW);  ledcWrite(LEDC_L_CH,  0);   }
}

void motorRight(int spd) {
  if      (spd > 0) { digitalWrite(MR_IN1, HIGH); digitalWrite(MR_IN2, LOW);  ledcWrite(LEDC_R_CH,  spd); }
  else if (spd < 0) { digitalWrite(MR_IN1, LOW);  digitalWrite(MR_IN2, HIGH); ledcWrite(LEDC_R_CH, -spd); }
  else              { digitalWrite(MR_IN1, LOW);  digitalWrite(MR_IN2, LOW);  ledcWrite(LEDC_R_CH,  0);   }
}

void setMotors(int l, int r) { motorLeft(l); motorRight(r); }
void motorsStop()            { motorLeft(0); motorRight(0); }

/* ═══════════════════════════════════════════════════
   SENSORS
   ─────────────────────────────────────────────────
   readIR(idx)  — returns raw ADC value (0–4095)
                  for sensor index 0 (left) or 1 (right)

   irOnEdge(idx)  — true when ADC > threshold_ir[idx]
                    (same comparison as MAXIMA: raw > threshold)

   MAXIMA pattern replicated:
     s[i] = (readMux(i) > threshold[i]) ? 1 : 0
   Here:
     irOnEdge(i) = (readIR(i) > threshold_ir[i])
════════════════════════════════════════════════════ */
int readIR(int idx) {
  return analogRead(idx == IR_LEFT ? TCRT_L_PIN : TCRT_R_PIN);
}

bool irOnEdge(int idx) {
  return readIR(idx) > threshold_ir[idx];
}

/* Convenience wrappers used throughout the firmware */
bool leftEdge()     { return irOnEdge(IR_LEFT);  }
bool rightEdge()    { return irOnEdge(IR_RIGHT); }
bool edgeDetected() { return leftEdge() || rightEdge(); }
bool onSurface()    { return !leftEdge() && !rightEdge(); }

/* ═══════════════════════════════════════════════════
   IR THRESHOLD PERSISTENCE  — MAXIMA style
   ─────────────────────────────────────────────────
   Keys match MAXIMA pattern: "irThrL", "irMaxL" …
════════════════════════════════════════════════════ */
void saveIRThresholds() {
  prefs.begin("dims", false);
  prefs.putInt("irThrL", threshold_ir[IR_LEFT]);
  prefs.putInt("irThrR", threshold_ir[IR_RIGHT]);
  prefs.putInt("irMaxL", maximum_ir[IR_LEFT]);
  prefs.putInt("irMaxR", maximum_ir[IR_RIGHT]);
  prefs.putInt("irMinL", minimum_ir[IR_LEFT]);
  prefs.putInt("irMinR", minimum_ir[IR_RIGHT]);
  prefs.end();
}

void loadIRThresholds() {
  prefs.begin("dims", true);
  threshold_ir[IR_LEFT]  = prefs.getInt("irThrL", 2048);
  threshold_ir[IR_RIGHT] = prefs.getInt("irThrR", 2048);
  maximum_ir[IR_LEFT]    = prefs.getInt("irMaxL", 4095);
  maximum_ir[IR_RIGHT]   = prefs.getInt("irMaxR", 4095);
  minimum_ir[IR_LEFT]    = prefs.getInt("irMinL", 0);
  minimum_ir[IR_RIGHT]   = prefs.getInt("irMinR", 0);
  prefs.end();
}

/* ═══════════════════════════════════════════════════
   SONAR
════════════════════════════════════════════════════ */
float readSonarMM() {
  unsigned int d = sonar.ping_cm();
  return (d == 0) ? 9999.0f : (float)d * 10.0f;
}

/* ═══════════════════════════════════════════════════
   ACCESSORIES
════════════════════════════════════════════════════ */
void bldcOn()  { myESC.writeMicroseconds(2000); }
void bldcOff() { myESC.writeMicroseconds(1000); }

void updateServoSweep();
void startServoSweep();
void stopServoSweep();

/* ═══════════════════════════════════════════════════
   OBSTACLE HANDLING
════════════════════════════════════════════════════ */
void showObstacleMsg() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_10x20_tf);
  oled.drawStr(5, 24, "OBSTACLE!");
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(20, 50, "Remove it...");
  oled.sendBuffer();
}

void waitForObstacleClear() {
  motorsStop(); bldcOff(); stopServoSweep();
  while (readSonarMM() < OBSTACLE_MM) {
    showObstacleMsg();
    motorsStop(); bldcOff();
    digitalWrite(BUZZER_PIN, HIGH); delay(150);
    digitalWrite(BUZZER_PIN, LOW);  delay(150);
  }
  delay(300);
}

void handleObstacleDetected() {
  motorsStop(); bldcOff(); stopServoSweep();
  waitForObstacleClear();
}

bool checkObstacle() {
  if (readSonarMM() < OBSTACLE_MM) { handleObstacleDetected(); return true; }
  return false;
}

/* ═══════════════════════════════════════════════════
   SERVO SWEEP (non-blocking)
════════════════════════════════════════════════════ */
void updateServoSweep() {
  if (!servoSweepActive) return;
  unsigned long now = millis();
  if (now - lastServoUpdate < SERVO_SWEEP_DELAY) return;
  lastServoUpdate = now;
  servoPos += SERVO_SWEEP_STEP * servoDir;
  if (servoPos >= SERVO_SWEEP_MAX) { servoPos = SERVO_SWEEP_MAX; servoDir = -1; }
  else if (servoPos <= SERVO_SWEEP_MIN) { servoPos = SERVO_SWEEP_MIN; servoDir = 1; }
  myServo.write(servoPos);
}

void startServoSweep() {
  servoSweepActive = true; servoPos = SERVO_SWEEP_MIN; servoDir = 1; lastServoUpdate = 0;
}
void stopServoSweep() { servoSweepActive = false; }

void accessoriesOn()  { startServoSweep(); bldcOn(); }
void accessoriesOff() { stopServoSweep(); myServo.write(90); bldcOff(); }

/* ═══════════════════════════════════════════════════
   TABLE DIMENSIONS & PERSISTENCE
════════════════════════════════════════════════════ */
float tableW = 0.0f;
float tableL = 0.0f;

void saveDims() {
  prefs.begin("dims", false);
  prefs.putFloat("W",   tableW);
  prefs.putFloat("L",   tableL);
  prefs.putInt  ("SPL", spl);
  prefs.putInt  ("SPR", spr);
  prefs.putFloat("WD",  wheelDia);
  prefs.end();
}

void loadDims() {
  prefs.begin("dims", true);
  tableW   = prefs.getFloat("W",   0.0f);
  tableL   = prefs.getFloat("L",   0.0f);
  spl      = prefs.getInt  ("SPL", 10);
  spr      = prefs.getInt  ("SPR", 10);
  wheelDia = prefs.getFloat("WD",  46.0f);
  prefs.end();
  spl      = constrain(spl,      1,    25);
  spr      = constrain(spr,      1,    25);
  wheelDia = constrain(wheelDia, 10.0f, 200.0f);
}

/* ═══════════════════════════════════════════════════
   DISPLAY HELPERS
════════════════════════════════════════════════════ */
void showMsg(const char* l1, const char* l2 = "", const char* l3 = "") {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 10, l1);
  oled.drawStr(0, 32, l2);
  oled.drawStr(0, 54, l3);
  oled.sendBuffer();
}

void drawHeader(const char* title) {
  oled.drawBox(0, 0, 128, 11);
  oled.setDrawColor(0);
  oled.drawStr(2, 9, title);
  oled.setDrawColor(1);
}

/* ═══════════════════════════════════════════════════
   SPIRAL LOGO
════════════════════════════════════════════════════ */
static const uint8_t SPIRAL_PTS[][2] = {
  {18, 8}, {110, 8}, {110,56}, {18,56},
  {18,20}, { 92,20}, { 92,44}, {36,44},
  {36,32}, { 74,32}, { 74,44}
};
static const uint8_t SPIRAL_N = sizeof(SPIRAL_PTS) / sizeof(SPIRAL_PTS[0]);

void _drawSpiralLines() {
  for (uint8_t i = 0; i < SPIRAL_N - 1; i++) {
    int x0 = SPIRAL_PTS[i][0],   y0 = SPIRAL_PTS[i][1];
    int x1 = SPIRAL_PTS[i+1][0], y1 = SPIRAL_PTS[i+1][1];
    oled.drawLine(x0, y0, x1, y1);
    if (x0 == x1) {
      for (int d = -2; d <= 2; d++) if (d) oled.drawLine(x0+d, y0, x1+d, y1);
    } else {
      for (int d = -2; d <= 2; d++) if (d) oled.drawLine(x0, y0+d, x1, y1+d);
    }
  }
}

void drawStartupLogo() {
  oled.clearBuffer(); _drawSpiralLines(); oled.sendBuffer();
}

void runLogoSplash() {
  oled.clearBuffer(); oled.sendBuffer();
  for (uint8_t i = 0; i < SPIRAL_N - 1; i++) {
    int x0 = SPIRAL_PTS[i][0],   y0 = SPIRAL_PTS[i][1];
    int x1 = SPIRAL_PTS[i+1][0], y1 = SPIRAL_PTS[i+1][1];
    oled.drawLine(x0, y0, x1, y1);
    if (x0 == x1) {
      for (int d = -2; d <= 2; d++) if (d) oled.drawLine(x0+d, y0, x1+d, y1);
    } else {
      for (int d = -2; d <= 2; d++) if (d) oled.drawLine(x0, y0+d, x1, y1+d);
    }
    oled.sendBuffer();
    delay(300);
  }
  delay(1500);
}

/* ═══════════════════════════════════════════════════
   BUTTON HANDLING
════════════════════════════════════════════════════ */
Btn btnOK   = { BTN_OK_PIN, true, false, false, false, 0 };
Btn btnBack = { BTN_BK_PIN, true, false, false, false, 0 };

void pollBtn(Btn& b) {
  b.shortFired = false; b.longFired = false;
  bool raw = digitalRead(b.pin);
  if (raw == LOW  && b.lastRaw == HIGH) { b.downAt = millis(); b.longEmitted = false; }
  if (raw == LOW  && !b.longEmitted && (millis() - b.downAt) >= LONG_PRESS_MS)
    { b.longFired = true; b.longEmitted = true; }
  if (raw == HIGH && b.lastRaw == LOW) {
    if (!b.longEmitted && (millis() - b.downAt) >= DEBOUNCE_MS) b.shortFired = true;
    b.longEmitted = false;
  }
  b.lastRaw = raw;
}

void pollButtons() { pollBtn(btnOK); pollBtn(btnBack); }

void waitForOKShort() {
  while (true) { pollButtons(); updateServoSweep(); if (btnOK.shortFired) return; delay(10); }
}

/* ═══════════════════════════════════════════════════
   MOVEMENT PRIMITIVES
════════════════════════════════════════════════════ */
static void _pivotLeft(float arcMM) {
  long target = mmToCount(arcMM); resetEncoders();
  motorLeft(-TURN_SPEED); motorRight(TURN_SPEED);
  noInterrupts(); long r = encR; interrupts();
  while (r < target) { delay(1); updateServoSweep(); noInterrupts(); r = encR; interrupts(); }
  motorsStop(); delay(120);
}

static void _pivotRight(float arcMM) {
  long target = mmToCount(arcMM); resetEncoders();
  motorLeft(TURN_SPEED); motorRight(-TURN_SPEED);
  noInterrupts(); long l = encL; interrupts();
  while (l < target) { delay(1); updateServoSweep(); noInterrupts(); l = encL; interrupts(); }
  motorsStop(); delay(120);
}

void rotateLeft90()   { _pivotLeft (ARC_90_MM);  }
void rotateRight90()  { _pivotRight(ARC_90_MM);  }
void rotateLeft180()  { _pivotLeft (ARC_180_MM); }
void rotateRight180() { _pivotRight(ARC_180_MM); }

/* ═══════════════════════════════════════════════════
   FORWARD / BACKWARD DRIVES
════════════════════════════════════════════════════ */
void driveForward(float mm) {
  if (mm <= 0) return;
  long target = mmToCount(mm); resetEncoders(); setMotors(speedL(), speedR());
  while (safeAvgCounts() < target) {
    if (checkObstacle()) {
      long rem = target - safeAvgCounts();
      if (rem > 0) {
        showMsg("Resuming...", "", ""); delay(400);
        startServoSweep(); bldcOn(); delay(300);
        resetEncoders(); target = rem; setMotors(speedL(), speedR());
      }
    }
    updateServoSweep(); delay(2);
  }
  motorsStop();
}

void driveBackward(float mm) {
  if (mm <= 0) return;
  long target = mmToCount(mm); resetEncoders();
  setMotors(-speedL(), -speedR());
  while (safeAvgCounts() < target) { updateServoSweep(); delay(2); }
  motorsStop();
}

float driveUntilEdge(unsigned long timeout_ms = 30000UL) {
  resetEncoders(); setMotors(speedL(), speedR());
  unsigned long t0 = millis();
  while (true) {
    bool le = leftEdge(), re = rightEdge();
    if (le && re) { motorsStop(); break; }
    if (re && !le) {
      motorLeft(-TURN_SPEED); motorRight(TURN_SPEED);
      while (rightEdge() && !leftEdge()) { updateServoSweep(); delay(1); }
      setMotors(speedL(), speedR());
    }
    if (le && !re) {
      motorLeft(TURN_SPEED); motorRight(-TURN_SPEED);
      while (leftEdge() && !rightEdge()) { updateServoSweep(); delay(1); }
      setMotors(speedL(), speedR());
    }
    if (checkObstacle()) {
      if (!edgeDetected()) { startServoSweep(); bldcOn(); delay(300); setMotors(speedL(), speedR()); }
    }
    updateServoSweep();
    if ((millis() - t0) > timeout_ms) { motorsStop(); break; }
    delay(2);
  }
  return countToMM(safeAvgCounts());
}

float driveBackUntilSurface(unsigned long timeout_ms = 8000UL) {
  resetEncoders(); setMotors(-speedL(), -speedR());
  unsigned long t0 = millis();
  while (!onSurface()) { updateServoSweep(); if ((millis() - t0) > timeout_ms) break; delay(2); }
  motorsStop(); return countToMM(safeAvgCounts());
}

/* ═══════════════════════════════════════════════════
   MAIN MENU  — 7 items, 5 visible, scrollable
════════════════════════════════════════════════════ */
const char* menuItems[] = {
  "1. Settings",
  "2. Spiral Clean",
  "3. ZigZag Clean",
  "4. Calibration",
  "5. Fwd Test",
  "6. IR Monitor",
  "7. IR Calibrate"    // ← NEW in v2.3
};
const int MENU_COUNT   = 7;
const int MENU_VISIBLE = 5;
int menuIdx       = 0;
int menuScrollTop = 0;

void drawMainMenu() {
  if (menuIdx < menuScrollTop)
    menuScrollTop = menuIdx;
  if (menuIdx >= menuScrollTop + MENU_VISIBLE)
    menuScrollTop = menuIdx - MENU_VISIBLE + 1;

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);

  oled.drawBox(0, 0, 128, 11);
  oled.setDrawColor(0);
  oled.drawStr(20, 9, "= MAIN MENU =");
  oled.setDrawColor(1);

  for (int i = 0; i < MENU_VISIBLE; i++) {
    int idx = menuScrollTop + i;
    if (idx >= MENU_COUNT) break;
    uint8_t ry = 11 + i * 10;
    uint8_t ty = ry + 9;

    if (idx == menuIdx) {
      oled.drawBox(0, ry, 120, 10);
      oled.setDrawColor(0);
      oled.drawStr(3, ty, ">");
      oled.drawStr(11, ty, menuItems[idx]);
      oled.setDrawColor(1);
    } else {
      oled.drawStr(11, ty, menuItems[idx]);
    }
  }

  oled.setFont(u8g2_font_4x6_tf);
  if (menuScrollTop > 0)
    oled.drawStr(122, 20, "^");
  if (menuScrollTop + MENU_VISIBLE < MENU_COUNT)
    oled.drawStr(122, 63, "v");

  oled.sendBuffer();
}

/* ═══════════════════════════════════════════════════
   CALIBRATION (table dimensions)
════════════════════════════════════════════════════ */
void runCalibration() {
  float fwd, bck;
  showMsg("CALIBRATING", "Side 1: Width", "Moving fwd...");
  fwd = driveUntilEdge(); bck = driveBackUntilSurface();
  tableW = max(0.0f, fwd - bck); rotateLeft90();

  showMsg("CALIBRATING", "Side 2: Length", "Moving fwd...");
  fwd = driveUntilEdge(); bck = driveBackUntilSurface();
  tableL = max(0.0f, fwd - bck); rotateLeft90();

  showMsg("CALIBRATING", "Side 3: Verify", "Traversing...");
  driveUntilEdge(); driveBackUntilSurface(); rotateLeft90();

  showMsg("CALIBRATING", "Side 4: Return", "Coming back...");
  driveUntilEdge(); driveBackUntilSurface(); rotateLeft90();

  saveDims();

  char bw[22], bl[22];
  snprintf(bw, sizeof(bw), "W:%6.1f mm", tableW);
  snprintf(bl, sizeof(bl), "L:%6.1f mm", tableL);
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(8, 10, "CALIBRATION DONE");
  oled.drawStr(0, 26, bw);
  oled.drawStr(0, 38, bl);
  oled.drawStr(0, 58, "Press OK for menu");
  oled.sendBuffer();
  waitForOKShort();
}

/* ═══════════════════════════════════════════════════
   SETTINGS  — 5 fields
════════════════════════════════════════════════════ */
void drawSettings(float w, float l, int tSpl, int tSpr, float tWd, int field) {
  char b0[26], b1[26], b2[26], b3[26], b4[26];
  snprintf(b0, sizeof(b0), "Width  : %5.0f mm", w);
  snprintf(b1, sizeof(b1), "Length : %5.0f mm", l);
  snprintf(b2, sizeof(b2), "SPL    : %2d /25", tSpl);
  snprintf(b3, sizeof(b3), "SPR    : %2d /25", tSpr);
  snprintf(b4, sizeof(b4), "Whl Dia: %5.1f mm", tWd);
  const char* labels[5] = { b0, b1, b2, b3, b4 };

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  drawHeader("=== SETTINGS ===");

  for (int i = 0; i < 5; i++) {
    uint8_t ry = 12 + i * 10;
    uint8_t ty = ry + 9;
    if (field == i) {
      oled.drawBox(0, ry, 128, 10);
      oled.setDrawColor(0);
      oled.drawStr(2, ty, labels[i]);
      oled.setDrawColor(1);
    } else {
      oled.drawStr(2, ty, labels[i]);
    }
  }
  oled.sendBuffer();
}

void runSettings() {
  float editW = tableW, editL = tableL, editWd = wheelDia;
  int editSpl = spl, editSpr = spr, field = 0;

  while (true) {
    drawSettings(editW, editL, editSpl, editSpr, editWd, field);
    pollButtons(); updateServoSweep();

    if (btnOK.shortFired) {
      switch (field) {
        case 0: editW   += 10.0f; break;
        case 1: editL   += 10.0f; break;
        case 2: editSpl  = min(editSpl + 1, 25); break;
        case 3: editSpr  = min(editSpr + 1, 25); break;
        case 4: editWd   = constrain(editWd + 0.5f, 10.0f, 200.0f); break;
      }
    }
    if (btnBack.shortFired) {
      switch (field) {
        case 0: editW   -= 10.0f; if (editW  < 0) editW  = 0; break;
        case 1: editL   -= 10.0f; if (editL  < 0) editL  = 0; break;
        case 2: editSpl  = max(editSpl - 1, 1); break;
        case 3: editSpr  = max(editSpr - 1, 1); break;
        case 4: editWd   = constrain(editWd - 0.5f, 10.0f, 200.0f); break;
      }
    }
    if (btnOK.longFired) {
      if (field < 4) { field++; }
      else {
        tableW = editW; tableL = editL;
        spl = editSpl; spr = editSpr; wheelDia = editWd;
        saveDims();
        showMsg("SAVED!", "All settings stored", "to flash memory.");
        delay(1300); return;
      }
    }
    if (btnBack.longFired) {
      showMsg("Cancelled.", "No changes saved.", ""); delay(900); return;
    }
    delay(40);
  }
}

/* ═══════════════════════════════════════════════════
   SPIRAL CLEAN
════════════════════════════════════════════════════ */
void runSpiralClean() {
  if (tableW < 1.0f || tableL < 1.0f) {
    showMsg("NOT CALIBRATED!", "Go to Calibration", "or Settings first.");
    delay(2500); return;
  }
  showMsg("SPIRAL CLEAN", "Starting in 500ms", ""); accessoriesOn(); delay(500);
  float curW = tableW, curL = tableL;
  while (curW > 1.0f && curL > 1.0f) {
    driveForward(curW); rotateLeft90();
    driveForward(curL); rotateLeft90();
    driveForward(curW); rotateLeft90();
    curL -= SPIRAL_STEP_MM; if (curL <= 0.0f) break;
    driveForward(curL); rotateLeft90();
    curW -= SPIRAL_STEP_MM; if (curW <= 0.0f) break;
  }
  motorsStop(); accessoriesOff();
  showMsg("SPIRAL DONE!", "", "Press OK for menu"); waitForOKShort();
}

/* ═══════════════════════════════════════════════════
   ZIGZAG CLEAN
════════════════════════════════════════════════════ */
void runZigZagClean() {
  if (tableW < 1.0f || tableL < 1.0f) {
    showMsg("NOT CALIBRATED!", "Go to Calibration", "or Settings first.");
    delay(2500); return;
  }
  showMsg("ZIGZAG CLEAN", "Starting in 500ms", ""); accessoriesOn(); delay(500);
  float covered = 0.0f; int strip = 0;
  while (covered < tableL - 0.5f) {
    driveForward(tableW);
    float step = min(tableL - covered, ZIGZAG_STEP_MM);
    covered += step; if (covered >= tableL - 0.5f) break;
    if ((strip % 2) == 0) { rotateLeft90();  driveForward(step); rotateRight90(); }
    else                   { rotateRight90(); driveForward(step); rotateLeft90();  }
    strip++;
  }
  motorsStop(); accessoriesOff();
  showMsg("ZIGZAG DONE!", "", "Press OK for menu"); waitForOKShort();
}

/* ═══════════════════════════════════════════════════
   FORWARD TEST — sub-menu + motor test runner
════════════════════════════════════════════════════ */
const char* fwdItems[3] = { "Left Motor", "Right Motor", "Both Motors" };
int fwdIdx = 0;

void drawFwdMenu() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  drawHeader("= FORWARD TEST =");

  for (int i = 0; i < 3; i++) {
    uint8_t ry = 12 + i * 13;
    uint8_t ty = ry + 10;
    if (i == fwdIdx) {
      oled.drawBox(0, ry, 128, 13);
      oled.setDrawColor(0);
      oled.drawStr(4, ty, ">");
      oled.drawStr(13, ty, fwdItems[i]);
      oled.setDrawColor(1);
    } else {
      oled.drawStr(13, ty, fwdItems[i]);
    }
  }

  oled.setFont(u8g2_font_4x6_tf);
  oled.drawHLine(0, 52, 128);
  oled.drawStr(0, 60, "OK:nav  HldOK:run");
  oled.drawStr(0, 63, "                HldBK:back");
  oled.sendBuffer();
}

void runMotorTest(bool doLeft, bool doRight) {
  const char* title =
    (doLeft && doRight) ? "= BOTH MOTORS  =" :
    doLeft              ? "= LEFT MOTOR   =" :
                          "= RIGHT MOTOR  =";
  int lPWM = doLeft  ? speedL() : 0;
  int rPWM = doRight ? speedR() : 0;
  char buf[28];

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  drawHeader(title);
  snprintf(buf, sizeof(buf), "L-PWM:%-3d  R-PWM:%-3d", lPWM, rPWM);
  oled.drawStr(0, 24, buf);
  snprintf(buf, sizeof(buf), "Wheel dia: %.1f mm", wheelDia);
  oled.drawStr(0, 36, buf);
  oled.drawStr(4, 50, "Starting in 1s...");
  oled.setFont(u8g2_font_4x6_tf);
  oled.drawStr(28, 63, "OK short = STOP");
  oled.sendBuffer();
  delay(1000);

  resetEncoders();
  motorLeft(lPWM);
  motorRight(rPWM);

  while (true) {
    pollButtons();
    if (btnOK.shortFired) {
      motorsStop();
      showMsg("STOPPED.", "Motors off.", "OK for sub-menu.");
      delay(1200);
      return;
    }

    if (checkObstacle()) {
      motorLeft(lPWM);
      motorRight(rPWM);
    }

    noInterrupts(); long cL = encL, cR = encR; interrupts();
    float distL = (float)cL * mmPerCount();
    float distR = (float)cR * mmPerCount();

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    drawHeader(title);
    snprintf(buf, sizeof(buf), "L:%3d PWM  R:%3d PWM", lPWM, rPWM);
    oled.drawStr(0, 22, buf);
    snprintf(buf, sizeof(buf), "L: %7.1f mm", distL);
    oled.drawStr(0, 34, buf);
    snprintf(buf, sizeof(buf), "R: %7.1f mm", distR);
    oled.drawStr(0, 46, buf);
    snprintf(buf, sizeof(buf), "Dia: %.1f mm", wheelDia);
    oled.drawStr(0, 57, buf);
    oled.setFont(u8g2_font_4x6_tf);
    oled.drawStr(88, 63, "OK=STOP");
    oled.sendBuffer();

    delay(80);
  }
}

void runForwardTest() {
  fwdIdx = 0;
  while (true) {
    drawFwdMenu();
    pollButtons();

    if (btnOK.shortFired)   fwdIdx = (fwdIdx + 1) % 3;
    if (btnBack.shortFired) fwdIdx = (fwdIdx - 1 + 3) % 3;

    if (btnOK.longFired) {
      switch (fwdIdx) {
        case 0: runMotorTest(true,  false); break;
        case 1: runMotorTest(false, true);  break;
        case 2: runMotorTest(true,  true);  break;
      }
    }
    if (btnBack.longFired) return;

    delay(20);
  }
}

/* ═══════════════════════════════════════════════════
   ██████████████████████████████████████████████████
   IR MONITOR  — MAXIMA style
   ──────────────────────────────────────────────────
   Reads raw ADC with analogRead(), compares against
   threshold_ir[] exactly like MAXIMA does:
     irOnEdge(i)  ≡  readIR(i) > threshold_ir[i]

   Display shows:
     • Big 0/1 digit  (0 = TABLE, 1 = EDGE)
     • Status label   (TABLE / EDGE!)
     • Raw ADC value  for each sensor
     • Current threshold for each sensor
     • OK short = EXIT
   ██████████████████████████████████████████████████
════════════════════════════════════════════════════ */
void runIRMonitor() {
  char buf[32];

  while (true) {
    pollButtons();
    if (btnOK.shortFired) return;

    /* ── Read raw ADC (same call as MAXIMA's readMux) ── */
    int lRaw = readIR(IR_LEFT);
    int rRaw = readIR(IR_RIGHT);

    /* ── Apply threshold (same logic as MAXIMA) ────────
       MAXIMA:  s[i] = (readMux(i) > threshold[i]) ? 1 : 0
       Here:    lVal = (lRaw > threshold_ir[L])    ? 1 : 0  */
    int lVal = (lRaw > threshold_ir[IR_LEFT])  ? 1 : 0;
    int rVal = (rRaw > threshold_ir[IR_RIGHT]) ? 1 : 0;

    oled.clearBuffer();

    /* ── Header ── */
    oled.setFont(u8g2_font_6x10_tf);
    drawHeader("=  IR  SENSORS  =");

    /* ── Column labels ── */
    oled.drawStr(20, 22, "LEFT");
    oled.drawStr(80, 22, "RIGHT");

    /* ── Vertical divider ── */
    oled.drawVLine(63, 12, 42);
    oled.drawVLine(64, 12, 42);

    /* ── Large 0/1 digit ── */
    oled.setFont(u8g2_font_10x20_tf);
    snprintf(buf, sizeof(buf), "%d", lVal);
    oled.drawStr(26, 43, buf);
    snprintf(buf, sizeof(buf), "%d", rVal);
    oled.drawStr(88, 43, buf);

    /* ── Status text ── */
    oled.setFont(u8g2_font_6x10_tf);
    if (lVal == 0) oled.drawStr(12, 55, "TABLE");
    else           oled.drawStr(10, 55, "EDGE!");
    if (rVal == 0) oled.drawStr(72, 55, "TABLE");
    else           oled.drawStr(70, 55, "EDGE!");

    /* ── Digit box outlines ── */
    oled.drawFrame(16, 25, 22, 22);
    oled.drawFrame(78, 25, 22, 22);

    /* ── Footer: raw ADC and threshold for both sensors ──
       Format:  L:raw/thr  R:raw/thr  OK=EXIT
       Same info MAXIMA shows: raw value + threshold      */
    oled.setFont(u8g2_font_4x6_tf);
    oled.drawHLine(0, 57, 128);
    snprintf(buf, sizeof(buf), "L:%4d/%4d R:%4d/%4d",
             lRaw, threshold_ir[IR_LEFT],
             rRaw, threshold_ir[IR_RIGHT]);
    oled.drawStr(0, 63, buf);

    oled.sendBuffer();
    delay(50);   /* 20 Hz */
  }
}

/* ═══════════════════════════════════════════════════
   ██████████████████████████████████████████████████
   IR CALIBRATION  — MAXIMA style  (NEW in v2.3)
   ──────────────────────────────────────────────────
   Identical approach to MAXIMA's cal():
     1. Collect max & min ADC for each sensor
     2. threshold = (max - min) * 0.5 + min
     3. Save to flash

   Two-step process:
     Step 1: Hold sensor over TABLE surface → press OK
     Step 2: Hold sensor over EDGE  / gap   → press OK
     → thresholds auto-calculated & saved
   ██████████████████████████████████████████████████
════════════════════════════════════════════════════ */
void runIRCalibration() {
  /* ── Step 1: sample TABLE surface ── */
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  drawHeader("= IR CALIBRATE =");
  oled.drawStr(0, 22, "Step 1: Place over");
  oled.drawStr(0, 32, "TABLE surface");
  oled.drawStr(0, 44, "then press OK");
  oled.setFont(u8g2_font_4x6_tf);
  oled.drawStr(0, 63, "BACK=cancel");
  oled.sendBuffer();

  /* wait for OK or BACK */
  while (true) {
    pollButtons();
    if (btnBack.shortFired) {
      showMsg("Cancelled.", "", ""); delay(800); return;
    }
    if (btnOK.shortFired) break;
    delay(20);
  }

  /* Sample 100 readings for the surface */
  int surf[2] = {0, 0};
  for (int n = 0; n < 100; n++) {
    surf[IR_LEFT]  += readIR(IR_LEFT);
    surf[IR_RIGHT] += readIR(IR_RIGHT);
    delay(5);
  }
  surf[IR_LEFT]  /= 100;
  surf[IR_RIGHT] /= 100;

  minimum_ir[IR_LEFT]  = surf[IR_LEFT];
  minimum_ir[IR_RIGHT] = surf[IR_RIGHT];

  /* Show surface readings */
  char buf[24];
  snprintf(buf, sizeof(buf), "L:%4d  R:%4d", surf[IR_LEFT], surf[IR_RIGHT]);
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  drawHeader("= IR CALIBRATE =");
  oled.drawStr(0, 22, "Surface sampled:");
  oled.drawStr(0, 33, buf);
  oled.drawStr(0, 47, "Now hold over EDGE");
  oled.drawStr(0, 57, "then press OK");
  oled.sendBuffer();
  delay(500);

  /* wait for OK */
  while (true) {
    pollButtons();
    if (btnBack.shortFired) {
      showMsg("Cancelled.", "", ""); delay(800); return;
    }
    if (btnOK.shortFired) break;
    delay(20);
  }

  /* ── Step 2: sample EDGE / gap ── */
  int edge[2] = {0, 0};
  for (int n = 0; n < 100; n++) {
    edge[IR_LEFT]  += readIR(IR_LEFT);
    edge[IR_RIGHT] += readIR(IR_RIGHT);
    delay(5);
  }
  edge[IR_LEFT]  /= 100;
  edge[IR_RIGHT] /= 100;

  maximum_ir[IR_LEFT]  = edge[IR_LEFT];
  maximum_ir[IR_RIGHT] = edge[IR_RIGHT];

  /* ── Calculate threshold (identical to MAXIMA's cal()) ──
     MAXIMA:  threshold[i] = (maximum[i] - minimum[i]) * 0.5 + minimum[i]  */
  threshold_ir[IR_LEFT]  = (maximum_ir[IR_LEFT]  - minimum_ir[IR_LEFT])  / 2
                           + minimum_ir[IR_LEFT];
  threshold_ir[IR_RIGHT] = (maximum_ir[IR_RIGHT] - minimum_ir[IR_RIGHT]) / 2
                           + minimum_ir[IR_RIGHT];

  /* ── Save to flash ── */
  saveIRThresholds();

  /* ── Show result ── */
  char tl[24], tr[24];
  snprintf(tl, sizeof(tl), "L: min%4d max%4d", minimum_ir[IR_LEFT],  maximum_ir[IR_LEFT]);
  snprintf(tr, sizeof(tr), "R: min%4d max%4d", minimum_ir[IR_RIGHT], maximum_ir[IR_RIGHT]);
  char tt[24];
  snprintf(tt, sizeof(tt), "Thr L:%4d R:%4d", threshold_ir[IR_LEFT], threshold_ir[IR_RIGHT]);

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  drawHeader("= IR CALIBRATE =");
  oled.drawStr(0, 22, "Done! Saved.");
  oled.drawStr(0, 33, tl);
  oled.drawStr(0, 43, tr);
  oled.drawStr(0, 54, tt);
  oled.setFont(u8g2_font_4x6_tf);
  oled.drawStr(28, 63, "OK = back to menu");
  oled.sendBuffer();

  waitForOKShort();
}

/* ═══════════════════════════════════════════════════
   SETUP
════════════════════════════════════════════════════ */
void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  oled.begin();
  oled.clearBuffer();
  oled.sendBuffer();

  pinMode(BTN_OK_PIN,  INPUT_PULLUP);
  pinMode(BTN_BK_PIN,  INPUT_PULLUP);
  pinMode(BUZZER_PIN,  OUTPUT);     digitalWrite(BUZZER_PIN, LOW);
  pinMode(TCRT_L_PIN,  INPUT);      /* ADC input — no pull-up mode */
  pinMode(TCRT_R_PIN,  INPUT);
  pinMode(ENC_L_PIN,   INPUT_PULLUP);
  pinMode(ENC_R_PIN,   INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), isrEncR, RISING);

  myServo.attach(SERVO_PIN);
  myServo.write(90);

  myESC.attach(ESC_PIN);
  myESC.writeMicroseconds(1000);

  runLogoSplash();
  drawStartupLogo();

  showMsg("Arming ESC...", "Please wait...", "Do not spin motors");
  delay(2500);

  motorsSetup();
  loadDims();           /* W, L, spl, spr, wheelDia */
  loadIRThresholds();   /* threshold_ir[], max, min  */

  char line1[24], line2[24];
  snprintf(line1, sizeof(line1), "W:%.0f L:%.0f mm", tableW, tableL);
  snprintf(line2, sizeof(line2), "Dia:%.1fmm S:%d/%d", wheelDia, spl, spr);
  showMsg("TABLE CLEANER v2.3", line1, line2);
  delay(2000);
}

/* ═══════════════════════════════════════════════════
   MAIN LOOP
════════════════════════════════════════════════════ */
void loop() {
  drawMainMenu();
  pollButtons();
  updateServoSweep();

  if (btnOK.shortFired)   menuIdx = (menuIdx + 1) % MENU_COUNT;
  if (btnBack.shortFired) menuIdx = (menuIdx - 1 + MENU_COUNT) % MENU_COUNT;

  if (btnOK.longFired) {
    switch (menuIdx) {
      case 0: runSettings();       break;
      case 1: runSpiralClean();    break;
      case 2: runZigZagClean();    break;
      case 3: runCalibration();    break;
      case 4: runForwardTest();    break;
      case 5: runIRMonitor();      break;
      case 6: runIRCalibration();  break;   // ← NEW in v2.3
    }
  }

  delay(20);
}

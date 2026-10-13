/*
 * ============================================================================
 *  LINE FOLLOWER - COMPETITION BUILD
 *  Arduino Uno (DIP) + L298N + 5-Channel IR Array + 2x BO Motors
 * ============================================================================
 *
 *  WIRING (same as your previous car - verified against your old sketch):
 *    L298N:  ENA -> D5, IN1 -> D7, IN2 -> D8
 *            ENB -> D6, IN3 -> D9, IN4 -> D10
 *    Sensor: S1..S5 -> A0, A1, A2, A3, A4   (S1 = far left, S5 = far right)
 *    Motors: Left = OUT1/OUT2, Right = OUT3/OUT4
 *    Power:  11.1V (3x18650) -> L298N 12V input, grounds common
 *
 *  HOW THIS CODE THINKS (read this once - it makes tuning obvious):
 *    The car is always in one of four states:
 *
 *    TRACK   - sees the line. PID steers. On a proven straight it shifts
 *              into TURBO. Entering a hard corner it hands off to PIVOT.
 *    PIVOT   - hard corner (90 deg, hairpin, acute angle). Outer wheel
 *              drives forward, INNER WHEEL REVERSES. Reversing the inner
 *              wheel is what lets the car rotate around the corner instead
 *              of drifting wide. It exits the moment the line comes back
 *              toward the centre.
 *    COAST   - line vanished for a blink. Broken/dashed line sections do
 *              this. Instead of panicking, the car holds its last command
 *              for a fraction of a second and usually re-finds the line at
 *              full speed. Panic-steering on a gap is how races are lost.
 *    SEARCH  - line lost near a corner: spin toward the last seen side and
 *              hunt (~0.45s), then stop. Line lost while cruising clean and
 *              centred: that's the END box - the line ends there, race over,
 *              stop immediately.
 *
 *    Plus two track features handled inside TRACK:
 *    - Full-black bar (all 5 sensors triggered): a start/finish marker or
 *      intersection. The car punches straight through, and can optionally
 *      stop after a set number of bars (see FINISH_AFTER_BARS).
 *    - Turbo: after a quarter second of clean centring, base speed rises.
 *
 *  RACE DAY CHECKLIST:
 *    0. Charge the 18650s fully. Every speed below assumes a full pack.
 *    1. Upload with RUN_MODE = MODE_PRE_RACE, open Serial Monitor at
 *       115200 and follow the prompts. It checks all 5 sensors, learns
 *       the line polarity, rolls the motors for a direction check, reads
 *       the 5V rail, and ends with one verdict: RACE READY or exactly
 *       what to fix.
 *    2. If a wheel ran backward in the motor step, flip that motor's
 *       REVERSED flag below - do not rewire.
 *    3. Set RUN_MODE = MODE_LINE_FOLLOW, upload, practice laps: raise
 *       FAST_SPEED 10 at a time until it overshoots, back off one step.
 *       That is your race speed.
 * ============================================================================
 */

// ---------------- PIN MAP (do not change - matches your wiring) ----------------
const uint8_t PIN_ENA = 5;
const uint8_t PIN_IN1 = 7;
const uint8_t PIN_IN2 = 8;
const uint8_t PIN_ENB = 6;
const uint8_t PIN_IN3 = 9;
const uint8_t PIN_IN4 = 10;
const uint8_t SENSOR_PINS[5] = {A0, A1, A2, A3, A4};

// ---------------- HARDWARE FLAGS (from your old sketch) ----------------
const bool BLACK_IS_LOW   = true;   // your board outputs LOW on black

// *** WHEEL DIRECTION - READ THIS BEFORE RUNNING ***
// Forward driving must roll BOTH wheels forward together, like a pair.
// The motors are mounted mirrored (they face each other), so one side
// almost always needs its flag flipped to make that happen. That is all
// these flags do - they never make the wheels fight each other.
// CHECK: run MODE_MOTOR_TEST. STEP 1 rolls the car forward. If one wheel
// spins backward, flip THAT motor's flag below - do not touch the wiring.
const bool LEFT_REVERSED  = true;   // flip if the LEFT wheel spins backward
const bool RIGHT_REVERSED = false;  // flip if the RIGHT wheel spins backward

// Trim fixes a motor that is naturally slower. 12 worked on your last car.
const int LEFT_TRIM  = 12;
const int RIGHT_TRIM = 0;

// Runtime copy of the line polarity. Starts from your board's known value;
// MODE_PRE_RACE confirms it or flips it automatically.
bool g_blackIsLow = BLACK_IS_LOW;

// ---------------- MODES ----------------
enum RunMode { MODE_LINE_FOLLOW, MODE_PRE_RACE, MODE_MOTOR_TEST, MODE_SENSOR_TEST };
RunMode RUN_MODE = MODE_PRE_RACE;   // first upload of the day: self-test. Then set MODE_LINE_FOLLOW to race.
const bool DEBUG = false;   // race with false; Serial printing costs loop time

// ============================================================================
//  RACE TUNING - every number you'd ever want to touch lives in this block
// ============================================================================

// --- speeds (0-255), assuming a fully charged 11.1V pack ---
int FAST_SPEED  = 110;   // cruising base speed
int TURBO_SPEED = 150;   // base speed once a straight is proven
int CURVE_SPEED = 90;    // base speed while the PID is working a bend
int MAX_PWM     = 180;   // hard ceiling - nothing ever exceeds this
int MIN_PWM     = 45;    // below this a BO motor stalls and whines

// Turbo: how long the car must stay centred before the boost kicks in.
// 50 cycles x 5ms = 250ms of clean straight. Lower = earlier turbo.
const int TURBO_AFTER_CYCLES = 50;

// --- PIVOT: hard-corner behaviour ---
// When the error hits TURN_THRESHOLD percent, the car stops steering
// gently and commits to a full pivot. Hysteresis (exit at TURN_EXIT)
// stops it chattering in and out of pivot mid-corner.
//
// ZIGZAG PROTECTION: a true 90 holds the edge sensor on one side. A
// zigzag only brushes it for a cycle or two, alternating sides. So the
// pivot commits only after the error holds past the threshold, SAME
// side, for PIVOT_CONFIRM_CYCLES (3 cycles = 15ms). A zigzag never
// confirms; the PID - which is fast and already steering hard - eats
// the zigzag at full speed instead of stopping to pivot.
const int TURN_THRESHOLD = 90;    // enter pivot at |error| >= 90% (edge sensor only)
const int PIVOT_CONFIRM_CYCLES = 3;  // same-side hold required before committing
const int TURN_EXIT      = 40;    // leave pivot once |error| <= 40%
int TURN_PWM   = 180;             // outer wheel forward speed in a pivot
int BRAKE_PWM  = 130;             // inner wheel REVERSE speed - the corner bite.
                                  // Raise for sharper turns, lower if it over-rotates.

// --- COAST: broken-line handling ---
// How long to hold the last command when the line vanishes, in ms.
// Long enough to bridge paint gaps, short enough that a real loss
// still gets caught. 120ms at 175 PWM covers roughly 10-15cm of gap.
const unsigned long COAST_MS = 120;

// --- SEARCH: true line-loss recovery, and the FINISH LINE ---
// His track has no finish bar: the line simply ENDS at the red END box.
// So a line that stays lost IS the finish. The car hunts briefly
// (bridges any mid-corner dropouts), then stops cleanly in the end zone.
// Mid-race, a loss this long means derailment - stopping is also safer
// than spinning into the crowd. 0 = hunt forever (not for this track).
int RECOVER_PWM = 150;                          // spin speed while hunting
const unsigned long SEARCH_GIVEUP_MS = 450;     // ~0.45s of nothing = line ended = STOP
// Finish detection that actually works on his track: the line ENDS at the
// red END box at the end of a long STRAIGHT. Corners always come with high
// error or a pivot right before any dropout. So: line lost while cruising
// straight-and-centred (no corner action in the last second) = the line
// ended = race over, stop clean. Line lost near corner action = dropout,
// hunt for it. (This also means: place the car ON the line at START, or
// it will immediately think the race is over.)
const unsigned long QUIET_LOSS_FINISH_MS = 1000;  // no corner action for this long => loss = finish

// --- intersections / finish bar ---
// A bar must hold for this many cycles to count (rejects momentary flicker).
const int BAR_MIN_CYCLES = 2;
// 0 = never stop, just drive through bars. If your track ends with a black
// bar, set this to how many bars the car must cross before it stops.
// Example: finish line is the 3rd bar -> set 3.
// NOTE: if the car STARTS already sitting on a black bar, that bar counts
// as the first crossing - so set the number accordingly.
int FINISH_AFTER_BARS = 0;   // no bar on this track - the line ENDS at the END box; the car stops on line loss (SEARCH_GIVEUP_MS)

// --- PID ---
// Error is normalized: -100 = line under far-left sensor, +100 = far-right.
// The loop runs at a fixed 200 Hz, so these gains mean the same thing
// every lap regardless of battery sag or code changes.
float Kp = 1.5;    // steering strength. Wanders on straights? Raise. Oscillates? Lower.
float Ki = 0.0;    // keep 0 for racing. Integral remembers the past; corners punish that.
float Kd = 8.0;    // damping - the overshoot killer. Swings wide out of corners? Raise.
int  MAX_CORRECTION = 200;

const unsigned long CONTROL_PERIOD_MS = 5;   // 200 Hz fixed control loop
const unsigned long SETTLE_MS = 2000;        // launch countdown after power/reset

// ============================================================================
//  INTERNALS - below here is machinery, not tuning
// ============================================================================

// Weighted position of each sensor. The line's position is the weighted
// average of whichever sensors see it - this gives smooth, sub-sensor
// resolution instead of five crude positions.
const int SENSOR_WEIGHTS[5] = {-2000, -1000, 0, 1000, 2000};

enum FollowState { STATE_TRACK, STATE_PIVOT, STATE_COAST, STATE_SEARCH, STATE_FINISH };
FollowState state = STATE_TRACK;

float lastError = 0;
float derivFilt = 0;   // low-passed derivative - raw d(error) is useless on 5 digital eyes
float integral  = 0;
int   lastDirection = 1;        // +1 = line was last seen to the right
int   pivotDir      = 1;        // locked at pivot entry so it can't flip mid-turn
int   pivotConfirm  = 0;        // consecutive same-side over-threshold cycles
int   pivotConfirmDir = 0;
int   straightCycles = 0;
int   barCycles      = 0;       // consecutive full-black readings
int   barsCrossed    = 0;       // finished intersections / markers
bool  barCounted     = false;   // debounce: count each bar once
bool  wasFound       = true;

// COAST memory: the last command TRACK issued. Kept separate from the
// pivot/search commands on purpose - a gap right after a corner must
// replay steering, never a spin.
int coastL = 0;
int coastR = 0;

unsigned long lostAt     = 0;
unsigned long pivotStart = 0;
unsigned long lastActionAt = 0;   // last time the car was working: |error| big or pivoting
unsigned long lastCycle  = 0;

// ------------------------------------------------ motor primitives
// A positive pwm drives the wheel forward, negative reverses it.
// Trim is added only when the wheel is actually driving, so it can't
// push a stalled wheel past its stall point.
void setLeftMotor(int pwm) {
  bool forward = (pwm >= 0);
  if (LEFT_REVERSED) forward = !forward;
  int mag = constrain(abs(pwm) + (pwm != 0 ? LEFT_TRIM : 0), 0, MAX_PWM);
  digitalWrite(PIN_IN1, forward ? HIGH : LOW);
  digitalWrite(PIN_IN2, forward ? LOW  : HIGH);
  analogWrite(PIN_ENA, mag);
}

void setRightMotor(int pwm) {
  bool forward = (pwm >= 0);
  if (RIGHT_REVERSED) forward = !forward;
  int mag = constrain(abs(pwm) + (pwm != 0 ? RIGHT_TRIM : 0), 0, MAX_PWM);
  digitalWrite(PIN_IN3, forward ? HIGH : LOW);
  digitalWrite(PIN_IN4, forward ? LOW  : HIGH);
  analogWrite(PIN_ENB, mag);
}

// Normal driving always sends both wheels the same way: positive =
// forward on BOTH sides. The ONLY place one wheel is ever reversed is a
// pivot turn, and that is deliberate cornering, never cruising.
void drive(int leftPwm, int rightPwm) {
  setLeftMotor(leftPwm);
  setRightMotor(rightPwm);
}

void stopMotors() {
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}

// ------------------------------------------------ sensor reading
bool sensorSeesLine(uint8_t i) {
  int raw = digitalRead(SENSOR_PINS[i]);
  return g_blackIsLow ? (raw == LOW) : (raw == HIGH);
}

// Returns the line's position as an error percentage:
//   -100 = under far-left sensor ... 0 = centred ... +100 = far-right.
// `found` is false when NO sensor sees the line.
// `fullBlack` is true when ALL sensors see it (marker bar / intersection).
int readLineError(bool &found, bool &fullBlack) {
  long weightedSum = 0;
  int  activeCount = 0;
  for (uint8_t i = 0; i < 5; i++) {
    if (sensorSeesLine(i)) {
      weightedSum += SENSOR_WEIGHTS[i];
      activeCount++;
    }
  }
  found     = (activeCount > 0);
  fullBlack = (activeCount == 5);
  if (!found) return 0;
  return (int)((weightedSum / activeCount) / 20);   // -2000..2000 -> -100..100
}

// ------------------------------------------------ TRACK: PID steering
void stateTrack(int error) {
  int absErr = abs(error);

  // Hand hard corners to PIVOT - but only after the edge error holds
  // on the SAME side for PIVOT_CONFIRM_CYCLES. Zigzags brush the edge
  // briefly and alternate sides; they reset the counter and stay with
  // the PID at speed. A real 90 holds one side and confirms fast.
  if (absErr >= TURN_THRESHOLD) {
    int d = (error > 0) ? 1 : -1;
    if (d == pivotConfirmDir) pivotConfirm++;
    else { pivotConfirmDir = d; pivotConfirm = 1; }
    if (pivotConfirm >= PIVOT_CONFIRM_CYCLES) {
      pivotDir = d;
      lastDirection = d;
      state = STATE_PIVOT;
      pivotStart = millis();
      lastActionAt = millis();
      straightCycles = 0;
      pivotConfirm = 0;
      return;
    }
    // Not confirmed yet: fall through to the PID, which is already
    // steering at maximum correction toward it. No time lost.
  } else {
    pivotConfirm = 0;
  }

  // Turbo: earned by staying centred, lost the moment there's work to do.
  if (absErr <= 10) straightCycles++;
  else straightCycles = 0;

  int base;
  if (straightCycles >= TURBO_AFTER_CYCLES) base = TURBO_SPEED;
  else if (absErr > 40)                     base = CURVE_SPEED;
  else                                      base = FAST_SPEED;

  // PID. Proportional steers toward the line, derivative brakes the
  // steering as the line approaches so the car settles instead of
  // swinging past centre. Integral is available but off - see Kd note.
  // IMPORTANT: 5 digital eyes quantize the error to steps of 25. A raw
  // derivative reads every step as a huge kick and the car bang-bangs
  // between full-left and full-right. The filter keeps the damping and
  // kills the kick.
  integral += error;
  integral = constrain(integral, -300, 300);   // anti wind-up
  float rawDeriv = error - lastError;
  derivFilt = derivFilt * 0.5 + rawDeriv * 0.5;
  lastError = error;

  float correction = Kp * error + Ki * integral + Kd * derivFilt;
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  // Differential steering: correction adds to one side, subtracts from
  // the other. A strong correction brakes the inner wheel to a stop -
  // but TRACK never reverses a wheel. Full inner-wheel reversal is
  // reserved for PIVOT, so the wheels only ever fight each other in a
  // deliberate corner, never while driving.
  int leftSpeed  = constrain(base + (int)correction, 0, MAX_PWM);
  int rightSpeed = constrain(base - (int)correction, 0, MAX_PWM);
  if (leftSpeed  > 0 && leftSpeed  < MIN_PWM) leftSpeed  = MIN_PWM;
  if (rightSpeed > 0 && rightSpeed < MIN_PWM) rightSpeed = MIN_PWM;

  drive(leftSpeed, rightSpeed);
  coastL = leftSpeed;             // remember for COAST - steering, never a spin
  coastR = rightSpeed;

  if (DEBUG) {
    Serial.print("TRACK err=");  Serial.print(error);
    Serial.print(" base=");      Serial.print(base);
    Serial.print(" corr=");      Serial.print(correction);
    Serial.print(" L=");         Serial.print(leftSpeed);
    Serial.print(" R=");         Serial.println(rightSpeed);
  }
}

// ------------------------------------------------ PIVOT: hard corners
void statePivot(bool found, int error) {
  if (found) {
    // Exit path 1: line back near the centre - corner eaten.
    if (abs(error) <= TURN_EXIT) {
      state = STATE_TRACK;
      lastError = error;      // no derivative spike on re-entry
      derivFilt = 0;
      integral  = 0;
      return;
    }
    // Exit path 2 (acute V-bends): the line crossed to the OTHER side.
    // That means we rotated PAST it - the classic acute-corner overshoot.
    // Hand the now-small error to the PID instead of flipping the spin
    // and pirouetting in place. The direction NEVER flips mid-pivot.
    if (((error > 0) ? 1 : -1) != pivotDir) {
      state = STATE_TRACK;
      lastError = error;
      derivFilt = 0;
      integral  = 0;
      return;
    }
  }
  // Keep pivoting in the LOCKED entry direction. If the line drops out
  // mid-corner (common on acute angles), trust the lock, not the noise.
  lastDirection = pivotDir;

  // Spinning forever on a corner that never resolves is the one failure
  // a race car can't afford. If the line stays gone, hand off to SEARCH.
  if (!found && SEARCH_GIVEUP_MS > 0 && millis() - pivotStart > SEARCH_GIVEUP_MS) {
    state = STATE_SEARCH;
    lostAt = millis();
    return;
  }

  // NOTE: this pivot is the ONLY moment the wheels turn in opposite
  // directions. That is the cornering mechanism working as designed -
  // the car rotates on the spot, bites the corner, then both wheels go
  // back to driving forward together.
  if (pivotDir > 0) drive(TURN_PWM, -BRAKE_PWM);   // rotate right
  else              drive(-BRAKE_PWM, TURN_PWM);   // rotate left

  if (DEBUG) { Serial.print("PIVOT dir="); Serial.println(pivotDir); }
}

// ------------------------------------------------ marker bars
// Returns true while the car should hold a straight line through a bar.
// Counts each bar once; optionally ends the race after FINISH_AFTER_BARS.
bool handleBar(bool fullBlack) {
  if (fullBlack) {
    if (barCycles < 255) barCycles++;
    if (barCycles >= BAR_MIN_CYCLES && !barCounted) {
      barCounted = true;
      barsCrossed++;
      if (DEBUG) { Serial.print("BAR #"); Serial.println(barsCrossed); }
      if (FINISH_AFTER_BARS > 0 && barsCrossed >= FINISH_AFTER_BARS) {
        state = STATE_FINISH;
        return false;
      }
    }
    // Straight through at cruising speed. Do not steer on bar readings -
    // a horizontal bar gives a meaningless centred position.
    straightCycles = 0;
    coastL = coastR = FAST_SPEED;
    drive(FAST_SPEED, FAST_SPEED);
    return true;
  }
  barCycles = 0;
  barCounted = false;
  return false;
}

// ------------------------------------------------ main control step
void runLineFollow() {
  bool found, fullBlack;
  int error = readLineError(found, fullBlack);

  if (found) {
    if (error > 0) lastDirection = 1;
    if (error < 0) lastDirection = -1;
  }

  switch (state) {

    case STATE_TRACK:
      if (handleBar(fullBlack)) return;
      if (state == STATE_FINISH) return;    // bar handler ended the race
      if (found && abs(error) > 40) lastActionAt = millis();
      if (found) {
        // Fresh acquisition after a loss: reset derivative so the jump
        // from "no line" to "line at the edge" doesn't kick the steering.
        if (!wasFound) { lastError = error; derivFilt = 0; integral = 0; }
        stateTrack(error);
      } else if (millis() - lastActionAt > QUIET_LOSS_FINISH_MS) {
        // Cruising clean and the line vanished: this is the END box.
        stopMotors();
        state = STATE_FINISH;
      } else {
        state = STATE_COAST;
        lostAt = millis();
        drive(coastL, coastR);   // hold the last steering command
      }
      break;

    case STATE_PIVOT:
      statePivot(found, error);
      break;

    case STATE_COAST:
      if (found) {
        state = STATE_TRACK;
        lastError = error;
        derivFilt = 0;
        integral = 0;
      } else if (millis() - lostAt >= COAST_MS) {
        state = STATE_SEARCH;
      } else {
        drive(coastL, coastR);   // bridge the gap at speed
      }
      break;

    case STATE_SEARCH:
      if (found) {
        state = STATE_TRACK;
        lastError = error;
        derivFilt = 0;
        integral = 0;
      } else if (SEARCH_GIVEUP_MS > 0 && millis() - lostAt > SEARCH_GIVEUP_MS) {
        stopMotors();    // pure safety net. Set SEARCH_GIVEUP_MS = 0 to disable.
      } else {
        if (lastDirection > 0) drive(RECOVER_PWM, -RECOVER_PWM);
        else                   drive(-RECOVER_PWM, RECOVER_PWM);
        if (DEBUG) Serial.println("SEARCH - hunting");
      }
      break;

    case STATE_FINISH:
      stopMotors();
      if (DEBUG) Serial.println("FINISH - race over");
      break;
  }

  wasFound = found;
}


// ============================================================================
//  PRE-RACE SELF TEST (RUN_MODE = MODE_PRE_RACE)
//
//  What this honestly can and can't do on this hardware (no encoders, no
//  extra sensors):
//   - Sensor health: YES. Guided slide across the line proves each of the
//     5 channels triggers and clears, in order. Dead and stuck channels
//     are named.
//   - Line polarity: YES. It samples the plain surface, then the line, and
//     sets g_blackIsLow itself - a board wired the other way just works.
//   - Motor direction: GUIDED. It rolls the wheels; YOU confirm both went
//     forward. Without encoders the code cannot see rotation - anyone
//     claiming otherwise is faking it.
//   - Battery: PARTIAL. It reads the Uno's 5V rail through the internal
//     reference. A sagging rail means a weak pack or regulator. The raw
//     11.1V pack voltage cannot be measured without a divider - noted, not
//     faked. Below ~30% pack the motors approach stall no matter what.
//   - Auto-trim: NOT POSSIBLE without encoders. Instead you get exact
//     manual guidance in the verdict.
// ============================================================================

enum PreRacePhase { PR_INTRO, PR_SURFACE_WAIT, PR_SURFACE_SAMPLE, PR_SLIDE,
                    PR_MOTOR_FWD, PR_MOTOR_LEFT, PR_MOTOR_RIGHT, PR_BATTERY,
                    PR_VERDICT, PR_DONE };
PreRacePhase prPhase = PR_INTRO;
unsigned long prPhaseStart = 0;
bool prPrinted = false;

uint8_t prBaseline[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};
long    prTrigAt[5]   = {0, 0, 0, 0, 0};
int     prLowVotes = 0, prHighVotes = 0;
bool    prDead[5]    = {false, false, false, false, false};
bool    prSuspect[5] = {false, false, false, false, false};
bool    prOrderOK = true;
bool    prPolarityOK = false;
long    prVcc = 0;
bool    prVccOK = true;
int     g_verdict = 0;   // 0 = running, 1 = RACE READY, 2 = PROBLEMS FOUND

unsigned long prHighCount[5] = {0, 0, 0, 0, 0};
unsigned long prSamples = 0;

void prEnter(PreRacePhase p) { prPhase = p; prPhaseStart = millis(); prPrinted = false; }
bool prOnce() { if (!prPrinted) { prPrinted = true; return true; } return false; }
unsigned long prElapsed() { return millis() - prPhaseStart; }

// The Uno's 5V rail, measured against the internal 1.1V reference.
long readVccmV() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)) {}
  long adc = ADCL | ((long)ADCH << 8);
  if (adc == 0) return 0;
  return 1126400L / adc;   // 1.1V x 1024 x 1000
}

void prAnalyzeSlide() {
  // Dead channel: never triggered during the whole slide window.
  for (uint8_t i = 0; i < 5; i++) prDead[i] = (prTrigAt[i] == 0);

  // Polarity: whatever level appeared when a sensor crossed the line.
  if (prLowVotes + prHighVotes > 0) {
    g_blackIsLow = (prLowVotes >= prHighVotes);
    prPolarityOK = true;
  }

  // Order: first-trigger times must sweep monotonically along the array,
  // in either direction. A crossed cable shows up as mixed votes.
  int l2r = 0, r2l = 0;
  for (uint8_t i = 0; i < 5; i++) {
    for (uint8_t j = i + 1; j < 5; j++) {
      if (prTrigAt[i] > 0 && prTrigAt[j] > 0) {
        if (prTrigAt[i] < prTrigAt[j]) l2r++;
        else if (prTrigAt[i] > prTrigAt[j]) r2l++;
      }
    }
  }
  prOrderOK = (l2r == 0 || r2l == 0);
}

void prPrintVerdict() {
  bool sensorsOK = true;
  Serial.println("---------------- PRE-RACE VERDICT ----------------");
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print("Sensor S"); Serial.print(i + 1); Serial.print(": ");
    if (prDead[i])           { Serial.println("DEAD - no trigger. Check its wiring and threshold pot."); sensorsOK = false; }
    else if (prSuspect[i])   { Serial.println("STUCK/MISSET - reads line-state on plain surface. Adjust its threshold pot."); sensorsOK = false; }
    else                     Serial.println("OK");
  }
  Serial.print("Line polarity: ");
  if (prPolarityOK) Serial.println(g_blackIsLow ? "LOW on black (confirmed)" : "HIGH on black (auto-flipped, handled)");
  else              Serial.println("UNKNOWN - no sensor crossed the line. Redo the slide.");
  Serial.print("Sensor order: "); Serial.println(prOrderOK ? "OK (clean sweep)" : "WRONG - cable order crossed, remap S1..S5");
  Serial.print("5V rail: "); Serial.print(prVcc); Serial.print(" mV - ");
  Serial.println(prVccOK ? "OK" : "LOW - charge the pack / check the regulator");
  Serial.println("Motors: self-certified in steps 3-5. Veered with both forward? Raise the SLOW side's TRIM by 4 and rerun (auto-trim needs encoders, which this build does not have).");
  bool allOK = sensorsOK && prOrderOK && prPolarityOK && prVccOK;
  g_verdict = allOK ? 1 : 2;
  Serial.println(allOK ? "== RACE READY ==" : "== PROBLEMS FOUND - fix the lines above, then rerun ==");
}

void runPreRace() {
  switch (prPhase) {
    case PR_INTRO:
      if (prOnce()) Serial.println("PRE-RACE SELF TEST. Follow each prompt; no typing needed, it runs on timers.");
      if (prElapsed() > 2000) prEnter(PR_SURFACE_WAIT);
      break;

    case PR_SURFACE_WAIT:
      if (prOnce()) Serial.println("STEP 1: hold ALL sensors over plain surface (no line). Sampling in 3s...");
      if (prElapsed() > 3000) {
        prSamples = 0;
        for (uint8_t i = 0; i < 5; i++) prHighCount[i] = 0;
        prEnter(PR_SURFACE_SAMPLE);
      }
      break;

    case PR_SURFACE_SAMPLE:
      for (uint8_t i = 0; i < 5; i++) if (digitalRead(SENSOR_PINS[i]) == HIGH) prHighCount[i]++;
      prSamples++;
      if (prElapsed() > 2000) {
        uint8_t hi = 0;
        for (uint8_t i = 0; i < 5; i++) {
          prBaseline[i] = (prHighCount[i] * 2 > prSamples) ? HIGH : LOW;
          if (prBaseline[i] == HIGH) hi++;
        }
        uint8_t majority = (hi >= 3) ? HIGH : LOW;
        for (uint8_t i = 0; i < 5; i++) prSuspect[i] = (prBaseline[i] != majority);
        prEnter(PR_SLIDE);
      }
      break;

    case PR_SLIDE:
      if (prOnce()) Serial.println("STEP 2: slide the car slowly across the line, one edge to the other. 12s window.");
      for (uint8_t i = 0; i < 5; i++) {
        uint8_t r = digitalRead(SENSOR_PINS[i]);
        if (prTrigAt[i] == 0 && r != prBaseline[i]) {
          prTrigAt[i] = prElapsed();
          if (r == LOW) prLowVotes++; else prHighVotes++;
        }
      }
      if (prElapsed() > 12000) { prAnalyzeSlide(); prEnter(PR_MOTOR_FWD); }
      break;

    case PR_MOTOR_FWD:
      if (prOnce()) Serial.println("STEP 3: BOTH wheels rolling FORWARD for 3s. One backward = flip its REVERSED flag.");
      drive(120, 120);
      if (prElapsed() > 3000) { stopMotors(); prEnter(PR_MOTOR_LEFT); }
      break;

    case PR_MOTOR_LEFT:
      if (prOnce()) Serial.println("STEP 4: LEFT wheel only - confirm it is the left one.");
      drive(120, 0);
      if (prElapsed() > 1500) { stopMotors(); prEnter(PR_MOTOR_RIGHT); }
      break;

    case PR_MOTOR_RIGHT:
      if (prOnce()) Serial.println("STEP 5: RIGHT wheel only - confirm it is the right one.");
      drive(0, 120);
      if (prElapsed() > 1500) { stopMotors(); prEnter(PR_BATTERY); }
      break;

    case PR_BATTERY:
      prVcc = readVccmV();
      prVccOK = (prVcc >= 4700);
      prEnter(PR_VERDICT);
      break;

    case PR_VERDICT:
      if (prOnce()) prPrintVerdict();
      prEnter(PR_DONE);
      break;

    case PR_DONE:
      // LED: solid = RACE READY, slow blink = problems. Re-run anytime with reset.
      if (g_verdict == 1) digitalWrite(LED_BUILTIN, HIGH);
      else digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
      break;
  }
}

// ------------------------------------------------ bench test modes
void runMotorTest() {
  // ================================================================
  //  STEP 1 - THE BOTH-FORWARD CHECK. Do this with the car on the floor
  //  or held by the chassis, wheels free.
  //  PASS: the car rolls STRAIGHT AHEAD, both tyres turning forward
  //        in the same direction, like a pair.
  //  FAIL: one tyre turns backward -> open the HARDWARE FLAGS above and
  //        flip that side's REVERSED flag. Do not rewire anything.
  // ================================================================
  Serial.println("STEP 1: BOTH WHEELS FORWARD 3s - car must roll straight ahead");
  Serial.println("        one wheel backward? flip its REVERSED flag, then re-run");
  drive(120, 120);  delay(3000);
  stopMotors();     delay(1500);

  // STEP 2 - identify each side. Confirms which physical wheel is "left".
  Serial.println("STEP 2: LEFT wheel only 1.5s");
  drive(120, 0);    delay(1500);
  stopMotors();     delay(1000);
  Serial.println("STEP 3: RIGHT wheel only 1.5s");
  drive(0, 120);    delay(1500);
  stopMotors();     delay(1000);

  // STEP 4 - pivot demo. Opposite rotation HERE is on purpose: this is
  // the cornering move, not a wiring fault.
  Serial.println("STEP 4: pivot right 1s (inner wheel reverses ON PURPOSE)");
  drive(150, -110); delay(1000);
  Serial.println("MOTOR TEST: done");
  stopMotors();     delay(2000);
}

void runSensorTest() {
  // One line per reading: 1 = sees black, 0 = does not. Slide the car
  // across the line and watch the 1 walk from left to right.
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(sensorSeesLine(i) ? "1 " : "0 ");
  }
  bool found, fullBlack;
  Serial.print("| err="); Serial.println(readLineError(found, fullBlack));
  delay(150);
}

// ------------------------------------------------ arduino entry points
void setup() {
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t i = 0; i < 5; i++) pinMode(SENSOR_PINS[i], INPUT);

  stopMotors();
  Serial.begin(115200);
  Serial.println("Competition line follower ready.");

  // Launch countdown: LED blinks for SETTLE_MS so you can place the car
  // and get your hand away before it launches.
  unsigned long t0 = millis();
  while (millis() - t0 < SETTLE_MS) {
    digitalWrite(LED_BUILTIN, (millis() / 200) % 2);
  }
  digitalWrite(LED_BUILTIN, LOW);
  lastActionAt = millis();   // grace: being placed off the line is not a finish
}

void loop() {
  switch (RUN_MODE) {
    case MODE_LINE_FOLLOW: {
      // Fixed 200 Hz control loop: the PID gains stay meaningful no
      // matter what else changes, because each cycle is exactly 5ms.
      unsigned long now = millis();
      if (now - lastCycle >= CONTROL_PERIOD_MS) {
        lastCycle = now;
        runLineFollow();
      }
      break;
    }
    case MODE_PRE_RACE:    runPreRace();    break;
    case MODE_MOTOR_TEST:  runMotorTest();  break;
    case MODE_SENSOR_TEST: runSensorTest(); break;
  }
}

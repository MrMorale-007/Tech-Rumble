/*
 * ============================================================================
 *  Tech Rumble 2 — Fully Autonomous 3-IR Line Follower
 *  File: TechRumbleRobot/line_follower.ino
 *
 *  STRICT HARDWARE ISOLATION & RULE 4 COMPLIANCE:
 *   - Zero GPIO, LEDC, ADC, or ISR code here. All sensor reads and motor
 *     commands go strictly through hardware.ino HAL functions and global
 *     snapshots (sensors, grip, robot).
 *   - NEVER reads PS4 drive sticks/triggers (sending drive commands during
 *     Line Follower loses the match per Rule 4!).
 *   - 100% non-blocking millis() state machine (called at 50 Hz from loop()).
 *
 *  ARENA FEATURES ENGINEERED FOR (see RULES_CHECKLIST.md & Arena Guide p. 3):
 *   - Board: 120 x 200 cm matte white (elevated! Ring-Out = instant loss),
 *            25 mm black line (386.1 cm total), mirrored Lane A / Lane B.
 *   - 90 deg sharp corner & 2 x 45 deg zig-zag: dynamic curve slowdown +
 *     lastLineDir pivot recovery without whipping the 20 g Sudden Death load.
 *   - 5 cm line gap: encoder-measured straight coast bridge (~85 mm max)
 *     when line disappears from straight tracking before triggering recovery.
 *   - 24 x 2.5 cm End Bars (both ends): debounced 111 detection after
 *     start-bar clearance (> 150 mm):
 *       * STAGE_KNOCKOUT: pulls cleanly into Changing Mode Zone, brakes,
 *                         and hands off to MODE_IDLE_SAFE.
 *       * STAGE_GROUP:    brakes cleanly at far/home End Bar and hands off
 *                         to MODE_IDLE_SAFE (two-way Loading Zone / deposit).
 * ============================================================================
 */

// ============================================================================
//  EASILY TUNABLE PID, SPEED & ARENA CONSTANTS
// ============================================================================

// PID Gains (weighted 3-IR error range: -2.0 .. +2.0)
static const float    LF_KP                    = 52.0f;   // Proportional steering gain
static const float    LF_KI                    = 0.8f;    // Integral gain (clamped anti-windup)
static const float    LF_KD                    = 14.0f;   // Derivative damping (tames 45 deg zig-zag)
static const float    LF_INTEGRAL_MAX          = 25.0f;   // Anti-windup integral clamp

// Speed Caps & Slew-Rate (Sudden Death 20 g load stability — Rule 3.2!)
static const int      LF_BASE_PWM              = 135;     // Nominal straightaway cruise PWM
static const int      LF_MIN_CRUISE_PWM        = 85;      // Minimum forward base PWM on tight turns
static const int      LF_MAX_PWM               = 175;     // Hard speed ceiling (protects 20 g load)
static const int      LF_CURVE_SLOWDOWN_PER_ERR = 20;     // Base PWM reduction per unit |error|
static const int      LF_GAP_BRIDGE_PWM        = 115;     // Steady straight PWM across 5 cm gap
static const int      LF_PIVOT_PWM             = 125;     // Controlled in-place pivot for 90/45 deg
static const int      LF_CMZ_PULLIN_PWM        = 105;     // Gentle creep into Knockout CMZ
static const int      LF_SLEW_STEP             = 16;      // Gentle ramp (PWM per 10 ms) for 20 g load

// Arena Geometry & Timing Thresholds (mm / ms)
static const uint32_t LF_PID_INTERVAL_MS       = 20UL;    // 50 Hz PID control update
static const float    LF_START_IGNORE_MM       = 150.0f;  // Ignore start End Bar for first 15 cm
static const uint32_t LF_END_BAR_DEBOUNCE_MS   = 45UL;    // Continuous 111 duration for real 24 cm bar
static const float    LF_GAP_BRIDGE_MAX_MM     = 85.0f;   // Covers 50 mm line gap + sensor margin
static const float    LF_CMZ_PULLIN_MM         = 120.0f;  // Pull into 30x40 cm Knockout CMZ past bar
static const uint32_t LF_LOST_LINE_TIMEOUT_MS  = 1800UL;  // Ring-Out fail-safe if line not reacquired
static const uint32_t LF_MAX_RUN_TIMEOUT_MS    = 180000UL;// 3 min run safety cap (< 5 min match limit)

// ============================================================================
//  INTERNAL LINE FOLLOWER STATE MACHINE
// ============================================================================

enum LfSubState : uint8_t {
  LF_STATE_FOLLOW       = 0,  // Normal 3-IR weighted PID line tracking
  LF_STATE_GAP_BRIDGE   = 1,  // Straight encoder-measured crossing of 5 cm line gap
  LF_STATE_RECOVER_TURN = 2,  // Sharp 90 deg / 45 deg corner pivot using lastLineDir
  LF_STATE_CMZ_PULLIN   = 3   // Knockout Stage: brief straight pull-in into CMZ after End Bar
};

static LfSubState s_lfState            = LF_STATE_FOLLOW;
static uint32_t   s_lfStartMs          = 0;
static uint32_t   s_lfLastPidMs        = 0;
static uint32_t   s_lfAllBlackStartMs  = 0;
static uint32_t   s_lfRecoverStartMs   = 0;

static float      s_lfPrevError        = 0.0f;
static float      s_lfIntegral         = 0.0f;
static float      s_lfLastValidError   = 0.0f;
static int8_t     s_lfLastLineDir      = 1;     // -1 = line last seen Left, +1 = Right
static bool       s_lfClearedStartBar  = false;

static float      s_lfGapStartMm       = 0.0f;
static float      s_lfCmzStartMm       = 0.0f;

// ============================================================================
//  INTERNAL HELPERS
// ============================================================================

static float getAverageTravelMm() {
  // Average unsigned encoder distance across both sides (robust to turns)
  const float mmL = encoderTicksToMm((int32_t)sensors.encAbsLeft);
  const float mmR = encoderTicksToMm((int32_t)sensors.encAbsRight);
  return 0.5f * (mmL + mmR);
}

static bool computeWeightedLineError(uint8_t mask, float &outErr) {
  // bit2 = Left, bit1 = Center, bit0 = Right (1 = black line)
  switch (mask) {
    case 0b010: outErr =  0.0f; return true;  // Centered
    case 0b110: outErr = -1.0f; return true;  // Slightly left
    case 0b100: outErr = -2.0f; return true;  // Hard left (90 deg / 45 deg entry)
    case 0b011: outErr =  1.0f; return true;  // Slightly right
    case 0b001: outErr =  2.0f; return true;  // Hard right (90 deg / 45 deg entry)
    case 0b101: outErr =  s_lfPrevError; return true; // Rare split/noise: hold last heading
    default:    return false;                 // 0b000 (gap/lost) or 0b111 (junction/end bar)
  }
}

static void runPidTracking(float error, uint32_t nowMs) {
  const uint32_t dtMs = nowMs - s_lfLastPidMs;
  if (dtMs < LF_PID_INTERVAL_MS) {
    return;
  }
  s_lfLastPidMs = nowMs;

  float dtSec = (float)dtMs * 0.001f;
  if (dtSec < 0.005f) dtSec = 0.005f;
  if (dtSec > 0.100f) dtSec = 0.100f;

  // Reset integral on center reacquisition or sign flip (prevents corner overshoot)
  if (error == 0.0f || (error * s_lfPrevError < 0.0f)) {
    s_lfIntegral = 0.0f;
  } else {
    s_lfIntegral += error * dtSec;
    if (s_lfIntegral >  LF_INTEGRAL_MAX) s_lfIntegral =  LF_INTEGRAL_MAX;
    if (s_lfIntegral < -LF_INTEGRAL_MAX) s_lfIntegral = -LF_INTEGRAL_MAX;
  }

  const float derivative = (error - s_lfPrevError) / dtSec;
  const float correction = (LF_KP * error) + (LF_KI * s_lfIntegral) + (LF_KD * derivative);

  s_lfPrevError      = error;
  s_lfLastValidError = error;

  if (error < -0.1f) {
    s_lfLastLineDir = -1;
  } else if (error > 0.1f) {
    s_lfLastLineDir = 1;
  }

  // Slow down smoothly on curves & zig-zags to protect 20 g Sudden Death load
  int basePwm = LF_BASE_PWM - (int)(fabsf(error) * (float)LF_CURVE_SLOWDOWN_PER_ERR);
  if (basePwm < LF_MIN_CRUISE_PWM) basePwm = LF_MIN_CRUISE_PWM;

  int leftPwm  = constrain(basePwm + (int)correction, -LF_MAX_PWM, LF_MAX_PWM);
  int rightPwm = constrain(basePwm - (int)correction, -LF_MAX_PWM, LF_MAX_PWM);
  setMotorTarget(leftPwm, rightPwm);
}

static void finishLineFollowerSafe(uint32_t nowMs, const char* reason) {
  brakeMotorsImmediate();
  Serial.print(F("[LF] Finished -> "));
  Serial.println(reason);
  enterMode(MODE_IDLE_SAFE, nowMs, reason);
}

// ============================================================================
//  PUBLIC LINE FOLLOWER ENTRY POINTS (Called by TechRumbleRobot.ino)
// ============================================================================

void initLineFollowerMode(uint32_t nowMs) {
  s_lfState           = LF_STATE_FOLLOW;
  s_lfStartMs         = nowMs;
  s_lfLastPidMs       = nowMs;
  s_lfAllBlackStartMs = 0;
  s_lfRecoverStartMs  = 0;

  s_lfPrevError       = 0.0f;
  s_lfIntegral        = 0.0f;
  s_lfLastValidError  = 0.0f;
  // Default recovery turn hint matches mirrored track geometry (Lane A vs B)
  s_lfLastLineDir     = (robot.lane == LANE_A) ? -1 : 1;
  s_lfClearedStartBar = false;

  s_lfGapStartMm      = 0.0f;
  s_lfCmzStartMm      = 0.0f;

  resetEncoders();
  setMotorSlewStep(LF_SLEW_STEP);

  // If carrying the 20 g load, ensure Sudden Death Grip-Lock remains active
  if (sensors.gripSwitchPressed) {
    setGripLock(true);
  }

  Serial.print(F("[LF] Autonomous Line Follower STARTED | Lane: "));
  Serial.print(robot.lane == LANE_A ? F("LANE_A") : F("LANE_B"));
  Serial.print(F(" | Stage: "));
  Serial.println(robot.stage == STAGE_GROUP ? F("GROUP (2-way)") : F("KNOCKOUT (CMZ)"));
}

void runLineFollower(uint32_t nowMs) {
  // Mode exit guard: never drive if mode changed externally
  if (robot.mode != MODE_LINE_FOLLOWER) {
    stopMotorsImmediate();
    return;
  }

  // Overall run timeout guard (well within 5 min Group / 10 min Knockout clock)
  if (nowMs - s_lfStartMs >= LF_MAX_RUN_TIMEOUT_MS) {
    finishLineFollowerSafe(nowMs, "LF_MAX_RUN_TIMEOUT");
    return;
  }

  const uint8_t mask     = sensors.irMask;
  const float   travelMm = getAverageTravelMm();

  // Wait until we have left the start End Bar before enabling End-Bar stop
  if (!s_lfClearedStartBar && travelMm >= LF_START_IGNORE_MM && mask != 0b111) {
    s_lfClearedStartBar = true;
  }

  // --- Sub-State: Knockout Stage Changing Mode Zone Pull-In ---
  if (s_lfState == LF_STATE_CMZ_PULLIN) {
    if (travelMm - s_lfCmzStartMm >= LF_CMZ_PULLIN_MM) {
      finishLineFollowerSafe(nowMs, "LF_KNOCKOUT_CMZ_ARRIVED");
    } else {
      setMotorTarget(LF_CMZ_PULLIN_PWM, LF_CMZ_PULLIN_PWM);
    }
    return;
  }

  // --- Junction / 24 cm End Bar Detection (111) ---
  if (mask == 0b111) {
    if (s_lfClearedStartBar) {
      if (s_lfAllBlackStartMs == 0) {
        s_lfAllBlackStartMs = nowMs;
      } else if (nowMs - s_lfAllBlackStartMs >= LF_END_BAR_DEBOUNCE_MS) {
        // Confirmed 24 x 2.5 cm End Bar!
        if (robot.stage == STAGE_KNOCKOUT) {
          s_lfState      = LF_STATE_CMZ_PULLIN;
          s_lfCmzStartMm = travelMm;
          Serial.println(F("[LF] Knockout End Bar detected -> pulling into CMZ..."));
          setMotorTarget(LF_CMZ_PULLIN_PWM, LF_CMZ_PULLIN_PWM);
        } else {
          // Group Stage: stop cleanly at far Loading Zone or home Starting Point bar
          finishLineFollowerSafe(nowMs, "LF_GROUP_END_BAR_STOP");
        }
        return;
      }
    }
    // While starting on the start bar or debouncing a brief 111 glance, drive straight
    setMotorTarget(LF_GAP_BRIDGE_PWM, LF_GAP_BRIDGE_PWM);
    return;
  } else {
    s_lfAllBlackStartMs = 0;
  }

  // --- Line Visible (any valid 1- or 2-sensor pattern) ---
  float err = 0.0f;
  if (computeWeightedLineError(mask, err)) {
    if (s_lfState != LF_STATE_FOLLOW) {
      // Cleanly reacquired line after gap or sharp turn: zero I & D kick!
      s_lfState          = LF_STATE_FOLLOW;
      s_lfIntegral       = 0.0f;
      s_lfPrevError      = err;
      s_lfLastPidMs      = nowMs - LF_PID_INTERVAL_MS;
      s_lfRecoverStartMs = 0;
    }
    runPidTracking(err, nowMs);
    return;
  }

  // --- All White (000): 5 cm Gap Bridge vs Sharp 90/45 deg Recovery ---
  if (s_lfState == LF_STATE_FOLLOW) {
    // If we were tracking straight (|lastValidError| <= 0.5), assume 5 cm line gap first!
    if (fabsf(s_lfLastValidError) <= 0.5f) {
      s_lfState      = LF_STATE_GAP_BRIDGE;
      s_lfGapStartMm = travelMm;
      setMotorTarget(LF_GAP_BRIDGE_PWM, LF_GAP_BRIDGE_PWM);
      return;
    }
    // Otherwise we exited off a sharp 90 deg corner or 45 deg zig-zag -> pivot immediately
    s_lfState          = LF_STATE_RECOVER_TURN;
    s_lfRecoverStartMs = nowMs;
  }

  if (s_lfState == LF_STATE_GAP_BRIDGE) {
    if (travelMm - s_lfGapStartMm < LF_GAP_BRIDGE_MAX_MM) {
      setMotorTarget(LF_GAP_BRIDGE_PWM, LF_GAP_BRIDGE_PWM);
      return;
    }
    // Exceeded 85 mm without finding line -> transition to lost-line pivot recovery
    s_lfState          = LF_STATE_RECOVER_TURN;
    s_lfRecoverStartMs = nowMs;
    Serial.println(F("[LF] Gap bridge exceeded 85 mm -> entering pivot recovery."));
  }

  // --- Sub-State: Sharp Corner / Lost-Line Pivot Recovery + Ring-Out Fail-Safe ---
  if (s_lfState == LF_STATE_RECOVER_TURN) {
    if (nowMs - s_lfRecoverStartMs >= LF_LOST_LINE_TIMEOUT_MS) {
      // Stop immediately so robot never drives off the elevated 120x200 cm board!
      finishLineFollowerSafe(nowMs, "LF_LOST_LINE_RINGOUT_FAILSAFE");
      return;
    }
    // Controlled pivot in last known line direction
    const int pivotL = (int)s_lfLastLineDir * LF_PIVOT_PWM;
    const int pivotR = -(int)s_lfLastLineDir * LF_PIVOT_PWM;
    setMotorTarget(pivotL, pivotR);
  }
}

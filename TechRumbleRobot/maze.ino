/*
 * ============================================================================
 *  Tech Rumble 2 — Autonomous Maze Solver (Learn & Replay via ESP32 NVS)
 *  File: TechRumbleRobot/maze.ino
 *
 *  STRICT HARDWARE ISOLATION & RULE 4 COMPLIANCE:
 *   - Zero GPIO, LEDC, ADC, or ISR code here. All sensor reads and motor
 *     commands go strictly through hardware.ino HAL functions and global
 *     snapshots (sensors, grip, robot).
 *   - NEVER reads PS4 drive sticks/triggers (sending drive commands during
 *     autonomous Maze loses the match per Rule 4!).
 *   - 100% non-blocking millis() state machine (called at 50 Hz from loop()).
 *
 *  ARENA GEOMETRY & FEATURES (see RULES_CHECKLIST.md & Arena Guide p. 2):
 *   - Maze footprint: ~335 x 350 cm, walls 20 cm high, corridors 35 cm clear
 *     (~5 cm clearance per side for a 25 cm robot -> tight ultrasonic P-centering).
 *   - Mirrored tracks: LANE_A uses Left-Hand Rule (L-S-R-B), LANE_B mirrors to
 *     Right-Hand Rule (R-S-L-B). Separate NVS keys ("path_a" / "path_b").
 *   - Dead-End Shortest-Path Optimization (applied iteratively on Learn run):
 *       LBR -> B | LBS -> R | RBL -> B | SBL -> R | SBS -> B | LBL -> S
 *       (plus mirrored Right-Hand rules: RBS -> L | SBR -> L | RBR -> S)
 *   - ESP32 Preferences (NVS): Saves simplified optimal path on goal/exit
 *     arrival so subsequent runs execute REPLAY directly.
 *   - Goal / Loading Zone / Exit Detection: Sustained open-zone detection
 *     (all 3 ultrasonics open) after minimum travel -> brakes, saves NVS
 *     path (if Learning), and hands off cleanly to MODE_IDLE_SAFE.
 * ============================================================================
 */

// ============================================================================
//  EASILY TUNABLE MAZE GEOMETRY, WALL-CENTERING & ENCODER CONSTANTS
// ============================================================================

// Ultrasonic thresholds (cm) tuned for 35 cm corridors & 20 cm high walls
static const float    MAZE_WALL_PRESENT_CM      = 24.0f;   // < 24 cm = wall present on side
static const float    MAZE_SINGLE_WALL_TGT_CM   = 9.5f;    // Target clearance from single wall
static const float    MAZE_FRONT_SLOW_CM        = 26.0f;   // Begin slowing for front wall
static const float    MAZE_FRONT_STOP_CM        = 14.0f;   // Stop before front scoop touches wall
static const float    MAZE_GOAL_OPEN_CM         = 42.0f;   // Wide open on F, L, R = Goal/LZ/Exit

// Wall-centering P-control & speed limits (gentle for 20 g Sudden Death load!)
static const float    MAZE_WALL_KP              = 5.5f;    // Steering PWM per cm centering error
static const int      MAZE_MAX_STEER_CORR       = 45;      // Cap steering correction PWM
static const int      MAZE_CRUISE_PWM           = 125;     // Smooth forward corridor speed
static const int      MAZE_SLOW_PWM             = 85;      // Approach speed near front wall
static const int      MAZE_MAX_PWM              = 160;     // Hard PWM cap (Sudden Death safe)
static const int      MAZE_TURN_PWM             = 120;     // Calibrated in-place pivot PWM
static const int      MAZE_SLEW_STEP            = 14;      // Smooth acceleration ramp (PWM/10ms)

// Wheel encoder calibration (68 mm CAD wheel OD, 20-slot disc -> ~10.68 mm/tick)
static const float    MAZE_CELL_MM              = 350.0f;  // Nominal corridor cell step (35 cm)
static const float    MAZE_JUNCTION_PULLIN_MM   = 80.0f;   // Center robot in opening before 90 deg
static const float    MAZE_POST_TURN_CLEAR_MM   = 180.0f;  // Clear intersection after 90/180 turn
static const float    MAZE_POST_STRAIGHT_CLR_MM = 260.0f;  // Clear remaining 35 cm opening after 'S'
static const float    MAZE_MIN_GOAL_DIST_MM     = 500.0f;  // Ignore open start area for first 50 cm
static const uint32_t MAZE_TURN_90_TICKS        = 15UL;    // Bench-tune for exact 90 deg pivot
static const uint32_t MAZE_TURN_180_TICKS       = 30UL;    // Bench-tune for exact 180 deg U-turn

// Non-blocking timers & rulebook safety timeouts (ms)
static const uint32_t MAZE_SETTLE_MS            = 140UL;   // Brake settle for fresh ultrasonic pings
static const uint32_t MAZE_GOAL_CONFIRM_MS      = 240UL;   // Sustained open space confirms goal/exit
static const uint32_t MAZE_TURN_TIMEOUT_MS      = 2200UL;  // Guard against stalled turn/encoder
static const uint32_t MAZE_STUCK_TIMEOUT_MS     = 2500UL;  // Anti-wall-wedge motor protection
static const uint32_t MAZE_MAX_RUN_TIMEOUT_MS   = 240000UL;// 4 min ceiling (< 5 min Group limit)

// Path buffer & ESP32 NVS namespace
static const uint8_t  MAZE_MAX_PATH_LEN         = 128;
static const char*    MAZE_NVS_NAMESPACE        = "tr2_maze";

// ============================================================================
//  INTERNAL MAZE STATE MACHINE & LEARN/REPLAY BUFFERS
// ============================================================================

enum MazeSubState : uint8_t {
  MAZE_STATE_ADVANCE     = 0,  // Wall-centered forward travel across corridor / cell
  MAZE_STATE_PULLIN      = 1,  // Brief pull-in to center wheelbase at side opening
  MAZE_STATE_SETTLE      = 2,  // Short brake settle for fresh ultrasonic readings
  MAZE_STATE_DECIDE      = 3,  // Choose L / S / R / B (Learn rule or Replay next step)
  MAZE_STATE_TURN        = 4   // Non-blocking encoder-counted 90 deg / 180 deg pivot
};

static Preferences  s_mazePrefs;
static MazeSubState s_mazeState          = MAZE_STATE_ADVANCE;
static MazeSubState s_mazeAfterSettle    = MAZE_STATE_DECIDE;

static bool         s_mazeReplayMode     = false; // true = execute saved NVS path, false = Learn
static char         s_mazePath[MAZE_MAX_PATH_LEN + 1];
static uint8_t      s_mazePathLen        = 0;
static uint8_t      s_mazeReplayIdx      = 0;

static uint32_t     s_mazeStartMs        = 0;
static uint32_t     s_mazePhaseStartMs   = 0;
static uint32_t     s_mazeGoalOpenMs     = 0;
static uint32_t     s_mazeLastMoveMs     = 0;
static uint32_t     s_mazeLastTickSum    = 0;

static float        s_mazeTotalDistMm    = 0.0f;
static float        s_mazeSegmentStartMm = 0.0f;
static float        s_mazePullinStartMm  = 0.0f;
static float        s_mazeClearTargetMm  = MAZE_POST_TURN_CLEAR_MM;
static bool         s_mazeJustTurned     = false;

static int8_t       s_mazeTurnDir        = 1;     // -1 = Left CCW, +1 = Right CW
static uint32_t     s_mazeTurnTargetTicks = 0;

// ============================================================================
//  INTERNAL HELPERS — NVS PERSISTENCE & PATH SIMPLIFICATION
// ============================================================================

static const char* mazePathKeyForLane(LaneSide lane) {
  return (lane == LANE_A) ? "path_a" : "path_b";
}

static const char* mazeValidKeyForLane(LaneSide lane) {
  return (lane == LANE_A) ? "valid_a" : "valid_b";
}

static bool loadMazePathFromNvs(LaneSide lane) {
  s_mazePathLen   = 0;
  s_mazePath[0]   = '\0';
  s_mazeReplayIdx = 0;

  if (!s_mazePrefs.begin(MAZE_NVS_NAMESPACE, true)) {
    return false;
  }
  const bool valid = s_mazePrefs.getBool(mazeValidKeyForLane(lane), false);
  String stored    = valid ? s_mazePrefs.getString(mazePathKeyForLane(lane), "") : String("");
  s_mazePrefs.end();

  if (!valid || stored.length() == 0 || stored.length() > MAZE_MAX_PATH_LEN) {
    return false;
  }

  s_mazePathLen = (uint8_t)stored.length();
  for (uint8_t i = 0; i < s_mazePathLen; i++) {
    const char c = stored[i];
    if (c != 'L' && c != 'R' && c != 'S' && c != 'B') {
      s_mazePathLen = 0;
      s_mazePath[0] = '\0';
      return false;
    }
    s_mazePath[i] = c;
  }
  s_mazePath[s_mazePathLen] = '\0';
  return true;
}

static void saveMazePathToNvs(LaneSide lane) {
  if (s_mazePathLen == 0) {
    return;
  }
  s_mazePath[s_mazePathLen] = '\0';
  if (!s_mazePrefs.begin(MAZE_NVS_NAMESPACE, false)) {
    Serial.println(F("[MAZE] NVS open failed — could not save shortest path!"));
    return;
  }
  s_mazePrefs.putString(mazePathKeyForLane(lane), String(s_mazePath));
  s_mazePrefs.putBool(mazeValidKeyForLane(lane), true);
  s_mazePrefs.end();

  Serial.print(F("[MAZE] Saved shortest path to NVS ("));
  Serial.print(lane == LANE_A ? F("LANE_A") : F("LANE_B"));
  Serial.print(F(", len="));
  Serial.print(s_mazePathLen);
  Serial.print(F("): "));
  Serial.println(s_mazePath);
}

static uint8_t turnCharToQuarterSteps(char c) {
  switch (c) {
    case 'S': return 0;
    case 'R': return 1;
    case 'B': return 2;
    case 'L': return 3;
    default:  return 0;
  }
}

static char quarterStepsToTurnChar(uint8_t steps) {
  switch (steps & 0x03U) {
    case 0: return 'S';
    case 1: return 'R';
    case 2: return 'B';
    case 3: return 'L';
    default: return 'S';
  }
}

static void simplifyMazePath() {
  /*
   * Iteratively collapses every dead-end backtrack triplet [X, 'B', Y]:
   *   LBR -> B | LBS -> R | RBL -> B | SBL -> R | SBS -> B | LBL -> S
   *   (and mirrored Right-Hand triplets: RBS -> L | SBR -> L | RBR -> S)
   * Using exact modular quarter-turn arithmetic: (dir(X) + 2 + dir(Y)) % 4.
   */
  while (s_mazePathLen >= 3 && s_mazePath[s_mazePathLen - 2] == 'B') {
    const uint8_t a = turnCharToQuarterSteps(s_mazePath[s_mazePathLen - 3]);
    const uint8_t b = turnCharToQuarterSteps(s_mazePath[s_mazePathLen - 1]);
    const char reduced = quarterStepsToTurnChar((uint8_t)((a + 2U + b) & 0x03U));
    s_mazePath[s_mazePathLen - 3] = reduced;
    s_mazePathLen -= 2;
    s_mazePath[s_mazePathLen] = '\0';
  }
}

static void recordMazeDecision(char moveChar) {
  if (s_mazeReplayMode) {
    return;
  }
  if (s_mazePathLen < MAZE_MAX_PATH_LEN) {
    s_mazePath[s_mazePathLen++] = moveChar;
    s_mazePath[s_mazePathLen]   = '\0';
    simplifyMazePath();
  }
}

// ============================================================================
//  INTERNAL HELPERS — SENSORS, WALL-CENTERING & TURN EXECUTION
// ============================================================================

static float getMazeEncoderAvgMm() {
  const float mmL = encoderTicksToMm((int32_t)sensors.encAbsLeft);
  const float mmR = encoderTicksToMm((int32_t)sensors.encAbsRight);
  return 0.5f * (mmL + mmR);
}

static bool isFrontOpen() {
  return (!sensors.usFrontValid || sensors.usFrontCm > MAZE_FRONT_STOP_CM + 8.0f);
}

static bool isLeftOpen() {
  return (!sensors.usLeftValid || sensors.usLeftCm >= MAZE_WALL_PRESENT_CM);
}

static bool isRightOpen() {
  return (!sensors.usRightValid || sensors.usRightCm >= MAZE_WALL_PRESENT_CM);
}

static bool isAllSidesWideOpen() {
  const bool fOpen = (!sensors.usFrontValid || sensors.usFrontCm >= MAZE_GOAL_OPEN_CM);
  const bool lOpen = (!sensors.usLeftValid  || sensors.usLeftCm  >= MAZE_GOAL_OPEN_CM);
  const bool rOpen = (!sensors.usRightValid || sensors.usRightCm >= MAZE_GOAL_OPEN_CM);
  return (fOpen && lOpen && rOpen);
}

static void startMazeSettle(uint32_t nowMs, MazeSubState nextState) {
  brakeMotorsImmediate();
  s_mazeState       = MAZE_STATE_SETTLE;
  s_mazeAfterSettle = nextState;
  s_mazePhaseStartMs = nowMs;
}

static void startMazeTurn(char moveChar, uint32_t nowMs) {
  if (moveChar == 'S') {
    // Straight through junction: clear the full 35 cm opening before next decision
    s_mazeSegmentStartMm = getMazeEncoderAvgMm();
    s_mazeClearTargetMm  = MAZE_POST_STRAIGHT_CLR_MM;
    s_mazeJustTurned     = true;
    s_mazeState          = MAZE_STATE_ADVANCE;
    s_mazeLastMoveMs     = nowMs;
    s_mazeLastTickSum    = sensors.encAbsLeft + sensors.encAbsRight;
    return;
  }

  if (moveChar == 'L') {
    s_mazeTurnDir         = -1;
    s_mazeTurnTargetTicks = MAZE_TURN_90_TICKS;
  } else if (moveChar == 'R') {
    s_mazeTurnDir         = 1;
    s_mazeTurnTargetTicks = MAZE_TURN_90_TICKS;
  } else {
    // 'B' = 180 deg U-turn (turn toward whichever side has slightly more room)
    s_mazeTurnDir         = (sensors.usLeftCm > sensors.usRightCm) ? -1 : 1;
    s_mazeTurnTargetTicks = MAZE_TURN_180_TICKS;
  }

  // Bank total distance before resetting segment encoders for exact turn ticks
  s_mazeTotalDistMm += getMazeEncoderAvgMm();
  resetEncoders();
  s_mazeSegmentStartMm = 0.0f;
  s_mazePhaseStartMs   = nowMs;
  s_mazeState          = MAZE_STATE_TURN;

  const int turnL = (int)s_mazeTurnDir * MAZE_TURN_PWM;
  const int turnR = -(int)s_mazeTurnDir * MAZE_TURN_PWM;
  setMotorTarget(turnL, turnR);
}

static void driveCorridorWallCentered() {
  // Smooth front-wall approach deceleration (avoids wall touch penalties!)
  int basePwm = MAZE_CRUISE_PWM;
  if (sensors.usFrontValid && sensors.usFrontCm < MAZE_FRONT_SLOW_CM) {
    const float span = MAZE_FRONT_SLOW_CM - MAZE_FRONT_STOP_CM;
    float ratio = (sensors.usFrontCm - MAZE_FRONT_STOP_CM) / span;
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    basePwm = MAZE_SLOW_PWM + (int)(ratio * (float)(MAZE_CRUISE_PWM - MAZE_SLOW_PWM));
  }

  const bool hasLeft  = (sensors.usLeftValid  && sensors.usLeftCm  < MAZE_WALL_PRESENT_CM);
  const bool hasRight = (sensors.usRightValid && sensors.usRightCm < MAZE_WALL_PRESENT_CM);

  // Wall-centering P-control:
  // Positive centerErr -> robot closer to right wall -> steer LEFT (negative corr)
  float centerErr = 0.0f;
  if (hasLeft && hasRight) {
    centerErr = sensors.usLeftCm - sensors.usRightCm;
  } else if (hasLeft) {
    centerErr = sensors.usLeftCm - MAZE_SINGLE_WALL_TGT_CM;
  } else if (hasRight) {
    centerErr = MAZE_SINGLE_WALL_TGT_CM - sensors.usRightCm;
  }

  int corr = (int)(-MAZE_WALL_KP * centerErr);
  corr = constrain(corr, -MAZE_MAX_STEER_CORR, MAZE_MAX_STEER_CORR);

  const int leftPwm  = constrain(basePwm + corr, -MAZE_MAX_PWM, MAZE_MAX_PWM);
  const int rightPwm = constrain(basePwm - corr, -MAZE_MAX_PWM, MAZE_MAX_PWM);
  setMotorTarget(leftPwm, rightPwm);
}

static void finishMazeSafe(uint32_t nowMs, bool saveIfLearned, const char* reason) {
  brakeMotorsImmediate();
  if (saveIfLearned && !s_mazeReplayMode) {
    saveMazePathToNvs(robot.lane);
  }
  Serial.print(F("[MAZE] Finished -> "));
  Serial.println(reason);
  enterMode(MODE_IDLE_SAFE, nowMs, reason);
}

// ============================================================================
//  PUBLIC MAZE ENTRY POINTS & NVS BENCH HELPERS
// ============================================================================

void clearSavedMazePaths() {
  if (!s_mazePrefs.begin(MAZE_NVS_NAMESPACE, false)) {
    Serial.println(F("[MAZE] NVS open failed — cannot clear paths."));
    return;
  }
  s_mazePrefs.clear();
  s_mazePrefs.end();
  s_mazePathLen   = 0;
  s_mazePath[0]   = '\0';
  s_mazeReplayIdx = 0;
  Serial.println(F("[MAZE] Cleared all saved NVS Learn/Replay maze paths."));
}

void printSavedMazePaths() {
  if (!s_mazePrefs.begin(MAZE_NVS_NAMESPACE, true)) {
    Serial.println(F("[MAZE] NVS not initialized yet."));
    return;
  }
  const bool   vA = s_mazePrefs.getBool("valid_a", false);
  const String pA = vA ? s_mazePrefs.getString("path_a", "") : String("<empty>");
  const bool   vB = s_mazePrefs.getBool("valid_b", false);
  const String pB = vB ? s_mazePrefs.getString("path_b", "") : String("<empty>");
  s_mazePrefs.end();

  Serial.print(F("[MAZE] NVS LANE_A: ")); Serial.println(pA);
  Serial.print(F("[MAZE] NVS LANE_B: ")); Serial.println(pB);
}

void initMazeMode(uint32_t nowMs) {
  s_mazeState          = MAZE_STATE_ADVANCE;
  s_mazeAfterSettle    = MAZE_STATE_DECIDE;
  s_mazeStartMs        = nowMs;
  s_mazePhaseStartMs   = nowMs;
  s_mazeGoalOpenMs     = 0;
  s_mazeLastMoveMs     = nowMs;
  s_mazeLastTickSum    = 0;

  s_mazeTotalDistMm    = 0.0f;
  s_mazeSegmentStartMm = 0.0f;
  s_mazePullinStartMm  = 0.0f;
  s_mazeJustTurned     = false;

  resetEncoders();
  setMotorSlewStep(MAZE_SLEW_STEP);

  // Preserve Sudden Death Grip-Lock if carrying the 20 g load out of Loading Zone
  if (sensors.gripSwitchPressed) {
    setGripLock(true);
  }

  // Automatically select REPLAY if an optimized path exists in NVS for this lane
  s_mazeReplayMode = loadMazePathFromNvs(robot.lane);

  Serial.print(F("[MAZE] STARTED | Mode: "));
  Serial.print(s_mazeReplayMode ? F("REPLAY (NVS)") : F("LEARN (Wall-Follow)"));
  Serial.print(F(" | Lane: "));
  Serial.print(robot.lane == LANE_A ? F("LANE_A (Left-Hand)") : F("LANE_B (Right-Hand)"));
  if (s_mazeReplayMode) {
    Serial.print(F(" | Path: "));
    Serial.println(s_mazePath);
  } else {
    Serial.println();
  }
}

void runMaze(uint32_t nowMs) {
  // Mode exit safety guard: never drive if mode changed externally
  if (robot.mode != MODE_MAZE) {
    stopMotorsImmediate();
    return;
  }

  // Rule 5 overall match safety timeout (< 5 min Group Stage / 10 min Knockout)
  if (nowMs - s_mazeStartMs >= MAZE_MAX_RUN_TIMEOUT_MS) {
    finishMazeSafe(nowMs, false, "MAZE_MAX_RUN_TIMEOUT");
    return;
  }

  const float segDistMm = getMazeEncoderAvgMm() - s_mazeSegmentStartMm;
  const float cumDistMm = s_mazeTotalDistMm + getMazeEncoderAvgMm();

  // --- Goal / Loading Zone / Exit Detection (sustained open space on F, L, R) ---
  if (cumDistMm >= MAZE_MIN_GOAL_DIST_MM &&
      (s_mazeState == MAZE_STATE_ADVANCE || s_mazeState == MAZE_STATE_PULLIN)) {
    if (isAllSidesWideOpen()) {
      if (s_mazeGoalOpenMs == 0) {
        s_mazeGoalOpenMs = nowMs;
      } else if (nowMs - s_mazeGoalOpenMs >= MAZE_GOAL_CONFIRM_MS) {
        finishMazeSafe(nowMs, true, "MAZE_GOAL_OR_EXIT_REACHED");
        return;
      }
    } else {
      s_mazeGoalOpenMs = 0;
    }
  } else {
    s_mazeGoalOpenMs = 0;
  }

  // --- Non-Blocking State Machine ---
  switch (s_mazeState) {
    case MAZE_STATE_ADVANCE: {
      // Anti-stuck watchdog (protects motors if wedged against a maze wall)
      const uint32_t tickSum = sensors.encAbsLeft + sensors.encAbsRight;
      if (tickSum >= s_mazeLastTickSum + 2UL) {
        s_mazeLastTickSum = tickSum;
        s_mazeLastMoveMs  = nowMs;
      } else if (nowMs - s_mazeLastMoveMs >= MAZE_STUCK_TIMEOUT_MS) {
        finishMazeSafe(nowMs, false, "MAZE_STUCK_FAILSAFE");
        return;
      }

      // 1. Front wall reached -> stop immediately before scoop touches wall!
      if (sensors.usFrontValid && sensors.usFrontCm <= MAZE_FRONT_STOP_CM) {
        startMazeSettle(nowMs, MAZE_STATE_DECIDE);
        return;
      }

      // Clear post-turn / post-straight lockout once past the current 35 cm opening
      if (s_mazeJustTurned && segDistMm >= s_mazeClearTargetMm) {
        s_mazeJustTurned = false;
      }

      // 2. Side opening / junction detection — IDENTICAL geometry trigger in both
      //    Learn and Replay so every recorded node matches 1-to-1 on Replay!
      if (!s_mazeJustTurned) {
        const bool anySideOpen = (isLeftOpen() || isRightOpen());
        if (anySideOpen) {
          s_mazePullinStartMm = getMazeEncoderAvgMm();
          s_mazeState         = MAZE_STATE_PULLIN;
          driveCorridorWallCentered();
          return;
        }
      }

      // 3. Pure straight walled corridor: advance smoothly with wall-centering P-control
      if (segDistMm >= MAZE_CELL_MM) {
        s_mazeSegmentStartMm = getMazeEncoderAvgMm();
      }
      driveCorridorWallCentered();
      break;
    }

    case MAZE_STATE_PULLIN: {
      // Pull robot center cleanly into the 35 cm intersection before pivoting 90 deg
      if (sensors.usFrontValid && sensors.usFrontCm <= MAZE_FRONT_STOP_CM) {
        startMazeSettle(nowMs, MAZE_STATE_DECIDE);
        return;
      }
      if (getMazeEncoderAvgMm() - s_mazePullinStartMm >= MAZE_JUNCTION_PULLIN_MM) {
        startMazeSettle(nowMs, MAZE_STATE_DECIDE);
        return;
      }
      driveCorridorWallCentered();
      break;
    }

    case MAZE_STATE_SETTLE: {
      brakeMotorsImmediate();
      if (nowMs - s_mazePhaseStartMs >= MAZE_SETTLE_MS) {
        s_mazeState = s_mazeAfterSettle;
      }
      break;
    }

    case MAZE_STATE_DECIDE: {
      const bool fOpen = isFrontOpen();
      const bool lOpen = isLeftOpen();
      const bool rOpen = isRightOpen();

      char moveChar = 'B';

      if (s_mazeReplayMode && s_mazeReplayIdx < s_mazePathLen) {
        // Execute next optimal move from ESP32 NVS shortest path
        moveChar = s_mazePath[s_mazeReplayIdx++];
        // Safety fallback: if NVS replay command points into a solid wall, fall back
        // to live wall-following so the robot never rams a wall!
        const bool blocked = (moveChar == 'S' && !fOpen) ||
                             (moveChar == 'L' && !lOpen) ||
                             (moveChar == 'R' && !rOpen);
        if (blocked) {
          Serial.println(F("[MAZE] REPLAY mismatch vs wall -> falling back to LEARN rule!"));
          s_mazeReplayMode = false;
        }
      }

      if (!s_mazeReplayMode) {
        // Mirrored Wall-Following Priority:
        //  LANE_A: Left-Hand Rule  (L -> S -> R -> B)
        //  LANE_B: Right-Hand Rule (R -> S -> L -> B)
        if (robot.lane == LANE_A) {
          if      (lOpen) moveChar = 'L';
          else if (fOpen) moveChar = 'S';
          else if (rOpen) moveChar = 'R';
          else            moveChar = 'B';
        } else {
          if      (rOpen) moveChar = 'R';
          else if (fOpen) moveChar = 'S';
          else if (lOpen) moveChar = 'L';
          else            moveChar = 'B';
        }

        // Log decision only at real junctions or dead-ends (not forced single corners
        // wait: logging all decisions makes Replay deterministic at every corner/junction!)
        recordMazeDecision(moveChar);
      }

      Serial.print(F("[MAZE] Decide -> "));
      Serial.print(moveChar);
      Serial.print(F(" | US F/L/R: "));
      Serial.print(sensors.usFrontCm, 1); Serial.print('/');
      Serial.print(sensors.usLeftCm,  1); Serial.print('/');
      Serial.println(sensors.usRightCm, 1);

      startMazeTurn(moveChar, nowMs);
      break;
    }

    case MAZE_STATE_TURN: {
      const uint32_t avgTicks = (sensors.encAbsLeft + sensors.encAbsRight) / 2UL;
      if (avgTicks >= s_mazeTurnTargetTicks) {
        // Turn complete: reset segment odometer, brief settle, then advance!
        resetEncoders();
        s_mazeSegmentStartMm = 0.0f;
        s_mazeClearTargetMm  = MAZE_POST_TURN_CLEAR_MM;
        s_mazeJustTurned     = true;
        s_mazeLastMoveMs     = nowMs;
        s_mazeLastTickSum    = 0;
        startMazeSettle(nowMs, MAZE_STATE_ADVANCE);
        return;
      }

      // Guard against stalled turn or disconnected encoder
      if (nowMs - s_mazePhaseStartMs >= MAZE_TURN_TIMEOUT_MS) {
        Serial.println(F("[MAZE] Turn timeout -> braking and re-evaluating."));
        resetEncoders();
        s_mazeSegmentStartMm = 0.0f;
        startMazeSettle(nowMs, MAZE_STATE_DECIDE);
        return;
      }

      const int turnL = (int)s_mazeTurnDir * MAZE_TURN_PWM;
      const int turnR = -(int)s_mazeTurnDir * MAZE_TURN_PWM;
      setMotorTarget(turnL, turnR);
      break;
    }
  }
}

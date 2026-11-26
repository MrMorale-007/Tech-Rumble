/*
 * ============================================================================
 *  Tech Rumble 2 — Autonomous Mini/Mega Sumo Combat Mode
 *  File: TechRumbleRobot/sumo.ino
 *
 *  STRICT HARDWARE ISOLATION:
 *   - Zero GPIO, LEDC, ADC, or ISR code here. All motor, servo, and sensor
 *     access goes strictly through hardware.ino HAL functions and global
 *     snapshots (sensors, grip, robot).
 *   - 100% non-blocking millis() state machine (called at 50 Hz from loop()).
 *
 *  RULEBOOK & SAFETY COMPLIANCE (see RULES_CHECKLIST.md):
 *   - Mandatory Start Countdown: Non-blocking 5-second countdown before first
 *     motion (tunable via SUMO_START_DELAY_MS; prevents false starts, Rule 5).
 *   - Arm Protection: Stows arm cleanly into POSE_COMBAT_TUCK (with Grip-Lock
 *     if holding the 20 g Sudden Death load, suspended strictly off chassis per
 *     Rule 3.1) or POSE_HOME_STOW (when empty, staying <= 25x25x50 cm, Rule 2).
 *   - Dohyo Ring-Edge Avoidance (HIGHEST PRIORITY): Continuously monitors all
 *     3x TCRT5000 IR sensors for the ring boundary; preempts Search/Track/Attack
 *     with immediate brake -> reverse -> pivot away to prevent Ring-Out (Rule 1).
 *   - Continuous Motion: Search sweep keeps moving so the robot never triggers
 *     the 15-second immobility/disabled rule (Rule 8.3).
 *   - Note on Rule 4: Tech Rumble 2 Fighting allows wireless Manual control
 *     (MODE_MANUAL); verify with Head Referee before running autonomous Sumo
 *     in Knockout Battle Royale, or use MODE_MANUAL with R2 Boost if required.
 * ============================================================================
 */

// ============================================================================
//  EASILY TUNABLE SUMO TIMING, SENSOR & POWER CONSTANTS
// ============================================================================

// Competition Start Delay & Arm Safety
static const uint32_t SUMO_START_DELAY_MS       = 5000UL;  // Mandatory 5 s non-blocking countdown
static const uint32_t SUMO_MAX_MATCH_MS         = 180000UL;// 3 min Battle Royale / Sumo safety cap

// Dohyo Ring-Edge IR Polarity & Escape Timing
// true  = Traditional black Sumo dohyo with WHITE ring border (IR bit == 0 is edge)
// false = Matte white arena with DARK border or elevated drop-off (IR bit == 1 is edge)
static const bool     SUMO_EDGE_IS_WHITE        = true;
static const uint32_t SUMO_EDGE_BRAKE_MS        = 60UL;    // Crisp active H-bridge brake
static const uint32_t SUMO_EDGE_REVERSE_MS      = 320UL;   // Pull cleanly away from ring edge
static const uint32_t SUMO_EDGE_PIVOT_MS        = 380UL;   // Turn away from edge back toward center

// Ultrasonic Opponent Detection Ranges (cm — tuned for ~100 cm radius Pocket / Dohyo)
static const float    SUMO_DETECT_RANGE_CM      = 75.0f;   // Side/Front acquisition range
static const float    SUMO_ATTACK_RANGE_CM      = 65.0f;   // Front ultrasonic lock -> Full Ram!
static const float    SUMO_CLOSE_RAM_CM         = 22.0f;   // Close contact max-torque push zone

// Motor PWM & Slew Tuning (4WD JGA25-370 high-torque combat response)
static const int      SUMO_SLEW_STEP            = 45;      // Snappy ramp (PWM / 10 ms)
static const int      SUMO_SEARCH_SPIN_PWM      = 130;     // Controlled radar sweep spin
static const int      SUMO_SEARCH_CREEP_PWM     = 135;     // Periodic forward sweep advance
static const int      SUMO_ALIGN_PWM            = 205;     // Fast pivot toward side opponent
static const int      SUMO_ATTACK_PWM           = 255;     // Full safe ramming torque
static const int      SUMO_ESCAPE_REV_PWM       = 215;     // Urgent reverse from ring edge
static const int      SUMO_ESCAPE_PIVOT_PWM     = 190;     // Fast turn away from ring edge

// Search / Track / Attack Non-Blocking Timers (ms)
static const uint32_t SUMO_SEARCH_SPIN_MS       = 850UL;   // Sweep phase duration
static const uint32_t SUMO_SEARCH_FORWARD_MS    = 450UL;   // Reposition phase (beats 15 s rule!)
static const uint32_t SUMO_ALIGN_TIMEOUT_MS     = 900UL;   // Return to search if target vanishes
static const uint32_t SUMO_ATTACK_LOST_HOLD_MS  = 220UL;   // Brief momentum hold on sensor blink

// ============================================================================
//  INTERNAL SUMO STATE MACHINE
// ============================================================================

enum SumoSubState : uint8_t {
  SUMO_STATE_COUNTDOWN    = 0,  // 5-second competition start countdown (motors stopped)
  SUMO_STATE_SEARCH_SPIN  = 1,  // Radar spin sweep scanning F/L/R ultrasonics
  SUMO_STATE_SEARCH_FWD   = 2,  // Short forward advance to cover ring & stay active
  SUMO_STATE_ALIGN        = 3,  // Rapidly pivot toward Left or Right ultrasonic target
  SUMO_STATE_ATTACK       = 4,  // Full-power straight ram on Front ultrasonic lock
  SUMO_STATE_EDGE_BRAKE   = 5,  // Highest-priority ring-edge active brake
  SUMO_STATE_EDGE_REVERSE = 6,  // Timed reverse away from dohyo boundary
  SUMO_STATE_EDGE_PIVOT   = 7   // Timed pivot back toward center of ring
};

static SumoSubState s_sumoState           = SUMO_STATE_COUNTDOWN;
static uint32_t     s_sumoEnteredMs       = 0;
static uint32_t     s_sumoPhaseStartMs    = 0;
static uint32_t     s_sumoFrontLostMs     = 0;
static uint8_t      s_sumoLastCountdownSec = 255;

static int8_t       s_sumoLastTargetDir   = 1;     // -1 = Left, +1 = Right
static int8_t       s_sumoEdgeTurnDir     = 1;     // -1 = pivot Left, +1 = pivot Right
static uint8_t      s_sumoSearchCycle     = 0;

// ============================================================================
//  INTERNAL HELPERS
// ============================================================================

static uint8_t readRingEdgeMask() {
  // sensors.irMask bits: bit2 = Left, bit1 = Center, bit0 = Right (1 = black, 0 = white)
  // Returns 3-bit mask where 1 means that sensor sees the RING BOUNDARY EDGE.
  if (SUMO_EDGE_IS_WHITE) {
    return (uint8_t)((~sensors.irMask) & 0x07U);
  }
  return (uint8_t)(sensors.irMask & 0x07U);
}

static bool isInEdgeEscape() {
  return (s_sumoState == SUMO_STATE_EDGE_BRAKE   ||
          s_sumoState == SUMO_STATE_EDGE_REVERSE ||
          s_sumoState == SUMO_STATE_EDGE_PIVOT);
}

static bool detectFrontOpponent() {
  return (sensors.usFrontValid && sensors.usFrontCm <= SUMO_ATTACK_RANGE_CM);
}

static bool detectLeftOpponent() {
  return (sensors.usLeftValid && sensors.usLeftCm <= SUMO_DETECT_RANGE_CM);
}

static bool detectRightOpponent() {
  return (sensors.usRightValid && sensors.usRightCm <= SUMO_DETECT_RANGE_CM);
}

static void triggerEdgeEscape(uint8_t edgeMask, uint32_t nowMs) {
  brakeMotorsImmediate();
  s_sumoState        = SUMO_STATE_EDGE_BRAKE;
  s_sumoPhaseStartMs = nowMs;

  const bool leftEdge  = (edgeMask & 0b100U) != 0;
  const bool rightEdge = (edgeMask & 0b001U) != 0;

  // Pivot AWAY from the detected border side
  if (leftEdge && !rightEdge) {
    s_sumoEdgeTurnDir = 1;    // Edge on left -> pivot RIGHT
  } else if (rightEdge && !leftEdge) {
    s_sumoEdgeTurnDir = -1;   // Edge on right -> pivot LEFT
  } else {
    // Center or both sides -> pivot away from last search direction
    s_sumoEdgeTurnDir = -s_sumoLastTargetDir;
  }

  Serial.print(F("[SUMO] RING EDGE DETECTED (mask="));
  Serial.print((edgeMask >> 2) & 1);
  Serial.print((edgeMask >> 1) & 1);
  Serial.print(edgeMask & 1);
  Serial.println(F(") -> BRAKE & ESCAPE!"));
}

static void enterSumoSearch(uint32_t nowMs) {
  s_sumoState        = SUMO_STATE_SEARCH_SPIN;
  s_sumoPhaseStartMs = nowMs;
  s_sumoFrontLostMs  = 0;
  resetEncoders();
}

static void enterSumoAlign(int8_t dir, uint32_t nowMs) {
  s_sumoLastTargetDir = (dir < 0) ? -1 : 1;
  s_sumoState         = SUMO_STATE_ALIGN;
  s_sumoPhaseStartMs  = nowMs;
  Serial.println(s_sumoLastTargetDir < 0 ? F("[SUMO] Target LEFT -> aligning...")
                                         : F("[SUMO] Target RIGHT -> aligning..."));
}

static void enterSumoAttack(uint32_t nowMs) {
  s_sumoState        = SUMO_STATE_ATTACK;
  s_sumoPhaseStartMs = nowMs;
  s_sumoFrontLostMs  = 0;
  Serial.print(F("[SUMO] FRONT LOCK ("));
  Serial.print(sensors.usFrontCm, 1);
  Serial.println(F(" cm) -> FULL RAM!"));
}

// ============================================================================
//  PUBLIC SUMO MODE ENTRY POINTS (Called by TechRumbleRobot.ino)
// ============================================================================

void initCombatSumoMode(uint32_t nowMs) {
  s_sumoState            = SUMO_STATE_COUNTDOWN;
  s_sumoEnteredMs        = nowMs;
  s_sumoPhaseStartMs     = nowMs;
  s_sumoFrontLostMs      = 0;
  s_sumoLastCountdownSec = 255;
  s_sumoLastTargetDir    = (robot.lane == LANE_A) ? 1 : -1;
  s_sumoEdgeTurnDir      = 1;
  s_sumoSearchCycle      = 0;

  stopMotorsImmediate();
  resetEncoders();
  setMotorSlewStep(SUMO_SLEW_STEP);

  // Stow arm into safe compact profile before any combat motion!
  // If carrying the 20 g Sudden Death load, keep Grip-Lock armed and tuck arm
  // clear of chassis deck (Rule 3.1); otherwise stow compactly within 25x25x50 cm.
  if (sensors.gripSwitchPressed || grip.gripLockActive) {
    setGripLock(true);
    applyArmPose(POSE_COMBAT_TUCK);
  } else {
    applyArmPose(POSE_HOME_STOW);
  }

  Serial.print(F("[SUMO] Armed — starting "));
  Serial.print(SUMO_START_DELAY_MS / 1000UL);
  Serial.println(F("s non-blocking start countdown. Arm stowed safely."));
}

void runCombatSumo(uint32_t nowMs) {
  // Mode exit safety guard: immediately stop motors if mode changed externally
  if (robot.mode != MODE_COMBAT_SUMO) {
    stopMotorsImmediate();
    return;
  }

  // --- 1. Mandatory Non-Blocking Start Countdown (No movement before signal!) ---
  if (s_sumoState == SUMO_STATE_COUNTDOWN) {
    stopMotorsImmediate();
    const uint32_t elapsed = nowMs - s_sumoPhaseStartMs;
    if (elapsed >= SUMO_START_DELAY_MS) {
      Serial.println(F("[SUMO] GO! Entering active opponent search."));
      enterSumoSearch(nowMs);
      return;
    }
    const uint8_t remSec = (uint8_t)(((SUMO_START_DELAY_MS - elapsed) + 999UL) / 1000UL);
    if (remSec != s_sumoLastCountdownSec) {
      s_sumoLastCountdownSec = remSec;
      Serial.print(F("[SUMO] Countdown: "));
      Serial.print(remSec);
      Serial.println(F("s..."));
    }
    return;
  }

  // Overall match safety timeout guard
  if (nowMs - s_sumoEnteredMs >= SUMO_MAX_MATCH_MS + SUMO_START_DELAY_MS) {
    brakeMotorsImmediate();
    enterMode(MODE_IDLE_SAFE, nowMs, "SUMO_MAX_MATCH_TIMEOUT");
    return;
  }

  // --- 2. HIGHEST PRIORITY: Dohyo Ring-Edge Avoidance (Preempts All Combat!) ---
  const uint8_t edgeMask = readRingEdgeMask();
  if (!isInEdgeEscape() && edgeMask != 0) {
    triggerEdgeEscape(edgeMask, nowMs);
  }

  // --- 3. Non-Blocking Combat & Escape State Machine ---
  switch (s_sumoState) {
    case SUMO_STATE_EDGE_BRAKE: {
      brakeMotorsImmediate();
      if (nowMs - s_sumoPhaseStartMs >= SUMO_EDGE_BRAKE_MS) {
        s_sumoState        = SUMO_STATE_EDGE_REVERSE;
        s_sumoPhaseStartMs = nowMs;
        setMotorTarget(-SUMO_ESCAPE_REV_PWM, -SUMO_ESCAPE_REV_PWM);
      }
      break;
    }

    case SUMO_STATE_EDGE_REVERSE: {
      setMotorTarget(-SUMO_ESCAPE_REV_PWM, -SUMO_ESCAPE_REV_PWM);
      if (nowMs - s_sumoPhaseStartMs >= SUMO_EDGE_REVERSE_MS) {
        s_sumoState        = SUMO_STATE_EDGE_PIVOT;
        s_sumoPhaseStartMs = nowMs;
      }
      break;
    }

    case SUMO_STATE_EDGE_PIVOT: {
      const int pivL = (int)s_sumoEdgeTurnDir * SUMO_ESCAPE_PIVOT_PWM;
      const int pivR = -(int)s_sumoEdgeTurnDir * SUMO_ESCAPE_PIVOT_PWM;
      setMotorTarget(pivL, pivR);
      if (nowMs - s_sumoPhaseStartMs >= SUMO_EDGE_PIVOT_MS) {
        // After escaping edge, immediately check if front sensor already sees opponent!
        if (detectFrontOpponent()) {
          enterSumoAttack(nowMs);
        } else {
          enterSumoSearch(nowMs);
        }
      }
      break;
    }

    case SUMO_STATE_SEARCH_SPIN: {
      // Check sensors in priority order: Front -> Closest Side
      if (detectFrontOpponent()) {
        enterSumoAttack(nowMs);
        return;
      }
      const bool lHit = detectLeftOpponent();
      const bool rHit = detectRightOpponent();
      if (lHit || rHit) {
        const int8_t dir = (lHit && (!rHit || sensors.usLeftCm <= sensors.usRightCm)) ? -1 : 1;
        enterSumoAlign(dir, nowMs);
        return;
      }

      // Spin sweep using last known target direction
      const int spinL = (int)s_sumoLastTargetDir * SUMO_SEARCH_SPIN_PWM;
      const int spinR = -(int)s_sumoLastTargetDir * SUMO_SEARCH_SPIN_PWM;
      setMotorTarget(spinL, spinR);

      if (nowMs - s_sumoPhaseStartMs >= SUMO_SEARCH_SPIN_MS) {
        s_sumoState        = SUMO_STATE_SEARCH_FWD;
        s_sumoPhaseStartMs = nowMs;
        s_sumoSearchCycle++;
        // Alternate sweep direction every 2 cycles to cover blind spots
        if ((s_sumoSearchCycle & 0x01U) == 0) {
          s_sumoLastTargetDir = -s_sumoLastTargetDir;
        }
      }
      break;
    }

    case SUMO_STATE_SEARCH_FWD: {
      if (detectFrontOpponent()) {
        enterSumoAttack(nowMs);
        return;
      }
      const bool lHit = detectLeftOpponent();
      const bool rHit = detectRightOpponent();
      if (lHit || rHit) {
        const int8_t dir = (lHit && (!rHit || sensors.usLeftCm <= sensors.usRightCm)) ? -1 : 1;
        enterSumoAlign(dir, nowMs);
        return;
      }

      setMotorTarget(SUMO_SEARCH_CREEP_PWM, SUMO_SEARCH_CREEP_PWM);
      if (nowMs - s_sumoPhaseStartMs >= SUMO_SEARCH_FORWARD_MS) {
        enterSumoSearch(nowMs);
      }
      break;
    }

    case SUMO_STATE_ALIGN: {
      // Lock acquired straight ahead -> Full Ram!
      if (detectFrontOpponent()) {
        enterSumoAttack(nowMs);
        return;
      }
      // Re-bias if opponent moved to opposite side
      if (s_sumoLastTargetDir < 0 && !detectLeftOpponent() && detectRightOpponent()) {
        s_sumoLastTargetDir = 1;
        s_sumoPhaseStartMs  = nowMs;
      } else if (s_sumoLastTargetDir > 0 && !detectRightOpponent() && detectLeftOpponent()) {
        s_sumoLastTargetDir = -1;
        s_sumoPhaseStartMs  = nowMs;
      }

      if (nowMs - s_sumoPhaseStartMs >= SUMO_ALIGN_TIMEOUT_MS) {
        enterSumoSearch(nowMs);
        return;
      }

      const int alignL = (int)s_sumoLastTargetDir * SUMO_ALIGN_PWM;
      const int alignR = -(int)s_sumoLastTargetDir * SUMO_ALIGN_PWM;
      setMotorTarget(alignL, alignR);
      break;
    }

    case SUMO_STATE_ATTACK: {
      if (detectFrontOpponent()) {
        s_sumoFrontLostMs = 0;
        // Full straight ram; subtle steering nudge if opponent starts sliding off one side
        int leftPwm  = SUMO_ATTACK_PWM;
        int rightPwm = SUMO_ATTACK_PWM;
        if (sensors.usFrontCm > SUMO_CLOSE_RAM_CM) {
          if (detectLeftOpponent() && !detectRightOpponent()) {
            leftPwm = SUMO_ATTACK_PWM - 35;
          } else if (detectRightOpponent() && !detectLeftOpponent()) {
            rightPwm = SUMO_ATTACK_PWM - 35;
          }
        }
        setMotorTarget(leftPwm, rightPwm);
        return;
      }

      // Opponent slipped off Front cone -> pivot immediately if a side sensor still sees them!
      if (detectLeftOpponent()) {
        enterSumoAlign(-1, nowMs);
        return;
      }
      if (detectRightOpponent()) {
        enterSumoAlign(1, nowMs);
        return;
      }

      // Brief momentum hold across ultrasonic ping jitter before resuming search
      if (s_sumoFrontLostMs == 0) {
        s_sumoFrontLostMs = nowMs;
        setMotorTarget(SUMO_ATTACK_PWM, SUMO_ATTACK_PWM);
      } else if (nowMs - s_sumoFrontLostMs >= SUMO_ATTACK_LOST_HOLD_MS) {
        enterSumoSearch(nowMs);
      }
      break;
    }

    default:
      enterSumoSearch(nowMs);
      break;
  }
}

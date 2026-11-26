/*
 * ============================================================================
 *  Tech Rumble 2 — Manual PS4 DualShock 4 Drive & 4-DOF Arm Control
 *  File: TechRumbleRobot/manual.ino
 *
 *  STRICT HARDWARE ISOLATION:
 *   - Zero GPIO, LEDC, ADC, or ISR code here. All motor, servo, and sensor
 *     access goes strictly through hardware.ino HAL functions and global
 *     snapshots (sensors, grip, robot).
 *   - 100% non-blocking millis() execution (called at 50 Hz from loop()).
 *
 *  DUALSHOCK 4 CONTROL MAP (MODE_MANUAL):
 *   Drive (Arcade Mix):
 *    - Left Stick Y         : Throttle (forward / reverse, deadzone + expo)
 *    - Right Stick X        : Steering (left / right, deadzone + expo)
 *    - L2 Trigger (hold)    : Precision Gear (42% drive + 50% arm jog speed —
 *                             ideal for Loading Zone pickup & Ramps ribs!)
 *    - R2 Trigger (hold)    : Boost / Serrated-Ramp Climb Gear (100% drive)
 *    - L3 (hold)            : Active H-bridge short-brake (hold on slopes)
 *
 *   4-DOF Arm Incremental Jogging (ignored while OPTIONS is held for mode switch):
 *    - D-Pad Left / Right   : Base Waist pan (MG996R Full Metal)
 *    - D-Pad Up / Down      : Shoulder raise / lower (hard-clamped <= 50 cm)
 *    - Right Stick Y        : Elbow extend / retract (vertical-dominant guard
 *                             so horizontal steering never drifts the elbow)
 *
 *   Arm Quick Presets (Face Buttons, rising-edge triggered):
 *    - Cross (X)            : POSE_HOME_STOW    (compact travel <= 25x25x50 cm)
 *    - Square               : POSE_PICKUP_READY (forward 20 g pickup reach)
 *    - Triangle             : POSE_CARRY_SAFE   (arms Grip-Lock if load held)
 *    - Circle (alone)       : POSE_DEPOSIT      (low Pocket placement; keeps grip)
 *
 *   Gripper & Sudden Death Safety:
 *    - R1 (hold)            : Incrementally close gripper; auto-stops at
 *                             grip-switch contact + gentle firm preload to
 *                             protect Half-Metal MG996R gears, with rumble!
 *    - L1 (tap)             : Open gripper freely when Grip-Lock is OFF
 *    - L1 + Circle (300 ms) : Deliberate guarded release when Grip-Lock is ON
 *                             (prevents accidental Sudden Death drops!)
 *    - Touchpad (300 ms)    : Manually toggle Sudden Death Grip-Lock ON/OFF
 * ============================================================================
 */

// ============================================================================
//  MANUAL TUNING CONSTANTS
// ============================================================================

static const int      MANUAL_STICK_DEADZONE      = 16;     // ~12.5% stick deadzone
static const int      MANUAL_ELBOW_DEADZONE      = 36;     // Stricter deadzone on RStickY
static const float    MANUAL_ELBOW_AXIS_RATIO    = 1.25f;  // |RY| must exceed 1.25 * |RX|
static const uint8_t  MANUAL_TRIGGER_THRESHOLD   = 100;    // L2/R2 active threshold (0..255)

// Drive scaling & smoothing
static const float    MANUAL_EXPO_BLEND          = 0.35f;  // 65% linear + 35% cubic-feel expo
static const float    MANUAL_STEER_GAIN          = 0.85f;  // Keeps turning crisp without spinout
static const int      MANUAL_PWM_PRECISION       = 110;    // L2 Precision Gear (~43%)
static const int      MANUAL_PWM_NORMAL          = 185;    // Default Gear (~73%)
static const int      MANUAL_PWM_BOOST           = 255;    // R2 Boost / Serrated Ramp (100%)
static const int      MANUAL_SLEW_PRECISION      = 12;     // Gentle ramp for 20 g Sudden Death load
static const int      MANUAL_SLEW_NORMAL         = 22;
static const int      MANUAL_SLEW_BOOST          = 38;

// Non-blocking arm incremental jog speeds (degrees per 20 ms tick = deg/s / 50)
static const uint32_t MANUAL_ARM_JOG_INTERVAL_MS = 20UL;
static const float    MANUAL_BASE_DEG_PER_TICK   = 1.4f;   // ~70 deg/s normal (35 deg/s precision)
static const float    MANUAL_SHLD_DEG_PER_TICK   = 1.2f;   // ~60 deg/s normal (30 deg/s precision)
static const float    MANUAL_ELBOW_DEG_PER_TICK  = 1.4f;   // ~70 deg/s max proportional
static const float    MANUAL_GRIP_CLOSE_PER_TICK = 1.8f;   // Smooth jaw closure toward switch
static const float    MANUAL_GRIP_PRELOAD_DEG    = 4.0f;   // Firm holding preload after switch click
static const float    MANUAL_TARGET_LEAD_CAP_DEG = 6.0f;   // Prevents target windup at hard stops

// Guarded release & lock-toggle hold timers
static const uint32_t MANUAL_RELEASE_HOLD_MS     = 300UL;  // Hold L1 + Circle to drop when locked
static const uint32_t MANUAL_LOCK_TOGGLE_MS      = 300UL;  // Hold Touchpad to toggle Grip-Lock

// ============================================================================
//  INTERNAL MANUAL MODE STATE (Reset cleanly on entering MODE_MANUAL)
// ============================================================================

static uint32_t s_manLastArmJogMs        = 0;
static uint32_t s_manReleaseHoldStartMs  = 0;
static uint32_t s_manLockHoldStartMs     = 0;
static bool     s_manLockToggleLatched   = false;
static bool     s_manReleaseLatched      = false;
static bool     s_manGripContactLatched  = false;

// Rising-edge button latches so holding a face button doesn't spam presets
static bool     s_prevCross              = false;
static bool     s_prevSquare             = false;
static bool     s_prevTriangle           = false;
static bool     s_prevCircle             = false;
static bool     s_prevL1                 = false;

// ============================================================================
//  INTERNAL HELPERS
// ============================================================================

static float applyStickDeadzoneExpo(int8_t rawVal, int deadzone) {
  const int v = (int)rawVal;
  const int av = abs(v);
  if (av <= deadzone) {
    return 0.0f;
  }
  const float maxRange = 127.0f - (float)deadzone;
  float norm = (float)(av - deadzone) / maxRange;
  if (norm > 1.0f) norm = 1.0f;

  // Blend linear + quadratic curve for fine control near center
  const float curved = (1.0f - MANUAL_EXPO_BLEND) * norm + MANUAL_EXPO_BLEND * (norm * norm);
  return (v < 0) ? -curved : curved;
}

static void jogArmJointSafe(ServoJoint joint, float deltaDeg) {
  const float curDeg = getServoCurrentDeg(joint);
  float nextTarget   = getServoTargetDeg(joint) + deltaDeg;

  // Keep target tightly bounded around physical position so releasing a button
  // stops arm motion immediately with zero lag or windup.
  if (nextTarget > curDeg + MANUAL_TARGET_LEAD_CAP_DEG) {
    nextTarget = curDeg + MANUAL_TARGET_LEAD_CAP_DEG;
  } else if (nextTarget < curDeg - MANUAL_TARGET_LEAD_CAP_DEG) {
    nextTarget = curDeg - MANUAL_TARGET_LEAD_CAP_DEG;
  }
  setServoTargetDeg(joint, nextTarget);
}

static void updateManualDrive() {
  // Active H-bridge brake while L3 is held (useful on elevated Ramps / slopes)
  if (PS4.L3()) {
    brakeMotorsImmediate();
    return;
  }

  // Select speed gear via analog triggers (Precision L2 takes safety priority over Boost R2)
  int maxPwm   = MANUAL_PWM_NORMAL;
  int slewStep = MANUAL_SLEW_NORMAL;
  if (PS4.L2Value() >= MANUAL_TRIGGER_THRESHOLD) {
    maxPwm   = MANUAL_PWM_PRECISION;
    slewStep = MANUAL_SLEW_PRECISION;
  } else if (PS4.R2Value() >= MANUAL_TRIGGER_THRESHOLD) {
    maxPwm   = MANUAL_PWM_BOOST;
    slewStep = MANUAL_SLEW_BOOST;
  }
  setMotorSlewStep(slewStep);

  // Arcade drive mix: Left Stick Y = throttle, Right Stick X = steering
  const float throttle = applyStickDeadzoneExpo(PS4.LStickY(), MANUAL_STICK_DEADZONE);
  const float steer    = applyStickDeadzoneExpo(PS4.RStickX(), MANUAL_STICK_DEADZONE) * MANUAL_STEER_GAIN;

  float leftNorm  = throttle + steer;
  float rightNorm = throttle - steer;

  // Proportional normalization preserves steering authority at full throttle
  const float peak = fmaxf(fabsf(leftNorm), fabsf(rightNorm));
  if (peak > 1.0f) {
    leftNorm  /= peak;
    rightNorm /= peak;
  }

  const int leftPwm  = (int)(leftNorm  * (float)maxPwm);
  const int rightPwm = (int)(rightNorm * (float)maxPwm);
  setMotorTarget(leftPwm, rightPwm);
}

static void updateManualArmAndGripper(uint32_t nowMs) {
  const bool precisionHeld = (PS4.L2Value() >= MANUAL_TRIGGER_THRESHOLD);
  const float speedScale   = precisionHeld ? 0.50f : 1.0f;

  // --- 1. Arm Preset Face Buttons (rising-edge triggered) ---
  const bool crossNow    = PS4.Cross();
  const bool squareNow   = PS4.Square();
  const bool triangleNow = PS4.Triangle();
  const bool circleNow   = PS4.Circle();
  const bool l1Now       = PS4.L1();
  const bool r1Now       = PS4.R1();

  if (crossNow && !s_prevCross) {
    applyArmPose(POSE_HOME_STOW);
    Serial.println(F("[MANUAL] Arm Preset -> HOME_STOW"));
  }
  if (squareNow && !s_prevSquare) {
    applyArmPose(POSE_PICKUP_READY);
    Serial.println(F("[MANUAL] Arm Preset -> PICKUP_READY"));
  }
  if (triangleNow && !s_prevTriangle) {
    if (sensors.gripSwitchPressed) {
      // Automatically arm Sudden Death Grip-Lock when lifting into Carry pose!
      armSuddenDeathLock(true);
      triggerPs4RumblePulse(0, 140, 180, nowMs);
      Serial.println(F("[MANUAL] Arm Preset -> CARRY_SAFE (Sudden Death Grip-Lock ARMED!)"));
    } else {
      Serial.println(F("[MANUAL] Arm Preset -> CARRY_SAFE"));
    }
    applyArmPose(POSE_CARRY_SAFE);
  }
  // Circle alone moves arm to Deposit pose (keeps Grip-Lock intact until L1+Circle release!)
  if (circleNow && !s_prevCircle && !l1Now) {
    applyArmPose(POSE_DEPOSIT);
    Serial.println(F("[MANUAL] Arm Preset -> DEPOSIT (hold L1 + Circle to release if locked)"));
  }

  // --- 2. Explicit Sudden Death Grip-Lock Toggle (Hold Touchpad 300 ms) ---
  if (PS4.Touchpad()) {
    if (s_manLockHoldStartMs == 0) {
      s_manLockHoldStartMs = nowMs;
    } else if (!s_manLockToggleLatched && (nowMs - s_manLockHoldStartMs >= MANUAL_LOCK_TOGGLE_MS)) {
      s_manLockToggleLatched = true;
      const bool nextArmed = !grip.gripLockActive;
      armSuddenDeathLock(nextArmed);
      if (!nextArmed) {
        setGripLock(false);
      }
      triggerPs4RumblePulse(nextArmed ? 0 : 100, nextArmed ? 160 : 0, 180, nowMs);
      Serial.print(F("[MANUAL] Sudden Death Grip-Lock -> "));
      Serial.println(grip.gripLockActive ? F("LOCKED") : F("UNLOCKED"));
    }
  } else {
    s_manLockHoldStartMs   = 0;
    s_manLockToggleLatched = false;
  }

  // --- 3. Incremental Non-Blocking 3-DOF Arm Jogging & Gripper Close ---
  if (nowMs - s_manLastArmJogMs >= MANUAL_ARM_JOG_INTERVAL_MS) {
    s_manLastArmJogMs = nowMs;

    // Ignore D-Pad while OPTIONS is held so mode switching never moves the arm!
    if (!PS4.Options()) {
      // Base Waist: D-Pad Left / Right
      if (PS4.Left()) {
        jogArmJointSafe(JOINT_BASE, -MANUAL_BASE_DEG_PER_TICK * speedScale);
      } else if (PS4.Right()) {
        jogArmJointSafe(JOINT_BASE, +MANUAL_BASE_DEG_PER_TICK * speedScale);
      }

      // Shoulder: D-Pad Up / Down (50 cm max height enforced inside hardware.ino)
      if (PS4.Up()) {
        jogArmJointSafe(JOINT_SHOULDER, +MANUAL_SHLD_DEG_PER_TICK * speedScale);
      } else if (PS4.Down()) {
        jogArmJointSafe(JOINT_SHOULDER, -MANUAL_SHLD_DEG_PER_TICK * speedScale);
      }
    }

    // Elbow: Right Stick Y with vertical-dominance guard so steering on RStickX
    // does not accidentally wobble the arm while driving on the elevated Ramps!
    const int rxAbs = abs((int)PS4.RStickX());
    const int ryAbs = abs((int)PS4.RStickY());
    if (ryAbs > MANUAL_ELBOW_DEADZONE && (float)ryAbs > (float)rxAbs * MANUAL_ELBOW_AXIS_RATIO) {
      const float elbowCmd = applyStickDeadzoneExpo(PS4.RStickY(), MANUAL_ELBOW_DEADZONE);
      jogArmJointSafe(JOINT_ELBOW, elbowCmd * MANUAL_ELBOW_DEG_PER_TICK * speedScale);
    }

    // Gripper Close (R1): incremental close with microswitch feedback & auto-stop!
    // Protects Half-Metal MG996R gripper gears from stalling while securing 20 g load.
    if (r1Now) {
      if (sensors.gripSwitchPressed) {
        if (!s_manGripContactLatched) {
          s_manGripContactLatched = true;
          clampGripperAtContact(MANUAL_GRIP_PRELOAD_DEG);
          triggerPs4RumblePulse(140, 80, 160, nowMs);
          Serial.println(F("[MANUAL] Grip switch CONTACT — clamped firmly & stopped closing."));
        }
      } else {
        s_manGripContactLatched = false;
        jogGripperClose(MANUAL_GRIP_CLOSE_PER_TICK * speedScale);
      }
    } else if (!sensors.gripSwitchPressed) {
      s_manGripContactLatched = false;
    }
  }

  // --- 4. Gripper Open (L1 = free open if unlocked; L1 + Circle 300 ms = guarded release) ---
  if (l1Now && circleNow) {
    if (s_manReleaseHoldStartMs == 0) {
      s_manReleaseHoldStartMs = nowMs;
    } else if (!s_manReleaseLatched && (nowMs - s_manReleaseHoldStartMs >= MANUAL_RELEASE_HOLD_MS)) {
      s_manReleaseLatched     = true;
      s_manGripContactLatched = false;
      requestGripperOpen(true); // Deliberate combo overrides Sudden Death Grip-Lock
      triggerPs4RumblePulse(120, 0, 200, nowMs);
      Serial.println(F("[MANUAL] Guarded combo confirmed — Gripper OPENED for Deposit!"));
    }
  } else {
    s_manReleaseHoldStartMs = 0;
    s_manReleaseLatched     = false;

    // Single tap on L1 (without Circle)
    if (l1Now && !s_prevL1 && !circleNow) {
      if (!grip.gripLockActive) {
        s_manGripContactLatched = false;
        requestGripperOpen(false);
      } else {
        // Blocked by Sudden Death Grip-Lock: warn driver with short sharp rumble
        requestGripperOpen(false);
        triggerPs4RumblePulse(200, 0, 100, nowMs);
      }
    }
  }

  // Save button edge states
  s_prevCross    = crossNow;
  s_prevSquare   = squareNow;
  s_prevTriangle = triangleNow;
  s_prevCircle   = circleNow;
  s_prevL1       = l1Now;
}

// ============================================================================
//  PUBLIC MANUAL MODE ENTRY POINTS (Called by TechRumbleRobot.ino)
// ============================================================================

void initManualMode(uint32_t nowMs) {
  s_manLastArmJogMs       = nowMs;
  s_manReleaseHoldStartMs = 0;
  s_manLockHoldStartMs    = 0;
  s_manLockToggleLatched  = false;
  s_manReleaseLatched     = false;
  s_manGripContactLatched = sensors.gripSwitchPressed;

  // Latch currently held buttons so entering MODE_MANUAL never misfires a preset
  if (PS4.isConnected()) {
    s_prevCross    = PS4.Cross();
    s_prevSquare   = PS4.Square();
    s_prevTriangle = PS4.Triangle();
    s_prevCircle   = PS4.Circle();
    s_prevL1       = PS4.L1();
  } else {
    s_prevCross = s_prevSquare = s_prevTriangle = s_prevCircle = s_prevL1 = false;
  }
}

void runManual(uint32_t nowMs) {
  // Defense-in-depth fail-safe: immediately stop all motors and hold servos
  // if DualShock 4 Bluetooth link drops!
  if (!PS4.isConnected()) {
    stopMotorsImmediate();
    freezeArmInPlace();
    return;
  }

  updateManualDrive();
  updateManualArmAndGripper(nowMs);
}

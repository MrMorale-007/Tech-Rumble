/*
 * ============================================================================
 *  Tech Rumble 2 — ESP32 DevKit V1 Competition Robot Firmware
 *  File: TechRumbleRobot/TechRumbleRobot.ino
 *  Repo: https://github.com/MrMorale-007/Tech-Rumble
 *
 *  LOCKED STACK:
 *   - Board:      ESP32 DevKit V1 (30-pin safe pinout)
 *   - Core:       ESP32 Arduino Core 2.0.17 (2.x LEDC API)
 *   - Controller: Sony PS4 DualShock 4 via PS4Controller (aed3) — NOT PS5
 *   - Structure:  Flat .ino tabs only (zero .h / .cpp files)
 *   - Isolation:  ALL GPIO, LEDC PWM, ADC, and IRAM_ATTR ISRs live strictly
 *                 inside hardware.ino.
 *   - Timing:     100% non-blocking millis() / micros() / ISR scheduling.
 *
 *  COMPETITION SAFETY INVARIANTS (see RULES_CHECKLIST.md):
 *   1. Boot into MODE_IDLE_SAFE (motors stopped, arm stowed) — prevents
 *      false starts before the referee's start signal (Rule 5).
 *   2. Never process PS4 drive inputs during autonomous MODE_LINE_FOLLOWER
 *      or autonomous MODE_MAZE (sending drive commands loses the match, Rule 4).
 *   3. Never drop servo/gripper holding torque on PS4 disconnect or mode
 *      switch — Sudden Death starts upon leaving Loading Zone (Rule 3.2).
 *   4. Hard-clamp arm servo angles in hardware.ino to stay <= 50 cm height
 *      at all times (Rule 2).
 * ============================================================================
 */

#include <PS4Controller.h>
#include <Preferences.h>

// ============================================================================
//  CONFIGURATION CONSTANTS
// ============================================================================

// Replace with the Bluetooth MAC address flashed onto your DualShock 4
// (using SixaxisPairTool or equivalent). Must be lowercase hex pairs.
static const char* PS4_MAC_ADDRESS = "01:02:03:04:05:06";

// Set to 1 to stop motors even during autonomous modes if PS4 disconnects
// (useful for tethered bench testing). Keep 0 for competition so brief RF
// dropouts across the 10 m Knockout gauntlet do not abort autonomous runs.
#define FAILSAFE_STOP_AUTONOMOUS_ON_DISCONNECT 0

// Enable non-blocking Serial bench-test & telemetry helper (active only in
// MODE_IDLE_SAFE). Set to 0 for minimal serial overhead on competition day.
#define ENABLE_SERIAL_DIAG 1

static const uint32_t CONTROL_TICK_MS         = 20UL;   // 50 Hz main control loop
static const uint32_t MODE_HOLD_CONFIRM_MS    = 500UL;  // Hold Options + D-Pad to switch modes
static const uint32_t ABORT_HOLD_CONFIRM_MS   = 300UL;  // Hold Share to emergency-stop to IDLE_SAFE
static const uint32_t CONFIG_HOLD_CONFIRM_MS  = 500UL;  // Hold L3/R3 in IDLE_SAFE to toggle Lane/Stage
static const uint32_t PS4_FEEDBACK_MIN_MS     = 250UL;  // Rate-limit PS4.sendToController()

// ============================================================================
//  SHARED TYPES & ENUMS (Declared at top for Arduino multi-tab auto-prototyping)
// ============================================================================

enum RobotMode : uint8_t {
  MODE_IDLE_SAFE     = 0,  // Default boot state: motors stopped, arm held, bench diag active
  MODE_MANUAL        = 1,  // Wireless PS4 drive + arm (Ramps & Loading Zone pickup)
  MODE_LINE_FOLLOWER = 2,  // Fully autonomous 3-IR PD line follower (PS4 drive locked out!)
  MODE_MAZE          = 3,  // Autonomous ultrasonic maze solver (PS4 drive locked out!)
  MODE_COMBAT_SUMO   = 4   // Knockout Battle Royale manual combat + Grip-Lock & arm tuck
};

enum LaneSide : uint8_t {
  LANE_A = 0,              // Left-side mirrored arena track
  LANE_B = 1               // Right-side mirrored arena track
};

enum StageType : uint8_t {
  STAGE_GROUP    = 0,      // 5 min single-arena two-way trials (wireless mode switches only)
  STAGE_KNOCKOUT = 1       // 10 min continuous gauntlet (CMZ mode switches + Battle Royale)
};

enum ArmPose : uint8_t {
  POSE_HOME_STOW    = 0,   // Compact safe boot/travel pose within 25x25x50 cm
  POSE_PICKUP_READY = 1,   // Forward reach for 20 g load pickup inside Loading Zone
  POSE_CARRY_SAFE   = 2,   // Stable carry pose (load held strictly by gripper, off chassis)
  POSE_DEPOSIT      = 3,   // Forward low release pose for 3-second Pocket deposit
  POSE_COMBAT_TUCK  = 4    // Low-CG retracted Battle Royale pose (load suspended off chassis)
};

enum ServoJoint : uint8_t {
  JOINT_BASE     = 0,
  JOINT_SHOULDER = 1,
  JOINT_ELBOW    = 2,
  JOINT_GRIPPER  = 3
};

struct SensorSnapshot {
  // HC-SR04 ultrasonics (filtered cm; 400.0f if out of range / timeout)
  float    usFrontCm;
  float    usLeftCm;
  float    usRightCm;
  bool     usFrontValid;
  bool     usLeftValid;
  bool     usRightValid;

  // TCRT5000 IR sensors (12-bit ADC1 raw 0..4095 + thresholded 3-bit mask)
  uint16_t irRawLeft;
  uint16_t irRawCenter;
  uint16_t irRawRight;
  uint8_t  irMask;         // bit2=Left, bit1=Center, bit0=Right (1 = black line)

  // 20-slot wheel encoders (68 mm CAD wheel OD -> ~10.68 mm/tick)
  int32_t  encTicksLeft;   // Signed directional ticks
  int32_t  encTicksRight;  // Signed directional ticks
  uint32_t encAbsLeft;     // Monotonic unsigned ticks
  uint32_t encAbsRight;    // Monotonic unsigned ticks
  float    distMmLeft;
  float    distMmRight;

  // Grip microswitch (debounced; true = 20 g load seated in jaws)
  bool     gripSwitchPressed;
};

struct GripState {
  bool loadDetected;       // Debounced microswitch state
  bool suddenDeathArmed;   // True once robot leaves Loading Zone with load
  bool gripLockActive;     // Blocks accidental single-tap gripper release
  bool loadLostAlarm;      // Latched if load disappears while suddenDeathArmed
};

struct RobotContext {
  RobotMode mode;
  RobotMode prevMode;
  LaneSide  lane;
  StageType stage;
  bool      ps4Connected;
  bool      ps4WasConnected;
  uint32_t  modeEnteredMs;
  uint32_t  lastControlTickMs;
};

// ============================================================================
//  EXPLICIT HARDWARE.INO API PROTOTYPES (Hardware Isolation Boundary)
// ============================================================================

void initHardware();
void updateHardware(uint32_t nowMs);
void readSensorSnapshot(SensorSnapshot &outSnap);

// Motors (2x L298N, 4x JGA25-370)
void setMotorSlewStep(int stepPer10Ms);
void setMotorTarget(int leftPwm, int rightPwm);
void stopMotorsImmediate();
void brakeMotorsImmediate();

// Servos & Gripper (4x MG996R with 50 cm height safety limits)
void applyArmPose(ArmPose pose);
void setServoTargetDeg(ServoJoint joint, float deg);
float getServoCurrentDeg(ServoJoint joint);
float getServoTargetDeg(ServoJoint joint);
void freezeArmInPlace();
void closeGripper();
void jogGripperClose(float stepDeg);
void clampGripperAtContact(float extraPreloadDeg);
bool requestGripperOpen(bool overrideComboHeld);
void armSuddenDeathLock(bool armed);
void setGripLock(bool locked);
bool isGripSwitchPressed();

// Encoders & IR Calibration Helpers
void resetEncoders();
float encoderTicksToMm(int32_t ticks);
void setIrThresholds(uint16_t leftT, uint16_t centerT, uint16_t rightT);

// Status LED & Non-Blocking Serial Bench Diagnostic (inside hardware.ino)
void updateStatusLed(uint32_t nowMs, RobotMode mode, bool ps4Connected, bool gripLocked);
void cancelBenchTest();
void handleSerialBenchTest(uint32_t nowMs, RobotMode currentMode);

// Mode Tab Entry Points (manual.ino, line_follower.ino, maze.ino; Step 5 adds sumo.ino)
void initManualMode(uint32_t nowMs);
void runManual(uint32_t nowMs);
void initLineFollowerMode(uint32_t nowMs);
void runLineFollower(uint32_t nowMs);
void initMazeMode(uint32_t nowMs);
void runMaze(uint32_t nowMs);
void clearSavedMazePaths();
void printSavedMazePaths();

// ============================================================================
//  GLOBAL ROBOT STATE
// ============================================================================

RobotContext   robot   = { MODE_IDLE_SAFE, MODE_IDLE_SAFE, LANE_A, STAGE_GROUP, false, false, 0, 0 };
SensorSnapshot sensors = {};
GripState      grip    = { false, false, false, false };

// Flag set from BT Core-0 disconnect callback; handled safely in Core-1 loop()
volatile bool g_ps4DisconnectFlag = false;

// Mode-switcher hold-to-confirm timers (non-blocking)
static uint32_t s_abortHoldStartMs  = 0;
static uint32_t s_modeHoldStartMs   = 0;
static uint8_t  s_pendingModeReq    = 0xFF;
static uint32_t s_laneHoldStartMs   = 0;
static uint32_t s_stageHoldStartMs  = 0;
static bool     s_laneToggleLatched = false;
static bool     s_stageToggleLatched = false;

// PS4 lightbar / rumble rate-limiter state
static uint8_t  s_ledR = 0, s_ledG = 0, s_ledB = 0;
static uint8_t  s_rumbleSmall = 0, s_rumbleLarge = 0;
static uint32_t s_rumbleEndMs = 0;
static volatile bool s_ps4FeedbackDirty = false;
static uint32_t s_lastPs4SendMs    = 0;

// One-time stub notice latch (removed as Steps 1.4–1.7 tabs are added)
static bool     s_stubNoticePrinted = false;

// ============================================================================
//  PS4 BLUETOOTH EVENT CALLBACKS (Minimal — Core 0 safe!)
// ============================================================================

void onPs4Connect() {
  s_ps4FeedbackDirty = true;
}

void onPs4Disconnect() {
  g_ps4DisconnectFlag = true;
}

// ============================================================================
//  HELPER FUNCTIONS — MODE NAMES, FEEDBACK & SAFE TRANSITIONS
// ============================================================================

const char* modeToString(RobotMode m) {
  switch (m) {
    case MODE_IDLE_SAFE:     return "IDLE_SAFE";
    case MODE_MANUAL:        return "MANUAL";
    case MODE_LINE_FOLLOWER: return "LINE_FOLLOWER";
    case MODE_MAZE:          return "MAZE";
    case MODE_COMBAT_SUMO:   return "COMBAT_SUMO";
    default:                 return "UNKNOWN";
  }
}

bool isAutonomousMode(RobotMode m) {
  return (m == MODE_LINE_FOLLOWER || m == MODE_MAZE);
}

void triggerPs4RumblePulse(uint8_t smallMotor, uint8_t largeMotor, uint32_t durationMs, uint32_t nowMs) {
  s_rumbleSmall = smallMotor;
  s_rumbleLarge = largeMotor;
  s_rumbleEndMs = nowMs + durationMs;
  s_ps4FeedbackDirty = true;
}

void refreshPs4Lightbar() {
  uint8_t r = 0, g = 0, b = 40;
  switch (robot.mode) {
    case MODE_IDLE_SAFE:
      // Blue = Lane A, Cyan = Lane B
      if (robot.lane == LANE_A) { r = 0; g = 0; b = 80; }
      else                      { r = 0; g = 80; b = 80; }
      break;
    case MODE_MANUAL:
      r = 0; g = 100; b = 0;       // Green
      break;
    case MODE_LINE_FOLLOWER:
      r = 110; g = 90; b = 0;      // Yellow
      break;
    case MODE_MAZE:
      r = 90; g = 0; b = 110;      // Purple
      break;
    case MODE_COMBAT_SUMO:
      r = 140; g = 0; b = 0;       // Red
      break;
  }
  if (r != s_ledR || g != s_ledG || b != s_ledB) {
    s_ledR = r;
    s_ledG = g;
    s_ledB = b;
    s_ps4FeedbackDirty = true;
  }
}

void servicePs4Feedback(uint32_t nowMs) {
  if (!robot.ps4Connected) {
    return;
  }
  if (s_rumbleEndMs != 0 && (int32_t)(nowMs - s_rumbleEndMs) >= 0) {
    s_rumbleSmall = 0;
    s_rumbleLarge = 0;
    s_rumbleEndMs = 0;
    s_ps4FeedbackDirty = true;
  }
  if (s_ps4FeedbackDirty && (nowMs - s_lastPs4SendMs >= PS4_FEEDBACK_MIN_MS)) {
    PS4.setLed(s_ledR, s_ledG, s_ledB);
    PS4.setRumble(s_rumbleSmall, s_rumbleLarge);
    PS4.sendToController();
    s_lastPs4SendMs = nowMs;
    s_ps4FeedbackDirty = false;
  }
}

void enterMode(RobotMode nextMode, uint32_t nowMs, const char* reason) {
  if (nextMode == robot.mode) {
    return;
  }

  // Cancel any bench test and stop motors cleanly on every mode transition.
  // NEVER drop servo/gripper holding torque here!
  cancelBenchTest();
  stopMotorsImmediate();

  robot.prevMode      = robot.mode;
  robot.mode          = nextMode;
  robot.modeEnteredMs = nowMs;
  s_stubNoticePrinted = false;

  // Mode-specific entry safety setup
  if (nextMode == MODE_IDLE_SAFE) {
    freezeArmInPlace();
  } else if (nextMode == MODE_MANUAL) {
    initManualMode(nowMs);
  } else if (nextMode == MODE_COMBAT_SUMO) {
    // Protect 20 g load immediately on entering Battle Royale
    setGripLock(true);
    applyArmPose(POSE_COMBAT_TUCK);
  } else if (nextMode == MODE_LINE_FOLLOWER) {
    initLineFollowerMode(nowMs);
  } else if (nextMode == MODE_MAZE) {
    initMazeMode(nowMs);
  } else if (isAutonomousMode(nextMode)) {
    // Reset encoders for deterministic autonomous distance/turn tracking
    resetEncoders();
  }

  refreshPs4Lightbar();
  triggerPs4RumblePulse(120, 0, 180, nowMs);

  Serial.print(F("[MODE] "));
  Serial.print(modeToString(robot.prevMode));
  Serial.print(F(" -> "));
  Serial.print(modeToString(robot.mode));
  Serial.print(F(" ("));
  Serial.print(reason);
  Serial.println(F(")"));
}

// ============================================================================
//  PS4 CONNECTION WATCHDOG & FAIL-SAFE STOP
// ============================================================================

void updatePs4Watchdog(uint32_t nowMs) {
  bool connectedNow = PS4.isConnected();

  if (g_ps4DisconnectFlag) {
    g_ps4DisconnectFlag = false;
    connectedNow = false;
  }

  robot.ps4Connected = connectedNow;

  // Edge: Newly connected
  if (connectedNow && !robot.ps4WasConnected) {
    Serial.println(F("[PS4] DualShock 4 CONNECTED."));
    refreshPs4Lightbar();
    triggerPs4RumblePulse(80, 0, 150, nowMs);
  }

  // Edge: Disconnected — execute fail-safe!
  if (!connectedNow && robot.ps4WasConnected) {
    Serial.println(F("[FAILSAFE] PS4 DualShock 4 DISCONNECTED!"));
    if (!isAutonomousMode(robot.mode) || FAILSAFE_STOP_AUTONOMOUS_ON_DISCONNECT) {
      // Stop drive motors immediately so robot cannot Ring-Out off the elevated arena,
      // while preserving full servo/gripper holding torque for the 20 g Sudden Death load!
      stopMotorsImmediate();
      freezeArmInPlace();
      enterMode(MODE_IDLE_SAFE, nowMs, "PS4_DISCONNECT_FAILSAFE");
    } else {
      // In competition autonomous mode (Maze / Line Follower), keep running legally
      // without wireless control per Rule 4.
      Serial.println(F("[FAILSAFE] Autonomous mode active — continuing without PS4."));
    }
  }

  // Continuous guard: never allow manual/combat modes to drive while disconnected
  if (!connectedNow && (robot.mode == MODE_MANUAL || robot.mode == MODE_COMBAT_SUMO)) {
    stopMotorsImmediate();
    freezeArmInPlace();
    enterMode(MODE_IDLE_SAFE, nowMs, "PS4_OFFLINE_GUARD");
  }

  robot.ps4WasConnected = connectedNow;
}

// ============================================================================
//  SAFE MODE SWITCHER (RULE 4 COMPLIANT)
// ============================================================================

void updateModeSwitcher(uint32_t nowMs) {
  if (!robot.ps4Connected) {
    s_abortHoldStartMs = 0;
    s_modeHoldStartMs  = 0;
    s_pendingModeReq   = 0xFF;
    return;
  }

  // 1. Emergency safe-stop to IDLE_SAFE from ANY mode: Hold SHARE for 300 ms
  if (PS4.Share()) {
    if (s_abortHoldStartMs == 0) {
      s_abortHoldStartMs = nowMs;
    } else if (nowMs - s_abortHoldStartMs >= ABORT_HOLD_CONFIRM_MS) {
      if (robot.mode != MODE_IDLE_SAFE) {
        enterMode(MODE_IDLE_SAFE, nowMs, "PS4_SHARE_ABORT");
      } else {
        stopMotorsImmediate();
        freezeArmInPlace();
      }
      s_abortHoldStartMs = 0;
      return;
    }
  } else {
    s_abortHoldStartMs = 0;
  }

  // 2. RULE 4 COMPLIANCE: During autonomous Line Follower or Maze, NEVER read
  //    drive sticks or triggers. Allow ONLY:
  //    - SHARE hold (handled above) to abort to IDLE_SAFE
  //    - OPTIONS + DOWN hold during Maze to enter MANUAL inside the Loading Zone
  //      (Group Stage has no Changing Mode Zones; switch is wireless per Rule 4).
  if (isAutonomousMode(robot.mode)) {
    if (robot.mode == MODE_MAZE && PS4.Options() && PS4.Down()) {
      if (s_modeHoldStartMs == 0) {
        s_modeHoldStartMs = nowMs;
      } else if (nowMs - s_modeHoldStartMs >= MODE_HOLD_CONFIRM_MS) {
        enterMode(MODE_MANUAL, nowMs, "MAZE_LOADING_ZONE_HANDOFF");
        s_modeHoldStartMs = 0;
      }
    } else {
      s_modeHoldStartMs = 0;
    }
    return;
  }

  // 3. Mode selection from IDLE_SAFE / MANUAL / COMBAT_SUMO:
  //    Hold OPTIONS + D-Pad for 500 ms:
  //      OPTIONS + UP    -> MODE_LINE_FOLLOWER
  //      OPTIONS + RIGHT -> MODE_MAZE
  //      OPTIONS + DOWN  -> MODE_MANUAL
  //      OPTIONS + LEFT  -> MODE_COMBAT_SUMO
  if (PS4.Options()) {
    uint8_t req = 0xFF;
    if      (PS4.Up())    req = MODE_LINE_FOLLOWER;
    else if (PS4.Right()) req = MODE_MAZE;
    else if (PS4.Down())  req = MODE_MANUAL;
    else if (PS4.Left())  req = MODE_COMBAT_SUMO;

    if (req != 0xFF) {
      if (req != s_pendingModeReq) {
        s_pendingModeReq  = req;
        s_modeHoldStartMs = nowMs;
      } else if (nowMs - s_modeHoldStartMs >= MODE_HOLD_CONFIRM_MS) {
        enterMode((RobotMode)req, nowMs, "PS4_OPTIONS_DPAD");
        s_pendingModeReq  = 0xFF;
        s_modeHoldStartMs = 0;
      }
    } else {
      s_pendingModeReq  = 0xFF;
      s_modeHoldStartMs = 0;
    }
  } else {
    s_pendingModeReq  = 0xFF;
    s_modeHoldStartMs = 0;
  }

  // 4. Pre-match arena config toggles (ONLY while in MODE_IDLE_SAFE):
  //    Hold L3 (500 ms) -> toggle LANE_A / LANE_B (mirrored track selector)
  //    Hold R3 (500 ms) -> toggle STAGE_GROUP / STAGE_KNOCKOUT
  if (robot.mode == MODE_IDLE_SAFE) {
    if (PS4.L3()) {
      if (s_laneHoldStartMs == 0) {
        s_laneHoldStartMs = nowMs;
      } else if (!s_laneToggleLatched && (nowMs - s_laneHoldStartMs >= CONFIG_HOLD_CONFIRM_MS)) {
        robot.lane = (robot.lane == LANE_A) ? LANE_B : LANE_A;
        s_laneToggleLatched = true;
        refreshPs4Lightbar();
        triggerPs4RumblePulse(100, 0, 120, nowMs);
        Serial.print(F("[CFG] Mirrored Lane -> "));
        Serial.println(robot.lane == LANE_A ? F("LANE_A") : F("LANE_B"));
      }
    } else {
      s_laneHoldStartMs   = 0;
      s_laneToggleLatched = false;
    }

    if (PS4.R3()) {
      if (s_stageHoldStartMs == 0) {
        s_stageHoldStartMs = nowMs;
      } else if (!s_stageToggleLatched && (nowMs - s_stageHoldStartMs >= CONFIG_HOLD_CONFIRM_MS)) {
        robot.stage = (robot.stage == STAGE_GROUP) ? STAGE_KNOCKOUT : STAGE_GROUP;
        s_stageToggleLatched = true;
        triggerPs4RumblePulse(0, 120, 160, nowMs);
        Serial.print(F("[CFG] Competition Stage -> "));
        Serial.println(robot.stage == STAGE_GROUP ? F("STAGE_GROUP (5 min, 2-way)") : F("STAGE_KNOCKOUT (10 min, Gauntlet)"));
      }
    } else {
      s_stageHoldStartMs   = 0;
      s_stageToggleLatched = false;
    }
  }
}

// ============================================================================
//  MODE HANDLERS (Step 1 Safe Dispatch — Steps 2+ replace stubs with .ino tabs)
// ============================================================================

void runModeIdleSafe(uint32_t nowMs) {
  (void)nowMs;
  // Motors remain stopped unless Serial bench-test motor sequence is active
  // inside hardware.ino.
}

void runModePlaceholderStub(const char* tabName) {
  stopMotorsImmediate();
  if (!s_stubNoticePrinted) {
    s_stubNoticePrinted = true;
    Serial.print(F("[STUB] "));
    Serial.print(tabName);
    Serial.println(F(" not yet implemented (Step 1 foundation only). Motors held safe."));
  }
}

// ============================================================================
//  ARDUINO SETUP & NON-BLOCKING MAIN LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  // Short non-blocking wait is avoided; small startup banner prints cleanly.
  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F(" Tech Rumble 2 — ESP32 DevKit V1 Firmware (Step 1)"));
  Serial.println(F(" Core 2.0.17 | PS4Controller (aed3) | Non-Blocking"));
  Serial.println(F("=================================================="));

  // 1. Initialize all hardware safely (motors OFF, arm smoothly stowed)
  initHardware();

  // 2. Initialize PS4 DualShock 4 Classic Bluetooth stack
  PS4.attachOnConnect(onPs4Connect);
  PS4.attachOnDisconnect(onPs4Disconnect);
  PS4.begin(PS4_MAC_ADDRESS);

  robot.mode              = MODE_IDLE_SAFE;
  robot.prevMode          = MODE_IDLE_SAFE;
  robot.modeEnteredMs     = millis();
  robot.lastControlTickMs = robot.modeEnteredMs;

  Serial.print(F("[PS4] Waiting for DualShock 4 on MAC: "));
  Serial.println(PS4_MAC_ADDRESS);
  Serial.println(F("[BOOT] Ready in MODE_IDLE_SAFE. Send '?' over Serial for bench tests."));
}

void loop() {
  const uint32_t nowMs = millis();

  // 1. High-frequency non-blocking hardware servicing:
  //    ultrasonics round-robin, IR sampling, grip debounce, motor slew-rate,
  //    20 ms smooth servo interpolation, and active bench test state machine.
  updateHardware(nowMs);

  // 2. Non-blocking Serial bench diagnostic console (guarded to MODE_IDLE_SAFE)
  #if ENABLE_SERIAL_DIAG
    handleSerialBenchTest(nowMs, robot.mode);
  #endif

  // 3. Fixed 50 Hz (20 ms) control scheduler
  if (nowMs - robot.lastControlTickMs >= CONTROL_TICK_MS) {
    robot.lastControlTickMs = nowMs;

    // Refresh clean sensor snapshot for control logic
    readSensorSnapshot(sensors);
    grip.loadDetected = sensors.gripSwitchPressed;
    if (grip.suddenDeathArmed && !grip.loadDetected) {
      grip.loadLostAlarm = true;
    }

    // Enforce PS4 disconnect fail-safe & safe mode transitions
    updatePs4Watchdog(nowMs);
    updateModeSwitcher(nowMs);

    // Dispatch current competition mode
    switch (robot.mode) {
      case MODE_IDLE_SAFE:
        runModeIdleSafe(nowMs);
        break;
      case MODE_MANUAL:
        runManual(nowMs);
        break;
      case MODE_LINE_FOLLOWER:
        runLineFollower(nowMs);
        break;
      case MODE_MAZE:
        runMaze(nowMs);
        break;
      case MODE_COMBAT_SUMO:
        // TODO(Step 5 / sumo.ino): replace with runCombatSumo(nowMs);
        runModePlaceholderStub("sumo.ino");
        break;
    }

    // Update Status LED & rate-limited PS4 lightbar/rumble feedback
    updateStatusLed(nowMs, robot.mode, robot.ps4Connected, grip.gripLockActive);
    servicePs4Feedback(nowMs);
  }
}

/*
 * ============================================================================
 *  Tech Rumble 2 — Hardware Abstraction Layer (Single Source of Truth)
 *  File: TechRumbleRobot/hardware.ino
 *
 *  STRICT HARDWARE ISOLATION:
 *   - ALL GPIO definitions, ESP32 Core 2.0.17 LEDC PWM setup, motor control,
 *     servo interpolation & 50 cm height clamps, HC-SR04 non-blocking ISRs,
 *     TCRT5000 ADC1 IR reads, 20-slot wheel encoder ISRs, grip switch debounce,
 *     Status LED patterns, and Serial bench-test helpers live strictly here.
 *   - Zero GPIO/PWM/ISR code is permitted in any other .ino tab.
 *
 *  PINOUT LOCKED TO CONTEXT.MD (30-Pin ESP32 DevKit V1 Safe):
 *   - GPIO 1/3 (UART0) untouched for USB Serial debugging/uploading.
 *   - GPIO 12 (MTDI) untouched to avoid flash-voltage bootloop.
 *   - ADC1 ONLY (GPIO 36/39/34) for IR sensors (Bluetooth blocks ADC2!).
 *   - Zero boot-glitch pins on all 4 MG996R servos (GPIO 16/17/21/22).
 *   - MANDATORY 1k/2k voltage dividers on all 3 HC-SR04 ECHO pins (5V -> 3.33V).
 * ============================================================================
 */

// ============================================================================
//  1. LOCKED GPIO PIN DEFINITIONS (30-Pin ESP32 DevKit V1)
// ============================================================================

// Drive Motors: 4x JGA25-370 via 2x L298N (Side-ganged: #1=Left FL+RL, #2=Right FR+RR)
#define PIN_MOTOR_L_EN      25   // LEDC HS Timer 0, Ch 0 (1 kHz, 8-bit)
#define PIN_MOTOR_L_IN1     26
#define PIN_MOTOR_L_IN2     27

#define PIN_MOTOR_R_EN      13   // LEDC HS Timer 0, Ch 1 (1 kHz, 8-bit)
#define PIN_MOTOR_R_IN1     14
#define PIN_MOTOR_R_IN2     23

// Robotic Arm Servos: 4x MG996R (External 5V Buck Converter ONLY + Common GND!)
#define PIN_SERVO_BASE      16   // LEDC HS Timer 1, Ch 2 (50 Hz, 16-bit) — Full Metal
#define PIN_SERVO_SHOULDER  17   // LEDC HS Timer 1, Ch 3 (50 Hz, 16-bit) — Full Metal
#define PIN_SERVO_ELBOW     21   // LEDC HS Timer 2, Ch 4 (50 Hz, 16-bit) — Full Metal
#define PIN_SERVO_GRIPPER   22   // LEDC HS Timer 2, Ch 5 (50 Hz, 16-bit) — Half Metal

// Line Sensors: 3x TCRT5000 (Input-only ADC1 pins — Bluetooth-safe!)
#define PIN_IR_LEFT         36   // VP (ADC1_CH0)
#define PIN_IR_CENTER       39   // VN (ADC1_CH3)
#define PIN_IR_RIGHT        34   //    (ADC1_CH6)

// Ultrasonics: 3x HC-SR04 (1k/2k voltage dividers MANDATORY on ECHO pins!)
#define PIN_US_FRONT_ECHO   35   // Input-only, ISR CHANGE
#define PIN_US_LEFT_ECHO    32   // ISR CHANGE
#define PIN_US_RIGHT_ECHO   33   // ISR CHANGE
#define PIN_US_FRONT_TRIG    5   // Dedicated Front trigger
#define PIN_US_SIDE_TRIG    15   // Shared Left+Right trigger (180 deg opposed across chassis)

// Wheel Encoders: 2x 20-Slot Optical Encoders (ISR RISING)
#define PIN_ENC_LEFT        18
#define PIN_ENC_RIGHT       19

// Grip Load Microswitch (INPUT_PULLUP, Active LOW when 20 g load is held)
#define PIN_GRIP_SWITCH      4

// Status LED (Built-in ESP32 DevKit V1 blue LED)
#define PIN_STATUS_LED       2

// ============================================================================
//  2. ESP32 ARDUINO CORE 2.0.17 LEDC CHANNEL & TIMER ALLOCATION
// ============================================================================

// HS Timer 0: DC Motors (1,000 Hz, 8-bit: 0..255)
#define LEDC_CH_MOTOR_L        0
#define LEDC_CH_MOTOR_R        1
#define LEDC_MOTOR_FREQ_HZ  1000
#define LEDC_MOTOR_RES_BITS    8

// HS Timer 1 & 2: Servos (50 Hz, 16-bit: 0..65535 -> ~0.3 us/count precision)
#define LEDC_CH_SERVO_BASE     2
#define LEDC_CH_SERVO_SHOULDER 3
#define LEDC_CH_SERVO_ELBOW    4
#define LEDC_CH_SERVO_GRIPPER  5
#define LEDC_SERVO_FREQ_HZ    50
#define LEDC_SERVO_RES_BITS   16

// ============================================================================
//  3. CALIBRATION CONSTANTS (Verify & update during Step 1 bench tests)
// ============================================================================

// Motor polarity flags (flip to true if a side runs backward during bench test 'm')
static const bool MOTOR_L_INVERT       = false;
static const bool MOTOR_R_INVERT       = false;
static const int  MOTOR_PWM_MAX        = 255;
static const int  MOTOR_DEADBAND_PWM   = 25;    // Below this, JGA25-370 stalls; clamp to 0
static const uint32_t MOTOR_SLEW_MS    = 10UL;  // Slew-rate update interval
static int        s_motorSlewStep      = 20;    // Max PWM change per 10 ms (smooth for 20 g load)

// MG996R PWM pulse bounds (microseconds)
static const float SERVO_MIN_US        = 500.0f;
static const float SERVO_MAX_US        = 2500.0f;
static const uint32_t SERVO_STEP_MS    = 20UL;  // 50 Hz servo interpolation frame
static const float SERVO_MAX_DEG_STEP  = 2.0f;  // ~100 deg/s smooth speed (no load whip!)

// HARD SERVO LIMITS — ENFORCES RULE 2 (MAX 50 CM VERTICAL HEIGHT AT ALL TIMES)
// Bench-verify with a tape measure before competition and tighten if needed!
static const float SERVO_LIMIT_MIN[4]  = {  15.0f,  20.0f,  25.0f,  30.0f }; // Base, Shld, Elb, Grip
static const float SERVO_LIMIT_MAX[4]  = { 165.0f, 135.0f, 145.0f, 125.0f }; // Shld/Elb capped <= 50cm
static const float ARM_HEIGHT_SUM_MAX  = 240.0f; // Extra envelope cap on (shoulder + elbow)

// Gripper calibrated endpoints (bench-tune using Serial 'a 3 <deg>')
static const float GRIPPER_OPEN_DEG    = 40.0f;
static const float GRIPPER_CLOSED_DEG  = 115.0f;

// Named Arm Poses: { Base, Shoulder, Elbow, Gripper }
// NOTE: Gripper angle in poses is NOT forced open if gripLockActive == true!
static const float ARM_POSES[5][4] = {
  { 90.0f,  35.0f,  55.0f, GRIPPER_OPEN_DEG   }, // POSE_HOME_STOW
  { 90.0f,  85.0f,  75.0f, GRIPPER_OPEN_DEG   }, // POSE_PICKUP_READY
  { 90.0f,  60.0f,  75.0f, GRIPPER_CLOSED_DEG }, // POSE_CARRY_SAFE (strictly off chassis!)
  { 90.0f,  95.0f,  80.0f, GRIPPER_CLOSED_DEG }, // POSE_DEPOSIT (driver opens with combo)
  { 90.0f,  45.0f,  60.0f, GRIPPER_CLOSED_DEG }  // POSE_COMBAT_TUCK (low-CG, load off deck)
};

// Wheel geometry from CAD (65 mm nominal tire, 68 mm actual OD, 20-slot disc)
static const float    WHEEL_OD_MM          = 68.0f;
static const uint16_t ENCODER_SLOTS        = 20;
static const float    WHEEL_MM_PER_TICK    = (3.14159265f * WHEEL_OD_MM) / (float)ENCODER_SLOTS;
static const uint32_t ENC_GLITCH_FILTER_US = 1500UL; // Rejects optical bounce (< 1.5 ms)

// HC-SR04 non-blocking ultrasonic timing
static const uint32_t US_PING_INTERVAL_MS  = 35UL;   // Alternating Front <-> Sides
static const uint32_t US_ECHO_MIN_US       = 120UL;  // ~2 cm minimum valid echo
static const uint32_t US_ECHO_MAX_US       = 23200UL;// ~400 cm maximum valid echo
static const uint32_t US_STALE_TIMEOUT_MS  = 160UL;  // Mark invalid if no echo within 2 cycles
static const float    US_MAX_RANGE_CM      = 400.0f;

// TCRT5000 IR configuration (matte white board, 25 mm black line)
static const bool     IR_BLACK_IS_HIGH     = true;   // Set false if module outputs LOW on black
static const uint32_t IR_SAMPLE_INTERVAL_MS = 5UL;   // 200 Hz sampling
static uint16_t       s_irThresh[3]        = { 2000, 2000, 2000 }; // L, C, R (12-bit 0..4095)

// Grip microswitch debounce
static const uint32_t GRIP_DEBOUNCE_MS     = 40UL;

// ============================================================================
//  4. INTERNAL HARDWARE STATE & THREAD-SAFE ISR STRUCTURES
// ============================================================================

static portMUX_TYPE s_hwMux = portMUX_INITIALIZER_UNLOCKED;

// --- Motors ---
static int      s_motorTargetL  = 0;
static int      s_motorTargetR  = 0;
static int      s_motorCurrentL = 0;
static int      s_motorCurrentR = 0;
static uint32_t s_lastMotorMs   = 0;
static volatile int8_t s_encDirL = 1;
static volatile int8_t s_encDirR = 1;

// --- Servos ---
static const uint8_t SERVO_LEDC_CH[4] = {
  LEDC_CH_SERVO_BASE, LEDC_CH_SERVO_SHOULDER, LEDC_CH_SERVO_ELBOW, LEDC_CH_SERVO_GRIPPER
};
static const uint8_t SERVO_PINS[4] = {
  PIN_SERVO_BASE, PIN_SERVO_SHOULDER, PIN_SERVO_ELBOW, PIN_SERVO_GRIPPER
};
static float    s_servoCurDeg[4] = { 90.0f, 35.0f, 55.0f, GRIPPER_OPEN_DEG };
static float    s_servoTgtDeg[4] = { 90.0f, 35.0f, 55.0f, GRIPPER_OPEN_DEG };
static uint32_t s_lastServoMs    = 0;

// --- Ultrasonic ISRs + 3-Sample Median Filter ---
struct UsChannelState {
  volatile uint32_t riseUs;
  volatile uint32_t pulseUs;
  volatile bool     newSample;
  volatile bool     inFlight;
  float             history[3];
  uint8_t           histIdx;
  float             filteredCm;
  uint32_t          lastValidMs;
  bool              valid;
};

static UsChannelState s_usFront = {};
static UsChannelState s_usLeft  = {};
static UsChannelState s_usRight = {};
static uint32_t       s_lastUsPingMs = 0;
static uint8_t        s_usPingPhase  = 0; // 0 = Front, 1 = Left+Right simultaneous

// --- Wheel Encoder ISRs ---
static volatile int32_t  s_encSignedL   = 0;
static volatile int32_t  s_encSignedR   = 0;
static volatile uint32_t s_encAbsL      = 0;
static volatile uint32_t s_encAbsR      = 0;
static volatile uint32_t s_encLastUsL   = 0;
static volatile uint32_t s_encLastUsR   = 0;

// --- IR Array ---
static uint16_t s_irRaw[3]      = { 0, 0, 0 };
static uint8_t  s_irMask        = 0;
static uint32_t s_lastIrMs      = 0;

// --- Grip Switch Debouncer ---
static bool     s_gripRawPrev     = false;
static bool     s_gripDebounced   = false;
static uint32_t s_gripDebounceMs  = 0;

// --- Status LED ---
static uint32_t s_lastLedToggleMs = 0;
static bool     s_ledState        = false;

// --- Non-Blocking Serial Bench-Test State Machine ---
static bool     s_diagStreamOn    = false;
static uint32_t s_lastDiagPrintMs = 0;
static uint8_t  s_motorTestStep   = 0;   // 0 = idle, 1..4 = L-fwd, L-rev, R-fwd, R-rev
static uint32_t s_motorTestMs     = 0;
static char     s_serialBuf[48];
static uint8_t  s_serialLen       = 0;

// ============================================================================
//  5. IRAM_ATTR INTERRUPT SERVICE ROUTINES (Zero Blocking!)
// ============================================================================

static inline void IRAM_ATTR handleUsEchoIsr(UsChannelState &ch, uint8_t pin) {
  const uint32_t nowUs = micros();
  const int level = digitalRead(pin);
  portENTER_CRITICAL_ISR(&s_hwMux);
  if (level == HIGH) {
    ch.riseUs   = nowUs;
    ch.inFlight = true;
  } else if (ch.inFlight) {
    ch.pulseUs   = nowUs - ch.riseUs;
    ch.inFlight  = false;
    ch.newSample = true;
  }
  portEXIT_CRITICAL_ISR(&s_hwMux);
}

void IRAM_ATTR isrUsFrontEcho() { handleUsEchoIsr(s_usFront, PIN_US_FRONT_ECHO); }
void IRAM_ATTR isrUsLeftEcho()  { handleUsEchoIsr(s_usLeft,  PIN_US_LEFT_ECHO);  }
void IRAM_ATTR isrUsRightEcho() { handleUsEchoIsr(s_usRight, PIN_US_RIGHT_ECHO); }

void IRAM_ATTR isrEncLeft() {
  const uint32_t nowUs = micros();
  portENTER_CRITICAL_ISR(&s_hwMux);
  if ((uint32_t)(nowUs - s_encLastUsL) >= ENC_GLITCH_FILTER_US) {
    s_encLastUsL = nowUs;
    s_encAbsL++;
    s_encSignedL += s_encDirL;
  }
  portEXIT_CRITICAL_ISR(&s_hwMux);
}

void IRAM_ATTR isrEncRight() {
  const uint32_t nowUs = micros();
  portENTER_CRITICAL_ISR(&s_hwMux);
  if ((uint32_t)(nowUs - s_encLastUsR) >= ENC_GLITCH_FILTER_US) {
    s_encLastUsR = nowUs;
    s_encAbsR++;
    s_encSignedR += s_encDirR;
  }
  portEXIT_CRITICAL_ISR(&s_hwMux);
}

// ============================================================================
//  6. INTERNAL MOTOR & SERVO LOW-LEVEL DRIVERS
// ============================================================================

static uint32_t servoDegToDuty16(float deg) {
  if (deg < 0.0f)   deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  const float pulseUs = SERVO_MIN_US + (deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  return (uint32_t)((pulseUs * 65535.0f) / 20000.0f);
}

static void writeServoHw(uint8_t idx, float deg) {
  ledcWrite(SERVO_LEDC_CH[idx], servoDegToDuty16(deg));
}

static void clampArmTargets() {
  for (uint8_t i = 0; i < 4; i++) {
    if (s_servoTgtDeg[i] < SERVO_LIMIT_MIN[i]) s_servoTgtDeg[i] = SERVO_LIMIT_MIN[i];
    if (s_servoTgtDeg[i] > SERVO_LIMIT_MAX[i]) s_servoTgtDeg[i] = SERVO_LIMIT_MAX[i];
  }
  // Extra 50 cm vertical height envelope guard on Shoulder + Elbow combined
  const float sumSE = s_servoTgtDeg[JOINT_SHOULDER] + s_servoTgtDeg[JOINT_ELBOW];
  if (sumSE > ARM_HEIGHT_SUM_MAX) {
    const float excess = sumSE - ARM_HEIGHT_SUM_MAX;
    s_servoTgtDeg[JOINT_ELBOW] -= excess;
    if (s_servoTgtDeg[JOINT_ELBOW] < SERVO_LIMIT_MIN[JOINT_ELBOW]) {
      s_servoTgtDeg[JOINT_ELBOW] = SERVO_LIMIT_MIN[JOINT_ELBOW];
    }
  }
}

static void applyMotorSideRaw(uint8_t in1Pin, uint8_t in2Pin, uint8_t ledcCh,
                              int pwm, bool invert, volatile int8_t &encDir) {
  if (pwm > MOTOR_PWM_MAX)  pwm = MOTOR_PWM_MAX;
  if (pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;

  if (abs(pwm) < MOTOR_DEADBAND_PWM) {
    // Coast stop (zero current draw, gentle on load)
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    ledcWrite(ledcCh, 0);
    return;
  }

  const int signedCmd = invert ? -pwm : pwm;
  if (signedCmd > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    ledcWrite(ledcCh, (uint32_t)signedCmd);
    encDir = (pwm > 0) ? 1 : -1;
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    ledcWrite(ledcCh, (uint32_t)(-signedCmd));
    encDir = (pwm > 0) ? 1 : -1;
  }
}

static void updateMotorsSlew(uint32_t nowMs) {
  if (nowMs - s_lastMotorMs < MOTOR_SLEW_MS) {
    return;
  }
  s_lastMotorMs = nowMs;

  int diffL = s_motorTargetL - s_motorCurrentL;
  if      (diffL >  s_motorSlewStep) s_motorCurrentL += s_motorSlewStep;
  else if (diffL < -s_motorSlewStep) s_motorCurrentL -= s_motorSlewStep;
  else                               s_motorCurrentL  = s_motorTargetL;

  int diffR = s_motorTargetR - s_motorCurrentR;
  if      (diffR >  s_motorSlewStep) s_motorCurrentR += s_motorSlewStep;
  else if (diffR < -s_motorSlewStep) s_motorCurrentR -= s_motorSlewStep;
  else                               s_motorCurrentR  = s_motorTargetR;

  applyMotorSideRaw(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, LEDC_CH_MOTOR_L,
                    s_motorCurrentL, MOTOR_L_INVERT, s_encDirL);
  applyMotorSideRaw(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, LEDC_CH_MOTOR_R,
                    s_motorCurrentR, MOTOR_R_INVERT, s_encDirR);
}

static void updateServosSmooth(uint32_t nowMs) {
  if (nowMs - s_lastServoMs < SERVO_STEP_MS) {
    return;
  }
  s_lastServoMs = nowMs;

  clampArmTargets();
  for (uint8_t i = 0; i < 4; i++) {
    const float diff = s_servoTgtDeg[i] - s_servoCurDeg[i];
    if (diff > SERVO_MAX_DEG_STEP) {
      s_servoCurDeg[i] += SERVO_MAX_DEG_STEP;
    } else if (diff < -SERVO_MAX_DEG_STEP) {
      s_servoCurDeg[i] -= SERVO_MAX_DEG_STEP;
    } else {
      s_servoCurDeg[i] = s_servoTgtDeg[i];
    }
    writeServoHw(i, s_servoCurDeg[i]);
  }
}

// ============================================================================
//  7. INTERNAL SENSOR SERVICING (Ultrasonics, IR Array, Grip Switch)
// ============================================================================

static float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

static void processUsChannel(UsChannelState &ch, uint32_t nowMs) {
  bool     hasNew  = false;
  uint32_t pulseUs = 0;

  portENTER_CRITICAL(&s_hwMux);
  if (ch.newSample) {
    hasNew       = true;
    pulseUs      = ch.pulseUs;
    ch.newSample = false;
  }
  portEXIT_CRITICAL(&s_hwMux);

  if (hasNew) {
    if (pulseUs >= US_ECHO_MIN_US && pulseUs <= US_ECHO_MAX_US) {
      const float cm = (float)pulseUs * 0.01715f; // (343 m/s) / 2 -> cm/us
      ch.history[ch.histIdx] = cm;
      ch.histIdx = (ch.histIdx + 1) % 3;
      ch.filteredCm  = median3(ch.history[0], ch.history[1], ch.history[2]);
      ch.lastValidMs = nowMs;
      ch.valid       = true;
    }
  }

  if (ch.valid && (nowMs - ch.lastValidMs > US_STALE_TIMEOUT_MS)) {
    ch.valid      = false;
    ch.filteredCm = US_MAX_RANGE_CM;
  }
}

static void triggerUsPin(uint8_t trigPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Standard 10 us HC-SR04 trigger pulse
  digitalWrite(trigPin, LOW);
}

static void updateUltrasonics(uint32_t nowMs) {
  processUsChannel(s_usFront, nowMs);
  processUsChannel(s_usLeft,  nowMs);
  processUsChannel(s_usRight, nowMs);

  if (nowMs - s_lastUsPingMs < US_PING_INTERVAL_MS) {
    return;
  }
  s_lastUsPingMs = nowMs;

  if (s_usPingPhase == 0) {
    portENTER_CRITICAL(&s_hwMux);
    s_usFront.inFlight = false;
    portEXIT_CRITICAL(&s_hwMux);
    triggerUsPin(PIN_US_FRONT_TRIG);
    s_usPingPhase = 1;
  } else {
    // Left and Right face 180 deg opposite across the chassis: simultaneous
    // trigger gives time-aligned corridor centering (leftCm - rightCm).
    portENTER_CRITICAL(&s_hwMux);
    s_usLeft.inFlight  = false;
    s_usRight.inFlight = false;
    portEXIT_CRITICAL(&s_hwMux);
    triggerUsPin(PIN_US_SIDE_TRIG);
    s_usPingPhase = 0;
  }
}

static void updateIrArray(uint32_t nowMs) {
  if (nowMs - s_lastIrMs < IR_SAMPLE_INTERVAL_MS) {
    return;
  }
  s_lastIrMs = nowMs;

  s_irRaw[0] = (uint16_t)analogRead(PIN_IR_LEFT);
  s_irRaw[1] = (uint16_t)analogRead(PIN_IR_CENTER);
  s_irRaw[2] = (uint16_t)analogRead(PIN_IR_RIGHT);

  uint8_t mask = 0;
  for (uint8_t i = 0; i < 3; i++) {
    const bool isBlack = IR_BLACK_IS_HIGH ? (s_irRaw[i] >= s_irThresh[i])
                                          : (s_irRaw[i] <= s_irThresh[i]);
    if (isBlack) {
      mask |= (uint8_t)(1U << (2 - i)); // bit2=Left, bit1=Center, bit0=Right
    }
  }
  s_irMask = mask;
}

static void updateGripSwitch(uint32_t nowMs) {
  // Active LOW microswitch wired to GND with INPUT_PULLUP
  const bool rawPressed = (digitalRead(PIN_GRIP_SWITCH) == LOW);
  if (rawPressed != s_gripRawPrev) {
    s_gripRawPrev    = rawPressed;
    s_gripDebounceMs = nowMs;
  }
  if (nowMs - s_gripDebounceMs >= GRIP_DEBOUNCE_MS) {
    s_gripDebounced = rawPressed;
  }
}

static void updateBenchMotorSequence(uint32_t nowMs) {
  if (s_motorTestStep == 0) {
    return;
  }
  const uint32_t elapsed = nowMs - s_motorTestMs;
  if (elapsed < 700UL) {
    return;
  }
  s_motorTestMs = nowMs;
  s_motorTestStep++;

  switch (s_motorTestStep) {
    case 2:
      Serial.println(F("[BENCH] Left motors REVERSE (PWM -120)..."));
      setMotorTarget(-120, 0);
      break;
    case 3:
      Serial.println(F("[BENCH] Right motors FORWARD (PWM +120)..."));
      setMotorTarget(0, 120);
      break;
    case 4:
      Serial.println(F("[BENCH] Right motors REVERSE (PWM -120)..."));
      setMotorTarget(0, -120);
      break;
    default:
      s_motorTestStep = 0;
      stopMotorsImmediate();
      Serial.println(F("[BENCH] Motor bench sequence COMPLETE."));
      break;
  }
}

// ============================================================================
//  8. PUBLIC HARDWARE API (Called by TechRumbleRobot.ino & Mode Tabs)
// ============================================================================

void initHardware() {
  // 1. Motor direction pins OFF first (zero twitch on power-up!)
  pinMode(PIN_MOTOR_L_IN1, OUTPUT); digitalWrite(PIN_MOTOR_L_IN1, LOW);
  pinMode(PIN_MOTOR_L_IN2, OUTPUT); digitalWrite(PIN_MOTOR_L_IN2, LOW);
  pinMode(PIN_MOTOR_R_IN1, OUTPUT); digitalWrite(PIN_MOTOR_R_IN1, LOW);
  pinMode(PIN_MOTOR_R_IN2, OUTPUT); digitalWrite(PIN_MOTOR_R_IN2, LOW);

  // 2. ESP32 Core 2.0.17 LEDC setup — HS Timer 0 for DC motors
  ledcSetup(LEDC_CH_MOTOR_L, LEDC_MOTOR_FREQ_HZ, LEDC_MOTOR_RES_BITS);
  ledcAttachPin(PIN_MOTOR_L_EN, LEDC_CH_MOTOR_L);
  ledcWrite(LEDC_CH_MOTOR_L, 0);

  ledcSetup(LEDC_CH_MOTOR_R, LEDC_MOTOR_FREQ_HZ, LEDC_MOTOR_RES_BITS);
  ledcAttachPin(PIN_MOTOR_R_EN, LEDC_CH_MOTOR_R);
  ledcWrite(LEDC_CH_MOTOR_R, 0);

  // 3. ESP32 Core 2.0.17 LEDC setup — HS Timers 1 & 2 for 4x MG996R servos
  for (uint8_t i = 0; i < 4; i++) {
    ledcSetup(SERVO_LEDC_CH[i], LEDC_SERVO_FREQ_HZ, LEDC_SERVO_RES_BITS);
    ledcAttachPin(SERVO_PINS[i], SERVO_LEDC_CH[i]);
    s_servoCurDeg[i] = ARM_POSES[POSE_HOME_STOW][i];
    s_servoTgtDeg[i] = ARM_POSES[POSE_HOME_STOW][i];
  }
  clampArmTargets();
  for (uint8_t i = 0; i < 4; i++) {
    s_servoCurDeg[i] = s_servoTgtDeg[i];
    writeServoHw(i, s_servoCurDeg[i]);
  }

  // 4. TCRT5000 IR sensors on ADC1 (12-bit, 11 dB attenuation -> 0..3.3 V)
  analogReadResolution(12);
  pinMode(PIN_IR_LEFT,   INPUT);
  pinMode(PIN_IR_CENTER, INPUT);
  pinMode(PIN_IR_RIGHT,  INPUT);
  analogSetPinAttenuation(PIN_IR_LEFT,   ADC_11db);
  analogSetPinAttenuation(PIN_IR_CENTER, ADC_11db);
  analogSetPinAttenuation(PIN_IR_RIGHT,  ADC_11db);

  // 5. HC-SR04 Ultrasonics (TRIG outputs LOW + ECHO CHANGE ISRs)
  pinMode(PIN_US_FRONT_TRIG, OUTPUT); digitalWrite(PIN_US_FRONT_TRIG, LOW);
  pinMode(PIN_US_SIDE_TRIG,  OUTPUT); digitalWrite(PIN_US_SIDE_TRIG,  LOW);
  pinMode(PIN_US_FRONT_ECHO, INPUT);
  pinMode(PIN_US_LEFT_ECHO,  INPUT);
  pinMode(PIN_US_RIGHT_ECHO, INPUT);

  for (uint8_t i = 0; i < 3; i++) {
    s_usFront.history[i] = US_MAX_RANGE_CM;
    s_usLeft.history[i]  = US_MAX_RANGE_CM;
    s_usRight.history[i] = US_MAX_RANGE_CM;
  }
  s_usFront.filteredCm = US_MAX_RANGE_CM;
  s_usLeft.filteredCm  = US_MAX_RANGE_CM;
  s_usRight.filteredCm = US_MAX_RANGE_CM;

  attachInterrupt(digitalPinToInterrupt(PIN_US_FRONT_ECHO), isrUsFrontEcho, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_US_LEFT_ECHO),  isrUsLeftEcho,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_US_RIGHT_ECHO), isrUsRightEcho, CHANGE);

  // 6. 20-Slot Wheel Encoders (RISING ISRs with pull-up)
  pinMode(PIN_ENC_LEFT,  INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT),  isrEncLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT), isrEncRight, RISING);

  // 7. Grip Microswitch (Active LOW to GND)
  pinMode(PIN_GRIP_SWITCH, INPUT_PULLUP);
  s_gripDebounced = (digitalRead(PIN_GRIP_SWITCH) == LOW);
  s_gripRawPrev   = s_gripDebounced;

  // 8. Built-in Status LED
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  const uint32_t nowMs = millis();
  s_lastMotorMs   = nowMs;
  s_lastServoMs   = nowMs;
  s_lastUsPingMs  = nowMs;
  s_lastIrMs      = nowMs;
  s_gripDebounceMs = nowMs;

  Serial.println(F("[HW] Init complete: 2x L298N (1kHz), 4x MG996R (50Hz/16-bit), 3x US ISR, 3x IR ADC1, 2x Enc ISR."));
}

void updateHardware(uint32_t nowMs) {
  updateBenchMotorSequence(nowMs);
  updateMotorsSlew(nowMs);
  updateServosSmooth(nowMs);
  updateUltrasonics(nowMs);
  updateIrArray(nowMs);
  updateGripSwitch(nowMs);
}

void readSensorSnapshot(SensorSnapshot &outSnap) {
  outSnap.usFrontCm    = s_usFront.filteredCm;
  outSnap.usLeftCm     = s_usLeft.filteredCm;
  outSnap.usRightCm    = s_usRight.filteredCm;
  outSnap.usFrontValid = s_usFront.valid;
  outSnap.usLeftValid  = s_usLeft.valid;
  outSnap.usRightValid = s_usRight.valid;

  outSnap.irRawLeft    = s_irRaw[0];
  outSnap.irRawCenter  = s_irRaw[1];
  outSnap.irRawRight   = s_irRaw[2];
  outSnap.irMask       = s_irMask;

  portENTER_CRITICAL(&s_hwMux);
  outSnap.encTicksLeft  = s_encSignedL;
  outSnap.encTicksRight = s_encSignedR;
  outSnap.encAbsLeft    = s_encAbsL;
  outSnap.encAbsRight   = s_encAbsR;
  portEXIT_CRITICAL(&s_hwMux);

  outSnap.distMmLeft        = encoderTicksToMm(outSnap.encTicksLeft);
  outSnap.distMmRight       = encoderTicksToMm(outSnap.encTicksRight);
  outSnap.gripSwitchPressed = s_gripDebounced;
}

// --- Motors Public API ---

void setMotorSlewStep(int stepPer10Ms) {
  if (stepPer10Ms < 5)   stepPer10Ms = 5;
  if (stepPer10Ms > 255) stepPer10Ms = 255;
  s_motorSlewStep = stepPer10Ms;
}

void setMotorTarget(int leftPwm, int rightPwm) {
  s_motorTargetL = constrain(leftPwm,  -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  s_motorTargetR = constrain(rightPwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
}

void stopMotorsImmediate() {
  s_motorTargetL  = 0;
  s_motorTargetR  = 0;
  s_motorCurrentL = 0;
  s_motorCurrentR = 0;
  digitalWrite(PIN_MOTOR_L_IN1, LOW);
  digitalWrite(PIN_MOTOR_L_IN2, LOW);
  ledcWrite(LEDC_CH_MOTOR_L, 0);
  digitalWrite(PIN_MOTOR_R_IN1, LOW);
  digitalWrite(PIN_MOTOR_R_IN2, LOW);
  ledcWrite(LEDC_CH_MOTOR_R, 0);
}

void brakeMotorsImmediate() {
  s_motorTargetL  = 0;
  s_motorTargetR  = 0;
  s_motorCurrentL = 0;
  s_motorCurrentR = 0;
  // Active L298N H-bridge short-brake (IN1=HIGH, IN2=HIGH, EN=255)
  digitalWrite(PIN_MOTOR_L_IN1, HIGH);
  digitalWrite(PIN_MOTOR_L_IN2, HIGH);
  ledcWrite(LEDC_CH_MOTOR_L, MOTOR_PWM_MAX);
  digitalWrite(PIN_MOTOR_R_IN1, HIGH);
  digitalWrite(PIN_MOTOR_R_IN2, HIGH);
  ledcWrite(LEDC_CH_MOTOR_R, MOTOR_PWM_MAX);
}

// --- Servos & Sudden Death Grip-Lock API ---

void applyArmPose(ArmPose pose) {
  const uint8_t idx = (uint8_t)pose;
  if (idx >= 5) return;

  s_servoTgtDeg[JOINT_BASE]     = ARM_POSES[idx][JOINT_BASE];
  s_servoTgtDeg[JOINT_SHOULDER] = ARM_POSES[idx][JOINT_SHOULDER];
  s_servoTgtDeg[JOINT_ELBOW]    = ARM_POSES[idx][JOINT_ELBOW];

  // SAFETY: Never open gripper via a pose preset while Grip-Lock is active!
  if (!grip.gripLockActive) {
    s_servoTgtDeg[JOINT_GRIPPER] = ARM_POSES[idx][JOINT_GRIPPER];
  } else {
    s_servoTgtDeg[JOINT_GRIPPER] = GRIPPER_CLOSED_DEG;
  }
  clampArmTargets();
}

void setServoTargetDeg(ServoJoint joint, float deg) {
  const uint8_t idx = (uint8_t)joint;
  if (idx >= 4) return;

  // SAFETY: Prevent direct joint commands from opening gripper when Grip-Lock is active
  if (joint == JOINT_GRIPPER && grip.gripLockActive) {
    if (GRIPPER_OPEN_DEG < GRIPPER_CLOSED_DEG && deg < s_servoCurDeg[JOINT_GRIPPER]) {
      return;
    }
    if (GRIPPER_OPEN_DEG > GRIPPER_CLOSED_DEG && deg > s_servoCurDeg[JOINT_GRIPPER]) {
      return;
    }
  }
  s_servoTgtDeg[idx] = deg;
  clampArmTargets();
}

float getServoCurrentDeg(ServoJoint joint) {
  return ((uint8_t)joint < 4) ? s_servoCurDeg[(uint8_t)joint] : 0.0f;
}

float getServoTargetDeg(ServoJoint joint) {
  return ((uint8_t)joint < 4) ? s_servoTgtDeg[(uint8_t)joint] : 0.0f;
}

void freezeArmInPlace() {
  // Keeps full PWM holding torque on all 4 servos (NEVER detaches LEDC!)
  for (uint8_t i = 0; i < 4; i++) {
    s_servoTgtDeg[i] = s_servoCurDeg[i];
  }
}

void closeGripper() {
  s_servoTgtDeg[JOINT_GRIPPER] = GRIPPER_CLOSED_DEG;
  clampArmTargets();
}

bool requestGripperOpen(bool overrideComboHeld) {
  if (grip.gripLockActive && !overrideComboHeld) {
    Serial.println(F("[GRIP] Release BLOCKED by Sudden Death Grip-Lock!"));
    return false;
  }
  grip.gripLockActive   = false;
  grip.suddenDeathArmed = false;
  grip.loadLostAlarm    = false;
  s_servoTgtDeg[JOINT_GRIPPER] = GRIPPER_OPEN_DEG;
  clampArmTargets();
  Serial.println(F("[GRIP] Gripper OPENED."));
  return true;
}

void armSuddenDeathLock(bool armed) {
  grip.suddenDeathArmed = armed;
  if (armed) {
    grip.gripLockActive = true;
    grip.loadLostAlarm  = false;
    closeGripper();
  }
}

void setGripLock(bool locked) {
  grip.gripLockActive = locked;
  if (locked) {
    closeGripper();
  }
}

bool isGripSwitchPressed() {
  return s_gripDebounced;
}

// --- Encoders & IR Calibration API ---

void resetEncoders() {
  portENTER_CRITICAL(&s_hwMux);
  s_encSignedL = 0;
  s_encSignedR = 0;
  s_encAbsL    = 0;
  s_encAbsR    = 0;
  portEXIT_CRITICAL(&s_hwMux);
}

float encoderTicksToMm(int32_t ticks) {
  return (float)ticks * WHEEL_MM_PER_TICK;
}

void setIrThresholds(uint16_t leftT, uint16_t centerT, uint16_t rightT) {
  s_irThresh[0] = leftT;
  s_irThresh[1] = centerT;
  s_irThresh[2] = rightT;
}

// --- Status LED API (GPIO 2) ---

void updateStatusLed(uint32_t nowMs, RobotMode mode, bool ps4Connected, bool gripLocked) {
  uint32_t periodMs = 500UL;
  if (mode == MODE_MANUAL) {
    // Solid ON in Manual (or tiny pulse if Grip-Lock armed)
    if (!gripLocked) {
      digitalWrite(PIN_STATUS_LED, HIGH);
      s_ledState = true;
      return;
    }
    periodMs = 1000UL;
  } else if (mode == MODE_IDLE_SAFE) {
    periodMs = ps4Connected ? 250UL : 600UL;
  } else if (mode == MODE_LINE_FOLLOWER) {
    periodMs = 150UL;
  } else if (mode == MODE_MAZE) {
    periodMs = 300UL;
  } else if (mode == MODE_COMBAT_SUMO) {
    periodMs = 80UL;
  }

  if (nowMs - s_lastLedToggleMs >= periodMs) {
    s_lastLedToggleMs = nowMs;
    s_ledState = !s_ledState;
    digitalWrite(PIN_STATUS_LED, s_ledState ? HIGH : LOW);
  }
}

void cancelBenchTest() {
  if (s_motorTestStep != 0) {
    s_motorTestStep = 0;
    stopMotorsImmediate();
  }
}

// ============================================================================
//  9. NON-BLOCKING SERIAL BENCH-TEST HELPER (Active Only in MODE_IDLE_SAFE)
// ============================================================================

static void printBenchMenu() {
  Serial.println(F("\n--- Tech Rumble 2 Serial Bench-Test Menu (IDLE_SAFE only) ---"));
  Serial.println(F("  ? or h       : Print this help menu"));
  Serial.println(F("  s            : Toggle 5 Hz live sensor telemetry stream"));
  Serial.println(F("  m            : Run safe 4-step motor direction test (L/R Fwd/Rev)"));
  Serial.println(F("  x            : Emergency STOP motors & cancel bench test"));
  Serial.println(F("  1..5         : Arm poses (1=HOME 2=PICKUP 3=CARRY 4=DEPOSIT 5=COMBAT)"));
  Serial.println(F("  g / o        : Close gripper / Open gripper (blocked if Grip-Lock ON)"));
  Serial.println(F("  u            : Toggle Grip-Lock ON/OFF (bench override)"));
  Serial.println(F("  r            : Reset wheel encoder counters to 0"));
  Serial.println(F("  a <j> <deg>  : Move servo joint j (0=Base 1=Shld 2=Elb 3=Grip) to deg"));
  Serial.println(F("  t <val>      : Set all 3 IR black/white thresholds (0..4095)"));
  Serial.println(F("-------------------------------------------------------------\n"));
}

static void printTelemetryLine() {
  SensorSnapshot snap;
  readSensorSnapshot(snap);
  Serial.print(F("[TEL] US(cm) F:"));
  Serial.print(snap.usFrontValid ? snap.usFrontCm : -1.0f, 1);
  Serial.print(F(" L:"));
  Serial.print(snap.usLeftValid  ? snap.usLeftCm  : -1.0f, 1);
  Serial.print(F(" R:"));
  Serial.print(snap.usRightValid ? snap.usRightCm : -1.0f, 1);
  Serial.print(F(" | IR("));
  Serial.print((snap.irMask >> 2) & 1);
  Serial.print((snap.irMask >> 1) & 1);
  Serial.print(snap.irMask & 1);
  Serial.print(F("): "));
  Serial.print(snap.irRawLeft); Serial.print(',');
  Serial.print(snap.irRawCenter); Serial.print(',');
  Serial.print(snap.irRawRight);
  Serial.print(F(" | ENC L:"));
  Serial.print(snap.encTicksLeft);
  Serial.print(F(" R:"));
  Serial.print(snap.encTicksRight);
  Serial.print(F(" | GRIP:"));
  Serial.print(snap.gripSwitchPressed ? F("HELD") : F("EMPTY"));
  Serial.print(grip.gripLockActive ? F("(LOCKED)") : F("(OPEN)"));
  Serial.print(F(" | ARM:"));
  Serial.print((int)s_servoCurDeg[0]); Serial.print(',');
  Serial.print((int)s_servoCurDeg[1]); Serial.print(',');
  Serial.print((int)s_servoCurDeg[2]); Serial.print(',');
  Serial.print((int)s_servoCurDeg[3]);
  Serial.print(F(" | PS4:"));
  Serial.println(robot.ps4Connected ? F("OK") : F("WAIT"));
}

static void executeBenchLine(const char* line, uint32_t nowMs, RobotMode currentMode) {
  if (line[0] == '\0') return;

  // 'x' emergency stop is ALWAYS allowed
  if (line[0] == 'x' || line[0] == 'X') {
    cancelBenchTest();
    stopMotorsImmediate();
    freezeArmInPlace();
    Serial.println(F("[BENCH] EMERGENCY STOP — motors 0, arm frozen."));
    return;
  }
  if (line[0] == '?' || line[0] == 'h' || line[0] == 'H') {
    printBenchMenu();
    return;
  }
  if (line[0] == 's' || line[0] == 'S') {
    s_diagStreamOn = !s_diagStreamOn;
    Serial.print(F("[BENCH] Sensor stream -> "));
    Serial.println(s_diagStreamOn ? F("ON") : F("OFF"));
    return;
  }

  // Block actuator bench commands outside MODE_IDLE_SAFE
  if (currentMode != MODE_IDLE_SAFE) {
    Serial.println(F("[BENCH] Ignored: switch to MODE_IDLE_SAFE first."));
    return;
  }

  switch (line[0]) {
    case 'm':
    case 'M':
      s_motorTestStep = 1;
      s_motorTestMs   = nowMs;
      Serial.println(F("[BENCH] Left motors FORWARD (PWM +120)..."));
      setMotorTarget(120, 0);
      break;
    case '1': applyArmPose(POSE_HOME_STOW);    Serial.println(F("[BENCH] Pose -> HOME_STOW"));    break;
    case '2': applyArmPose(POSE_PICKUP_READY); Serial.println(F("[BENCH] Pose -> PICKUP_READY")); break;
    case '3': applyArmPose(POSE_CARRY_SAFE);   Serial.println(F("[BENCH] Pose -> CARRY_SAFE"));   break;
    case '4': applyArmPose(POSE_DEPOSIT);      Serial.println(F("[BENCH] Pose -> DEPOSIT"));      break;
    case '5': applyArmPose(POSE_COMBAT_TUCK);  Serial.println(F("[BENCH] Pose -> COMBAT_TUCK"));  break;
    case 'g':
    case 'G':
      closeGripper();
      Serial.println(F("[BENCH] Gripper CLOSE."));
      break;
    case 'o':
    case 'O':
      requestGripperOpen(false);
      break;
    case 'u':
    case 'U':
      setGripLock(!grip.gripLockActive);
      Serial.print(F("[BENCH] Grip-Lock -> "));
      Serial.println(grip.gripLockActive ? F("LOCKED") : F("UNLOCKED"));
      break;
    case 'r':
    case 'R':
      resetEncoders();
      Serial.println(F("[BENCH] Encoders reset to 0."));
      break;
    case 'a':
    case 'A': {
      int j = -1;
      float deg = 90.0f;
      if (sscanf(line + 1, "%d %f", &j, &deg) == 2 && j >= 0 && j < 4) {
        setServoTargetDeg((ServoJoint)j, deg);
        Serial.print(F("[BENCH] Servo ")); Serial.print(j);
        Serial.print(F(" target -> ")); Serial.println(s_servoTgtDeg[j], 1);
      } else {
        Serial.println(F("[BENCH] Usage: a <joint 0..3> <deg>"));
      }
      break;
    }
    case 't':
    case 'T': {
      int thr = 0;
      if (sscanf(line + 1, "%d", &thr) == 1 && thr >= 0 && thr <= 4095) {
        setIrThresholds((uint16_t)thr, (uint16_t)thr, (uint16_t)thr);
        Serial.print(F("[BENCH] IR threshold -> ")); Serial.println(thr);
      } else {
        Serial.println(F("[BENCH] Usage: t <0..4095>"));
      }
      break;
    }
    default:
      Serial.println(F("[BENCH] Unknown cmd. Send '?' for help."));
      break;
  }
}

void handleSerialBenchTest(uint32_t nowMs, RobotMode currentMode) {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (s_serialLen > 0) {
        s_serialBuf[s_serialLen] = '\0';
        executeBenchLine(s_serialBuf, nowMs, currentMode);
        s_serialLen = 0;
      }
    } else if (s_serialLen < sizeof(s_serialBuf) - 1) {
      s_serialBuf[s_serialLen++] = c;
    }
  }

  if (s_diagStreamOn && (nowMs - s_lastDiagPrintMs >= 200UL)) {
    s_lastDiagPrintMs = nowMs;
    printTelemetryLine();
  }
}

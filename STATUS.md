# Tech Rumble 2 — Project Status Tracker

**Last Updated:** 2026-11-26  
**Current Phase:** Phase 1 — Step 3 Complete (Awaiting Approval for Step 4)  
**Git Branch:** `main` (tracking `origin/main` → `https://github.com/MrMorale-007/Tech-Rumble`)

---

## Completed Milestones

### Phase 0: Repo Setup, Audit & Workspace Scaffolding
- [x] **Git Initialization & Remote Sync:** Initialized local git repository, connected `origin` to `https://github.com/MrMorale-007/Tech-Rumble`, fetched all history, and checked out `main` cleanly without modifying local `docs/`.
- [x] **Remote Repo Audit:** Verified `origin/main` has 2 commits (`ca359c7`, `c9bd2a4`) containing only `README.md`.
- [x] **Rulebook & Arena Guide Audit:** Extracted and visually verified all 9 pages of `docs/RULE BOOK .pdf`, all 5 pages of `docs/Tech Rumble Arena Guide.pdf`, both CAD renders in `docs/Car Images/`, and the SolidWorks inventory in `docs/Car Design/`.
- [x] **BOM Cross-Audit ([Car Components.docx](file:///home/goodkid/Desktop/Tech%20Rumble/docs/Car%20Components.docx)):** Confirmed 100% component match with `CONTEXT.md` and `hardware.ino` (30/38-pin DevKit V1, 4× JGA25-370, 2× L298N, 3× Full Metal + 1× Half Metal MG996R, 3× HC-SR04 + 1k/2k dividers, 3× TCRT5000, 2× 20-slot encoders, grip switch, 5V buck, `PB-1601`, 3S LiPo/3×18650, `470–1000 µF` decoupling caps). Flagged 2 Rule 9.2 inspection items missing from the BOM: main battery lead fuse and LiPo protective casing.
- [x] **Rules Checklist:** Created [RULES_CHECKLIST.md](file:///home/goodkid/Desktop/Tech%20Rumble/RULES_CHECKLIST.md) with all limits, Sudden Death rules, timeouts, Battle Royale rules, and arena dimensions.
- [x] **Context & Pinout Lock:** Created [CONTEXT.md](file:///home/goodkid/Desktop/Tech%20Rumble/CONTEXT.md) with locked BOM, 30-pin safe ESP32 DevKit V1 pinout, and Core `2.0.17` LEDC timer allocation.
- [x] **Always-On Workspace Sync Rule:** Created [.agents/rules/sync.md](file:///home/goodkid/Desktop/Tech%20Rumble/.agents/rules/sync.md) (`trigger: always_on`), including automatic post-step `STATUS.md` sync, git commit, and push to `origin`.
- [x] **Phase 1 Plan:** Created [IMPLEMENTATION_PLAN.md](file:///home/goodkid/Desktop/Tech%20Rumble/IMPLEMENTATION_PLAN.md).

### Phase 1 — Step 1: Core State Machine, Hardware Layer & Bench-Test Helper
- [x] **Created [TechRumbleRobot.ino](file:///home/goodkid/Desktop/Tech%20Rumble/TechRumbleRobot/TechRumbleRobot.ino):**
  - Strictly flat `.ino` structure (zero `.h`/`.cpp` files; zero GPIO/PWM/ISR code outside `hardware.ino`).
  - `PS4_MAC_ADDRESS` placeholder (`"01:02:03:04:05:06"`) with `PS4Controller` (`aed3`) DualShock 4 integration.
  - Default boot into `MODE_IDLE_SAFE` (prevents Rule 5 false starts).
  - Non-blocking `millis()` scheduler (`20 ms` / `50 Hz` control tick + continuous hardware servicing).
  - Rule 4 compliant mode switcher (`Share` 300 ms hold = emergency stop to `MODE_IDLE_SAFE`; `Options + D-Pad` 500 ms hold = mode select; `L3`/`R3` in `IDLE_SAFE` = mirrored `LANE_A`/`LANE_B` and `STAGE_GROUP`/`STAGE_KNOCKOUT` toggles; PS4 drive inputs completely locked out in autonomous `MODE_LINE_FOLLOWER` and `MODE_MAZE`).
  - PS4 disconnect fail-safe watchdog: immediately stops drive motors and freezes arm/gripper with full holding torque on disconnect in manual/combat modes (never drops the 20 g Sudden Death load).
- [x] **Created [hardware.ino](file:///home/goodkid/Desktop/Tech%20Rumble/TechRumbleRobot/hardware.ino):**
  - Exact 30-pin safe ESP32 DevKit V1 pinout matching `CONTEXT.md` (no `GPIO 12`, no `GPIO 1/3`, ADC1-only IR sensors).
  - ESP32 Core `2.0.17` LEDC allocation: HS Timer 0 (Ch `0–1`, `1 kHz`, 8-bit) for 4× JGA25-370 motors via 2× L298N with slew-rate limiting & active short-brake; HS Timers 1–2 (Ch `2–5`, `50 Hz`, 16-bit) for 4× MG996R servos with smooth `20 ms` interpolation, hard 50 cm height limits, 5 named poses, and Sudden Death Grip-Lock release guard.
  - Non-blocking `IRAM_ATTR` `CHANGE` ISRs for 3× HC-SR04 ultrasonics (alternating Front vs simultaneous 180°-opposed Left/Right trigger, 3-sample median filter, zero `pulseIn()`).
  - Non-blocking ADC1 12-bit TCRT5000 IR array sampler (`GPIO 36, 39, 34`) with live-tunable black/white thresholds.
  - Non-blocking `IRAM_ATTR` `RISING` ISRs for 2× 20-slot wheel encoders (`GPIO 18, 19`, `1.5 ms` glitch filter, signed directional + monotonic counts, `68 mm` CAD tire OD conversion `~10.68 mm/tick`).
  - Debounced (`40 ms`) grip microswitch on `GPIO 4` (`INPUT_PULLUP`, Active LOW) and non-blocking Status LED patterns on `GPIO 2`.
  - Non-blocking Serial bench-test helper (`?`, `s`, `m`, `x`, `1..5`, `g`, `o`, `u`, `r`, `a <j> <deg>`, `t <val>`) guarded to `MODE_IDLE_SAFE`.
- [x] **Verification Results:**
  - Confirmed strictly `.ino` files only inside `TechRumbleRobot/` (zero `.h`/`.cpp`).
  - Confirmed zero `delay()`, zero `pulseIn()`, zero Core `3.x` `ledcAttach()`, and zero GPIO/PWM/ISR code in `TechRumbleRobot.ino`.
  - Ran full concatenated `g++ -std=gnu++17 -Wall -Wextra -fsyntax-only` check against ESP32 Core `2.0.17` & `PS4Controller` (`aed3`) API stubs → **PASSED with 0 errors and 0 warnings**. *(Arduino IDE hardware compile/flash check pending on user's board.)*

### Phase 1 — Step 2: Manual PS4 Drive & 4-DOF Robotic Arm Control (`manual.ino`)
- [x] **Created [manual.ino](file:///home/goodkid/Desktop/Tech%20Rumble/TechRumbleRobot/manual.ino):**
  - **Arcade Drive:** `LStickY` throttle + `RStickX` steering with deadzone filtering (`16`), smooth linear+quadratic expo curve (`0.35`), proportional peak normalization, and 3 speed gears (`L2` Precision 43% + gentle slew for Loading Zone & Ramps ribs, Default Normal 73%, `R2` Boost 100% for serrated ramp, `L3` active short-brake).
  - **4-DOF Arm Incremental Control (non-blocking `millis()`):** D-Pad Left/Right → Base Waist, D-Pad Up/Down → Shoulder (ignored while `Options` is held for mode switching!), `RStickY` → Elbow (with `36` deadzone + `1.25×` vertical-dominance guard so horizontal steering never drifts the elbow), target lead cap (`6°`) preventing stop windup, and 50% precision jog scaling when `L2` is held.
  - **Gripper Control with Microswitch Feedback & Sudden Death Lock:** `R1` incrementally closes gripper and auto-stops + clamps with gentle `4°` preload upon `sensors.gripSwitchPressed` contact (protecting the Half-Metal MG996R gears!) with PS4 rumble confirmation; `L1` opens freely when unlocked, or requires `L1 + Circle` (`300 ms` hold) when Sudden Death Grip-Lock is active; `Touchpad` (`300 ms` hold) toggles Grip-Lock explicitly.
  - **Face-Button Arm Presets (rising-edge triggered):** `Cross` (`POSE_HOME_STOW`), `Square` (`POSE_PICKUP_READY`), `Triangle` (`POSE_CARRY_SAFE`, auto-arming Sudden Death Grip-Lock when load is detected), `Circle` alone (`POSE_DEPOSIT`, keeping Grip-Lock intact until `L1 + Circle` release).
  - **Defense-in-Depth Disconnect Safety:** Immediately stops motors and freezes servos with full holding torque if `!PS4.isConnected()`.
- [x] **Updated [hardware.ino](file:///home/goodkid/Desktop/Tech%20Rumble/TechRumbleRobot/hardware.ino) & [TechRumbleRobot.ino](file:///home/goodkid/Desktop/Tech%20Rumble/TechRumbleRobot/TechRumbleRobot.ino):**
  - Added direction-agnostic `jogGripperClose()` and `clampGripperAtContact()` HAL helpers; updated `applyArmPose()`, `armSuddenDeathLock()`, and `setGripLock()` to preserve the contact-clamped gripper angle so Half-Metal gears never stall.
  - Wired `initManualMode()` in `enterMode()` and `runManual()` in `loop()`.
- [x] **Verification Results:**
  - Confirmed zero `.h`/`.cpp` files, zero `delay()`/`pulseIn()`/`ledcAttach()`, and zero raw GPIO/LEDC/ISR calls outside `hardware.ino`.
  - Ran concatenated 3-tab `g++ -std=gnu++17 -Wall -Wextra -fsyntax-only` check (`TechRumbleRobot.ino` + `hardware.ino` + `manual.ino`) → **PASSED with 0 errors and 0 warnings**.

### Phase 1 — Step 3: Autonomous 3-IR Line Follower (`line_follower.ino`)
- [x] **Created [line_follower.ino](file:///home/goodkid/Desktop/Tech%20Rumble/TechRumbleRobot/line_follower.ino):**
  - **Weighted 3-IR Non-Blocking PID:** Maps `sensors.irMask` (`Left`, `Center`, `Right` → `-2.0 .. +2.0`) through a `50 Hz` (`20 ms`) PID controller (`LF_KP`, `LF_KI`, `LF_KD` tunable constants at top of file) with integral anti-windup, zero-crossing reset, reacquisition D-kick suppression, and dynamic curve slowdown (`LF_BASE_PWM = 135`, `LF_MAX_PWM = 175`, gentle `LF_SLEW_STEP = 16` for Sudden Death 20 g load stability).
  - **Arena Feature Handlers (Arena Guide p. 3):**
    - **`5 cm` Line Gap (`000` from straight tracking):** Bridges straight using encoder distance (`LF_GAP_BRIDGE_MAX_MM = 85 mm`) before entering pivot recovery.
    - **Sharp `90°` Corner & `2 × 45°` Zig-Zag (`000` off curve / overshoot):** Controlled in-place pivot using `lastLineDir` (seeded by mirrored `LANE_A` / `LANE_B` hint) until line is reacquired.
    - **`24 × 2.5 cm` End Bars (`111`):** Start-bar ignore window (`150 mm`) + `45 ms` debounce; in `STAGE_KNOCKOUT` pulls `120 mm` into the Changing Mode Zone and brakes to `MODE_IDLE_SAFE`; in `STAGE_GROUP` brakes cleanly at the far/home End Bar for two-way Loading Zone pickup & return.
  - **Rulebook Compliance & Fail-Safes:** Zero `PS4.` calls in `line_follower.ino` (Rule 4 autonomous drive lockout), preserves Sudden Death Grip-Lock if load is held, `1800 ms` lost-line Ring-Out brake fail-safe, `180 s` max run cap, and immediate motor stop on `Share` abort or external mode exit.
- [x] **Wired into [TechRumbleRobot.ino](file:///home/goodkid/Desktop/Tech%20Rumble/TechRumbleRobot/TechRumbleRobot.ino):** Added `initLineFollowerMode()` in `enterMode()` and `runLineFollower()` in `loop()`.
- [x] **Verification Results:**
  - Confirmed zero `.h`/`.cpp` files, zero `delay()`/`pulseIn()`/`ledcAttach()`, zero GPIO/LEDC/ISR calls outside `hardware.ino`, and zero `PS4.` calls in `line_follower.ino`.
  - Ran 4-tab alphabetical concatenation `g++ -std=gnu++17 -Wall -Wextra -fsyntax-only` (`TechRumbleRobot.ino` + `hardware.ino` + `line_follower.ino` + `manual.ino`) → **PASSED with 0 errors and 0 warnings**.

---

## Remaining Phase 1 Roadmap (Paused — Waiting for Step 4 Approval)
- [ ] **Step 4:** Autonomous Maze Solver (`TechRumbleRobot/maze.ino`) — 35 cm corridor dual-wall centering PID, mirrored Lane A/B encoder 90° turns, Loading Zone stop/handoff, gentle exit run.
- [ ] **Step 5:** Battle Royale / Combat Mode (`TechRumbleRobot/sumo.ino`) — Knockout Shared Pocket high-torque manual combat drive, `POSE_COMBAT_TUCK`, 15 s immobility alert, edge/proximity PS4 feedback.
- [ ] **Step 6:** Final Arduino IDE Core `2.0.17` compile check & full bench/field validation.

---

## Bench Calibration Values To Record (Using Step 1 Serial Bench Helper)
- [ ] `PS4_MAC_ADDRESS` in `TechRumbleRobot.ino` — replace placeholder `"01:02:03:04:05:06"` with paired DualShock 4 MAC.
- [ ] `MOTOR_L_INVERT` / `MOTOR_R_INVERT` — verify forward/reverse with Serial command `m`.
- [ ] `SERVO_LIMIT_MIN/MAX`, `ARM_HEIGHT_SUM_MAX`, `GRIPPER_OPEN_DEG`, `GRIPPER_CLOSED_DEG`, and `ARM_POSES` — tune with `a <j> <deg>` and tape-measure verify `<= 50 cm` height & 20 g load held clear of chassis deck.
- [ ] `IR_BLACK_IS_HIGH` and `s_irThresh[3]` — tune with `s` and `t <val>` on matte white board + 25 mm black line.
- [ ] Ultrasonic `1k/2k` voltage divider ECHO check (`s` stream: confirm clean cm readings on F, L, R).
- [ ] Wheel encoder tick polarity & counts per 90° pivot turn (`r` + manual spin).

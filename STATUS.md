# Tech Rumble 2 — Project Status Tracker

**Last Updated:** 2026-11-26  
**Current Phase:** Phase 1 — Step 1 Complete (Awaiting Approval for Step 2)  
**Git Branch:** `main` (tracking `origin/main` → `https://github.com/MrMorale-007/Tech-Rumble`)

---

## Completed Milestones

### Phase 0: Repo Setup, Audit & Workspace Scaffolding
- [x] **Git Initialization & Remote Sync:** Initialized local git repository, connected `origin` to `https://github.com/MrMorale-007/Tech-Rumble`, fetched all history, and checked out `main` cleanly without modifying local `docs/`.
- [x] **Remote Repo Audit:** Verified `origin/main` has 2 commits (`ca359c7`, `c9bd2a4`) containing only `README.md`.
- [x] **Rulebook & Arena Guide Audit:** Extracted and visually verified all 9 pages of `docs/RULE BOOK .pdf`, all 5 pages of `docs/Tech Rumble Arena Guide.pdf`, both CAD renders in `docs/Car Images/`, and the SolidWorks inventory in `docs/Car Design/`.
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

---

## Remaining Phase 1 Roadmap (Paused — Waiting for Step 2 Approval)
- [ ] **Step 2:** Manual Drive, Ramps & Arm Control (`TechRumbleRobot/manual.ino`) — PS4 multi-gear drive, smooth arm jogging + presets, guarded `L1+R1+Circle` Grip-Lock release.
- [ ] **Step 3:** Autonomous Line Follower (`TechRumbleRobot/line_follower.ino`) — 3-IR PD follower, 90° corner & 45° zig-zag memory, 5 cm encoder-bridged gap crossing, 24 cm end-bar Group/Knockout handling.
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

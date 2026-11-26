---
trigger: always_on
---

# Tech Rumble 2 — Mandatory Workspace Sync Rule

Before editing any code or making architectural decisions in this workspace, you **MUST** read all of the following files first:

1. `CONTEXT.md` — Locked hardware BOM, ESP32 DevKit V1 pinout, LEDC timer allocation, and stack constraints.
2. `RULES_CHECKLIST.md` — Official Tech Rumble 2 rules, Sudden Death locks, timeouts, and arena dimensions.
3. `IMPLEMENTATION_PLAN.md` — Active step-by-step firmware plan and deliverables.
4. `STATUS.md` — Current progress, completed steps, active blockers, and calibration notes.
5. `TechRumbleRobot/hardware.ino` — Live single source of truth for all GPIO, LEDC, servo, sensor, and ISR definitions (once created in Phase 1).

## Non-Negotiable Engineering Constraints

- **Target Stack:** ESP32 DevKit V1 on **ESP32 Arduino Core `2.0.17` (`2.x`)** using Core `2.x` LEDC API (`ledcSetup`, `ledcAttachPin`, `ledcWrite`) and **`PS4Controller` (`aed3`)** for DualShock 4 (never PS5).
- **Flat Sketch Structure:** Strictly `.ino` tabs only inside `TechRumbleRobot/` (`TechRumbleRobot.ino`, `hardware.ino`, `manual.ino`, `line_follower.ino`, `maze.ino`, `sumo.ino`). **Zero `.h` or `.cpp` files.**
- **Hardware Isolation:** All GPIO, PWM/LEDC, ADC, and `IRAM_ATTR` ISR code MUST stay strictly inside `TechRumbleRobot/hardware.ino`.
- **Non-Blocking Execution:** All loops and routines MUST be non-blocking (`millis()` / `micros()` / ISR). Never use `delay()` in active control loops or blocking `pulseIn()`.
- **Competition Safety Invariants:**
  - Never process PS4 drive commands during autonomous `MODE_LINE_FOLLOWER` or autonomous `MODE_MAZE` (sending drive commands loses the match).
  - Never drop servo/gripper holding torque on PS4 disconnect or mode switch; protect gripper release with Grip-Lock when Sudden Death is armed.
  - Enforce hard servo angle limits in `hardware.ino` so the arm never exceeds the 50 cm vertical height limit.

## Post-Step Sync & Git Push Requirement

After completing each step or task (or pausing with a partial milestone), you **MUST**:

1. Update `STATUS.md` with what was completed, any pin/calibration updates, verification results, and the exact next step.
2. Stage, commit (with a clear descriptive commit message), and push **all** changes to `origin` (`main`) before finishing your turn.

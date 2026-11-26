# Tech Rumble 2 — Phase 1 Implementation Plan

> **Status:** Pending User Approval (Phase 0 complete — zero `.ino` files written yet).  
> **Scope:** Build the complete, modular, non-blocking ESP32 DevKit V1 firmware inside `TechRumbleRobot/` using strictly flat `.ino` tabs (`TechRumbleRobot.ino`, `hardware.ino`, `manual.ino`, `line_follower.ino`, `maze.ino`, `sumo.ino`).

---

## Step 1.1 — Flat Sketch Scaffolding & Core State Machine (`TechRumbleRobot.ino`)
- Create `TechRumbleRobot/` with all 6 `.ino` tabs (zero `.h`/`.cpp` files).
- Define global enums & shared state structs in `TechRumbleRobot.ino`:
  - `RobotMode`: `MODE_IDLE_SAFE` (default boot state — prevents false starts per Rule 5), `MODE_MANUAL`, `MODE_LINE_FOLLOWER`, `MODE_MAZE`, `MODE_COMBAT_SUMO`.
  - `LaneSide`: `LANE_A` / `LANE_B` (supports mirrored competition tracks).
  - `StageType`: `STAGE_GROUP` (two-way runs, wireless mode switches) vs `STAGE_KNOCKOUT` (continuous gauntlet, CMZ transitions).
  - `GripState`: `suddenDeathArmed`, `gripLockActive`, `loadDetected`.
- Implement the top-level non-blocking `loop()` scheduler:
  1. `updateHardware()` (sensors, encoders, smooth servo interpolation, status LED).
  2. `updateControllerWatchdog()` (detects PS4 disconnect → stops motors immediately while holding arm/gripper torque!).
  3. Safe mode-transition handler (hold-to-confirm wireless combo so accidental taps cannot trigger mode switches; strict lockout of drive sticks during autonomous modes per Rule 4).
  4. Dispatch active mode handler (`runManual()`, `runLineFollower()`, `runMaze()`, `runCombatSumo()`).

## Step 1.2 — Hardware Abstraction Layer (`hardware.ino`)
- Centralize all pin `#define` constants from [CONTEXT.md](file:///home/goodkid/Desktop/Tech%20Rumble/CONTEXT.md) (30-pin safe DevKit V1 layout).
- **LEDC Init (Core `2.0.17`):**
  - HS Timer 0 (Ch `0–1`, `1 kHz`, 8-bit): Left & Right L298N `EN` PWM.
  - HS Timers 1–2 (Ch `2–5`, `50 Hz`, 16-bit): Base, Shoulder, Elbow, Gripper MG996R servos.
- **Motor Control API:** `driveMotors(int leftPwm, int rightPwm)`, `brakeMotors()`, `stopMotors()` with configurable per-side inversion flags, minimum PWM deadband, and non-blocking slew-rate limiter (prevents sudden jerks that could dislodge the 20 g load).
- **Servo & Arm API:**
  - Non-blocking `millis()` interpolation (`updateServos()`) with tunable deg/sec speed.
  - Hard min/max angle clamps enforcing the strict **50 cm vertical height limit** (Rule 2).
  - Named arm presets: `POSE_HOME_STOW`, `POSE_PICKUP_READY`, `POSE_CARRY_SAFE`, `POSE_DEPOSIT`, `POSE_COMBAT_TUCK` (holds 20 g load strictly by gripper, never touching chassis deck per Rule 3.1).
- **Grip Switch & Sudden Death Guard:** Debounced `isLoadGripped()` on `PIN_GRIP_SWITCH` (`INPUT_PULLUP`), auto-arming Grip-Lock once load is grasped, and gentle slip-hold protection.
- **Non-Blocking Ultrasonic Driver:** `IRAM_ATTR` `CHANGE` ISRs on Front/Left/Right ECHO pins + alternating round-robin trigger scheduler (`Front` vs simultaneous `Left+Right` every `30 ms`), echo timeout guard (`25 ms`), and 3-sample median filter.
- **TCRT5000 IR Driver:** Non-blocking ADC1 sampler (`GPIO 36, 39, 34`) with configurable black/white threshold & digital bitmask helper (`0b000`–`0b111`).
- **20-Slot Encoder ISRs:** `IRAM_ATTR` rising-edge counters on `GPIO 18, 19` with `getLeftTicks()`, `getRightTicks()`, `resetEncoders()`, and millimeter conversion (`68 mm` wheel OD → `~10.68 mm/tick`).
- **Status LED (`GPIO 2`):** Non-blocking heartbeat/blink patterns distinguishing `IDLE_SAFE`, `MANUAL`, `LINE_FOLLOWER`, `MAZE`, `COMBAT_SUMO`, and `PS4_DISCONNECTED`.

## Step 1.3 — Bench Diagnostic & Calibration Harness (inside `TechRumbleRobot.ino` / `hardware.ino`)
- Add a compile-time gated (`#define ENABLE_SERIAL_DIAG 1`) non-blocking Serial command interface for safe bench bring-up:
  - Sensor live stream (`IR` raw ADC + bitmask, `US` F/L/R cm, `ENC` L/R ticks, `GRIP` switch state).
  - Individual motor direction & PWM ramp test.
  - Individual servo angle sweep to record exact mechanical limits and measure max vertical arm height (`≤ 50 cm`).
  - IR black/white threshold calibration on actual matte arena print.

## Step 1.4 — Manual Drive, Ramps & Arm Control (`manual.ino`)
- Initialize `PS4Controller` (`aed3`) with configurable DualShock 4 Bluetooth MAC address.
- **Drive Control:** Smooth arcade drive (Left Stick Y = throttle, Right Stick X or Left Stick X = steering, configurable) with 3 speed gears:
  - **Precision Gear (`L2` hold):** Low speed + gentle ramp for Maze Loading Zone pickup & Ramps (`8 ribs × 2.5 cm`, hump, serrated ramp).
  - **Normal Gear:** Balanced speed for flat strips and S-curves.
  - **Boost Gear (`R2` hold):** High torque for climbing serrated ramp or aggressive repositioning.
- **Arm Control:**
  - Smooth manual joint jogging (Right Stick / D-Pad) + instant preset poses (`Triangle` = Carry Safe, `Square` = Pickup Ready, `Cross` = Home Stow).
  - **Gripper Control with Sudden Death Protection:**
    - `R1` closes/clamps gripper and engages `gripLockActive` once `isLoadGripped()` confirms contact.
    - Opening gripper while `gripLockActive == true` requires deliberate combo (`L1 + R1 + Circle` held `300 ms`) — prevents accidental drop during Ramps or Combat!
    - Inside Loading Zone (or before Sudden Death armed), single press toggles freely for fast pickup retries.
- **Wireless Mode Switching:** Hold `Options + D-Pad` direction (`500 ms`) to enter `LINE_FOLLOWER`, `MAZE`, or `COMBAT_SUMO`, or `Share` to immediately return to `MODE_IDLE_SAFE` (essential for Group Stage wireless-only mode changes).

## Step 1.5 — Autonomous Line Follower (`line_follower.ino`)
- Non-blocking `millis()` PD loop tuned for 3× TCRT5000 on `25 mm` black line (`120 × 200 cm` matte white board).
- **Arena-Specific State Machine:**
  - `LF_FOLLOW`: Weighted 3-sensor PD tracking (`100`, `110`, `010`, `011`, `001`) with smooth speed reduction on tight `R 12` curves and S-bends.
  - `LF_SHARP_TURN`: Handles the **`90°` sharp corner** and **`45°` zig-zag** using `lastLineDir` memory + controlled pivot turn until center sensor reacquires line.
  - `LF_GAP_BRIDGE`: When all 3 sensors read WHITE (`000`) from straight tracking, drives straight using encoder distance up to `~7–8 cm` to cleanly cross the **`5 cm` line gap** before falling back to `LF_SHARP_TURN` recovery.
  - `LF_END_BAR`: Debounced all-black (`111`) detection for the **`24 × 2.5 cm` end bars**:
    - In **Knockout Stage:** Stops cleanly inside Changing Mode Zone and switches to `MODE_IDLE_SAFE`.
    - In **Group Stage:** Supports two-way run (stops at far Loading Zone end bar for pickup/turnaround, then returns to Starting Point end bar).
- **Rule 4 Compliance:** Strictly ignores all PS4 drive inputs while running (only `Share` emergency abort returns to `MODE_IDLE_SAFE`).

## Step 1.6 — Autonomous Maze Solver (`maze.ino`)
- Non-blocking `millis()` state machine for `35 cm` corridors (`20 cm` high walls, `~5 cm` side clearance):
  - `MAZE_CORRIDOR`: Dual-wall centering PID (`left_cm - right_cm`) when both walls present; single-wall hold (`~15–17 cm` target on active follow wall) when one side opens.
  - `MAZE_APPROACH_TURN`: Front ultrasonic deceleration (`< 25 cm`) down to turn trigger (`~14–16 cm`, tuned to chassis length).
  - `MAZE_TURN_90`: Non-blocking encoder-counted 90° pivot turn (`LANE_A` vs `LANE_B` mirrored wall-follow preference: Left-Hand vs Right-Hand rule) + post-turn straight stabilization.
  - `MAZE_LOADING_ZONE_DETECT`: Detects the wide T-shaped central Loading Zone (`45 × 120 cm` / open left & right > `38 cm` simultaneously) → stops motors, flashes Status LED, and transitions safely to `MODE_MANUAL` (or `MODE_IDLE_SAFE` awaiting driver takeover) for manual 20 g pickup per Rule 4 & 7.2.
  - `MAZE_EXIT_RUN`: Post-pickup autonomous exit run (gentle slew-rate acceleration to keep the 20 g load steady) until maze exit / CMZ clear-out (`F/L/R` all open).
- **Rule 4 Compliance:** Strictly ignores all PS4 drive inputs while autonomous.

## Step 1.7 — Battle Royale / Combat Mode (`sumo.ino`)
- Designed for Knockout Stage **Shared Pocket Zone** (`R ≈ 100 cm` elevated platform, **Manual wireless** per Rule 4 & 8).
- **Combat Arm Protection:** Entering `MODE_COMBAT_SUMO` automatically moves arm to `POSE_COMBAT_TUCK` (low CG, protected behind chassis perimeter while keeping 20 g load suspended strictly in gripper — never resting on chassis!) and enforces `gripLockActive = true`.
- **High-Torque Combat Drive:** Full-power responsive arcade/tank drive with aggressive traction control, quick-pivot spin boost (`L1` / `R1` or stick), and **12-second movement reminder** (PS4 rumble/LED alert) to avoid the **15-second disabled rule** (Rule 8.3).
- **Edge & Opponent Awareness Assist (Non-Intrusive):**
  - Uses front TCRT5000 IR sensors / ultrasonic proximity to trigger PS4 LED color & rumble warnings (and optional toggleable edge-brake assist so we don't self Ring-Out off the elevated platform).
  - Keeps driver in full manual control per Rule 4 (`Fighting: Manual (wireless)`).

## Step 1.8 — Verification, Compilation & Field Test Plan
- Compile-verify `TechRumbleRobot/` against ESP32 DevKit V1 / Arduino Core `2.0.17` (`arduino-cli` if available, or full static audit + Arduino IDE checklist).
- Execute structured bench → dry-run → competition verification:
  1. Technical inspection check (`≤ 25×25×50 cm`, `≤ 5 kg`, `≤ 13.0 V`, `PB-1601` kill switch cuts all 12V, main fuse, LiPo casing).
  2. PS4 disconnect failsafe test (motors stop immediately, gripper holds 20 g load!).
  3. Group Stage 3-trial test (Maze, Line Follower two-way, Ramps two-way) within 5 min each.
  4. Knockout continuous gauntlet test (`Maze → CMZ → LF → CMZ → Ramps → Pocket Zone → 3 s Deposit`) within 10 min.

---

## Decisions to Confirm Before Starting Phase 1
1. **Ultrasonic TRIG Pin Assignment:** Approve the 30-pin safe layout with shared `PIN_US_SIDE_TRIG` (`GPIO 15`) for 180°-opposed Left/Right ultrasonics + dedicated `PIN_STATUS_LED` (`GPIO 2`), OR prefer 3 separate TRIG pins (using `GPIO 2` as `PIN_US_RIGHT_TRIG` and omitting a separate Status LED)?
2. **L298N Wiring Topology:** Confirm the 2× L298N drivers are wired **side-ganged** (L298N #1 = Left FL+RL tied in parallel, L298N #2 = Right FR+RR tied in parallel → 6 GPIOs total).
3. **PS4 DualShock 4 Bluetooth MAC Address:** Do you already have the ESP32/PS4 paired MAC address ready to drop into `TechRumbleRobot.ino`, or should we include a placeholder (`"01:02:03:04:05:06"`) and quick SixaxisPairTool note?
4. **Line Follower Group Stage Pickup:** In Group Stage Line Follower (Rule 6.2), confirm whether the far-end pickup is handled via wireless switch to Manual in the Loading Zone (same as Maze) or if you want an autonomous stop-and-wait trigger at the far end bar.

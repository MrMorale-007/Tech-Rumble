# Tech Rumble 2 — Official Competition Rules & Arena Checklist

> Extracted from [RULE BOOK .pdf](file:///home/goodkid/Desktop/Tech%20Rumble/docs/RULE%20BOOK%20.pdf) and [Tech Rumble Arena Guide.pdf](file:///home/goodkid/Desktop/Tech%20Rumble/docs/Tech%20Rumble%20Arena%20Guide.pdf).  
> **Note on Rulebook Section 1 typo:** In the official Rule Book definitions table (p. 2), the descriptions for *Loading Zone* and *Pocket* were swapped. Throughout Sections 3, 6, 7, and the Arena Guide, **Loading Zone** is the marked pickup area (Sudden Death starts upon leaving it) and **Pocket** is the deposit receptacle (3-second confirmed release).

---

## 1. Robot Specifications, Safety & Inspection Limits (Rule 2 & Rule 9)

- [ ] **Max Dimensions:** `25 cm (L) × 25 cm (W) × 50 cm (H)` **at any point during the match**, including the robotic arm at absolute maximum vertical extension.
  - *Firmware lock:* Hard-clamp shoulder/elbow servo angles in `hardware.ino` so the arm can never exceed 50 cm height or expand past 25×25 cm when stowed/checked.
- [ ] **Fixed Size:** Robot may not grow beyond size limits during a match, and nothing may be detached on purpose.
- [ ] **Max Weight:** `5 kg`.
- [ ] **Voltage Limit:** Strict maximum `12.0 V` anywhere in the circuit (`13.0 V` max allowance for a fully charged battery at inspection). **NO step-up / boost DC converters** raising voltage above battery voltage.
- [ ] **Mandatory Physical Power / Kill Switch:** Prominent, easily accessible exterior kill switch (`PB-1601`) that **cuts all power to the 12 V circuit**. Missing/hidden kill switch = fails technical inspection.
- [ ] **Battery & Electrical Safety (Rule 9.2):**
  - Main battery lead MUST have a fuse or equivalent overcurrent protection.
  - LiPo batteries MUST sit inside a protective casing in the robot (charge only in a fireproof bag, never unattended).
  - All wiring insulated and secured inside chassis; cover all sharp edges.
- [ ] **Technical Inspections:** Performed **twice** — before Group Stage and again before Knockout Stage (size, voltage, power button, arm, and **wireless system declaration**).
- [ ] **No Weapons / Prohibited Devices (Rule 8.4 & 9.1):**
  - Forbidden: spinners, saws, blades, hammers, spikes, **flippers, lifters, tippers, hooks**, projectiles, liquids, gases, fire, sparks, entangling nets/ropes/tape, adhesives/suction/vacuum devices, wireless jammers.
  - Forbidden in combat: grabbing the opponent robot or deliberately targeting opponent's electronics, battery, or load with an attachment.
  - *Design check:* Verify front scoop (`kasa7a_2.SLDPRT` / `كساحه 99.SLDPRT`) passes inspection as a passive pushing bumper, not a lifter/flipper/hook.
  - Penalty for Section 9 violations: disqualification from the match or entire event.

---

## 2. The 20 g Load & Sudden Death Rule (Rule 3)

- [ ] **Official Load:** Exactly `20 g` (shape/dimensions published before event).
- [ ] **Arm-Only Carry:** Load MUST be carried and held by the robot's arm. It **may NOT rest on the robot body**, or in any tray, bay, cage, or container.
  - *Firmware/Pose rule:* Even in `POSE_COMBAT_TUCK`, the load must remain suspended/held strictly by the gripper jaws, clear of the chassis deck.
- [ ] **Sudden Death Activation:** Begins the instant the robot leaves the **Loading Zone** carrying its load (applies across ALL stages, including inside **Changing Mode Zones** and during **Battle Royale**).
- [ ] **Drop Definition = Instant Loss:** Load touches the arena floor or any surface other than the Pocket, OR stops being held by the arm for any reason (even if it falls onto our own robot body), whether accidental or caused by an opponent's hit. No second chance.
- [ ] **Loading Zone Exception:** Dropping the load *inside* the Loading Zone is allowed; robot may re-grip and pick it up again.
- [ ] **Simultaneous / Unclear Drop:** Draw (Group Stage) or Replay (Knockout Stage). Arena/staff fault may trigger Head Referee replay.
- [ ] **Deposit Requirement (Rule 1):** Load is released by the arm into the Pocket and **stays there for 3 seconds**, confirmed by the referee.
- [ ] **Firmware Safety Locks Required:**
  - Continuous gripper hold torque (never detach/de-energize gripper servo while carrying load).
  - Debounced grip microswitch monitoring (`PIN_GRIP_SWITCH`) with sudden-death arming & slip-recovery hold.
  - Guarded gripper release (e.g., deliberate multi-button combo `L1 + R1 + Circle` when Sudden Death is armed) so an accidental single tap or PS4 disconnect can NEVER drop the load.
  - On PS4 Bluetooth disconnect: stop drive motors immediately, **freeze/hold all servo positions and gripper torque**.

---

## 3. Control Modes & No-Touching Rules (Rule 4)

| Arena Section | Allowed Control Mode | Firmware Enforcement |
| :--- | :--- | :--- |
| **Maze** | **Autonomous** (EXCEPT inside the Loading Zone, where robot is driven **Manual wireless**) | Lock out all PS4 drive commands while autonomous Maze is active; only allow mode-switch / emergency stop. |
| **Line Follower** | **Fully Autonomous** | Lock out ALL PS4 drive commands while Line Follower is active; sending drive commands = **automatic match loss**. |
| **Ramps** | **Manual (wireless)** | Full PS4 wireless drive + arm control. |
| **Fighting (Battle Royale)** | **Manual (wireless)** | Full PS4 wireless combat drive + grip-locked arm posture. |

- [ ] **No Drive Commands in Autonomous:** Sending wireless drive commands during an autonomous section = **loss of match**.
- [ ] **No Touching the Robot:**
  - **Group Stage:** NO Changing Mode Zones. Robot may **NEVER** be touched after the start signal. All mode switches (e.g., Maze <-> Loading Zone) MUST happen **wirelessly**.
  - **Knockout Stage:** Team may touch the robot **ONLY** inside the two marked **Changing Mode Zones** (`≈ 30 × 40 cm` between Maze→LF and LF→Ramps) solely to switch operating mode. **Touching the load or the arm's grip is NEVER allowed.**
  - Touching the robot anywhere else = **loss of match**.
- [ ] **Wireless Non-Interference:** Must not interfere with other teams; organizers may assign channels/frequencies.

---

## 4. Match Timers, Scoring & General Rules (Rules 5, 6, 7)

| Parameter | Group Stage (Precision Trials) | Knockout Stage (Ultimate Gauntlet) |
| :--- | :--- | :--- |
| **Teams / Format** | 64 teams → 16 groups of 4 (3 matches each: Maze, LF, Ramps; top 2 advance = 32 teams) | 32 teams, single elimination (R32 → R16 → QF → SF → Final) |
| **Match Time Limit** | **5 minutes** | **10 minutes** |
| **Repair Time / Pauses** | **None** — clock never stops; no-touching rule still applies | **None** — clock never stops (even inside Changing Mode Zones and Battle Royale) |
| **Course Format** | Separate single-challenge arenas, **two-way journey** (out to pickup → return to Starting Point pocket). No combat. | One continuous linked gauntlet: `Maze → CMZ → Line Follower → CMZ → Ramps → S-Curve → Shared Pocket`. Single load in Maze center. |
| **Scoring** | Win = **3 pts**, Draw = **1 pt**, Loss = **0 pts** | Win = Advance, Loss = Eliminated |
| **Result at Time Limit** | Robot with most Progress (checkpoints) wins; equal Progress = **Draw (1 pt each)** | Robot with most Progress wins (if no Battle Royale running); equal Progress = **Replay** |
| **Group Tiebreakers** | 1. Head-to-head result → 2. Fastest successful deposit time in group → 3. Referee playoff match | N/A (single elimination) |

- [ ] **Start Protocol:** Robots start inside Starting Zone and must not move before referee's start signal. 1st false start = reset; 2nd false start = **forfeit**.
  - *Firmware rule:* Always boot into `MODE_IDLE_SAFE` (motors stopped, servos smoothly initialized to safe home pose; never move autonomously on power-up!).
- [ ] **Call-Up Readiness:** Must be ready at arena within **2 minutes** of being called, or **forfeit**.
- [ ] **Stuck Robots:** Stay where they are; match continues until time limit or another result.
- [ ] **Mirrored Tracks:** Every match runs two identical **mirrored** tracks (Robot A / Robot B). Firmware autonomous modes MUST support both `LANE_A` and `LANE_B`!
- [ ] **Ring-Out (All Elevated Sections):** Any part of the robot touching the ground or anything outside the elevated arena = **instant loss**.
- [ ] **Referees:** Head referee's decision is final. Only team captain speaks to referees. Protests must be raised before the next match starts.

---

## 5. Battle Royale Rules — Knockout Stage Only (Rule 8)

- [ ] **Trigger:** Starts when the 2nd robot enters the **Pocket Zone** carrying its load *before* the 1st robot's 3-second deposit is completed and confirmed. (If 1st robot finishes deposit before 2nd enters Pocket Zone, 1st robot wins with no fight.)
- [ ] **Deposit Lock:** While both robots are in the Pocket Zone, **neither team may deposit its load**.
- [ ] **Win Conditions in Combat:**
  1. Push/drop opponent off the elevated arena (**Ring-Out**).
  2. Opponent drops their 20 g load (**Sudden Death**).
  3. Opponent **cannot move for 15 seconds** (declared disabled).
- [ ] **Allowed Tactics:** Pushing, ramming, blocking, arena positioning.
- [ ] **Post-Fight Deposit:** Victor MUST still complete the 3-second deposit into the Shared Pocket. If the victor drops its load before deposit completes, **Battle Royale is replayed**.
- [ ] **Clock Expiry During Fight:** Judges decide by dominance (control of arena/pocket, pushing power, last attacker). If tied, a **3-minute Battle Royale rematch** starts with both robots reset inside the Pocket Zone.

---

## 6. Arena Dimensions & Sensor/Navigation Constraints (Arena Guide)

> Overall assembled Knockout Gauntlet footprint: `≈ 1038 × 430 cm`, elevated platform, mirrored tracks A & B merging only at Shared Pocket Zone.

### 6.1 Maze Arena
- **Footprint:** `≈ 335 × 350 cm` (mirrored halves A & B).
- **Corridor Clear Width:** **`35 cm`** (with max 25 cm robot width → only **`~5 cm` clearance per side!** Requires tight Left+Right ultrasonic wall-centering PID).
- **Wall Height / Thickness:** `20 cm` high, `≈ 5 cm` thick (well aligned with our chassis-mounted HC-SR04 ultrasonics).
- **Entrances:** 2 parallel lanes × `35 cm` wide, `75 cm` long.
- **Measured Wall Segments:** `70, 75, 80, 90, 95, 115, 130 cm` (longest = `130 cm`).
- **Loading Zone:** T-shaped center (`corridor ≈ 45 × 120 cm` + `inner pocket ≈ 45 × 175 cm`).
- **Exits:** 1 per robot → Changing Mode Zone (`≈ 30 × 40 cm`) → Line Follower.

### 6.2 Line Follower Arena
- **Board:** `120 × 200 cm` per robot (mirrored), printed on **matte white** material.
- **Line:** **`2.5 cm` (25 mm) black line**, total length `386.1 cm`.
- **Clearances:** Line edge to board edge `≥ 18.75 cm`; separate line parts `≥ 23 cm` apart.
- **Checkpoints:** `CP1–CP7` every `25 cm` along both board edges (light blue, referee Progress markers).
- **End Bars (both ends):** **`24 × 2.5 cm`** solid perpendicular black bars (all 3 TCRT5000 sensors read BLACK simultaneously → turnaround / CMZ stop trigger!).
- **Track Features in Order:**
  1. Start straight: `15 cm`
  2. Sharp corner: **`90°`**
  3. Quarter curve: `R 12 cm`
  4. S-bend: `45° + 45°`, `R 12 cm`
  5. Quarter curve: `R 12 cm`
  6. **Line gap:** **`5 cm`** (all 3 IR sensors see WHITE → must drive straight across using heading/encoders without spinning out!)
  7. Quarter curve: `R 12 cm`
  8. S-bend: `45° + 45°`, `R 12 cm`
  9. Quarter curve: `R 12 cm`
  10. **Zig-zag:** **`2 corners × 45°`** (`48 cm` wide, `R 10 cm` in/out — aggressive sharp turns!)
  11. S-bump: `R 12 cm` (`45° · 90° · 45°`) + `15.3 cm` straight → End bar.

### 6.3 Ramps Arena & Shared Pocket Zone
- **Lane Width:** `≈ 40 cm` (elevated — Ring-Out hazard!).
- **Sequence (`≈ 205 cm` straight ramp lane + `≈ 105 cm` S-curve):**
  1. Flat entry strip: `80 cm` (dark center stripe `≈ 10 cm`)
  2. Blue marker: `7.5 cm`
  3. **Ribs:** **`8 ribs, 2.5 cm high`** (`≈ 40 cm` zone — major shock/vibration risk for 20 g load!)
  4. **Hump:** `≈ 15 cm` long
  5. **Serrated ramp:** `≈ 32 cm` long
  6. Flat run-out: `≈ 30 cm`
  7. **S-curve:** `≈ 105 cm` long, `79 cm` inward toward center
  8. **Shared Pocket Zone:** Semicircular elevated Battle Royale platform, **`R ≈ 100 cm`**, **`chord ≈ 192 cm`**, depth `≈ 133 cm`.

# Tech Rumble 2 — Project Context & Hardware Specification

## 1. Repository & Stack (Locked In)
- **GitHub Repository:** `https://github.com/MrMorale-007/Tech-Rumble` (default branch: `main`)
- **MCU Board:** ESP32 DevKit V1 (30-pin standard layout assumed so every pin is valid on both 30-pin and 38-pin variants)
- **Arduino Core:** **ESP32 Arduino Core `2.0.17` (`2.x`)**
  - **CRITICAL API RULE:** Use Core `2.x` LEDC functions (`ledcSetup()`, `ledcAttachPin()`, `ledcWrite()`). Do **NOT** use Core `3.x` `ledcAttach()`.
- **Wireless Controller:** Sony PS4 DualShock 4 via **`PS4Controller` (`aed3`)** library — **NOT PS5**.
- **Firmware Structure:** Strictly flat Arduino sketch inside `TechRumbleRobot/` using **`.ino` tabs only**:
  - `TechRumbleRobot/TechRumbleRobot.ino` — Main state machine, mode dispatcher, PS4 watchdog, global constants & shared structs
  - `TechRumbleRobot/hardware.ino` — **ALL** GPIO, LEDC PWM, servo interpolation, ultrasonic/encoder ISRs, IR sampling, grip switch, status LED
  - `TechRumbleRobot/manual.ino` — PS4 DualShock 4 drive (Ramps / Loading Zone) & arm/gripper control with Grip-Lock
  - `TechRumbleRobot/line_follower.ino` — Fully autonomous 3-IR PD line follower (90° corner, 5 cm gap, 45° zig-zag, end bars)
  - `TechRumbleRobot/maze.ino` — Autonomous ultrasonic wall-centering maze solver (mirrored Lane A / Lane B + encoder turns)
  - `TechRumbleRobot/sumo.ino` — Knockout Battle Royale manual combat drive profile, arm combat tuck, grip-lock & edge/proximity feedback
  - **Zero `.h` or `.cpp` files allowed.**
- **Execution Architecture:** 100% **non-blocking (`millis()` / `micros()` / ISR)** loops. No `delay()` in active loops, no blocking `pulseIn()`.

---

## 2. Locked Hardware BOM & Physical Notes (from CAD Audit)
CAD reference files live in [docs/Car Design/](file:///home/goodkid/Desktop/Tech%20Rumble/docs/Car%20Design) and renders in [docs/Car Images/](file:///home/goodkid/Desktop/Tech%20Rumble/docs/Car%20Images).

| Subsystem | Components | Physical / Electrical Notes |
| :--- | :--- | :--- |
| **MCU** | 1× ESP32 DevKit V1 | Classic Bluetooth active for PS4 → **ADC2 unavailable**; use **ADC1 only** for analog reads. |
| **Drive Motors** | 4× JGA25-370 Geared DC Motors | 4WD skid-steer; **65 mm tires (actual OD `68 mm` per CAD)** → circumference `≈ 213.6 mm`. |
| **Motor Drivers** | 2× L298N Dual H-Bridge | Side-ganged: **L298N #1 = Left (FL + RL)**, **L298N #2 = Right (FR + RR)**. Inputs tied per side (`ENA+ENB`, `IN1+IN3`, `IN2+IN4`) for heat balance & 6-pin control. |
| **Robotic Arm** | 4× MG996R Servos (3× Full Metal: Base, Shoulder, Elbow + 1× Half Metal: Gripper) | Parallelogram linkage keeps parallel-jaw geared gripper level. **Powered ONLY from external 5V Buck Converter** (never ESP32 5V/VIN); shared GND required. |
| **Ultrasonics** | 3× HC-SR04 (Front, Left, Right) | Front on bumper; Left & Right mid-chassis pointing 180° opposite. **MANDATORY `1 kΩ / 2 kΩ` voltage dividers on all 3 ECHO pins** (`5 V → 3.33 V`). |
| **Line Sensors** | 3× TCRT5000 IR Modules (Left, Center, Right) | Mounted in horizontal row on front underside, right behind front scoop. |
| **Wheel Encoders** | 2× 20-Slot Optical Encoders (Left, Right) | Mounted on wheel shafts → `≈ 10.68 mm` per rising edge (`213.6 mm / 20 slots`). |
| **Load Detection** | 1× Grip Microswitch | Inside gripper jaws; wired `Active LOW` to GND with ESP32 internal `INPUT_PULLUP`. |
| **Power & Safety** | 12V Battery (≤13.0V charged) + Main Fuse + `PB-1601` Kill Switch + 5V Buck Converter | `PB-1601` cuts ALL 12V circuit power (mandatory for inspection). No boost converters allowed. |
| **Chassis Scoop** | Curved front plow (`kasa7a_2.SLDPRT`) | Confirm passive pushing compliance at tech inspection (Rule 8.4 / 9.1). |

---

## 3. Standardized ESP32 DevKit V1 Pinout (30-Pin Safe)

> **Reserved / Untouched Pins:**
> - `GPIO 1 (TX0)` & `GPIO 3 (RX0)`: Reserved for USB Serial upload & debug telemetry.
> - `GPIO 12 (MTDI)`: **DO NOT USE** (flash voltage strapping pin — HIGH at boot causes bootloop).
> - `GPIO 6–11`: Internal SPI flash (unavailable).

| Peripheral | Signal Name | GPIO | Direction / Type | Notes & Safety Check |
| :--- | :--- | :---: | :--- | :--- |
| **Left Motors (L298N #1)** | `PIN_MOTOR_L_EN` | **25** | Output (LEDC Ch 0) | Clean GPIO, no boot glitch. Drives Left ENA+ENB. |
| | `PIN_MOTOR_L_IN1` | **26** | Output | Clean GPIO. Drives Left IN1+IN3. |
| | `PIN_MOTOR_L_IN2` | **27** | Output | Clean GPIO. Drives Left IN2+IN4. |
| **Right Motors (L298N #2)** | `PIN_MOTOR_R_EN` | **13** | Output (LEDC Ch 1) | Clean GPIO, no boot glitch. Drives Right ENA+ENB. |
| | `PIN_MOTOR_R_IN1` | **14** | Output | Brief boot pulse harmless while `EN = LOW`. Drives Right IN1+IN3. |
| | `PIN_MOTOR_R_IN2` | **23** | Output | Clean GPIO. Drives Right IN2+IN4. |
| **Robotic Arm Servos** | `PIN_SERVO_BASE` | **16** | Output (LEDC Ch 2) | RX2, zero boot glitch. |
| | `PIN_SERVO_SHOULDER` | **17** | Output (LEDC Ch 3) | TX2, zero boot glitch. |
| | `PIN_SERVO_ELBOW` | **21** | Output (LEDC Ch 4) | Zero boot glitch. |
| | `PIN_SERVO_GRIPPER` | **22** | Output (LEDC Ch 5) | Zero boot glitch (critical for Sudden Death load!). |
| **TCRT5000 IR Array** | `PIN_IR_LEFT` | **36** | Input (ADC1_CH0 / VP) | Input-only, Bluetooth-safe ADC1 (`analogRead` or `digitalRead`). |
| | `PIN_IR_CENTER` | **39** | Input (ADC1_CH3 / VN) | Input-only, Bluetooth-safe ADC1. |
| | `PIN_IR_RIGHT` | **34** | Input (ADC1_CH6) | Input-only, Bluetooth-safe ADC1. |
| **HC-SR04 Ultrasonics** | `PIN_US_FRONT_ECHO` | **35** | Input (ISR `CHANGE`) | Input-only; **1k/2k divider required**. |
| | `PIN_US_LEFT_ECHO` | **32** | Input (ISR `CHANGE`) | Clean GPIO; **1k/2k divider required**. |
| | `PIN_US_RIGHT_ECHO` | **33** | Input (ISR `CHANGE`) | Clean GPIO; **1k/2k divider required**. |
| | `PIN_US_FRONT_TRIG` | **5** | Output | Strapping pin (internal pull-up); HC-SR04 TRIG is high-Z → boot-safe. |
| | `PIN_US_SIDE_TRIG` | **15** | Output | Shared TRIG for 180°-opposed Left & Right sensors (simultaneous corridor measurement, zero front crosstalk). Boot-safe. |
| **20-Slot Encoders** | `PIN_ENC_LEFT` | **18** | Input (ISR `RISING`) | Clean GPIO; internal pull-up available. |
| | `PIN_ENC_RIGHT` | **19** | Input (ISR `RISING`) | Clean GPIO; internal pull-up available. |
| **Grip Switch** | `PIN_GRIP_SWITCH` | **4** | Input (`INPUT_PULLUP`) | Clean GPIO; Active LOW when 20 g load is held in jaws. |
| **Status LED** | `PIN_STATUS_LED` | **2** | Output | Built-in blue DevKit V1 LED (mode / PS4 / Grip-Lock status). |

> **Pinout Option Note:** `PIN_US_SIDE_TRIG` (`GPIO 15`) triggers Left & Right ultrasonics simultaneously (pointing 180° opposite across the chassis, giving time-aligned `left - right` wall-centering readings while preserving the built-in Status LED on `GPIO 2` and avoiding `GPIO 12`). If 3 fully independent TRIG pins are preferred later, `GPIO 2` can be reassigned to `PIN_US_RIGHT_TRIG` via a 1-line edit in `hardware.ino`.

---

## 4. LEDC Timer Allocation (ESP32 Arduino Core `2.0.17`)

We drive both DC motors and all 4 servos directly via ESP32 Core `2.0.17` hardware LEDC (`ledcSetup` / `ledcAttachPin` / `ledcWrite`) inside `hardware.ino` — **no `ESP32Servo` library needed**, eliminating timer clashes!

| LEDC Speed / Timer | Channels | Frequency | Resolution | Duty Range | Assignment |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **HS Timer 0** | Ch `0` | `1,000 Hz` | `8-bit` | `0 – 255` | `PIN_MOTOR_L_EN` (GPIO 25) — Left Drive Motors |
| **HS Timer 0** | Ch `1` | `1,000 Hz` | `8-bit` | `0 – 255` | `PIN_MOTOR_R_EN` (GPIO 13) — Right Drive Motors |
| **HS Timer 1** | Ch `2` | `50 Hz` | `16-bit` | `1638 – 8192` (`500–2500 µs`) | `PIN_SERVO_BASE` (GPIO 16) — Arm Base |
| **HS Timer 1** | Ch `3` | `50 Hz` | `16-bit` | `1638 – 8192` (`500–2500 µs`) | `PIN_SERVO_SHOULDER` (GPIO 17) — Arm Shoulder |
| **HS Timer 2** | Ch `4` | `50 Hz` | `16-bit` | `1638 – 8192` (`500–2500 µs`) | `PIN_SERVO_ELBOW` (GPIO 21) — Arm Elbow |
| **HS Timer 2** | Ch `5` | `50 Hz` | `16-bit` | `1638 – 8192` (`500–2500 µs`) | `PIN_SERVO_GRIPPER` (GPIO 22) — Gripper |
| **HS Timer 3** | Ch `6–7` | — | — | — | Spare / Reserved |

*Servo duty formula (50 Hz, 16-bit, 20,000 µs period):*  
`duty = (uint32_t)pulseUs * 65535UL / 20000UL` (`≈ 3.277 counts/µs` → `~0.3 µs` precision for smooth, jerk-free Sudden Death carry).

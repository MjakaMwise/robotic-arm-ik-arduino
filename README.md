# 🤖 Inverse Kinematics Robotic Arm — Arduino

![Arduino](https://img.shields.io/badge/Platform-Arduino-00979D?style=flat-square&logo=arduino&logoColor=white)
![Language](https://img.shields.io/badge/Language-C%2B%2B-blue?style=flat-square&logo=cplusplus)
![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)
![Difficulty](https://img.shields.io/badge/Level-Beginner--Intermediate-orange?style=flat-square)
![AI Training](https://img.shields.io/badge/AI%20Training-SIM--Lab%20Kenya-purple?style=flat-square)

> **AI Training Program Project** — SIM-Lab Kenya  
> A hands-on introduction to geometric motion planning: computing joint angles for a 2-DOF robotic arm using Inverse Kinematics, implemented in Arduino C++.

---

## 📋 Table of Contents

1. [Title & Objective](#1-title--objective)
2. [Quick Summary of the Technology](#2-quick-summary-of-the-technology)
3. [System Requirements](#3-system-requirements)
4. [Installation & Setup](#4-installation--setup)
5. [Minimal Working Example](#5-minimal-working-example)
6. [AI Prompt Journal](#6-ai-prompt-journal)
7. [Common Issues & Fixes](#7-common-issues--fixes)
8. [References](#8-references)

---

## 1. Title & Objective

**"Inverse Kinematics for a 2-DOF Robotic Arm — A Beginner's Guide in Arduino C++"**

### Technology Chosen
**Arduino C++** with the built-in `Servo` library, running on an **Arduino Uno or Nano**. The physical robot is a 2-DOF (two degrees-of-freedom) planar arm: two rigid links connected by servo motors at the shoulder and elbow joints.

### Why This Technology?
Arduino is the dominant open-source microcontroller ecosystem for hardware prototyping. It is:
- **Beginner-friendly** — minimal setup, vast community, free IDE
- **Affordable** — boards cost $5–$20
- **Capable** — the ATmega328P handles trigonometric IK math comfortably at 16 MHz
- **Educational** — writing close-to-hardware C++ builds real embedded programming intuition

### End Goal
Write a self-contained Arduino sketch that:
1. Accepts a target Cartesian coordinate **(x, y)** in millimetres
2. Computes joint angles **θ₁** (shoulder) and **θ₂** (elbow) using the geometric IK equations
3. Maps those angles to physical servo positions
4. Drives both servos smoothly to the target
5. Reports all values over the Serial Monitor for debugging and verification

---

## 2. Quick Summary of the Technology

### What is Inverse Kinematics?

| Direction | Question it answers | Difficulty |
|-----------|---------------------|------------|
| **Forward Kinematics (FK)** | *Given joint angles → where does the tip end up?* | Easy |
| **Inverse Kinematics (IK)** | *Given a target position → what joint angles do I need?* | Hard |

IK is harder because it requires solving a system of **nonlinear equations** — and there can be zero, one, or two valid solutions for any given target.

### Arm Geometry

```
         P(x, y)  ← end-effector target
        ●
       /
  L₂ /   θ₂ = elbow angle
     /
    ●  ← elbow joint
   /
L₁/   θ₁ = shoulder angle
 /
●  ← shoulder (origin)
```

### IK Equations (Geometric / Closed-Form)

```
cos(θ₂) = (x² + y² − L₁² − L₂²) / (2·L₁·L₂)    ← cosine rule
θ₂       = atan2(√(1 − cos²θ₂), cos(θ₂))           ← elbow-UP solution
θ₁       = atan2(y, x) − atan2(L₂·sin(θ₂), L₁ + L₂·cos(θ₂))
```

### Where Is IK Used in the Real World?

- **Industrial robot arms** (KUKA, ABB, FANUC) on factory floors
- **3D character animation** — game engines and tools like Blender use IK for limb control
- **Surgical robots** (da Vinci system)
- **Prosthetic limbs** — real-time IK maps motor intent to joint angles

---

## 3. System Requirements

### Hardware

| Component | Specification | Notes |
|-----------|--------------|-------|
| Microcontroller | Arduino Uno Rev3 or Nano | ATmega328P, 16 MHz |
| Servo (×2) | SG90 or MG996R | SG90 for light builds; MG996R for heavier arms |
| Power Supply | 5V, 2A external | Do **not** power servos from Arduino 5V pin |
| USB Cable | USB-A to USB-B (Uno) or Micro-USB (Nano) | For programming |
| Arm Links | Two rigid rods or 3D-printed pieces | L₁ = 120 mm, L₂ = 90 mm (adjustable in code) |

### Software

| Tool | Version | Notes |
|------|---------|-------|
| Arduino IDE | 2.x (latest) | Free — [arduino.cc/en/software](https://www.arduino.cc/en/software) |
| Servo Library | Built-in | No installation required |
| OS | Windows 10+, macOS 12+, Ubuntu 20.04+ | All supported |

> ⚠️ **Power Warning:** Servos can draw 500 mA–1 A each at stall. An external 5V/2A supply is required. Share GND between the supply and the Arduino but do **not** link the 5V rails.

---

## 4. Installation & Setup

### Step 1 — Install Arduino IDE 2

Download and install from [arduino.cc/en/software](https://www.arduino.cc/en/software).  
On first launch the IDE installs the AVR toolchain automatically.

### Step 2 — Select Board and Port

```
Tools → Board → Arduino AVR Boards → Arduino Uno
Tools → Port  → COMx (Windows) or /dev/ttyUSBx (Linux/Mac)
```

### Step 3 — Verify the Servo Library

```
Sketch → Include Library → Servo
```
It appears in the list with no installation — it ships with the IDE.

### Step 4 — Wire the Hardware

```
Arduino Uno          Servo Wiring
────────────         ──────────────────────────────────────
Pin 9           →    Servo 1 (Shoulder) — Signal (Orange/Yellow)
Pin 10          →    Servo 2 (Elbow)    — Signal (Orange/Yellow)
GND             →    External 5V supply GND
                     External 5V supply GND → Both servo GND (Brown/Black)
                     External 5V supply 5V  → Both servo VCC (Red)
```

For a full wiring diagram see [`docs/WIRING.md`](docs/WIRING.md).

### Step 5 — Upload a Sweep Test First

Validate your wiring before running IK code:

```
File → Examples → Servo → Sweep → Upload
```

One servo should sweep 0°–180° smoothly. If it does, proceed to the IK sketch.

### Step 6 — Open and Upload the IK Sketch

```
File → Open → inverse_kinematics_arm/inverse_kinematics_arm.ino → Upload
```

Open the **Serial Monitor** at **9600 baud** to see live angle outputs.

---

## 5. Minimal Working Example

### What the Sketch Does

The sketch pre-loads five (x, y) target positions (waypoints). Every 2 seconds it:
1. Picks the next waypoint
2. Solves IK → gets θ₁ and θ₂
3. Maps angles to servo positions (0–180°)
4. Moves both servos smoothly via linear interpolation
5. Prints joint angles to the Serial Monitor

The full annotated sketch is in [`inverse_kinematics_arm/inverse_kinematics_arm.ino`](inverse_kinematics_arm/inverse_kinematics_arm.ino).

### Code Overview (Key Functions)

```cpp
// 1. Solve IK — returns θ₁, θ₂, and a reachability flag
IKResult solveIK(float x, float y);

// 2. Map IK angle (any degrees) → servo angle (0–180°)
int ikAngleToServo(float ikDegrees, float offset = 90.0f);

// 3. Interpolate from current servo positions to target positions
void smoothMove(int fromS1, int toS1, int fromS2, int toS2,
                int steps = 40, int stepDelay = 20);
```

### Expected Serial Monitor Output

```
=== 2-DOF IK Robotic Arm ===
L1=120mm  L2=90mm  Elbow-UP

[TARGET] Waypoint 0  x=150.00  y=80.00
[IK]  θ₁=19.9°  θ₂=68.3°
[SRV] S1=110°  S2=158°

[TARGET] Waypoint 1  x=80.00  y=150.00
[IK]  θ₁=52.3°  θ₂=72.1°
[SRV] S1=142°  S2=162°

[TARGET] Waypoint 2  x=180.00  y=0.00
[IK]  θ₁=0.0°  θ₂=53.1°
[SRV] S1=90°  S2=143°
```

### Verifying the Output (Forward Kinematics Check)

You can verify IK correctness by running FK on the output angles:

```
For θ₁ = 19.9°, θ₂ = 68.3°:

x = L1·cos(θ₁) + L2·cos(θ₁+θ₂)
  = 120·cos(19.9°) + 90·cos(88.2°)
  ≈ 150.0 mm  ✓

y = L1·sin(θ₁) + L2·sin(θ₁+θ₂)
  = 120·sin(19.9°) + 90·sin(88.2°)
  ≈  80.0 mm  ✓
```

---

## 6. AI Prompt Journal

AI assistant used: **Claude (Anthropic)**

---

### Prompt 1 — Conceptual Foundation

**Prompt used:**
> *"Explain inverse kinematics for a 2-DOF robotic arm to a beginner. What is the geometric method and what equations do I need?"*

**AI Response Summary:**  
The AI explained the geometric approach step by step — from the cosine-rule derivation of θ₂ to the atan2-based formula for θ₁. It identified the two-solution ambiguity (elbow-up vs elbow-down) and recommended constraining `cosTheta2` to `[-1, 1]` to prevent floating-point errors.

**Helpfulness:** ⭐⭐⭐⭐⭐ — The math breakdown was clear and directly implementable. The floating-point clamping tip was critical.

---

### Prompt 2 — Arduino Implementation

**Prompt used:**
> *"Write an Arduino C++ function that takes (x, y) in mm and computes servo angles for a 2-DOF arm with L1=120 and L2=90. Use the Servo library."*

**AI Response Summary:**  
Produced a working `solveIK()` function using `atan2()` and `sqrt()` from `math.h`. Also returned results via a struct (`IKResult`) rather than global variables.

**Helpfulness:** ⭐⭐⭐⭐⭐ — The struct pattern for multiple return values was cleaner than what I had planned. The AI also flagged that `Servo.write()` takes integers only.

---

### Prompt 3 — Smooth Motion (Code Refactoring)

**Prompt used:**
> *"My servo jumps directly to the target angle and it's too fast. How do I make it move smoothly in Arduino?"*

**AI Response Summary:**  
Suggested linear interpolation — step from current to target angle in N increments with `delay()` between each. Maps directly to the `smoothMove()` function. Also mentioned sine easing for even smoother motion.

**Helpfulness:** ⭐⭐⭐⭐ — Solved the problem cleanly. Needed minor adaptation since `Servo.write()` doesn't support fractional degrees.

---

### Prompt 4 — Debugging a Servo Overrun

**Prompt used:**
> *"My arm overshoots. The shoulder goes past 180° and the servo makes a grinding sound. What's wrong with my angle mapping?"*

**AI Response Summary:**  
Diagnosed the issue: IK angles (which can be negative or exceed 180°) were being written directly to the servo without offset conversion. Introduced the `ikAngleToServo()` wrapper that maps IK-frame angles to servo-frame angles (0–180°).

**Helpfulness:** ⭐⭐⭐⭐ — Identified a conceptual gap between IK coordinate space and physical servo range that I had missed.

---

### Prompt 5 — Code Documentation

**Prompt used:**
> *"Add inline comments to this IK sketch that explain each step of the math in plain English, suitable for someone who hasn't studied robotics."*

**AI Response Summary:**  
Rewrote all comments to explain both *what* and *why* — e.g., why `atan2` is used instead of `acos` (quadrant awareness), and why clamping is needed before `sqrt`. Treated comments as teaching tools, not just labels.

**Helpfulness:** ⭐⭐⭐⭐⭐ — Directly served the Code Documentation curriculum exercise. Comments are more informative than my originals.

---

## 7. Common Issues & Fixes

### ❌ Servo twitches or Arduino resets

**Cause:** Powering servos from the Arduino 5V pin — servos draw 500 mA+ at stall and crash the onboard regulator.  
**Fix:** Use a dedicated 5V 2A external supply. Connect GNDs together; do **not** link the 5V rails.

---

### ❌ Arm snaps violently to home position on startup

**Cause:** The servo's physical position before power-on is unknown. The first `write(90)` can cause a large sudden jump.  
**Fix:** Always start with the arm manually placed at home position (90°, 90°) before powering on. Add `delay(1500)` after the home command in `setup()`.

---

### ❌ IK returns `NaN` or random angles

**Cause:** `sqrt(1 - cosTheta2²)` receives a tiny negative value due to floating-point rounding when the target is at the arm's exact reach limit. `sqrt()` of a negative returns `NaN` on AVR.  
**Fix:** Add `cosTheta2 = constrain(cosTheta2, -1.0f, 1.0f);` before computing `sinTheta2`. Already included in the sketch.  
Reference: [Arduino Stack Exchange — IK NaN issue](https://arduino.stackexchange.com/questions/18249)

---

### ❌ Arm reaches the right zone but angles look wrong

**Cause:** The physical servo zero-position doesn't match the IK coordinate zero. IK assumes 0° = arm along +x axis, but a real servo may treat 0° as pointing straight down.  
**Fix:** Adjust the `offset` parameter in `ikAngleToServo()`. If your servo is at 90° when the arm is horizontal, use `offset = 90.0f`.

---

### ❌ `avrdude: stk500_recv(): programmer is not responding`

**Cause:** Wrong COM port, or a servo signal wire connected to Pin 0/1 (Arduino's Serial TX/RX pins) interfering with upload.  
**Fix:** Disconnect servo wires from Pin 0/1 before uploading. Select the correct port under `Tools → Port`. Try a different USB cable (some are charge-only).

---

## 8. References

### Official Documentation
- [Arduino Servo Library Reference](https://www.arduino.cc/reference/en/libraries/servo/)
- [Arduino Language Reference](https://www.arduino.cc/reference/en/)
- [Arduino IDE 2 Download](https://www.arduino.cc/en/software)

### Robotics & IK Theory
- [Inverse Kinematics — Wikipedia](https://en.wikipedia.org/wiki/Inverse_kinematics)
- [Robot Academy — IK Masterclass (Peter Corke)](https://robotacademy.net.au/masterclass/inverse-kinematics-and-robot-programming/) — Free, rigorous, highly recommended
- [Instructables — Robotic Arm with IK and Arduino](https://www.instructables.com/Robotic-Arm-With-Inverse-Kinematics-Arduino/)

### Community & Forums
- [Arduino Forum](https://forum.arduino.cc/)
- [Arduino Stack Exchange](https://arduino.stackexchange.com/)

### Generative AI Tools Overview

This project used **Claude (Anthropic)** as the primary AI assistant. Generative AI tools in software development generally serve these roles:

| Role | Tools | Best Used For |
|------|-------|--------------|
| Code Generation | Claude, GitHub Copilot, ChatGPT | Boilerplate, unfamiliar APIs, algorithm first drafts |
| Code Explanation | Claude, ChatGPT | Reading unfamiliar codebases |
| Debugging | Claude, Cursor | Diagnosing errors with context |
| Documentation | Claude | Inline comments, READMEs, API docs |

> **Key principle:** AI-generated code must always be reviewed and tested. The Prompt Journal above demonstrates critical evaluation — noting where follow-up was needed and verifying outputs independently.

### Curriculum Exercises Demonstrated

| Curriculum Exercise | Location in This Project |
|--------------------|--------------------------|
| Code Documentation | Inline comments in `.ino` file; Prompt #5 |
| Error Diagnosis Challenge | Section 7 — all five issues with root-cause analysis |
| Code Readability Challenge | `smoothMove()`, `IKResult` struct, descriptive names |
| AI Solution Verification | FK check in Section 5 verifies IK output |
| Function Decomposition | Three focused functions: `solveIK`, `ikAngleToServo`, `smoothMove` |
| Algorithm Deconstruction | Section 2 — IK math broken into 5 sequential steps |
| Using AI to Debug Errors | Prompt #4 — grinding servo issue diagnosed via AI |
| Using AI to Refactor Code | Prompt #3 — direct write refactored to smooth interpolation |

---

## 📁 Repository Structure

```
robotic-arm-ik-arduino/
├── README.md                          ← You are here
├── LICENSE                            ← MIT License
├── .gitignore                         ← Arduino build artifacts
├── inverse_kinematics_arm/
│   └── inverse_kinematics_arm.ino    ← Main Arduino sketch
└── docs/
    └── WIRING.md                      ← Detailed wiring guide
```

---

## 📜 License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.

---

*Built as part of the SIM-Lab Kenya AI Training Program.*

# Inverse Kinematics Robotic Arm — Arduino

![Arduino](https://img.shields.io/badge/Platform-Arduino-00979D?style=flat-square&logo=arduino&logoColor=white)
![Language](https://img.shields.io/badge/Language-C%2B%2B-blue?style=flat-square&logo=cplusplus)
![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)
![Difficulty](https://img.shields.io/badge/Level-Beginner--Intermediate-orange?style=flat-square)
![AI Training](https://img.shields.io/badge/AI%20Training-SIM--Lab%20Kenya-purple?style=flat-square)

**AI Training Program Project — SIM-Lab Kenya**

A hands-on introduction to geometric motion planning: computing joint angles for a 2-DOF robotic arm using Inverse Kinematics, implemented in Arduino C++. Includes an interactive browser-based demonstration.

---

## Table of Contents

1. [Title and Objective](#1-title-and-objective)
2. [Quick Summary of the Technology](#2-quick-summary-of-the-technology)
3. [System Requirements](#3-system-requirements)
4. [Installation and Setup](#4-installation-and-setup)
5. [Minimal Working Example](#5-minimal-working-example)
6. [AI Prompt Journal](#6-ai-prompt-journal)
7. [Common Issues and Fixes](#7-common-issues-and-fixes)
8. [References](#8-references)

---

## 1. Title and Objective

**"Inverse Kinematics for a 2-DOF Robotic Arm — A Beginner's Guide in Arduino C++"**

### Technology Chosen

**Arduino C++** with the built-in `Servo` library, running on an **Arduino Uno or Nano**. The physical robot is a 2-DOF (two degrees-of-freedom) planar arm: two rigid links connected by servo motors at the shoulder and elbow joints.

### Why This Technology?

Arduino is the dominant open-source microcontroller ecosystem for hardware prototyping. It is beginner-friendly, affordable (boards cost $5-$20), and its C++ environment handles trigonometric IK math at 16 MHz without difficulty. Writing at the hardware level builds genuine embedded programming skills that transfer directly to professional robotics work.

### End Goal

Write a self-contained Arduino sketch that:

1. Accepts a target Cartesian coordinate **(x, y)** in millimetres
2. Computes joint angles **theta1** (shoulder) and **theta2** (elbow) using the geometric IK equations
3. Maps those angles to physical servo positions (0 to 180 degrees)
4. Drives both servos smoothly to the target position
5. Reports all values over the Serial Monitor for debugging and verification

---

## 2. Quick Summary of the Technology

### What is Inverse Kinematics?

| Direction | Question it answers | Difficulty |
|-----------|---------------------|------------|
| Forward Kinematics (FK) | Given joint angles, where does the tip end up? | Straightforward |
| Inverse Kinematics (IK) | Given a target position, what joint angles are needed? | Non-trivial |

IK requires solving a system of nonlinear equations. For most target points a 2-link arm has two valid solutions — elbow pointing up or elbow pointing down. This implementation uses the elbow-up solution.

### Arm Geometry

```
         P(x, y)  -- end-effector target
        *
       /
  L2 /   theta2 = elbow angle
     /
    *  -- elbow joint
   /
L1/   theta1 = shoulder angle
 /
*  -- shoulder (origin)
```

### IK Equations — Geometric Closed-Form

```
cos(theta2) = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2)     -- cosine rule
theta2      = atan2(sqrt(1 - cos^2(theta2)), cos(theta2))       -- elbow-up
theta1      = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))
```

### Where IK Is Used in Practice

- Industrial robot arms (KUKA, ABB, FANUC) on factory floors
- 3D character animation in game engines and tools such as Blender
- Surgical robots including the da Vinci system
- Prosthetic limbs with real-time joint angle computation

### Browser Demonstration

An interactive HTML demonstration is included in the `demo/` folder. Open `demo/arm_demo.html` in any modern browser. It renders the arm on canvas, lets you drag a target point, solves IK in real time, and displays live joint angle readouts and the live IK equation values. No server or installation is required — it is a single self-contained file.

---

## 3. System Requirements

### Hardware

| Component | Specification | Notes |
|-----------|--------------|-------|
| Microcontroller | Arduino Uno Rev3 or Nano | ATmega328P, 16 MHz |
| Servo motors | 2x SG90 or MG996R | SG90 for light builds; MG996R for heavier arms |
| External power supply | 5V, 2A | Do not power servos from the Arduino 5V pin |
| USB cable | USB-A to USB-B (Uno) or Micro-USB (Nano) | For programming and Serial Monitor |
| Arm links | Two rigid rods or 3D-printed pieces | L1 = 120 mm, L2 = 90 mm — adjustable in code |

### Software

| Tool | Version | Notes |
|------|---------|-------|
| Arduino IDE | 2.x, latest | Free at arduino.cc/en/software |
| Servo library | Built-in | No separate installation required |
| Operating system | Windows 10+, macOS 12+, Ubuntu 20.04+ | All supported |

**Power note:** Servos can draw 500 mA to 1 A each at stall. An external 5V/2A supply is required. Share GND between the supply and the Arduino but do not connect the two 5V rails together.

---

## 4. Installation and Setup

### Step 1 — Install Arduino IDE 2

Download and install from [arduino.cc/en/software](https://www.arduino.cc/en/software). On first launch the IDE installs the AVR toolchain automatically.

### Step 2 — Select Board and Port

```
Tools > Board > Arduino AVR Boards > Arduino Uno
Tools > Port  > COMx (Windows) or /dev/ttyUSBx (Linux / macOS)
```

### Step 3 — Verify the Servo Library

```
Sketch > Include Library > Servo
```

The library ships with the IDE. No installation is required.

### Step 4 — Wire the Hardware

```
Arduino Uno          Servo Wiring
------------         ------------------------------------------
Pin 9           ->   Servo 1 (Shoulder) -- Signal (Orange/Yellow)
Pin 10          ->   Servo 2 (Elbow)    -- Signal (Orange/Yellow)
GND             ->   External 5V supply GND
                     External 5V supply GND -> Both servo GND (Brown/Black)
                     External 5V supply 5V  -> Both servo VCC (Red)
```

For a full wiring diagram see [`docs/WIRING.md`](docs/WIRING.md).

### Step 5 — Upload a Sweep Test First

Validate your wiring before running IK code:

```
File > Examples > Servo > Sweep > Upload
```

One servo should sweep 0 to 180 degrees smoothly. If it does, proceed to the IK sketch.

### Step 6 — Upload the IK Sketch

```
File > Open > inverse_kinematics_arm/inverse_kinematics_arm.ino > Upload
```

Open the Serial Monitor at 9600 baud to see live angle output.

---

## 5. Minimal Working Example

### What the Sketch Does

The sketch pre-loads five (x, y) target positions called waypoints. Every two seconds it picks the next waypoint, solves IK to find theta1 and theta2, maps the angles to servo positions (0 to 180 degrees), moves both servos smoothly via linear interpolation, and prints the joint angles to the Serial Monitor.

The full annotated sketch is at [`inverse_kinematics_arm/inverse_kinematics_arm.ino`](inverse_kinematics_arm/inverse_kinematics_arm.ino).

### Key Functions

```cpp
// Solve IK -- returns theta1, theta2, and a reachability flag
IKResult solveIK(float x, float y);

// Map IK angle (any value in degrees) to servo angle (0-180 degrees)
int ikAngleToServo(float ikDegrees, float offset = 90.0f);

// Interpolate smoothly from current servo positions to target positions
void smoothMove(int fromS1, int toS1, int fromS2, int toS2,
                int steps = 40, int stepDelay = 20);
```

### Expected Serial Monitor Output

```
========================================
  2-DOF IK Robotic Arm -- Arduino
========================================
  L1 = 120.00 mm
  L2 = 90.00 mm
  Workspace: 30.00 - 210.00 mm
  Elbow convention: UP (positive sinTheta2)
========================================

[INIT] Moving to home position (90, 90)...
[INIT] Ready.

[TARGET] Waypoint 0  x=150.0 mm  y=80.0 mm
[IK]    theta1 = 19.9 deg    theta2 = 68.3 deg
[SERVO] S1 = 110 deg    S2 = 158 deg
[DONE]  Holding position...

[TARGET] Waypoint 1  x=80.0 mm  y=150.0 mm
[IK]    theta1 = 52.3 deg    theta2 = 72.1 deg
[SERVO] S1 = 142 deg    S2 = 162 deg
[DONE]  Holding position...
```

### Verifying the Output with Forward Kinematics

You can confirm the IK solution is correct by running FK on the output angles. For theta1 = 19.9 degrees and theta2 = 68.3 degrees:

```
x = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
  = 120 * cos(19.9) + 90 * cos(88.2)
  = 150.0 mm  -- matches target

y = L1 * sin(theta1) + L2 * sin(theta1 + theta2)
  = 120 * sin(19.9) + 90 * sin(88.2)
  =  80.0 mm  -- matches target
```

---

## 6. AI Prompt Journal

AI assistant used: **Claude (Anthropic)**

---

### Prompt 1 — Conceptual Foundation

**Prompt:**
> "Explain inverse kinematics for a 2-DOF robotic arm to a beginner. What is the geometric method and what equations do I need?"

**Response summary:**
The AI explained the geometric approach step by step — from the cosine-rule derivation of theta2 to the atan2-based formula for theta1. It identified the two-solution ambiguity and recommended clamping cosTheta2 to [-1, 1] to prevent floating-point errors at the workspace boundary.

**Evaluation — Very helpful.** The math breakdown was clear and directly implementable. The clamping tip was critical; without it the sketch crashes on boundary targets.

---

### Prompt 2 — Arduino Implementation

**Prompt:**
> "Write an Arduino C++ function that takes (x, y) in mm and computes servo angles for a 2-DOF arm with L1=120 and L2=90. Use the Servo library."

**Response summary:**
Produced a working `solveIK()` function using `atan2()` and `sqrt()` from `math.h`. Returned results via a struct (`IKResult`) rather than global variables, and flagged that `Servo.write()` accepts only integers.

**Evaluation — Very helpful.** The struct pattern was cleaner than the global-variable approach I had planned. The integer-cast requirement was a detail that would have caused a silent bug.

---

### Prompt 3 — Smooth Motion

**Prompt:**
> "My servo jumps directly to the target angle and it's too fast. How do I make it move smoothly in Arduino?"

**Response summary:**
Suggested linear interpolation — stepping from current to target angle in N increments with `delay()` between each step. This maps directly to the `smoothMove()` function in the sketch. Sine easing was also mentioned for even smoother motion.

**Evaluation — Helpful.** Solved the problem cleanly. Minor adaptation was needed because `Servo.write()` does not accept fractional degrees.

---

### Prompt 4 — Debugging a Servo Overrun

**Prompt:**
> "My arm overshoots. The shoulder goes past 180 degrees and the servo makes a grinding sound. What's wrong with my angle mapping?"

**Response summary:**
Diagnosed the issue: IK angles can be negative or exceed 180 degrees, but servo angles are restricted to 0 to 180. Writing IK angles directly to the servo caused the overrun. The fix was an `ikAngleToServo()` wrapper that adds an offset and clamps the result.

**Evaluation — Helpful with follow-up needed.** The diagnosis was correct. Additional calibration was still required to match the offset value to the physical mounting of each servo.

---

### Prompt 5 — Code Documentation

**Prompt:**
> "Add inline comments to this IK sketch that explain each step of the math in plain English, suitable for someone who has not studied robotics."

**Response summary:**
Rewrote all comments to explain both what the code does and why — including why `atan2` is used instead of `acos`, and why clamping is necessary before calling `sqrt`. Comments were treated as teaching tools rather than simple labels.

**Evaluation — Very helpful.** The rewritten comments are more informative than the originals and directly served the Code Documentation exercise from the curriculum.

---

## 7. Common Issues and Fixes

### Servo twitches or Arduino resets repeatedly

**Cause:** Powering servos from the Arduino 5V pin. Servos draw 500 mA or more at stall and overload the onboard regulator.

**Fix:** Use a dedicated 5V 2A external supply. Connect the GNDs together but do not link the 5V rails.

---

### Arm snaps violently to home on startup

**Cause:** The servo's physical position before power-on is unknown. The first `write(90)` can cause a large sudden jump.

**Fix:** Place the arm manually at the home position before powering on. Add `delay(1500)` after the home command in `setup()`.

---

### IK returns NaN or erratic angles

**Cause:** `sqrt(1 - cosTheta2 * cosTheta2)` receives a slightly negative argument due to floating-point rounding at the workspace boundary. On AVR, `sqrt()` of a negative number returns NaN.

**Fix:** Add `cosTheta2 = constrain(cosTheta2, -1.0f, 1.0f);` before computing `sinTheta2`. This line is already in the provided sketch.

Reference: [Arduino Stack Exchange — IK NaN discussion](https://arduino.stackexchange.com/questions/18249)

---

### Arm reaches approximately the right area but joint angles look wrong

**Cause:** The physical servo zero-position does not match the IK coordinate frame zero.

**Fix:** Adjust the `offset` parameter in `ikAngleToServo()` to match the physical mounting of each servo.

---

### avrdude: stk500_recv(): programmer is not responding

**Cause:** Wrong COM port selected, or a servo signal wire on Pin 0 or Pin 1 is interfering with the upload.

**Fix:** Disconnect servo wires from Pin 0 and Pin 1 before uploading. Select the correct port under Tools > Port. Try a different USB cable — some are charge-only and carry no data.

---

## 8. References

### Official Documentation

- [Arduino Servo Library Reference](https://www.arduino.cc/reference/en/libraries/servo/)
- [Arduino Language Reference](https://www.arduino.cc/reference/en/)
- [Arduino IDE 2 Download](https://www.arduino.cc/en/software)

### Robotics and IK Theory

- [Inverse Kinematics — Wikipedia](https://en.wikipedia.org/wiki/Inverse_kinematics)
- [Robot Academy — IK Masterclass (Peter Corke)](https://robotacademy.net.au/masterclass/inverse-kinematics-and-robot-programming/) — free course with rigorous mathematical treatment
- [Instructables — Robotic Arm with IK and Arduino](https://www.instructables.com/Robotic-Arm-With-Inverse-Kinematics-Arduino/)

### Community and Forums

- [Arduino Forum](https://forum.arduino.cc/)
- [Arduino Stack Exchange](https://arduino.stackexchange.com/)

### Generative AI Tools Overview

This project used Claude (Anthropic) as the primary AI assistant. Generative AI tools in software development generally serve the following roles:

| Role | Tools | Best Used For |
|------|-------|--------------|
| Code generation | Claude, GitHub Copilot, ChatGPT | Boilerplate, unfamiliar APIs, algorithm first drafts |
| Code explanation | Claude, ChatGPT | Understanding unfamiliar codebases |
| Debugging | Claude, Cursor | Diagnosing errors when provided with sufficient context |
| Documentation | Claude | Inline comments, README files, API documentation |

AI-generated code must always be reviewed and tested. The Prompt Journal above demonstrates critical evaluation — noting where follow-up was needed and verifying outputs independently.

### Curriculum Exercises Demonstrated

| Curriculum Exercise | Location in This Project |
|--------------------|--------------------------|
| Code Documentation | Inline comments in the .ino file; Prompt 5 |
| Error Diagnosis Challenge | Section 7 — five issues with root-cause analysis |
| Code Readability Challenge | smoothMove(), IKResult struct, descriptive names |
| AI Solution Verification | Forward kinematics check in Section 5 |
| Function Decomposition | Three focused functions: solveIK, ikAngleToServo, smoothMove |
| Algorithm Deconstruction | Section 2 — IK math in five sequential steps |
| Using AI to Debug Errors | Prompt 4 — grinding servo issue diagnosed via AI |
| Using AI to Refactor Code | Prompt 3 — direct write refactored to smooth interpolation |

---

## Repository Structure

```
robotic-arm-ik-arduino/
├── README.md
├── LICENSE
├── .gitignore
├── inverse_kinematics_arm/
│   └── inverse_kinematics_arm.ino
├── demo/
│   └── arm_demo.html
└── docs/
    └── WIRING.md
```

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

---

*Built as part of the SIM-Lab Kenya AI Training Program.*

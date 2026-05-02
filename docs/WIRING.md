# Wiring Guide — 2-DOF IK Robotic Arm

## Connection Table

| Arduino Pin | Wire Colour | Connects To |
|-------------|-------------|-------------|
| Pin 9       | Orange / Yellow | Servo 1 (Shoulder) — Signal |
| Pin 10      | Orange / Yellow | Servo 2 (Elbow) — Signal |
| GND         | Black | External 5V supply — Negative terminal |
| *(no connection)* | | External 5V supply Positive → Both servo Red (VCC) wires |
| *(no connection)* | | External 5V supply Negative → Both servo Brown/Black (GND) wires |

## ASCII Wiring Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                        ARDUINO UNO                          │
│                                                             │
│   Pin 9  ──────────────────────────── SERVO 1 (Shoulder)   │
│                                       [Signal: Orange/Yel]  │
│   Pin 10 ──────────────────────────── SERVO 2 (Elbow)      │
│                                       [Signal: Orange/Yel]  │
│   GND ────────────┐                                         │
│                   │                                         │
└───────────────────┼─────────────────────────────────────────┘
                    │
                    │  (GND shared — CRITICAL)
                    │
┌───────────────────┼─────────────────────────────────────────┐
│  EXTERNAL         │                                         │
│  5V / 2A  ────────┘ (−) ────────────── Both servo GND      │
│  SUPPLY    ────────── (+) ────────────── Both servo VCC     │
│                                       [Red wires]           │
└─────────────────────────────────────────────────────────────┘
```

## ⚠️ Critical Power Warning

**DO NOT** power servos from the Arduino's 5V pin.

- Hobby servos can draw **500 mA – 1 A each at stall**
- The Arduino's onboard 5V regulator is rated for **~400 mA total**
- Overloading it causes **voltage drops, random resets, and permanent damage**

**Always use an external 5V 2A supply for the servos.**  
The only connection between the Arduino and the external supply is a **shared GND**.

## Servo Connector Pinout (Standard 3-Wire)

```
  Servo connector (looking at wire-end):
  ┌─────────────────────┐
  │  GND  VCC  Signal   │
  │  [●]  [●]   [●]    │
  │ Brown  Red  Orange  │
  └─────────────────────┘
  (colours may vary by brand — always verify with datasheet)
```

## Servo Signal Pins Used

| Joint | Pin | Servo Type (Recommended) |
|-------|-----|--------------------------|
| Shoulder (Joint 1) | **Pin 9** | MG996R (for heavier arms) or SG90 |
| Elbow (Joint 2) | **Pin 10** | SG90 (elbow load is lighter) |

> **Note:** Pins 9 and 10 on the Arduino Uno produce 50 Hz PWM, which is the standard signal frequency for hobby servos.

## Adjusting for Your Servo's Zero Position

Physical servos may have different mechanical zero positions depending on how they are mounted. If the arm's position doesn't match the expected angles:

1. Open `inverse_kinematics_arm.ino`
2. Find the `ikAngleToServo()` calls in `loop()`
3. Adjust the `offset` parameter:
   - `offset = 90.0f` → servo at 90° when IK angle is 0° (arm horizontal)
   - `offset = 0.0f`  → servo at 0° when IK angle is 0°
   - Negative offsets are also valid if your servo is mounted inverted

```cpp
// Example: adjust offset for shoulder servo
int s1 = ikAngleToServo(ik.theta1_deg, 90.0f);  // ← change 90.0f here
int s2 = ikAngleToServo(ik.theta2_deg, 90.0f);  // ← and here for elbow
```

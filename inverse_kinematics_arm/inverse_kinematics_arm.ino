/*
 * ================================================================
 *  Project  : 2-DOF Planar Robotic Arm — Inverse Kinematics
 *  Platform : Arduino Uno / Nano (ATmega328P, 16 MHz)
 *  Library  : Servo.h (built-in — no installation required)
 *
 *  Author   : SIM-Lab Kenya — AI Training Program
 *  License  : MIT
 *
 *  Physical Setup:
 *    Servo 1 (Shoulder) → Pin 9
 *    Servo 2 (Elbow)    → Pin 10
 *    Link 1 length (L1) = 120 mm  (shoulder → elbow)
 *    Link 2 length (L2) =  90 mm  (elbow → end-effector tip)
 *
 *  Coordinate System:
 *    Origin (0, 0) is at the shoulder joint.
 *    +x is horizontal (forward), +y is vertical (upward).
 *
 *  IK Method: Geometric / Trigonometric (closed-form)
 *    - Two solutions exist for most targets (elbow-up, elbow-down)
 *    - This code uses the ELBOW-UP solution (positive sinTheta2)
 *
 *  IK Equations:
 *    cos(θ₂) = (x² + y² − L1² − L2²) / (2·L1·L2)
 *    θ₂       = atan2(+√(1 − cos²θ₂), cos(θ₂))       ← elbow-up
 *    θ₁       = atan2(y, x) − atan2(L2·sinθ₂, L1 + L2·cosθ₂)
 * ================================================================
 */

#include <Servo.h>      // Built-in servo PWM driver
#include <math.h>       // atan2(), sqrt(), cos(), sin(), abs()


// ── PIN DEFINITIONS ──────────────────────────────────────────────
const int SERVO_SHOULDER_PIN = 9;
const int SERVO_ELBOW_PIN    = 10;


// ── ARM GEOMETRY (millimetres) ───────────────────────────────────
// Change these to match your physical arm's link lengths.
const float L1 = 120.0;   // Length of link 1 (shoulder to elbow joint)
const float L2 =  90.0;   // Length of link 2 (elbow joint to tip)

// Workspace limits — target distance from shoulder must be within these.
const float MAX_REACH = L1 + L2;          // Arm fully extended  = 210 mm
const float MIN_REACH = abs(L1 - L2);     // Arm fully folded    =  30 mm


// ── SERVO OBJECTS ────────────────────────────────────────────────
Servo shoulderServo;   // Controls joint 1 (base rotation / shoulder)
Servo elbowServo;      // Controls joint 2 (elbow bend)


// ── DATA STRUCTURES ──────────────────────────────────────────────

/*
 * Point2D — a Cartesian target position for the end-effector.
 * Units: millimetres, relative to the shoulder joint (origin).
 */
struct Point2D {
  float x;
  float y;
};

/*
 * IKResult — output of the IK solver.
 * Contains both joint angles and a flag indicating whether
 * the target was reachable (inside the arm's workspace).
 */
struct IKResult {
  float theta1_deg;   // Shoulder angle in degrees (IK coordinate frame)
  float theta2_deg;   // Elbow angle in degrees (IK coordinate frame)
  bool  reachable;    // false = target is outside the arm's workspace
};


// ── WAYPOINTS ────────────────────────────────────────────────────
/*
 * A list of (x, y) positions for the arm tip to visit in sequence.
 * All units in millimetres. Origin is at the shoulder joint.
 * Adjust or extend this array to change the arm's movement path.
 */
const Point2D waypoints[] = {
  { 150.0,  80.0 },   // Waypoint A — extended upper-right region
  {  80.0, 150.0 },   // Waypoint B — upper area (arm more vertical)
  { 180.0,   0.0 },   // Waypoint C — fully horizontal (max x at y=0)
  { 100.0, 100.0 },   // Waypoint D — mid-field diagonal
  {  60.0,  60.0 },   // Waypoint E — near origin; tests tight elbow fold
};

const int NUM_WAYPOINTS = sizeof(waypoints) / sizeof(waypoints[0]);


// ── FUNCTION: solveIK ────────────────────────────────────────────
/*
 * Compute shoulder (θ₁) and elbow (θ₂) angles for a given target.
 *
 * Approach: Geometric / closed-form — no iteration required.
 *
 * Step 1: Use the cosine rule to find cos(θ₂).
 *         The three sides of the triangle are L1, L2, and the
 *         straight-line distance from shoulder to target.
 *
 * Step 2: Take the positive square root of (1 − cos²θ₂) to get
 *         sin(θ₂). Positive = elbow-UP solution.
 *         Use constrain() first — floating-point arithmetic can
 *         push cosTheta2 just outside [−1, 1], making sqrt() return NaN.
 *
 * Step 3: Use atan2() to get θ₂ in the correct quadrant.
 *         atan2(y, x) is preferred over acos() because it handles
 *         all four quadrants correctly.
 *
 * Step 4: Decompose θ₁ into two sub-angles and subtract them:
 *         - The angle from origin to target (atan2(y, x))
 *         - The angle the second link "offsets" the first
 *
 * Step 5: Convert radians → degrees for servo use.
 */
IKResult solveIK(float x, float y) {
  IKResult result;

  // Distance² from shoulder to target
  float distSq = x * x + y * y;
  float dist   = sqrt(distSq);

  // --- Reachability check ---
  // If the target is beyond the arm's stretch or inside its minimum
  // fold radius, there is no valid IK solution.
  if (dist > MAX_REACH || dist < MIN_REACH) {
    result.reachable = false;
    Serial.print("[IK] UNREACHABLE — distance from origin: ");
    Serial.print(dist);
    Serial.print(" mm  (valid range: ");
    Serial.print(MIN_REACH);
    Serial.print("–");
    Serial.print(MAX_REACH);
    Serial.println(" mm)");
    return result;
  }

  // --- Step 1: Cosine rule → cos(θ₂) ---
  float cosTheta2 = (distSq - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

  // --- Step 2: Clamp and compute sin(θ₂) ---
  // Must clamp before sqrt() to avoid NaN on boundary targets.
  cosTheta2 = constrain(cosTheta2, -1.0f, 1.0f);
  float sinTheta2 = sqrt(1.0f - cosTheta2 * cosTheta2);  // ELBOW-UP (+)

  // --- Step 3: θ₂ via atan2 ---
  float theta2_rad = atan2(sinTheta2, cosTheta2);

  // --- Step 4: θ₁ via atan2 decomposition ---
  float k1 = L1 + L2 * cosTheta2;
  float k2 = L2 * sinTheta2;
  float theta1_rad = atan2(y, x) - atan2(k2, k1);

  // --- Step 5: Radians → Degrees ---
  result.theta1_deg = theta1_rad * (180.0f / PI);
  result.theta2_deg = theta2_rad * (180.0f / PI);
  result.reachable  = true;

  return result;
}


// ── FUNCTION: ikAngleToServo ─────────────────────────────────────
/*
 * Convert an IK-frame angle (can be negative or > 90°) to a
 * physical servo angle (must be in [0, 180°]).
 *
 * IK angles are in the arm's mathematical coordinate frame.
 * Physical servos have their own zero reference that depends on
 * how they are mounted.
 *
 * Parameters:
 *   ikDegrees — the angle from solveIK (degrees, any value)
 *   offset    — servo angle when IK angle = 0° (default: 90°)
 *               Adjust this to match your physical servo mounting.
 *
 * Returns: servo angle clamped to [0, 180].
 */
int ikAngleToServo(float ikDegrees, float offset = 90.0f) {
  int servoAngle = (int)(ikDegrees + offset);
  return constrain(servoAngle, 0, 180);
}


// ── FUNCTION: smoothMove ─────────────────────────────────────────
/*
 * Move both servos from their current positions to new targets
 * using linear interpolation, so the arm glides rather than snaps.
 *
 * Parameters:
 *   fromS1, toS1  — shoulder servo: start and end angle
 *   fromS2, toS2  — elbow servo:    start and end angle
 *   steps         — how many interpolation steps (more = smoother)
 *   stepDelay     — milliseconds between steps (more = slower)
 *
 * Note: This is a blocking function — it occupies the CPU for
 * (steps × stepDelay) ms. For non-blocking motion, consider
 * a timer-based approach.
 */
void smoothMove(int fromS1, int toS1,
                int fromS2, int toS2,
                int steps = 40, int stepDelay = 20) {
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;               // Normalised t ∈ [0.0, 1.0]
    int s1 = (int)(fromS1 + t * (toS1 - fromS1));
    int s2 = (int)(fromS2 + t * (toS2 - fromS2));
    shoulderServo.write(s1);
    elbowServo.write(s2);
    delay(stepDelay);
  }
}


// ── GLOBAL STATE ─────────────────────────────────────────────────
int currentWaypoint = 0;
int prevS1 = 90;    // Last shoulder servo position (start at home)
int prevS2 = 90;    // Last elbow servo position (start at home)


// ── SETUP ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.println("========================================");
  Serial.println("  2-DOF IK Robotic Arm — Arduino");
  Serial.println("========================================");
  Serial.print("  L1 = "); Serial.print(L1); Serial.println(" mm");
  Serial.print("  L2 = "); Serial.print(L2); Serial.println(" mm");
  Serial.print("  Workspace: "); Serial.print(MIN_REACH);
  Serial.print(" – "); Serial.print(MAX_REACH); Serial.println(" mm");
  Serial.println("  Elbow convention: UP (positive sinTheta2)");
  Serial.println("========================================\n");

  // Attach servos to their PWM pins
  shoulderServo.attach(SERVO_SHOULDER_PIN);
  elbowServo.attach(SERVO_ELBOW_PIN);

  // Move to home position: arm pointing upward at 90° / 90°
  // Physical position depends on your servo mounting offsets.
  Serial.println("[INIT] Moving to home position (90°, 90°)...");
  shoulderServo.write(90);
  elbowServo.write(90);
  delay(1500);   // Allow servos to reach home before starting waypoints
  Serial.println("[INIT] Ready.\n");
}


// ── LOOP ─────────────────────────────────────────────────────────
void loop() {

  // --- 1. Get the current target waypoint ---
  Point2D target = waypoints[currentWaypoint];

  Serial.print("[TARGET] Waypoint ");
  Serial.print(currentWaypoint);
  Serial.print("  x="); Serial.print(target.x, 1);
  Serial.print(" mm  y="); Serial.print(target.y, 1);
  Serial.println(" mm");

  // --- 2. Solve IK for this target ---
  IKResult ik = solveIK(target.x, target.y);

  if (ik.reachable) {

    // --- 3. Convert IK angles to servo angles ---
    int s1 = ikAngleToServo(ik.theta1_deg, 90.0f);
    int s2 = ikAngleToServo(ik.theta2_deg, 90.0f);

    // Print joint angles for Serial Monitor verification
    Serial.print("[IK]    θ₁ = "); Serial.print(ik.theta1_deg, 1);
    Serial.print("°    θ₂ = "); Serial.print(ik.theta2_deg, 1);
    Serial.println("°");

    Serial.print("[SERVO] S1 = "); Serial.print(s1);
    Serial.print("°    S2 = "); Serial.print(s2);
    Serial.println("°");

    // --- 4. Move smoothly to the new position ---
    smoothMove(prevS1, s1, prevS2, s2, /*steps=*/40, /*stepDelay=*/20);

    // Save servo positions for next iteration's smooth start
    prevS1 = s1;
    prevS2 = s2;

    Serial.println("[DONE]  Holding position...\n");

  } else {
    // Target was unreachable — skip it and continue to next waypoint
    Serial.println("[SKIP]  Waypoint skipped (unreachable).\n");
  }

  // --- 5. Advance to next waypoint (wraps around) ---
  currentWaypoint = (currentWaypoint + 1) % NUM_WAYPOINTS;

  // Hold current position before moving to the next waypoint
  delay(2000);
}

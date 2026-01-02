# MotionExecutor Integration Review

This review summarizes critical findings after comparing the integration plan with the actual MotionExecutor and DriverManager implementations. It focuses on behavioral correctness, API mismatches, and risks that could cause regressions.

## Findings (Highest Risk First)

1. **Reference-point mismatch (critical):**
   MotionExecutor commands are in terms of the ACTIVE_REFERENCE_POINT, not the robot center. Pedro Pathing uses robot-center poses. Feeding GameManager targets directly into MotionExecutor will introduce systematic offsets unless the active reference point is set to robot center or poses are converted. This is not addressed in the plan and will cause field-position errors.
   - Evidence: `MotionConfig.ACTIVE_REFERENCE_POINT` and MotionState comments about reference point coordinates.
   - Files: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/MotionConfig.java:282`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/MotionState.java:34`

2. **Control loop timing assumptions (critical):**
   MotionExecutor primitives use a fixed control-loop period with `Thread.sleep(CONTROL_LOOP_PERIOD_MS)`. Removing the blocking loop and calling it from an OpMode loop creates variable dt, which affects velocity ramping and stall detection. The adapter needs explicit loop-rate control or dt handling.
   - Evidence: `moveToPose()` uses fixed sleep; velocity ramping uses wall-clock time.
   - Files: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/MotionExecutor.java:739`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/MotionExecutor.java:1180`

3. **Double-update risk (high):**
   `MotionExecutor.setVelocity()` calls `updateState()` internally. If the adapter also calls `updateState()` before `setVelocity()`, odometry is updated twice per cycle and state may become inconsistent (read/write timing drift).
   - Evidence: `setVelocity()` begins with `updateState()`.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/MotionExecutor.java:301`

4. **Alignment mapping bug (high):**
   `MotionExecutor.rotate()` expects a **relative** angle delta, while `DriverManager.startAlignCycle()` uses absolute headings. Mapping align to `rotate()` directly will overshoot by current heading. Must convert absolute target to delta or use a method that accepts absolute heading.
   - Evidence: `rotate()` calls `spinInPlace()` which adds delta to current heading.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/MotionExecutor.java:1031`

5. **Odometry initialization assumption (high):**
   The plan assumes MotionExecutor constructor resets odometry, but it does not. Only `resetOdometry()` or `setFieldOrigin()` establishes the field coordinate system. Missing this will misalign field coordinates from the start.
   - Evidence: dedicated reset methods; no reset in constructor.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/MotionExecutor.java:1064`

6. **Unit confusion risk (medium):**
   MotionExecutor uses **degrees** for heading throughout. The plan comment "Convert from radians" while calling `getHeading(AngleUnit.DEGREES)` suggests unit confusion that could propagate into implementation.
   - Evidence: MotionState heading uses degrees.
   - Files: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/MotionState.java:34`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:361`

7. **Path behavior regression (medium):**
   Current Pedro Pathing uses a Bezier with heading interpolation. Replacing it with a straight `moveToPose()` changes the path shape and may reduce smoothness or collision avoidance. The plan does not acknowledge or mitigate this change.
   - Evidence: DriverManager builds a Bezier line and sets heading interpolation.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/DriverManager.java:96`

## Recommended Adjustments

- **Reference-point strategy:** Either set `ACTIVE_REFERENCE_POINT` to robot center for autonomous or convert GameManager targets into reference-point coordinates before executing motion.
- **Adapter timing:** Implement a single-iteration step for `moveToPose()` and enforce a fixed loop rate (or compute dt and adapt controllers accordingly).
- **Avoid double updates:** Let `setVelocity()` handle state updates or ensure only one update per cycle.
- **Alignment correctness:** Convert absolute heading targets to deltas before calling `rotate()` or use a motion method that accepts absolute target heading.
- **Explicit odometry init:** Call `resetOdometry()` or `setFieldOrigin()` at the beginning of auto with known starting pose.
- **Preserve path intent:** If path shape matters, consider building short waypoint sequences or using MotionExecutor primitives that approximate curvature rather than a single endpoint.

## Additional Notes

- `DriveHardware.getDistanceController()` and `getHeadingController()` are available; reusing them is viable, but the adapter must reset them exactly as `moveToPose()` does.
- Stall detection in `moveToPose()` depends on error deltas over time; changing timing or skipping iterations will alter detection behavior.


## Second Review (integration.md v2.0)

This second review reflects a careful read of the updated integration plan.

### Findings (Highest Risk First)

1. **Double-update assumption still unsafe (critical):**
   The plan asserts double update is "NOT AN ISSUE" while the adapter still calls `updateState()` and then `setVelocity()` (which calls `updateState()` again). The blocking loop doing this does not guarantee correctness in a non-blocking context. This needs an explicit decision and justification (or removal of the extra update).
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:11`
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:356`

2. **Heading error calculation ignores wraparound (high):**
   The adapter pseudo-logic uses `Math.abs(targetHeading - currentHeading)` which fails near ±180° and can prevent convergence. Use normalized heading error (like `MotionState.getHeadingError()`) or `motionState.atTarget()`.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:366`

3. **Wrong velocity parameter in align cycle (high):**
   `startAlignCycle()` passes `MotionConfig.MAX_ANGULAR_VELOCITY` as the *linear* max velocity to `startMoveToPose()`. This clamps translation incorrectly and is a semantic mismatch.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:621`

4. **Velocity ramping attribution is incorrect (medium):**
   The plan claims velocity ramping is built into `setVelocity()`. In MotionExecutor, ramping is applied in `moveToPose()` before calling `setVelocity()`. Direct `setVelocity()` calls do not ramp, so the plan is misleading.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:276`

5. **Inconsistent timing source (medium):**
   `update(double nowSec)` ignores `nowSec` and uses `System.currentTimeMillis()`. This makes timing harder to test and creates two sources of truth. Either use `nowSec` consistently or enforce a fixed loop period explicitly.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:336`

6. **Reference-point switch needs scope (medium):**
   Recommending `ACTIVE_REFERENCE_POINT` be robot center can break subsystems that rely on intake/scoring offsets. The plan should specify when and how this is toggled (autonomous-only? runtime config?).
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:246`

### Open Questions / Assumptions

- Is it acceptable to change `ACTIVE_REFERENCE_POINT` globally, or do you need a runtime toggle for autonomous only?
- Should completion be based on MotionExecutor's `motionState.atTarget()` or a custom error check?
- If timing is variable, should stall detection thresholds be adjusted as well?


## Third Review (Follow-up Decisions)

### Double update strategy (updateState vs setVelocity)

- **Recommended**: Use **exactly one `updateState()` per control cycle**. Keep the explicit update at the top of the adapter loop and add a way for `setVelocity()` to **skip its internal update** when the adapter already updated state.
- **Rationale**: The correct control loop order is sensor update → compute error → command actuators. Double-updating creates uneven timing (two quick updates, then a long gap), which can distort odometry deltas, ramping, and stall detection. Removing the explicit update instead makes the controller run one full cycle behind the latest pose, which is also suboptimal.
- **If MotionExecutor cannot be changed immediately**: keep the explicit update and accept the double update **as a temporary compromise**, but document the timing risk and plan to remove the redundant update once a skip-update path is available.

### Reference point selection (ACTIVE_REFERENCE_POINT)

- **Recommended**: Treat the reference point as a **runtime‑selectable mode**, not a global constant. Set it explicitly during autonomous initialization and restore it for TeleOp or other subsystems.
- **Rationale**: Autonomous may want robot‑center coordinates for pathing parity, while TeleOp and other features may depend on intake/scoring offsets. A permanent global change can silently break those behaviors.
- **Action**: Add an explicit “mode switch” API (e.g., `setReferencePointForMode(AUTO/TELEOP)`) and log the active reference point at mode transitions.


## Fourth Review (After Decision: Keep Double Update)

This review reflects the explicit decision to keep the double `updateState()` behavior for now. Issues below assume that choice is intentional and documented.

### Findings (Highest Risk First)

1. **Heading error wraparound still incorrect (high):**
   The adapter example uses `Math.abs(targetHeading - currentHeading)` which fails near ±180° and can prevent convergence. Use a normalized heading error or `motionState.atTarget()`.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:366`

2. **Align cycle passes angular limit as linear max velocity (high):**
   `startAlignCycle()` calls `startMoveToPose()` with `MotionConfig.MAX_ANGULAR_VELOCITY` in the linear-velocity slot. This is a unit mismatch and will clamp translation incorrectly.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:621`

3. **Double-update justification should be explicit (medium):**
   The plan should state why the uneven timing is acceptable and what telemetry/tests will detect regressions (stall detection drift, odometry jitter). The current text calls it "NOT AN ISSUE" without a stated acceptance criterion.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:11`

4. **Velocity ramping attribution remains misleading (medium):**
   The plan says ramping is built into `setVelocity()`, but MotionExecutor ramps before calling `setVelocity()` in `moveToPose()`. The adapter needs its own ramping (which it does), so this statement should be corrected.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:276`

5. **Timing source inconsistency (medium):**
   The adapter uses `System.currentTimeMillis()` and ignores the `nowSec` parameter. Pick one source for timing and note why (testability and determinism).
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:336`

6. **Reference-point scope still unclear (medium):**
   The plan recommends setting `ACTIVE_REFERENCE_POINT` to robot center without specifying when it is restored or how TeleOp is protected. This should be defined as an autonomous-only mode switch.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:246`

### Open Questions

- What concrete acceptance criteria confirm that the double-update timing does not degrade stall detection or odometry stability?
- Will the reference point be toggled per mode (AUTO vs TELEOP), and where is that decision enforced?


## Fifth Review (Latest Read)

### Findings (Highest Risk First)

1. **Reference point runtime switching is still inconsistent with actual types (critical):**
   The document now notes that `ACTIVE_REFERENCE_POINT` is `public static final ComponentPosition` (correct), but the earlier guidance still suggests runtime switching without a concrete, type-correct API. If you keep it compile-time only (Option 1), remove any runtime switch code examples to avoid confusion.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:285`

2. **Control mode placeholder is now clear (good), but avoid implying functional behavior elsewhere (medium):**
   The placeholder section is explicit, but other parts still show control-mode APIs in use and may read as functional. Ensure every mention reiterates “no behavioral effect yet.”
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:671`

3. **Double-update compromise is documented (good), but acceptance criteria missing (medium):**
   The plan flags timing risk but doesn’t specify how you will detect regressions (e.g., log odometry deltas, stall detection rate). Add measurable acceptance checks.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:615`

4. **Stall detection now present (good), but ensure parity with MotionExecutor (medium):**
   The adapter includes stall timing logic. The plan should explicitly state whether thresholds match MotionExecutor and whether it checks “progress” on position the same way.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:514`

5. **Velocity ramping statements now consistent (good), but fix older “Features to Maintain” line if still present (low):**
   You corrected ramping in the adapter section, but scan the summary lists to ensure none still say ramping is in `setVelocity()`.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:401`

### Open Questions

- Is the final decision **compile-time reference point only**, or do you intend to add a runtime API later? The document currently mixes both ideas.
- What are the concrete acceptance criteria for the double-update compromise (what telemetry, what thresholds)?


## Sixth Review (Latest Read After Intake Reference Point Decision)

### Findings (Highest Risk First)

1. **`startAlignCycle()` snippet uses undefined variables in telemetry (medium):**
   The example logs `currentHeadingDeg`, `currentX`, and `currentY`, but those symbols are never defined in the snippet. This makes the example misleading and non-compilable if copied. Use values from `currentRobotCenter` (or define the variables explicitly).
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:874`

2. **`startAlignCycle()` comment still mentions relative delta but implementation doesn’t use it (low):**
   The snippet says “convert absolute target heading to relative delta,” but then uses `moveToPose()` with the absolute heading. That’s fine, but the comment should be updated to avoid confusion.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:835`

3. **Reference-point conversion in `startCycle()` assumes target heading units are correct (low):**
   `convertRobotCenterToReferencePoint()` uses `getHeading(AngleUnit.DEGREES)` internally. If GameManager ever provides a Pose2D with radians but doesn’t set units properly, conversion will be wrong. Consider a short note that Pose2D must be constructed with correct units or explicitly normalized before conversion.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:703`

### Open Questions

- Should the `startAlignCycle()` example explicitly show the variables used in telemetry to prevent copy‑paste errors?


## Seventh Review (Post‑Update Codebase Read)

Reviewed `motion/` and `autonomous/` folders after latest code changes, then re‑read `integration.md`.

### Findings (Highest Risk First)

1. **Plan uses outdated reference‑point source (critical):**
   The codebase no longer has `MotionConfig.ACTIVE_REFERENCE_POINT`; reference point is now managed in `FieldPositions` via `FieldPositions.getActiveReferencePoint()` (default `SCORING_POINT`). The plan still describes MotionExecutor using `ACTIVE_REFERENCE_POINT` and implies a default that may not match. Update the doc to reference `FieldPositions` and explicitly set the desired reference point (if intake is intended).
   - Files: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/FieldPositions.java:16`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/CoordinateTransformer.java:33`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:92`

2. **Odometry init API mismatch (high):**
   `integration.md` calls `motionExecutor.setFieldOrigin(...)`, but MotionExecutor now exposes `resetToFieldOrigin(Pose2D)` and FieldPositions provides `setReferencePointInitialPosition(...)`. The plan should be updated to the new API to avoid implementation drift.
   - Files: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/MotionExecutor.java:1083`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/FieldPositions.java:118`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:329`

3. **Adapter control law in plan diverges from MotionExecutor (high):**
   The plan’s example uses `distanceController.calculate(distanceError)` and `headingController.calculate(headingError)`, then computes `angleToTarget - currentHeading` for `vx/vy`. MotionExecutor’s `moveToPose()` uses `distanceController.calculate(-currentDistance)` and `headingController.calculate(currentHeading)` with a **field‑centric** direction vector from `(deltaX, deltaY)` (no heading subtraction). The plan should match MotionExecutor’s actual control logic if fidelity is the goal.
   - Files: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/MotionExecutor.java:817`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:180`

4. **Implementation status mismatch (medium):**
   `integration.md` claims critical issues are resolved and Pedro Pathing is removed, but `DriverManager.java` is still a TODO skeleton (no MotionExecutor adapter, no conversions). The plan should reflect that the code is not yet implemented.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/DriverManager.java:32`

5. **Reference‑point default conflict with stated intent (medium):**
   The decision text says to follow MotionExecutor’s current reference point (intake), but `FieldPositions` defaults to `SCORING_POINT`. If intake is intended, the plan should include an explicit call to `FieldPositions.setActiveReferencePoint(RobotConstants.INTAKE_POINT_REF)` during auto init.
   - Files: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/motion/FieldPositions.java:22`, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/calibration/RobotConstants.java:277`

### Open Questions

- Should the adapter replicate MotionExecutor’s exact control law (negative distance error + field‑centric direction), or is a different law acceptable?
- Where will the reference point be explicitly set for autonomous if intake is desired (FieldPositions API vs a centralized init step)?


## Eighth Review (Updated integration.md)

### Findings (Highest Risk First)

1. **Inconsistent “resolved” statement about double update (high):**
   The plan now says “Double Update - latest code handles odometry updates correctly, no concern,” but the adapter section still calls `motionExecutor.updateState()` and then `setVelocity()`, which calls `updateState()` again. That is still the documented temporary compromise. The “no concern” phrasing is inaccurate and should be softened or removed.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:58`

2. **Reference point default mismatch vs stated intent (medium):**
   The plan states default reference point is `SCORING_POINT` and notes runtime changes via `FieldPositions.setActiveReferencePoint()`, but it never specifies the intended value for autonomous (intake vs scoring). If intake is desired, the plan should add an explicit initialization step that sets it.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:40`

3. **Adapter control law depends on `getCurrentRobotCenterPose()` which is not in code (medium):**
   The pseudocode uses `transformer.getCurrentRobotCenterPose()`, but CoordinateTransformer does not define that method. The plan should use the existing conversion from `motionExecutor.getMotionState().getCurrentPose()` via `convertReferencePointToRobotCenter(...)` to avoid phantom APIs.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:150`

4. **Angle usage in adapter pseudocode still mixes frames (medium):**
   The pseudocode uses `targetHeading` and converts reference point to robot center, but does not clarify which frame the heading is in after conversion. In MotionExecutor, the heading is field‑absolute degrees in both frames. The plan should explicitly state this to prevent future confusion.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:134`

5. **Control mode default mismatch (low):**
   “Key Learnings” says default is HYBRID, but later the plan says placeholder always uses PURE_FEEDBACK. That’s fine for the adapter, but the section should clarify that adapter behavior is fixed regardless of MotionExecutor default.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:26`

### Open Questions

- Where will autonomous explicitly set the active reference point (intake vs scoring), and where will it be restored, if needed?
- Should the adapter strictly mirror MotionExecutor’s control law or allow future deviations?


## Ninth Review (Latest integration.md)

### Findings (Highest Risk First)

1. **Static call to `CoordinateTransformer.convertRobotCenterToReferencePoint()` is invalid (high):**
   The method is an instance method, but the plan calls it statically in `startAlignCycle()`. This will not compile if copied literally. Use `transformer.convertRobotCenterToReferencePoint(...)` instead.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:365`

2. **Stall detection state not reset in `startMoveToPose()` (medium):**
   The adapter pseudocode uses `stallTimer` and `lastPositionError` in `update()`, but `startMoveToPose()` does not reset them. This can cause immediate false stall detection from stale values. Initialize `stallTimer.reset()` and set `lastPositionError` from the initial error.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:139`

3. **Reference point intent (intake) not explicitly set (medium):**
   The plan states the default reference point is `SCORING_POINT` but doesn’t add an explicit auto init step to switch to intake if that’s the chosen behavior. Add a line to call `FieldPositions.setActiveReferencePoint(RobotConstants.INTAKE_POINT_REF)` during auto init.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:40`

4. **Adapter snippet uses `driveHardware` without defining it (low):**
   `startMoveToPose()` references `driveHardware.getDistanceController()` but doesn’t show how `driveHardware` is obtained (e.g., `DriveHardware driveHardware = motionExecutor.getDriveHardware();`). Add that line for clarity and to prevent copy‑paste errors.
   - File: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/StateMachines/integration.md:136`


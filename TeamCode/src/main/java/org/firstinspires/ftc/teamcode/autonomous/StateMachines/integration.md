# MotionExecutor Integration Plan

## Overview

**Goal**: Integrate `MotionExecutor` into the StateMachines architecture to replace Pedro Pathing, while maintaining **fully non-blocking behavior** for concurrent subsystem orchestration.

**Key Challenge**: MotionExecutor uses blocking `while` loops, but StateMachines requires non-blocking `update()` methods.

**Solution**: Create a `MotionExecutorAdapter` that extracts MotionExecutor's control loop into a single-iteration `update()` method.

---

## Key Learnings from Debug Folder

**From `BaseMotionTest.java`, `SimpleMotionTest.java`, `SimpleAutoTest.java`:**

1. **Odometry Initialization**:
   - Use `motionExecutor.resetToFieldOrigin()` or `resetToFieldOrigin(Pose2D)` (NOT `setFieldOrigin()`)
   - Reference point initial position managed via `FieldPositions.getReferencePointInitialPosition()`

2. **Reference Point Management**:
   - Default reference point is `SCORING_POINT` (FieldPositions line 24)
   - Can be changed at runtime via `FieldPositions.setActiveReferencePoint(ComponentPosition)`
   - Access via `FieldPositions.getActiveReferencePoint()`

3. **Control Mode**:
   - Examples show `PURE_FEEDBACK` being used
   - Default is `HYBRID` but can be set via `setControlMode()`

4. **MotionExecutor API Usage Pattern**:
   - Initialize → `resetToFieldOrigin()` → `updateState()` → `moveToPose()` → Check completion
   - All coordinates are in reference point frame unless explicitly converted

---

## Current Status

### StateMachine (Non-Blocking)
- `update()` called every OpMode loop cycle
- Returns immediately - multiple subsystems run concurrently
- State-driven: `IDLE` → `MOVING` → `DONE`

**Example: DriverManager (Current - Pedro Pathing)**
```java
public void update(double nowSec) {
    follower.update();  // Non-blocking
    
    if (state == MOVING && !follower.isBusy()) {
        finish(DriveResult.ARRIVED_OK);
    }
}
```

### MotionExecutor (Blocking)
- `moveToPose()` contains `while` loop - **blocks until complete**
- Cannot run other subsystems concurrently
- Designed for simple sequential sequences

**Example: FVMotionTest**
```java
motionExecutor.moveToPose(0, 0, 180, 10);
// ⚠️ BLOCKS until robot reaches target
```

**Key Differences:**

| Aspect | StateMachine | MotionExecutor |
|--------|-------------|----------------|
| **Execution** | Non-blocking, state-driven | Blocking, command-driven |
| **Concurrency** | Multiple subsystems together | Sequential only |
| **Timing** | Variable (OpMode loop) | Fixed (`Thread.sleep(20)`) |

---

## Integration Architecture

```
┌─────────────────┐
│  GameManager    │  Orchestrates game states (non-blocking)
└────────┬────────┘
         │ startCycle(target)
         ▼
┌─────────────────┐
│  DriverManager  │  Manages drive state lifecycle
└────────┬────────┘
         │ startMoveToPose(x, y, heading)
         ▼
┌─────────────────┐
│MotionExecutor   │  Extracts control loop from blocking methods
│   Adapter       │  Non-blocking update() interface
└────────┬────────┘
         │ setVelocity(vx, vy, omega)
         ▼
┌─────────────────┐
│MotionExecutor   │  Core motion control (PID, odometry, kinematics)
└─────────────────┘
```

---

## Critical Issues & Solutions

### ✅ Resolved Issues

1. **Double Update** - Latest code handles odometry updates correctly, no concern
2. **Heading Wraparound** - Use `MotionState.getHeadingError()` for proper normalization
3. **Variable Timing** - Calculate `dt` from `nowSec` parameter for velocity ramping
4. **PID Reset** - Reset controllers in `startMoveToPose()` (matching `moveToPose()` lines 765-767)
5. **Stall Detection** - Implement matching MotionExecutor's thresholds and timing
6. **Odometry Init** - Explicitly call `resetToFieldOrigin(Pose2D)` at startup

### ⚠️ Design Decisions

1. **Reference Point Mismatch**:
   - **MotionExecutor**: Uses `FieldPositions.getActiveReferencePoint()` (defaults to `SCORING_POINT`)
   - **GameManager**: Uses robot center coordinates
   - **Solution**: Convert robot center → reference point in `DriverManager.startCycle()` using `CoordinateTransformer.convertRobotCenterToReferencePoint()`
   - **Note**: Reference point is managed via `FieldPositions.setActiveReferencePoint()` - defaults to `SCORING_POINT` at runtime

2. **Units**:
   - **MotionExecutor**: Degrees for heading
   - **GameManager**: Uses `Pose2D` (supports both)
   - **Solution**: Convert to degrees when calling adapter

3. **Alignment**:
   - **Do NOT use `rotate()`** - it expects relative angles
   - **Use `moveToPose()`** at current position with target heading

4. **Control Mode API**:
   - **Status**: Placeholder - exposed but currently non-functional
   - **Current**: Always uses PURE_FEEDBACK (direct PID)
   - **Future**: Can implement HYBRID/FEEDFORWARD modes later

5. **Path Shape**:
   - **Pedro Pathing**: Bézier curves (smooth)
   - **MotionExecutor**: Straight lines
   - **Acceptable**: Simplifies control, test during validation

---

## Implementation Steps

### Step 1: Create MotionExecutorAdapter

**File**: `motion/MotionExecutorAdapter.java`

**Key Responsibilities**:
1. Extract control loop from `MotionExecutor.moveToPose()` (lines 820-872)
2. Execute one iteration per `update()` call (no `while` loop)
3. Manage state: `IDLE` → `EXECUTING` → `COMPLETE`/`FAILED`/`ABORTED`
4. Handle PID reset, stall detection, velocity ramping, timeout
5. **CRITICAL**: Match MotionExecutor's exact control law (robot center coordinates, negative distance, field-centric direction vector)

**Critical Implementation Details**:

```java
public class MotionExecutorAdapter {
    private MotionExecutor motionExecutor;
    private AdapterState state = AdapterState.IDLE;
    
    private double targetX, targetY, targetHeading;
    private double maxVelocity, timeoutSec;
    private ElapsedTime timer, stallTimer;
    private double lastUpdateTimeSec = 0.0;
    private double previousVx = 0.0, previousVy = 0.0, previousOmega = 0.0;
    private double lastPositionError = Double.MAX_VALUE;
    
    public void startMoveToPose(double x, double y, double heading, 
                                double velocity, double timeout) {
        // Store target, reset state
        this.targetX = x;
        this.targetY = y;
        this.targetHeading = heading;
        this.maxVelocity = velocity;
        this.timeoutSec = timeout;
        this.state = AdapterState.EXECUTING;
        this.timer.reset();
        this.stallTimer.reset();  // CRITICAL: Reset stall timer (matches MotionExecutor line 755)
        this.lastUpdateTimeSec = 0.0;
        
        // Update state and set target to calculate initial error (matches MotionExecutor lines 758, 765, 768, 781)
        motionExecutor.updateState();
        MotionState motionState = motionExecutor.getMotionState();
        motionState.setTarget(x, y, heading);
        this.lastPositionError = motionState.getPositionError();  // Initialize from actual initial error
        
        // Reset PID controllers (CRITICAL!) - do this after setting target
        DriveHardware driveHardware = motionExecutor.getDriveHardware();
        driveHardware.getDistanceController().reset(0.0);
        driveHardware.getHeadingController().reset(heading);
    }
    
    public void update(double nowSec) {
        if (state != AdapterState.EXECUTING) return;
        
        // Calculate dt for velocity ramping
        double dt = (lastUpdateTimeSec > 0) 
            ? Math.max(nowSec - lastUpdateTimeSec, 0.001)
            : MotionConfig.CONTROL_LOOP_PERIOD_MS / 1000.0;
        lastUpdateTimeSec = nowSec;
        
        // Update odometry
        motionExecutor.updateState();
        
        // Check timeout
        if (timer.seconds() > timeoutSec) {
            finish(MotionResult.FAILED, "Timeout");
            return;
        }
        
        // Check completion (uses proper heading wraparound handling)
        MotionState motionState = motionExecutor.getMotionState();
        motionState.setTarget(targetX, targetY, targetHeading);
        if (motionState.atTarget()) {
            finish(MotionResult.SUCCESS, "Arrived");
            return;
        }
        
        // Stall detection (matches MotionExecutor thresholds)
        double positionError = motionState.getPositionError();
        if (Math.abs(lastPositionError - positionError) < MotionConfig.STALL_VELOCITY_THRESHOLD * 0.02) {
            if (stallTimer.milliseconds() > MotionConfig.STALL_DETECTION_TIME_MS) {
                finish(MotionResult.FAILED, "Stalled");
                return;
            }
        } else {
            stallTimer.reset();
        }
        lastPositionError = positionError;
        
        // CRITICAL: Match MotionExecutor's exact control law (lines 761-863)
        // MotionExecutor converts reference point target to robot center target, then uses robot center for control
        
        CoordinateTransformer transformer = motionExecutor.getCoordinateTransformer();
        
        // Convert reference point target to robot center target (MotionExecutor line 761-762)
        Pose2D robotCenterTarget = transformer.convertReferencePointToRobotCenter(
            targetX, targetY, targetHeading);
        
        // Get current robot center pose (MotionExecutor line 823)
        // NOTE: getCurrentRobotCenterPose() exists in CoordinateTransformer (line 143)
        // It internally does: getCurrentPose() → convertReferencePointToRobotCenter()
        // This matches exactly how MotionExecutor accesses it
        Pose2D currentRobotCenter = transformer.getCurrentRobotCenterPose();
        
        // Calculate direction vector toward target (field-centric, from deltaX/deltaY)
        double robotCenterTargetX = robotCenterTarget.getX(DistanceUnit.INCH);
        double robotCenterTargetY = robotCenterTarget.getY(DistanceUnit.INCH);
        double currentRobotCenterX = currentRobotCenter.getX(DistanceUnit.INCH);
        double currentRobotCenterY = currentRobotCenter.getY(DistanceUnit.INCH);
        
        double deltaX = robotCenterTargetX - currentRobotCenterX;
        double deltaY = robotCenterTargetY - currentRobotCenterY;
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        DriveHardware driveHardware = motionExecutor.getDriveHardware();
        // CRITICAL: Negate distance - PID error = setpoint(0) - (-distance) = +distance (MotionExecutor line 840)
        double desiredLinearSpeed = driveHardware.getDistanceController().calculate(-distanceToTarget);
        
        // Convert speed and direction to velocity components (field-centric)
        double vx, vy;
        if (distanceToTarget > 0.001) {
            double directionX = deltaX / distanceToTarget;  // Normalized direction (lines 845-846)
            double directionY = deltaY / distanceToTarget;
            vx = desiredLinearSpeed * directionX;  // Line 847
            vy = desiredLinearSpeed * directionY;  // Line 848
        } else {
            vx = 0.0;
            vy = 0.0;
        }
        
        // Apply velocity limits (lines 857-858)
        vx = Math.max(-maxVelocity, Math.min(maxVelocity, vx));
        vy = Math.max(-maxVelocity, Math.min(maxVelocity, vy));
        
        // Heading controller uses current heading (not error) - MotionExecutor line 861
        double omega = driveHardware.getHeadingController().calculate(motionState.getHeading());
        omega = Math.max(-MotionConfig.MAX_ANGULAR_VELOCITY,
                Math.min(MotionConfig.MAX_ANGULAR_VELOCITY, omega));
        
        // Apply velocity ramping (prevents jerky motion)
        double[] ramped = applyVelocityRampingWithDt(vx, vy, omega, dt);
        
        // Apply velocity (always FIELD_CENTRIC to match moveToPose())
        motionExecutor.setVelocity(ramped[0], ramped[1], ramped[2], 
                                   MotionState.CoordinateMode.FIELD_CENTRIC);
    }
    
    private double[] applyVelocityRampingWithDt(double vx, double vy, double omega, double dt) {
        // Apply ramping with variable dt (matches MotionExecutor logic)
        // ... implementation ...
    }
}
```

### Step 2: Refactor DriverManager

**File**: `DriverManager.java`

**Key Changes**:

1. **Replace Follower with MotionExecutorAdapter**:
```java
// Remove:
private final Follower follower;

// Add:
private final MotionExecutor motionExecutor;
private final MotionExecutorAdapter motionAdapter;
```

2. **Update Constructor**:
```java
public DriverManager(HardwareMap hardwareMap, Telemetry telemetry) {
    // Get hardware
    DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
    DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
    DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
    DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRightDrive");
    GoBildaPinpointDriver odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    
    // Create MotionExecutor and adapter
    this.motionExecutor = new MotionExecutor(frontLeft, frontRight, backLeft, backRight, odometry);
    this.motionAdapter = new MotionExecutorAdapter(motionExecutor);
}
```

3. **Update `startCycle()` - Coordinate Conversion**:
```java
public void startCycle(DriveGoalKind kind, Pose2D target, double timeoutSec) {
    // Convert robot center (GameManager) → reference point (MotionExecutor)
    FieldPose refPointTarget = motionExecutor.getCoordinateTransformer()
        .convertRobotCenterToReferencePoint(target);
    
    // Extract coordinates (already in inches and degrees)
    double x = refPointTarget.x;
    double y = refPointTarget.y;
    double heading = refPointTarget.heading;
    
    // Start motion
    motionAdapter.startMoveToPose(x, y, heading, MotionConfig.MAX_LINEAR_VELOCITY, timeoutSec);
    
    state = DriveState.MOVING;
    result = DriveResult.NONE;
}
```

4. **Update `startAlignCycle()` - Use moveToPose()**:
```java
public void startAlignCycle(double headingDeg, double timeoutSec) {
    // Get current reference point pose
    motionExecutor.updateState();
    Pose2D currentRefPointPose = motionExecutor.getMotionState().getCurrentPose();
    
    // Convert reference point to robot center for calculation
    CoordinateTransformer transformer = motionExecutor.getCoordinateTransformer();
    double currentX = currentRefPointPose.getX(DistanceUnit.INCH);
    double currentY = currentRefPointPose.getY(DistanceUnit.INCH);
    double currentHeading = currentRefPointPose.getHeading(AngleUnit.DEGREES);
    
    Pose2D currentRobotCenter = transformer.convertReferencePointToRobotCenter(
        currentX, currentY, currentHeading);
    
    // Create target robot center (same position, new heading)
    Pose2D targetRobotCenter = new Pose2D(
        DistanceUnit.INCH,
        currentRobotCenter.getX(DistanceUnit.INCH),
        currentRobotCenter.getY(DistanceUnit.INCH),
        AngleUnit.DEGREES,
        headingDeg
    );
    
    // Convert robot center target back to reference point coordinates
    // NOTE: convertRobotCenterToReferencePoint() is a static method (CoordinateTransformer line 79)
    FieldPose refPointTarget = CoordinateTransformer.convertRobotCenterToReferencePoint(
        targetRobotCenter);
    
    // Use moveToPose() with small linear velocity (focus on rotation, minimal translation)
    motionAdapter.startMoveToPose(refPointTarget.x, refPointTarget.y, refPointTarget.heading,
                                  1.0, timeoutSec);  // Small linear velocity for pure alignment
    
    state = DriveState.MOVING;
    result = DriveResult.NONE;
    timer.reset();
    
    telemetry.addData("Drive", "Start ALIGN cycle to %.1f° at (%.1f, %.1f)",
        headingDeg, currentX, currentY);
}
```

5. **Update `update()`**:
```java
public void update(double nowSec) {
    motionAdapter.update(nowSec);
    
    if (state == DriveState.IDLE || state == DriveState.DONE) return;
    
    if (motionAdapter.isComplete()) {
        MotionResult result = motionAdapter.getResult();
        finish(result.success ? DriveResult.ARRIVED_OK : DriveResult.PATH_FAILED);
    }
}
```

6. **Update `setStartingPose()`**:
```java
    public void setStartingPose(Pose2D startingPose) {
        // CRITICAL: MotionExecutor doesn't reset odometry automatically
        // Use resetToFieldOrigin(Pose2D) - this is the correct API
        // startingPose is already in robot center coordinates, convert to reference point first
        FieldPose refPointPose = motionExecutor.getCoordinateTransformer()
            .convertRobotCenterToReferencePoint(startingPose);
        
        Pose2D fieldOrigin = new Pose2D(
            DistanceUnit.INCH,
            refPointPose.x,
            refPointPose.y,
            AngleUnit.DEGREES,
            refPointPose.heading
        );
        
        motionExecutor.resetToFieldOrigin(fieldOrigin);
    }
```

### Step 3: Update VortexAutoOpMode

**Remove Pedro Pathing references**:
- Remove `import com.pedropathing.geometry.Pose;`
- Use `Pose2D` directly instead of Pedro `Pose`
- Update comments to remove Pedro references

---

## Testing Checklist

### Unit Testing
- [ ] MotionExecutorAdapter: `startMoveToPose()` → `update()` → `isComplete()` flow
- [ ] Coordinate conversion: robot center → reference point
- [ ] PID reset on new motion start
- [ ] Stall detection triggers correctly
- [ ] Velocity ramping works with variable `dt`
- [ ] Heading wraparound near ±180° boundaries

### Integration Testing
- [ ] DriverManager: All `DriveGoalKind` types work
- [ ] Timeout and abort behavior
- [ ] `setStartingPose()` initializes odometry correctly

### System Testing
- [ ] Full `VortexAutoOpMode` sequence
- [ ] Concurrent subsystems (drive + intake + shoot)
- [ ] Autonomous sequence completion
- [ ] Motion accuracy matches or exceeds Pedro Pathing

---

## Implementation Status

### ⚠️ Not Yet Implemented

**Current State**: `DriverManager.java` is currently a TODO skeleton with Pedro Pathing code removed. Integration plan is ready, but code implementation is pending.

**Critical Fixes Applied to Plan**:
- ✅ Reference point API updated to `FieldPositions.getActiveReferencePoint()` (defaults to `SCORING_POINT`)
- ✅ Odometry init API updated to `resetToFieldOrigin(Pose2D)`
- ✅ Adapter control law corrected to match MotionExecutor exactly:
  - Uses `calculate(-distanceToTarget)` (negative distance)
  - Uses robot center coordinates for control
  - Calculates direction from (deltaX, deltaY) - no angle subtraction
  - Uses `calculate(motionState.getHeading())` for heading (current heading, not error)

**Remaining Work:**
1. Implement `MotionExecutorAdapter.java` with corrected control law
2. Refactor `DriverManager.java` to use adapter (currently TODO skeleton)
3. Test thoroughly at each stage
4. Verify motion accuracy matches MotionExecutor behavior

---

## Open Questions

1. **Hardware Names**: Verify motor names match config (`frontLeftDrive`, `frontRightDrive`, etc.)
2. **Control Mode Usage**: Currently non-functional placeholder - implement later if needed
3. **Path Smoothness**: Test acceptability of straight lines vs Bézier curves

---

## Reference

- **MotionExecutor**: `motion/MotionExecutor.java`
  - `moveToPose()`: Lines 739-882 (control loop to extract)
  - Control law: Lines 820-872 (robot center coordinates, negative distance, field-centric)
  - `setVelocity()`: Lines 301-342 (called by adapter)
  - PID reset: Lines 765-767 (must replicate in adapter)
  - Stall detection: Lines 796-817 (must replicate in adapter)

- **FieldPositions**: `motion/FieldPositions.java`
  - `getActiveReferencePoint()`: Gets current reference point (defaults to `SCORING_POINT`)
  - `setActiveReferencePoint()`: Sets reference point at runtime
  - `getReferencePointInitialPosition()`: Gets initial position
  - `resetToFieldOrigin()`: Resets odometry to field origin

- **CoordinateTransformer**: `motion/CoordinateTransformer.java`
  - `convertRobotCenterToReferencePoint()`: Main conversion method

- **MotionState**: `motion/MotionState.java`
  - `getHeadingError()`: Proper heading wraparound handling
  - `atTarget()`: Robust completion check

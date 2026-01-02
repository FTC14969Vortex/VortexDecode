# VortexAutoOpMode Behavior Analysis

This document compares `VortexAutoOpMode` (state machine implementation) with `FullAutoOperateTest` (reference blocking implementation) to understand behaviors, similarities, and differences.

---

## 1. Constants, Settings, and Configurations

### Time-Based Settings
- **`autoTotalTimeSec`**: `30.0` seconds
  - Total autonomous period duration
  - Used for time-based aborts and parking transitions
  - Always tracked for logging/telemetry even when time limits are disabled

- **`parkReserveSec`**: `5.0` seconds
  - Time reserved for parking at the end of autonomous
  - When `timeRemaining <= parkReserveSec`, robot transitions to parking
  - Ensures robot has enough time to reach parking position

- **`driveTimeoutSec`**: `4.0` seconds
  - Maximum time allowed for drive operations
  - Safety timeout to prevent infinite drive cycles

- **`intakeTimeoutSec`**: `3.0` seconds
  - Maximum time allowed for intake operations
  - Can be ignored during intake drive (see `ignoreTimeoutDuringDrive`)

- **`shootTimeoutSec`**: `3.0` seconds
  - Maximum time allowed for shooting operations
  - Separate from spin-up timeout (3s) and firing timeout (10s)

### Game Configuration
- **`ballsPerSpot`**: `3`
  - Number of balls to collect per intake spot
  - Matches FullAutoOperateTest's 3-shot sequence

- **`SHOOT_DISTANCE`**: `45.0` inches
  - Fallback shooting distance if CameraServo is unavailable
  - Used when vision system fails or is not initialized

### Logging Configuration
- **`ENABLE_DETAILED_LOGGING`**: `true`
  - Enables detailed file logging for all state machines
  - Creates separate log files for GameManager, DriverManager, IntakeManager, ShootManager
  - Logs saved to app-scoped external files directory

### Alliance Configuration
- **Alliance Selection**: Runtime selection via gamepad
  - Press **X** for BLUE alliance (Tag 20)
  - Press **B** for RED alliance (Tag 24)
  - Default: BLUE alliance
  - Field positions automatically mirrored for RED alliance

### Vision Configuration
- **AprilTag Processor**: TAG_36h11 family
- **Output Units**: Inches (distance), Degrees (angle)
- **Camera**: "Webcam 1"
- **Auto Odometry Correction**: `false` (disabled)
- **Camera Servo Position**: Center (for alignment)

### State Machine Configuration
- **`enforceAutoTimeLimit`**: `true` (default, not explicitly set in VortexAutoOpMode)
  - When `true`: Time-gated behavior (competition mode)
    - Enforces 30-second hard limit
    - Starts parking when 5 seconds remaining
    - Aborts intake if < 1 second remaining
    - Times out alignment after 3 seconds
  - When `false`: Run-to-completion behavior (parity testing mode)
    - No time-based aborts
    - No alignment timeout (waits until alignment completes)
    - No critical intake abort
    - Runs until all cycles complete (matches FullAutoOperateTest)

- **`dryRun`**: `false`
  - Hardware operations are enabled
  - Matches FullAutoOperateTest behavior

---

## 2. Behaviors That Are the Same as FullAutoOperateTest

### Sequence Flow
- ✅ **Initial shoot sequence**: Drive to shooting position → align → shoot 3 preloaded samples
- ✅ **Intake-shoot cycles**: For each of 3 intake positions:
  - Drive to intake start position
  - Intake while driving to finish position
  - Drive to shooting position
  - Align and shoot
- ✅ **Parking**: Final movement to parking position

### Intake Behavior
- ✅ **Full power during intake**: `1.0` power when actively intaking
- ✅ **Travel power**: `0.5` power when traveling to shooting position
- ✅ **Gate position**: Set to `GATE_INTAKE` before starting intake
- ✅ **Intake stops during travel**: Intake stops when driving to intake start position

### Shooting Behavior
- ✅ **3 shots per cycle**: Uses 3 flipper shots (120°, 150°, 180°)
- ✅ **Gate control**: Close gate before spin-up, open gate before shooting
- ✅ **Flipper sequence**: Turn flipper → wait (increasing delay) → reset flipper
- ✅ **Flywheel velocity**: Based on AprilTag distance via CameraServo
- ✅ **Shoot anyway on spin-up failure**: Proceeds to shoot even if flywheel doesn't reach target velocity

### Vision and Alignment
- ✅ **AprilTag-based alignment**: Uses CameraServo to calculate optimal shooting angle (when vision available)
- ✅ **Distance-based velocity**: Flywheel velocity calculated from AprilTag distance
- ✅ **Alignment before shooting**: Aligns to shooting angle before shooting when vision is available and within timeout
  - If `cameraServo == null`: Skips alignment, proceeds directly to shooting
  - If alignment times out (3s when `enforceAutoTimeLimit=true`): Proceeds to shoot anyway
  - If alignment fails: Proceeds to shoot anyway
- ✅ **200ms settle time**: Brief settle after successful alignment before shooting

### Parallel Preparation
- ✅ **Gate closes during drive**: Gate closes while driving to shooting position
- ✅ **Flywheel spin-up during drive**: Flywheel target velocity captured at start of drive, then maintained during drive
  - Target velocity is calculated once when drive starts (based on AprilTag distance at that moment)
  - `updateSpinUp()` is called during drive to maintain velocity, but target is not recomputed
  - Velocity control continues during alignment and between shots
- ✅ **Intake travel power**: Intake runs at 0.5 power during drive to shooting position

### Field Positions
- ✅ **Same field positions**: Uses same `FieldPositions` constants
- ✅ **Mirroring for RED**: RED alliance positions are mirrored correctly
- ✅ **Start position**: `START_NEAR`
- ✅ **Shooting position**: `SHOOTING_NEAR` (single position for all shots)
- ✅ **Intake positions**: `INTAKE_1_START/FINISH`, `INTAKE_2_START/FINISH`, `INTAKE_3_START/FINISH`
- ✅ **Parking position**: `PARKING_NEAR`

### Subsystem Initialization
- ✅ **Intake starts stopped**: `stopIntake()` called during init
- ✅ **Gate starts in INTAKE position**: `setGatePosition(GATE_INTAKE)` during init
- ✅ **Flipper starts reset**: `resetFlipper()` called during init
- ✅ **Odometry reset**: Position and IMU reset before setting starting pose

---

## 3. Behaviors That Are Different from FullAutoOperateTest

### Time Management
- ❌ **Time-gated behavior**: VortexAutoOpMode enforces 30-second time limit
  - FullAutoOperateTest: Runs until manually stopped (no time limit)
  - VortexAutoOpMode: Forces parking at 30 seconds or when 5 seconds remaining

- ❌ **Time-based aborts**: VortexAutoOpMode aborts operations when time is low
  - FullAutoOperateTest: Never aborts based on time
  - VortexAutoOpMode: Aborts intake if < 1 second remaining (when `enforceAutoTimeLimit=true`)

- ❌ **Alignment timeout**: VortexAutoOpMode has 3-second alignment timeout
  - FullAutoOperateTest: Waits indefinitely for alignment to complete
  - VortexAutoOpMode: Proceeds to shoot after 3 seconds even if not aligned (when `enforceAutoTimeLimit=true`)

### Execution Model
- ❌ **Non-blocking state machines**: VortexAutoOpMode uses non-blocking FSMs
  - FullAutoOperateTest: Uses blocking operations (`sleep()`, `setToShootingVelocity()`, `moveToPose()`)
  - VortexAutoOpMode: All operations are non-blocking, updates called in loop

- ❌ **State-based transitions**: VortexAutoOpMode uses explicit state machine
  - FullAutoOperateTest: Linear sequence with blocking calls
  - VortexAutoOpMode: State-based transitions (INIT → DRIVE_TO_SHOOT → ALIGN → SHOOT → etc.)

### Intake Timeout Handling
- ❌ **Timeout during intake drive**: VortexAutoOpMode can ignore timeout during intake drive
  - FullAutoOperateTest: No timeout during intake (intake runs until drive completes)
  - VortexAutoOpMode: Uses `ignoreTimeoutDuringDrive` flag to match this behavior

### Flywheel Control
- ❌ **Non-blocking spin-up**: VortexAutoOpMode uses non-blocking `startSpinUp()` and `updateSpinUp()`
  - FullAutoOperateTest: Uses blocking `setToShootingVelocity()` with timeout
  - VortexAutoOpMode: Maintains velocity control between shots without blocking

- ❌ **Spin-up timeout behavior**: VortexAutoOpMode can proceed even if spin-up times out
  - FullAutoOperateTest: Logs warning but proceeds to shoot
  - VortexAutoOpMode: Sets `spinUpTimedOut` flag and skips `isAtTarget()` checks

### Gate Timing
- ❌ **Gate open delay**: VortexAutoOpMode opens gate first, then waits `KICKER_OPEN_DELAY_MS`
  - FullAutoOperateTest: Opens gate, then immediately waits `KICKER_OPEN_DELAY_MS` before shots
  - VortexAutoOpMode: Same behavior, but implemented in state machine

### Error Handling
- ❌ **Initial drive failure**: VortexAutoOpMode explicitly handles initial drive failure
  - FullAutoOperateTest: No explicit handling (would throw exception)
  - VortexAutoOpMode: Transitions to ball 0 if initial drive to shoot fails

### Parking Behavior
- ❌ **Automatic parking**: VortexAutoOpMode automatically parks when time expires (when `enforceAutoTimeLimit=true`)
  - FullAutoOperateTest: Manual stop required
  - VortexAutoOpMode: Transitions to parking state and processes it in the same update cycle
  - When `timeElapsed >= autoTotalTimeSec`: Calls `goToParkState()` then continues processing so `handleParkDrive()` runs
  - When `enforceAutoTimeLimit=false`: No automatic parking, runs to completion

---

## 4. New Behaviors That FullAutoOperateTest Doesn't Have

### State Machine Architecture
- 🆕 **Hierarchical state machines**: Multiple FSMs working together
  - `GameManager`: Top-level orchestration
  - `DriverManager`: Drive operations FSM
  - `IntakeManager`: Intake operations FSM
  - `ShootManager`: Shooting operations FSM

- 🆕 **Non-blocking updates**: All operations are non-blocking
  - Main loop calls `update()` on each manager every cycle
  - No `Thread.sleep()` or blocking waits
  - Enables responsive telemetry and time-based aborts

### Logging and Debugging
- 🆕 **Detailed state machine logging**: Per-manager log files
  - GameManager logs: State transitions, timing, ball index
  - DriverManager logs: Drive goals, results, pose updates
  - IntakeManager logs: Intake state, ball detection, timeouts
  - ShootManager logs: Shooting state, flywheel velocity, shot sequence

- 🆕 **Telemetry integration**: Log summaries displayed on telemetry
  - Real-time state information
  - Time tracking (elapsed, remaining)
  - Subsystem status

### Time Management Features
- 🆕 **Configurable time limits**: `enforceAutoTimeLimit` flag
  - Competition mode (`enforceAutoTimeLimit=true`): Time-gated behavior (safe)
    - Enforces 30-second hard limit
    - Starts parking when 5 seconds remaining
    - Aborts intake if < 1 second remaining
    - Times out alignment after 3 seconds
  - Parity mode (`enforceAutoTimeLimit=false`): Run-to-completion (matches FullAutoOperateTest)
    - No time-based aborts
    - No alignment timeout
    - No critical intake abort
    - Runs until all cycles complete
  - Allows same codebase for both competition and testing

- 🆕 **Park reserve time**: Automatic transition to parking (only when `enforceAutoTimeLimit=true`)
  - Starts parking when `timeRemaining <= parkReserveSec`
  - Ensures robot has time to reach parking position
  - Aborts current operations gracefully
  - When `enforceAutoTimeLimit=false`: No automatic parking

- 🆕 **Critical time abort**: Emergency intake abort (only when `enforceAutoTimeLimit=true`)
  - Aborts intake if < 1 second remaining
  - Prevents robot from getting stuck in intake when time is critical
  - When `enforceAutoTimeLimit=false`: Never aborts intake based on time

### Error Recovery
- 🆕 **Graceful failure handling**: Explicit error recovery paths
  - Initial drive failure → go to ball 0
  - Drive failures → advance to next ball or park
  - Alignment failures → proceed to shoot anyway (with timeout)
  - Spin-up failures → proceed to shoot anyway

- 🆕 **State consistency**: Proper cleanup on transitions
  - Flywheel stops when transitioning away from shooting
  - Intake stops when transitioning to parking
  - Gate positions set correctly for each operation

### Vision Integration
- 🆕 **Runtime alliance selection**: Gamepad-based selection
  - Select BLUE (X button) or RED (B button) during init
  - Automatically configures CameraServo target tag
  - Mirrors field positions for RED alliance

- 🆕 **Odometry fallback**: CoordinateTransformer integration
  - CameraServo can use odometry for distance prediction
  - Falls back to odometry when AprilTag not detected
  - More robust than vision-only approach

### Parallel Operations
- 🆕 **Maintained flywheel velocity**: Velocity control throughout sequence
  - Target velocity captured once at start of drive to shoot (based on AprilTag distance at that moment)
  - `updateSpinUp()` called during drive to maintain velocity (target not recomputed)
  - Velocity maintained during alignment
  - Velocity updated between shots (target recomputed for each shot)
  - Non-blocking velocity control throughout shooting sequence

- 🆕 **Intake passive power**: Power applied even when intake FSM is IDLE
  - Travel power (0.5) applied during drive to shooting
  - Intake FSM doesn't need to be RUNNING for travel power
  - Enables parallel intake operation during movement

### Safety Features
- 🆕 **Timeout safety fallbacks**: Multiple timeout layers
  - Drive timeout: Prevents infinite drive cycles (always active)
  - Intake timeout: Prevents infinite intake (can be ignored during drive, always active)
  - Spin-up timeout: Proceeds to shoot even if flywheel doesn't reach target (always active)
  - Firing timeout: Prevents infinite shooting sequence (always active)
  - Alignment timeout: Proceeds to shoot even if alignment incomplete (only when `enforceAutoTimeLimit=true`)
  - Critical intake abort: Aborts intake if < 1 second remaining (only when `enforceAutoTimeLimit=true`)

- 🆕 **State validation**: Checks before state transitions
  - Validates drive completion before starting intake
  - Validates alignment completion before shooting
  - Prevents invalid state transitions

### Configuration Flexibility
- 🆕 **Configurable timeouts**: Separate timeouts for different operations
  - `driveTimeoutSec`: Drive operations
  - `intakeTimeoutSec`: Intake operations
  - `shootTimeoutSec`: Shooting operations (separate from spin-up and firing)

- 🆕 **Configurable ball count**: `ballsPerSpot` parameter
  - Can be adjusted for different game strategies
  - Currently set to 3 (matches FullAutoOperateTest)

---

## Summary

**VortexAutoOpMode** maintains **behavioral parity** with FullAutoOperateTest for core autonomous operations (intake, shooting, alignment) while adding:

1. **Time management** for competition safety
2. **Non-blocking architecture** for responsive operation
3. **Error recovery** for robust operation
4. **Detailed logging** for debugging
5. **Configurable behavior** for different use cases

The key difference is the **execution model**: FullAutoOperateTest uses blocking operations, while VortexAutoOpMode uses non-blocking state machines. This enables time-based aborts and more responsive operation, but requires careful state management to maintain behavioral parity.


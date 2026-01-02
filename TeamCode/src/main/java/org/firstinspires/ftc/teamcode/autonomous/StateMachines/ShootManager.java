package org.firstinspires.ftc.teamcode.autonomous.StateMachines;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.StateMachineLogger;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.vision.CameraServo;



public class ShootManager {

    public enum ShooterState {
        IDLE,        // not doing anything
        SPINNING_UP, // ramping flywheel to target velocity
        FIRING,      // running flipper/kicker sequence
        DONE         // finished this cycle (success or failure)
    }

    public enum ShooterResult {
        NONE,        // no cycle yet
        SUCCESS,     // shot sequence completed
        JAM_FAILED,  // could not spin up or detected jam
        ABORTED      // cancelled due to time or gameManager
    }
    
    private final FlyWheel flyWheel;
    private final Kicker kicker;
    private final Flipper flipper;
    private final Intake intake;
    private final Telemetry telemetry;
    private final CameraServo cameraServo;  // Optional - can be null if not using vision
    private boolean dryRun = false;
    private StateMachineLogger logger;  // Optional logger for detailed debugging

    private ShooterState state  = ShooterState.IDLE;
    private ShooterState previousState = ShooterState.IDLE;  // Track previous state for logging
    private ShooterResult result = ShooterResult.NONE;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime firingTimer = new ElapsedTime();  // For firing sub-state timing

    // parameters for this cycle
    private double targetDistanceInch = 0.0;  // Legacy - not used with CameraServo
    private double spinUpTimeoutSec = 3.0;  // Timeout for spin-up phase
    private double firingTimeoutSec = 10.0;  // Timeout for firing phase (separate from spin-up, safety fallback)

    // internal
    private int shotIndex = 0;
    private final int maxShots = 3; // or 1, or configurable
    
    // Firing sub-state machine
    private enum FiringSubState {
        IDLE,
        PREPARE_SHOT,      // Close gate, ensure flywheel at target
        OPEN_GATE,         // Open gate for shooting
        TURNING_FLIPPER,   // Turn flipper to angle
        WAITING_FLIPPER_MOVE,  // Wait for flipper movement
        RESETTING_FLIPPER,     // Reset flipper
        WAITING_FLIPPER_RESET  // Wait for flipper reset
    }
    
    private FiringSubState firingSubState = FiringSubState.IDLE;
    
    // Track if spin-up timed out - if true, skip isAtTarget() check in PREPARE_SHOT
    // This matches FullAutoOperateTest behavior where shooting proceeds even if spin-up fails
    private boolean spinUpTimedOut = false;
    
    // Shooting parameters (from FullAutoOperateTest)
    private static final double INITIAL_FLIPPER_ANGLE = 120.0;
    private static final double ANGLE_INCREMENT = 30.0;
    private static final int KICKER_OPEN_DELAY_MS = 300;
    private static final int BASE_FLIPPER_DELAY_MS = 150;
    private static final int FLIPPER_DELAY_INCREMENT_MS = 50;
    private static final int FLIPPER_RESET_DELAY_MS = 200;
    
    // Default firing timeout (safety fallback only - FullAutoOperateTest never times out during shooting)
    private static final double DEFAULT_FIRING_TIMEOUT_SEC = 10.0;

    public ShootManager(FlyWheel flyWheel,
                        Kicker kicker,
                        Flipper flipper,
                        Intake intake,
                        Telemetry telemetry) {
        this(flyWheel, kicker, flipper, intake, telemetry, null, false, null);
    }

    public ShootManager(FlyWheel flyWheel,
                        Kicker kicker,
                        Flipper flipper,
                        Intake intake,
                        Telemetry telemetry,
                        boolean dryRun) {
        this(flyWheel, kicker, flipper, intake, telemetry, null, dryRun, null);
    }
    
    public ShootManager(FlyWheel flyWheel,
                        Kicker kicker,
                        Flipper flipper,
                        Intake intake,
                        Telemetry telemetry,
                        CameraServo cameraServo,
                        boolean dryRun) {
        this(flyWheel, kicker, flipper, intake, telemetry, cameraServo, dryRun, null);
    }
    
    public ShootManager(FlyWheel flyWheel,
                        Kicker kicker,
                        Flipper flipper,
                        Intake intake,
                        Telemetry telemetry,
                        CameraServo cameraServo,
                        boolean dryRun,
                        StateMachineLogger logger) {
        this.flyWheel = flyWheel;
        this.kicker = kicker;
        this.flipper = flipper;
        this.intake = intake;
        this.telemetry = telemetry;
        this.cameraServo = cameraServo;
        this.dryRun = dryRun;
        this.logger = logger;
    }

// -------- Public API for GameManager --------

    public ShooterState getState()  { return state; }
    public ShooterResult getResult(){ return result; }

    public boolean isIdle() { return state == ShooterState.IDLE; }
    public boolean isDone() { return state == ShooterState.DONE; }

    public void resetCycle() {
        // stop everything and clear
        if (!dryRun) {
            flyWheel.stopSpinUp();  // Use high-level API
            kicker.setGatePosition(Kicker.GATE_CLOSE);
            flipper.resetFlipper();
        }
        if (logger != null) {
            logger.logStateTransition(previousState, ShooterState.IDLE, null, "resetCycle()");
        }
        previousState = state;
        state  = ShooterState.IDLE;
        result = ShooterResult.NONE;
        shotIndex = 0;
        firingSubState = FiringSubState.IDLE;
    }

    /** GameManager calls this only when:
     *  - robot is already aligned to goal (by Drive subsystem)
     *  - we know our distance to goal (for flywheel tuning)
     */
    public void startCycle(double distanceInch, double spinUpTimeoutSec) {
        startCycle(distanceInch, spinUpTimeoutSec, DEFAULT_FIRING_TIMEOUT_SEC);  // Default firing timeout (safety fallback)
    }
    
    /** GameManager calls this with separate timeouts for spin-up and firing. */
    public void startCycle(double distanceInch, double spinUpTimeoutSec, double firingTimeoutSec) {
        if (state != ShooterState.IDLE) return;

        this.targetDistanceInch = distanceInch;
        this.spinUpTimeoutSec = spinUpTimeoutSec;
        this.firingTimeoutSec = firingTimeoutSec;
        this.shotIndex = 0;
        this.spinUpTimedOut = false;  // Reset timeout flag for new cycle

        if (dryRun) {
            // Testing mode: skip hardware, immediately complete
            finish(ShooterResult.SUCCESS);
        } else {
            // Normal mode: prepare mechanisms for shooting and begin spin-up
            // Close gate and set intake to travel power
            kicker.setGatePosition(Kicker.GATE_CLOSE);
            if (intake != null) {
                intake.setIntakePower(0.5);  // Travel power (from FullAutoOperateTest)
            }
            
            // Start non-blocking spin-up
            startSpinUp();
            timer.reset();
            if (logger != null) {
                logger.logStateTransition(previousState, ShooterState.SPINNING_UP, ShooterState.FIRING, 
                        String.format("startCycle(dist=%.1f)", distanceInch));
            }
            previousState = state;
            state  = ShooterState.SPINNING_UP;
            result = ShooterResult.NONE;
        }

        telemetry.addData("Shooter", "Start cycle: dist=%.1f spinUpTimeout=%.1fs firingTimeout=%.1fs",
                distanceInch, spinUpTimeoutSec, firingTimeoutSec);
    }

    public void abortCycle() {
        if (state == ShooterState.IDLE) return;
        finish(ShooterResult.ABORTED);
        telemetry.addData("Shooter", "Aborted");
    }
    
    /**
     * Close gate for parallel preparation (called during drive to shoot position).
     * This matches FullAutoOperateTest behavior where gate is closed in preparation thread.
     * Uses the class's dryRun field to determine whether to skip hardware calls.
     */
    public void closeGateForPrep() {
        if (!dryRun) {
            kicker.setGatePosition(Kicker.GATE_CLOSE);
        }
    }
    
    /**
     * Set gate to INTAKE position (called when starting intake cycle).
     * This matches FullAutoOperateTest behavior where gate is set to INTAKE before intaking.
     */
    public void setGateForIntake() {
        if (!dryRun) {
            kicker.setGatePosition(Kicker.GATE_INTAKE);
        }
    }

    public void update(double nowSec) {
        // Update CameraServo vision system (if available)
        if (cameraServo != null) {
            cameraServo.update();
        }
        
        switch (state) {
            case IDLE:
            case DONE:
                return;

            case SPINNING_UP:
                updateSpinUp();
                return;

            case FIRING:
                // CRITICAL: Call updateSpinUp() to maintain velocity control between shots
                flyWheel.updateSpinUp();
                updateFiring();
                return;
        }
    }

// -------- Internal state handlers --------

    private void updateSpinUp() {
        // FIXED: Match FullAutoOperateTest behavior - continue to firing even if spin-up times out
        // FullAutoOperateTest logs a warning but still attempts shooting when spin-up fails
        // This ensures shots are attempted even if flywheel didn't reach target (safer than skipping entirely)
        if (timer.seconds() > spinUpTimeoutSec) {
            telemetry.addData("Shooter", "⚠️ Spin-up timeout - proceeding to fire anyway (matching FullAutoOperateTest)");
            // Set flag to skip isAtTarget() check in PREPARE_SHOT (flywheel may never reach target)
            spinUpTimedOut = true;
            // Transition to firing phase anyway (matching FullAutoOperateTest line 252-257)
            // This attempts shooting even if flywheel didn't reach target velocity
            if (logger != null) {
                logger.logStateTransition(previousState, state, ShooterState.FIRING, 
                        "Spin-up timeout - proceeding to fire anyway");
            }
            previousState = state;
            state = ShooterState.FIRING;
            firingSubState = FiringSubState.PREPARE_SHOT;
            timer.reset();  // use timer for firing sequence timeout
            firingTimer.reset();  // use firingTimer for sub-state timing
            return;
        }

        // Get target velocity from CameraServo (vision-based) or fallback to distance-based
        double targetVelocity;
        if (cameraServo != null) {
            targetVelocity = cameraServo.getFlywheelVelocity();
        } else {
            // Fallback: use distance-based calculation (legacy)
            targetVelocity = org.firstinspires.ftc.teamcode.utils.RobotUtil.getRequiredFlyWheelVelocity(targetDistanceInch);
        }
        
        // FIXED: Update target RPM whenever camera target changes (matching FullAutoOperateTest)
        // startSpinUp() is idempotent - won't reset if same target, but will update if target changed
        // This prevents target drift where isAtTarget() compares against new target while flywheel chases old one
        flyWheel.startSpinUp(targetVelocity);
        
        // Update FlyWheel's internal state machine
        flyWheel.updateSpinUp();
        
        // Check if at target (FlyWheel handles mode switching internally)
        if (flyWheel.isAtTarget(targetVelocity, 0.95)) {
            // Good enough: go to firing phase
            if (logger != null) {
                logger.logStateTransition(previousState, state, ShooterState.FIRING, 
                        "Flywheel at target velocity");
            }
            previousState = state;
            state = ShooterState.FIRING;
            firingSubState = FiringSubState.PREPARE_SHOT;
            timer.reset();  // use timer for firing sequence timeout
            firingTimer.reset();  // use firingTimer for sub-state timing
            telemetry.addData("Shooter", "Spin-up OK (target=%.0f rpm)", targetVelocity);
        }
    }
    
    private void startSpinUp() {
        // Get target velocity from CameraServo (vision-based) or fallback
        double targetVelocity;
        if (cameraServo != null) {
            targetVelocity = cameraServo.getFlywheelVelocity();
        } else {
            // Fallback: use distance-based calculation (legacy)
            targetVelocity = org.firstinspires.ftc.teamcode.utils.RobotUtil.getRequiredFlyWheelVelocity(targetDistanceInch);
        }
        
        // Start non-blocking spin-up (idempotent - won't reset if already spinning toward same target)
        flyWheel.startSpinUp(targetVelocity);
        timer.reset();
    }

    private void updateFiring() {
        // FIXED: Use separate firing timeout (safety fallback only)
        // FullAutoOperateTest never times out during shooting - it runs until all shots complete
        // This timeout is only a safety mechanism for stuck states
        if (timer.seconds() > firingTimeoutSec) {
            finish(ShooterResult.ABORTED);
            telemetry.addData("Shooter", "Firing timeout (safety fallback)");
            return;
        }

        // Non-blocking flipper sequence using sub-state machine
        switch (firingSubState) {
            case PREPARE_SHOT:
                // FIXED: Ensure flywheel is at target before opening gate (matching FullAutoOperateTest)
                // FullAutoOperateTest waits for spin-up to complete (line 250) before opening gate (line 260)
                // This prevents early feeding if velocity is still recovering
                // EXCEPTION: If spin-up timed out, skip isAtTarget() check and proceed anyway (matching FullAutoOperateTest line 252-257)
                kicker.setGatePosition(Kicker.GATE_CLOSE);
                if (cameraServo != null) {
                    double targetVelocity = cameraServo.getFlywheelVelocity();
                    flyWheel.startSpinUp(targetVelocity); // Idempotent - won't reset if same target
                    // FIXED: If spin-up timed out, skip isAtTarget() check and proceed to open gate
                    // This matches FullAutoOperateTest behavior where shooting proceeds even if spin-up failed
                    if (spinUpTimedOut || flyWheel.isAtTarget(targetVelocity, 0.95)) {
                        // Flywheel at target OR spin-up timed out - open gate and proceed
                        kicker.setGatePosition(Kicker.GATE_SHOOT);
                        firingTimer.reset();
                        firingSubState = FiringSubState.OPEN_GATE;
                    } else {
                        // Still waiting for flywheel - stay in PREPARE_SHOT
                        // updateSpinUp() is called in update() to maintain velocity control
                        return;
                    }
                } else {
                    // No camera - open gate immediately (legacy behavior)
                    kicker.setGatePosition(Kicker.GATE_SHOOT);
                    firingTimer.reset();
                    firingSubState = FiringSubState.OPEN_GATE;
                }
                break;
                
            case OPEN_GATE:
                // Wait KICKER_OPEN_DELAY_MS after opening gate before turning flipper
                // This matches FullAutoOperateTest line 261: sleep(KICKER_OPEN_DELAY_MS) after opening gate
                if (firingTimer.milliseconds() >= KICKER_OPEN_DELAY_MS) {
                    firingTimer.reset();
                    firingSubState = FiringSubState.TURNING_FLIPPER;
                }
                break;
                
            case TURNING_FLIPPER:
                if (shotIndex >= maxShots) {
                    // All shots done
                    finish(ShooterResult.SUCCESS);
                    telemetry.addData("Shooter", "All shots completed: %d", maxShots);
                    return;
                }
                
                // Get target velocity and ensure flywheel is at target before proceeding (matching FullAutoOperateTest)
                // This ensures flywheel has recovered between shots for consistent shooting
                double targetVelocity;
                if (cameraServo != null) {
                    targetVelocity = cameraServo.getFlywheelVelocity();
                } else {
                    // Fallback: use distance-based calculation (legacy)
                    targetVelocity = org.firstinspires.ftc.teamcode.utils.RobotUtil.getRequiredFlyWheelVelocity(targetDistanceInch);
                }
                
                // Start spin-up (idempotent - won't reset if same target)
                flyWheel.startSpinUp(targetVelocity);
                
                // Wait for flywheel to reach target before turning flipper (matching FullAutoOperateTest behavior)
                // This ensures shot consistency by waiting for flywheel recovery
                if (flyWheel.isAtTarget(targetVelocity, 0.95)) {
                    // Flywheel at target - proceed with shot
                    double flipperAngle = INITIAL_FLIPPER_ANGLE + (shotIndex * ANGLE_INCREMENT);
                    flipper.turnFlipper(flipperAngle);
                    firingTimer.reset();
                    firingSubState = FiringSubState.WAITING_FLIPPER_MOVE;
                } else {
                    // Still waiting for flywheel to reach target - stay in this state
                    // updateSpinUp() is called in update() to maintain velocity control
                    telemetry.addData("Shooter", "Waiting for flywheel recovery (shot %d/%d)", shotIndex + 1, maxShots);
                }
                break;
                
            case WAITING_FLIPPER_MOVE:
                // Wait for flipper movement (time increases with shot number)
                int flipperWaitTime = BASE_FLIPPER_DELAY_MS + (shotIndex * FLIPPER_DELAY_INCREMENT_MS);
                if (firingTimer.milliseconds() >= flipperWaitTime) {
                    firingTimer.reset();
                    firingSubState = FiringSubState.RESETTING_FLIPPER;
                }
                break;
                
            case RESETTING_FLIPPER:
                flipper.resetFlipper();
                firingTimer.reset();
                firingSubState = FiringSubState.WAITING_FLIPPER_RESET;
                break;
                
            case WAITING_FLIPPER_RESET:
                if (firingTimer.milliseconds() >= FLIPPER_RESET_DELAY_MS) {
                    shotIndex++;
                    firingSubState = FiringSubState.TURNING_FLIPPER;  // Next shot
                }
                break;
                
            case IDLE:
                // Should not happen in FIRING state
                break;
        }
    }

    /** One place that stops mechanisms and marks DONE+result. */
    private void finish(ShooterResult finalResult) {
        if (!dryRun) {
            // FIXED: Use non-blocking stopSpinUp() instead of blocking fastStop()
            // This maintains non-blocking FSM requirement while still stopping the flywheel
            // Note: fastStop() contains Thread.sleep() which blocks the FSM update loop
            flyWheel.stopSpinUp();  // Non-blocking: just sets power to 0 and resets state
            flipper.resetFlipper();
            // Close gate first, then set to intake position to allow future intake cycles
            kicker.setGatePosition(Kicker.GATE_CLOSE);
            kicker.setGatePosition(Kicker.GATE_INTAKE);
        }
        if (logger != null) {
            logger.logStateTransition(previousState, state, ShooterState.DONE, 
                    String.format("finish(%s)", finalResult));
        }
        previousState = state;
        state  = ShooterState.DONE;
        result = finalResult;
        firingSubState = FiringSubState.IDLE;
        spinUpTimedOut = false;  // Reset timeout flag
    }
}



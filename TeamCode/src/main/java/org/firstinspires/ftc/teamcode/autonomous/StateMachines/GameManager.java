package org.firstinspires.ftc.teamcode.autonomous.StateMachines;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.motion.FieldPose;
import org.firstinspires.ftc.teamcode.utils.StateMachineLogger;


/**
 * GameManager (Option B)
 *
 * - Top-level AUTO state machine (GameState)
 * - Orchestrates Drive / Intake / Shooter via their cycle-style APIs
 * - Non-blocking: call update(nowSec) once per loop
 */
public class GameManager {

    // ------------------------------------------------------------
    // GAME STATE
    // ------------------------------------------------------------

    public enum GameState {
        INIT,

        AUTO_DRIVE_TO_BALL,   // drive to ball spot[ballIndex]
        AUTO_INTAKE,          // run intake at that spot

        AUTO_DRIVE_TO_SHOOT,  // drive to shoot spot[ballIndex]
        AUTO_ALIGN_TO_SHOOT,  // align to shooting angle (vision-based)
        AUTO_SHOOT,           // shoot balls

        PARK_DRIVE,           // drive to park pose

        DONE
    }

    // ------------------------------------------------------------
    // FIELDS
    // ------------------------------------------------------------

    private final DriverManager drive;
    private final IntakeManager intake;
    private final ShootManager shooter;
    private final Telemetry telemetry;
    private final StateMachineLogger logger;  // Optional logger for detailed debugging

    private GameState state = GameState.INIT;
    private GameState previousState = GameState.INIT;  // Track previous state for logging
    
    // Timebase for logging (updated in update() to ensure consistency)
    private double nowSecForLogging = 0.0;

    // Field layout (global static config) - using FieldPose (reference-point frame)
    private final FieldPose[] ballSpots;
    private final FieldPose[] ballFinishSpots;  // Finish positions for intake (INTAKE_*_FINISH)

    private final FieldPose defaultShootSpot;
    private final FieldPose parkPose;
    
    // CameraServo reference for alignment (optional - can be null if not using vision)
    private final org.firstinspires.ftc.teamcode.vision.CameraServo cameraServo;
    
    // FlyWheel reference for parallel preparation during drive to shoot
    private final org.firstinspires.ftc.teamcode.subsystems.FlyWheel flyWheel;
    
    // Track if flywheel spin-up has been started (to avoid redundant updateSpinUp() calls)
    private boolean flywheelSpinUpStarted = false;

    private int ballIndex = -1;  // which BALL_SPOT we are at (-1 = initial shoot, then 0, 1, 2...)

    // Auto timing
    // NOTE: Time gating behavior differs from FullAutoOperateTest
    // FullAutoOperateTest runs until manual stop, while GameManager enforces autoTotalTimeSec and parkReserveSec
    // This is a design choice: GameManager prioritizes reliable parking over completing all cycles
    // If identical sequencing to FullAutoOperateTest is required, set enforceAutoTimeLimit=false or autoTotalTimeSec very high
    private final double autoTotalTimeSec;   // e.g. 30.0
    private final double parkReserveSec;     // time reserved for parking at the end
    private final boolean enforceAutoTimeLimit;  // If false, time gating is disabled (matches FullAutoOperateTest run-to-completion)
    private double autoStartTimeSec = 0.0;

    // Subsystem timeouts / params
    private final int    ballsPerSpot;
    private final double defaultShootDistanceInch; // Ultimate fallback distance if array access fails

    // Timeout configuration (necessary for non-blocking FSM, unlike FullAutoOperateTest which blocks)
    // driveTimeoutSec: Required for DriverManager to prevent infinite waits in non-blocking FSM
    // intakeTimeoutSec: Safety fallback only (intake completion is tied to drive completion)
    //                   FullAutoOperateTest has no intake timeout - it blocks until drive completes
    // shootTimeoutSec: Used for spin-up timeout (matches FullAutoOperateTest FLYWHEEL_SPINUP_TIMEOUT=3s)
    //                  Also used for firing timeout (FullAutoOperateTest has no firing timeout - runs until done)
    private double driveTimeoutSec = 4;   // Required for non-blocking FSM
    private double intakeTimeoutSec = 10; // Safety fallback only (should not trigger during normal operation)
    private double shootTimeoutSec = 3;   // Spin-up timeout (matches FullAutoOperateTest), firing timeout is safety fallback
    
    // Velocity profiles (matching FullAutoOperateTest)
    private static final double TRAVEL_VELOCITY = 30.0; // inches/sec for travel movements
    private static final double INTAKE_VELOCITY = 20.0; // inches/sec during intake movements
    
    // Settle time after alignment (matching FullAutoOperateTest)
    private static final double ALIGNMENT_SETTLE_TIME_MS = 200.0; // milliseconds

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime alignmentSettleTimer = new ElapsedTime(); // Timer for post-alignment settle
    private boolean alignmentSettlingStarted = false; // Track if settle period has started for current alignment

    // ------------------------------------------------------------
    // CONSTRUCTOR
    // ------------------------------------------------------------

    public GameManager(
            DriverManager drive,
            IntakeManager intake,
            ShootManager shooter,
            Telemetry telemetry,
            FieldPose[] ballSpots,
            FieldPose[] ballFinishSpots,
            FieldPose   parkPose,
            FieldPose   defaultShootSpot,
            double autoTotalTimeSec,
            double parkReserveSec,
            int    ballsPerSpot,
            double defaultShootDistanceInch,
            double driveTimeoutSec,
            double intakeTimeoutSec,
            double shootTimeoutSec,
            org.firstinspires.ftc.teamcode.vision.CameraServo cameraServo,
            org.firstinspires.ftc.teamcode.subsystems.FlyWheel flyWheel,
            StateMachineLogger logger
    ) {
        this(drive, intake, shooter, telemetry, ballSpots, ballFinishSpots, parkPose, defaultShootSpot,
                autoTotalTimeSec, parkReserveSec, ballsPerSpot, defaultShootDistanceInch,
                driveTimeoutSec, intakeTimeoutSec, shootTimeoutSec, cameraServo, flyWheel, logger, true);
    }
    
    public GameManager(
            DriverManager drive,
            IntakeManager intake,
            ShootManager shooter,
            Telemetry telemetry,
            FieldPose[] ballSpots,
            FieldPose[] ballFinishSpots,
            FieldPose   parkPose,
            FieldPose   defaultShootSpot,
            double autoTotalTimeSec,
            double parkReserveSec,
            int    ballsPerSpot,
            double defaultShootDistanceInch,
            double driveTimeoutSec,
            double intakeTimeoutSec,
            double shootTimeoutSec,
            org.firstinspires.ftc.teamcode.vision.CameraServo cameraServo,
            org.firstinspires.ftc.teamcode.subsystems.FlyWheel flyWheel,
            StateMachineLogger logger,
            boolean enforceAutoTimeLimit
    ) {
        this.drive   = drive;
        this.intake  = intake;
        this.shooter = shooter;
        this.telemetry = telemetry;
        this.cameraServo = cameraServo;
        this.flyWheel = flyWheel;
        this.logger = logger;

        this.ballSpots  = ballSpots;
        this.ballFinishSpots = ballFinishSpots;
        this.defaultShootSpot = defaultShootSpot;
        this.parkPose   = parkPose;

        this.autoTotalTimeSec = autoTotalTimeSec;
        this.parkReserveSec   = parkReserveSec;
        this.ballsPerSpot     = ballsPerSpot;
        this.defaultShootDistanceInch = defaultShootDistanceInch;
        this.driveTimeoutSec  = driveTimeoutSec;
        this.intakeTimeoutSec = intakeTimeoutSec;
        this.shootTimeoutSec  = shootTimeoutSec;
        this.enforceAutoTimeLimit = enforceAutoTimeLimit;

        // Validation checks
        if (ballSpots == null || ballSpots.length == 0) {
            telemetry.addData("GM/WARNING", "ballSpots is null or empty");
        }
        if (defaultShootSpot == null) {
            telemetry.addData("GM/WARNING", "defaultShootSpot is null");
        }
        if (parkPose == null) {
            telemetry.addData("GM/WARNING", "parkPose is null");
        }
        if (defaultShootDistanceInch <= 0) {
            telemetry.addData("GM/WARNING", "defaultShootDistanceInch is invalid: %.1f", defaultShootDistanceInch);
        }
    }

    // ------------------------------------------------------------
    // PUBLIC API
    // ------------------------------------------------------------

    public GameState getState() {
        return state;
    }

    public boolean isDone() {
        return state == GameState.DONE;
    }

    /** Call once when AUTO actually starts (after waitForStart). */
    public void startAuto(double nowSec) {
        autoStartTimeSec = nowSec;
        ballIndex = -1;  // Start with initial shoot (-1), then go to 0, 1, 2...
        flywheelSpinUpStarted = false;  // Reset spin-up tracking

        if (logger != null) {
            logger.logSettings(String.format("autoTotalTimeSec=%.2f, parkReserveSec=%.2f, ballsPerSpot=%d, " +
                    "driveTimeoutSec=%.2f, intakeTimeoutSec=%.2f, shootTimeoutSec=%.2f, " +
                    "defaultShootDistanceInch=%.2f, ballSpots.length=%d, hasCameraServo=%s, hasFlyWheel=%s",
                    autoTotalTimeSec, parkReserveSec, ballsPerSpot, driveTimeoutSec, intakeTimeoutSec,
                    shootTimeoutSec, defaultShootDistanceInch,
                    ballSpots != null ? ballSpots.length : 0,
                    cameraServo != null, flyWheel != null));
        }

        drive.resetCycle();
        intake.resetCycle();
        shooter.resetCycle();

        state = GameState.INIT;
        stateTimer.reset();

        telemetry.addData("GM", "Auto start: state=INIT (will do initial shoot first)");
    }

    /**
     * Main non-blocking update.
     * In Option B, OpMode should call:
     *
     *   drive.update(nowSec);
     *   intake.update(nowSec);
     *   shooter.update(nowSec);
     *   gameManager.update(nowSec);
     */
    public void update(double nowSec) {
        // Update timebase for logging (ensures all logging uses consistent time from last update)
        nowSecForLogging = nowSec;
        
        if (state == GameState.DONE) return;

        // Time gating (only enforced if enforceAutoTimeLimit is true)
        // When false, matches FullAutoOperateTest run-to-completion behavior
        if (enforceAutoTimeLimit) {
            // FIXED: When out of time, always go to parking instead of just stopping
            // This ensures robot attempts to park even when time expires, rather than stopping wherever it is
            if (timeElapsed(nowSec) >= autoTotalTimeSec) {
                if (state != GameState.PARK_DRIVE && state != GameState.DONE) {
                    goToParkState(nowSec);
                }
                // CRITICAL FIX: Don't return here - continue processing so handleParkDrive() can run
                // If we just transitioned to PARK_DRIVE, we need to process it in this same update cycle
                // Otherwise the park drive never starts
                if (state == GameState.PARK_DRIVE) {
                    // Continue to state machine processing below so handleParkDrive() runs
                } else {
                    // State didn't change (already DONE or transition failed) - safe to return
                    return;
                }
            } else {
                // if close to the end, force transition to parking, unless already parking/done
                // This ensures we can abort intake/drive/shoot and go to parking even if timeouts are disabled
                if (timeRemaining(nowSec) <= parkReserveSec &&
                        state != GameState.PARK_DRIVE &&
                        state != GameState.DONE) {
                    goToParkState(nowSec);
                    // Continue processing - if we just transitioned to PARK_DRIVE, handleParkDrive() needs to run
                }
            }
            
            // Additional safety: if time is critically low (< 1 second), abort intake immediately
            // This ensures intake can be stopped even if ignoreTimeoutDuringDrive is true
            // FIXED: Only apply this safety check when enforceAutoTimeLimit is true
            // When false (run-to-completion mode), don't abort intake based on time
            if (timeRemaining(nowSec) < 1.0 && 
                (state == GameState.AUTO_INTAKE || intake.getState() == IntakeManager.IntakeState.RUNNING)) {
                intake.abortCycle();
                telemetry.addData("GM", "Critical time remaining (%.1fs) - aborting intake", timeRemaining(nowSec));
            }
        }

        switch (state) {
            case INIT:
                handleInit(nowSec);
                break;

            case AUTO_DRIVE_TO_BALL:
                handleAutoDriveToBall(nowSec);
                break;

            case AUTO_INTAKE:
                handleAutoIntake(nowSec);
                break;

            case AUTO_DRIVE_TO_SHOOT:
                handleAutoDriveToShoot(nowSec);
                break;

            case AUTO_ALIGN_TO_SHOOT:
                handleAutoAlignToShoot(nowSec);
                break;

            case AUTO_SHOOT:
                handleAutoShoot(nowSec);
                break;

            case PARK_DRIVE:
                handleParkDrive(nowSec);
                break;

            case DONE:
            default:
                break;
        }

        telemetry.addData("GM/State", state);
        telemetry.addData("GM/ballIndex", ballIndex);
        telemetry.addData("GM/timeRemain", "%.1f", timeRemaining(nowSec));
    }

    // ------------------------------------------------------------
    // LOGGING HELPERS
    // ------------------------------------------------------------
    
    /**
     * Log state transition and current status.
     * Uses nowSecForLogging (updated in update()) to ensure consistent timebase.
     * @param nextState The next state to transition to
     * @param reason Reason for the transition
     */
    private void logStateTransition(GameState nextState, String reason) {
        if (logger != null) {
            logger.logStateTransition(previousState, state, nextState, reason);
            
            // Log current status using tracked timebase for consistency
            String status = String.format("ballIndex=%d, timeElapsed=%.2fs, timeRemaining=%.2fs, " +
                    "driveState=%s, intakeState=%s, shooterState=%s",
                    ballIndex, timeElapsed(nowSecForLogging),
                    timeRemaining(nowSecForLogging),
                    drive.getState(), intake.getState(), shooter.getState());
            logger.logStatus(status);
        }
        previousState = state;
        state = nextState;
    }
    
    // ------------------------------------------------------------
    // STATE HANDLERS
    // ------------------------------------------------------------

    /** INIT → go to initial shoot position first, then proceed to ball pickup cycles. */
    private void handleInit(double nowSec) {
        // Validate defaultShootSpot is not null
        if (defaultShootSpot == null) {
            telemetry.addData("GM/ERROR", "defaultShootSpot is null - going to park");
            goToParkState(nowSec);
            return;
        }
        
        // Update flywheel spin-up during drive (only if spin-up has been started)
        if (flywheelSpinUpStarted && flyWheel != null && cameraServo != null && drive.getState() == DriverManager.DriveState.MOVING) {
            cameraServo.update();  // Update vision to get current distance
            double shootingVelocity = cameraServo.getFlywheelVelocity();
            flyWheel.updateSpinUp();  // Maintain velocity control during drive
        }
        
        // Drive to initial shoot position for preload
        if (drive.isIdle()) {
            // FIXED: Close gate during parallel prep (matching FullAutoOperateTest.java:183)
            // This prevents feed movement while driving/alignment
            shooter.closeGateForPrep();
            
            // FIXED: Enable passive power and set travel power when moving to shooting position
            // This applies power even when intake FSM is IDLE (matching FullAutoOperateTest)
            intake.enablePassivePower(true);
            intake.setIntakePower(0.5);  // INTAKE_TRAVEL_POWER (from FullAutoOperateTest)
            
            // Start parallel flywheel preparation (matches FullAutoOperateTest pattern)
            if (cameraServo != null && flyWheel != null) {
                cameraServo.update();  // Update vision to get current distance
                double shootingVelocity = cameraServo.getFlywheelVelocity();
                flyWheel.startSpinUp(shootingVelocity);  // Start non-blocking spin-up (parallel with drive)
                flywheelSpinUpStarted = true;  // Mark that spin-up has been started (CRITICAL: enables updateSpinUp() during drive)
                telemetry.addData("GM", "Starting flywheel spin-up in parallel: %.0f RPM", shootingVelocity);
            }
            
            drive.resetCycle();
            drive.startCycle(
                    DriverManager.DriveGoalKind.GOTO_SHOOT_SPOT,
                    defaultShootSpot,
                    driveTimeoutSec
            );
            logStateTransition(GameState.AUTO_DRIVE_TO_SHOOT, "Starting initial drive to shoot position");
            stateTimer.reset();
            telemetry.addData("GM", "Goto INITIAL_SHOOT (defaultShootSpot)");
            return;
        }

        if (drive.getState() == DriverManager.DriveState.DONE) {
            DriverManager.DriveResult res = drive.getResult();

            if (res == DriverManager.DriveResult.ARRIVED_OK) {
                // FIXED: Initial shoot now includes alignment + settle (matching FullAutoOperateTest.java:221-225)
                // Transition to alignment if CameraServo is available, otherwise go directly to shooting
                if (cameraServo != null) {
                    // Reset drive cycle so handleAutoAlignToShoot can start alignment
                    drive.resetCycle();
                    logStateTransition(GameState.AUTO_ALIGN_TO_SHOOT, "Initial shoot: starting alignment");
                    stateTimer.reset();
                    telemetry.addData("GM", "Drive to initial shoot OK → AUTO_ALIGN_TO_SHOOT");
                } else {
                    // No vision system - skip alignment and go directly to shooting
                    shooter.resetCycle();
                    shooter.startCycle(defaultShootDistanceInch, shootTimeoutSec);
                    logStateTransition(GameState.AUTO_SHOOT, String.format("Initial shoot (no vision, dist=%.1f)", defaultShootDistanceInch));
                    stateTimer.reset();
                    telemetry.addData("GM", "Drive to initial shoot OK → AUTO_SHOOT (no vision, dist=%.1f)", defaultShootDistanceInch);
                }
            }
            // REMOVED: Dead failure branch - initial drive failure is now handled in handleAutoDriveToShoot
            // When handleInit transitions to AUTO_DRIVE_TO_SHOOT state, drive completion (including failures)
            // will be handled by handleAutoDriveToShoot, not handleInit. This branch was unreachable.
        }
    }   

    /** Drive to BALL_SPOT[ballIndex]. */
    private void handleAutoDriveToBall(double nowSec) {
        // If drive is idle (not started, or reset), start it.
        if (drive.isIdle()) {
            startDriveToBallSpot(nowSec);
            return;
        }

        if (drive.getState() == DriverManager.DriveState.DONE) {
            DriverManager.DriveResult res = drive.getResult();

            if (res == DriverManager.DriveResult.ARRIVED_OK) {
                // Transition to AUTO_INTAKE state (will start forward drive in handleAutoIntake)
                logStateTransition(GameState.AUTO_INTAKE, "Drive to ball OK");
                stateTimer.reset();
                telemetry.addData("GM", "Drive to ball OK → AUTO_INTAKE (will drive forward)");
            } else {
                // drive failed / align failed / aborted → go next spot or park
                advanceBallIndexOrPark(nowSec);
            }
        }
    }

    /** Drive to intake finish position while intaking at current ball spot. */
    private void handleAutoIntake(double nowSec) {
        // Critical: Ensure previous ball-spot drive is fully complete before starting intake drive
        // Check if drive state is DONE (previous drive finished) and it was a drive to ball START spot
        if (drive.getState() == DriverManager.DriveState.DONE && 
            drive.getGoalKind() == DriverManager.DriveGoalKind.GOTO_BALL_SPOT) {
            // Previous drive to ball START spot is complete, start intake cycle and drive to finish position
            // FIXED: Set gate to INTAKE position before starting intake (matching FullAutoOperateTest.java:353)
            // This ensures gate is open even if previous shoot prep closed it
            shooter.setGateForIntake();
            
            // Set intake to full power when actively intaking
            intake.setIntakePower(1.0);  // INTAKE_FULL_POWER (from FullAutoOperateTest)
            
            // Start intake cycle
            intake.resetCycle();
            intake.startCycle(ballsPerSpot, (long) intakeTimeoutSec);
            
            // FIXED: Disable timeout during intake drive to match FullAutoOperateTest behavior
            // Intake will run until drive completes or target balls reached, timeout is ignored
            // This prevents premature stop if drive runs long (matching FullAutoOperateTest where intake
            // runs through the whole finish move without timeout)
            intake.setIgnoreTimeoutDuringDrive(true);
            
            // Drive to finish position (matches FullAutoOperateTest pattern: INTAKE_*_FINISH)
            if (ballFinishSpots == null || ballIndex >= ballFinishSpots.length) {
                telemetry.addData("GM/ERROR", "ballFinishSpots invalid - going to park");
                goToParkState(nowSec);
                return;
            }
            
            FieldPose finishTarget = ballFinishSpots[ballIndex];
            drive.resetCycle(); // Reset to IDLE state to prepare for new drive
            drive.startCycle(
                    DriverManager.DriveGoalKind.GOTO_BALL_FINISH,  // Use new goal kind for intake finish
                    finishTarget,
                    driveTimeoutSec
            );
            stateTimer.reset();
            telemetry.addData("GM", "Starting drive to intake finish position (ballIndex=%d)", ballIndex);
            return;
        }

        // Monitor intake drive progress (intake completion is tied to drive completion)
        // Check if drive is DONE and it was the intake finish drive
        if (drive.getState() == DriverManager.DriveState.DONE && 
            drive.getGoalKind() == DriverManager.DriveGoalKind.GOTO_BALL_FINISH) {
            DriverManager.DriveResult res = drive.getResult();

            if (res == DriverManager.DriveResult.ARRIVED_OK) {
                // Intake drive completed successfully → complete intake cycle (GOT_BALLS)
                // FIXED: Explicitly complete intake when drive finishes (matches FullAutoOperateTest)
                // This ensures intake stops when drive completes, not when timeout fires
                // The timeout is disabled during drive, so intake runs until drive completes
                intake.completeCycle(IntakeManager.IntakeResult.GOT_BALLS);
                telemetry.addData("GM", "Intake drive OK → intake complete, drive to shoot");
                startDriveToShootSpot(nowSec);
            } else {
                // Intake drive failed → treat as intake failure → next ball spot or park
                intake.abortCycle(); // Abort intake if drive failed
                telemetry.addData("GM", "Intake drive failed (%s) → next spot or park", res);
                advanceBallIndexOrPark(nowSec);
            }
        }
    }

    /** Drive to SHOOT_SPOT[ballIndex]. */
    private void handleAutoDriveToShoot(double nowSec) {
        // Update flywheel spin-up during drive (only if spin-up has been started)
        if (flywheelSpinUpStarted && flyWheel != null && cameraServo != null) {
            cameraServo.update();  // Update vision to get current distance
            double shootingVelocity = cameraServo.getFlywheelVelocity();
            flyWheel.updateSpinUp();  // Maintain velocity control during drive
        }
        
        if (drive.isIdle()) {
            startDriveToShootSpot(nowSec);
            return;
        }

        if (drive.getState() == DriverManager.DriveState.DONE) {
            DriverManager.DriveResult res = drive.getResult();

            if (res == DriverManager.DriveResult.ARRIVED_OK) {
                // Transition to alignment if CameraServo is available, otherwise go directly to shooting
                if (cameraServo != null) {
                    // Reset drive cycle so handleAutoAlignToShoot can start alignment
                    drive.resetCycle();
                    logStateTransition(GameState.AUTO_ALIGN_TO_SHOOT, String.format("Drive to shoot OK (ballIndex=%d)", ballIndex));
                    stateTimer.reset();
                    telemetry.addData("GM", "Drive to shoot OK → AUTO_ALIGN_TO_SHOOT (ballIndex=%d)", ballIndex);
                } else {
                    // No vision system - skip alignment and go directly to shooting
                    shooter.resetCycle();
                    shooter.startCycle(defaultShootDistanceInch, shootTimeoutSec);
                    logStateTransition(GameState.AUTO_SHOOT, String.format("Drive to shoot OK (no vision, ballIndex=%d, dist=%.1f)", ballIndex, defaultShootDistanceInch));
                    stateTimer.reset();
                    telemetry.addData("GM", "Drive to shoot OK → AUTO_SHOOT (no vision, ballIndex=%d, dist=%.1f)", ballIndex, defaultShootDistanceInch);
                }
            } else {
                // Drive failed - handle based on ballIndex
                if (ballIndex == -1) {
                    // FIXED: Initial drive failed → explicitly go to ball 0 (make behavior explicit)
                    // This makes the "initial drive failed → go to ball 0" behavior unambiguous
                    // and keeps the state machine non-blocking
                    // Stop flywheel if it was spinning (safety and state consistency)
                    if (flyWheel != null && flywheelSpinUpStarted) {
                        flyWheel.stopSpinUp();
                        flywheelSpinUpStarted = false;  // Reset flag
                    }
                    ballIndex = 0;
                    telemetry.addData("GM", "Initial drive to shoot failed (%s) → going to ball 0", res);
                    startDriveToBallSpot(nowSec);
                } else {
                    // Cannot reach shoot spot for current ball → next ball or park
                    advanceBallIndexOrPark(nowSec);
                }
            }
        }
    }
    
    /** Align to shooting angle using CameraServo vision. */
    private void handleAutoAlignToShoot(double nowSec) {
        // FIXED: Maintain flywheel velocity control during alignment (matching FullAutoOperateTest behavior)
        // FullAutoOperateTest sets velocity control before shooting, so we maintain it during alignment
        if (flywheelSpinUpStarted && flyWheel != null && cameraServo != null) {
            cameraServo.update();  // Update vision to get current distance
            double shootingVelocity = cameraServo.getFlywheelVelocity();
            flyWheel.updateSpinUp();  // Maintain velocity control during alignment
        }
        
        if (cameraServo == null) {
            // No vision system - skip alignment
            shooter.resetCycle();
            shooter.startCycle(defaultShootDistanceInch, shootTimeoutSec);
            logStateTransition(GameState.AUTO_SHOOT, "Alignment timeout - shooting anyway");
            return;
        }
        
        // Check for alignment timeout (only when enforceAutoTimeLimit is true)
        // FIXED: FullAutoOperateTest doesn't time-limit alignment - it waits until alignment completes
        // When enforceAutoTimeLimit is false (run-to-completion mode), don't timeout alignment
        // When true (timed mode), use timeout as safety fallback to prevent infinite alignment
        if (enforceAutoTimeLimit && stateTimer.seconds() > 3.0) {  // Alignment timeout (safety fallback in timed mode)
            telemetry.addData("GM", "Alignment timeout (%.1fs) → proceeding to shoot", stateTimer.seconds());
            // Critical: Stop drive before starting to shoot (prevents robot from moving while firing)
            drive.abortCycle();
            shooter.resetCycle();
            shooter.startCycle(0.0, shootTimeoutSec);
            logStateTransition(GameState.AUTO_SHOOT, String.format("Alignment timeout (%.1fs)", stateTimer.seconds()));
            alignmentSettlingStarted = false; // Reset flag for next alignment
            stateTimer.reset();
            return;
        }
        
        // Check if drive is idle (ready to start alignment) or done (alignment complete)
        if (drive.isIdle()) {
            // Drive is idle - start alignment
            // Update vision before getting shooting angle
            cameraServo.update();
            // FIXED: getShootingAngle() returns ABSOLUTE heading (not relative delta)
            // Per FullAutoOperateTest.java:390-400, it's an absolute heading
            double targetHeading = cameraServo.getShootingAngle();
            
            // Normalize to [-180, 180] range
            while (targetHeading > 180) targetHeading -= 360;
            while (targetHeading < -180) targetHeading += 360;
            
            drive.startAlignCycle(targetHeading, 3.0);
            alignmentSettlingStarted = false; // Reset flag when starting new alignment
            return;
        }

        // Check if alignment is complete
        if (drive.getState() == DriverManager.DriveState.DONE) {
            if (drive.getResult() == DriverManager.DriveResult.ARRIVED_OK) {
                // Alignment complete - start settle period if not already started
                if (!alignmentSettlingStarted) {
                    // First time alignment completes - start settle period
                    alignmentSettlingStarted = true;
                    alignmentSettleTimer.reset();
                    telemetry.addData("GM", "Alignment OK → settling (%.0fms)", ALIGNMENT_SETTLE_TIME_MS);
                    return; // Wait for settle time
                }
                
                // Check if settle time has elapsed
                if (alignmentSettleTimer.milliseconds() >= ALIGNMENT_SETTLE_TIME_MS) {
                    // Settle complete - start shooting
                    shooter.resetCycle();
                    shooter.startCycle(0.0, shootTimeoutSec);  // Distance not needed with CameraServo
                    logStateTransition(GameState.AUTO_SHOOT, String.format("Alignment OK + settled (ballIndex=%d)", ballIndex));
                    alignmentSettlingStarted = false; // Reset flag for next alignment
                    stateTimer.reset();
                    telemetry.addData("GM", "Alignment OK + settled → AUTO_SHOOT (ballIndex=%d)", ballIndex);
                }
                // Otherwise, still settling - wait for next update
                return;
            } else {
                // Alignment failed - proceed to shooting anyway (may still work)
                telemetry.addData("GM", "Alignment failed (%s) → proceeding to shoot", drive.getResult());
                shooter.resetCycle();
                shooter.startCycle(0.0, shootTimeoutSec);
                logStateTransition(GameState.AUTO_SHOOT, String.format("Alignment failed (%s)", drive.getResult()));
                alignmentSettlingStarted = false; // Reset flag for next alignment
                stateTimer.reset();
            }
        }
        // If drive is MOVING, just wait (will check again next update)
    }

    /** Run shooter at current shoot spot. */
    private void handleAutoShoot(double nowSec) {
        if (shooter.getState() == ShootManager.ShooterState.IDLE) {
            // If idle unexpectedly, start
            // Always use defaultShootDistanceInch (single shoot position)
            shooter.startCycle(defaultShootDistanceInch, shootTimeoutSec);
            return;
        }

        if (shooter.getState() == ShootManager.ShooterState.DONE) {
            ShootManager.ShooterResult res = shooter.getResult();

            // After initial shoot, transition to first ball pickup
            if (ballIndex == -1) {
                ballIndex = 0;  // Start with first ball spot (was -1 for initial shoot)
                startDriveToBallSpot(nowSec);
                return;
            }

            // For cycle shoots, treat all results the same: move on
            switch (res) {
                case SUCCESS:
                case JAM_FAILED:
                case ABORTED:
                default:
                    advanceBallIndexOrPark(nowSec);
                    break;
            }
        }
    }

    /** Drive to PARK position; after that we are DONE. */
    private void handleParkDrive(double nowSec) {
        // If drive was aborted (from goToParkState), reset it first
        if (drive.getState() == DriverManager.DriveState.DONE && 
            drive.getResult() == DriverManager.DriveResult.ABORTED) {
            drive.resetCycle();
        }
        
        if (drive.isIdle()) {
            // FIXED: Ensure intake is completely stopped when starting park drive
            // Disable passive power and explicitly stop intake (safety requirement)
            intake.enablePassivePower(false);
            intake.stopIntake();  // Always stops hardware, regardless of state
            
            drive.resetCycle();
            drive.startCycle(
                    DriverManager.DriveGoalKind.GOTO_PARK,
                    parkPose,
                    driveTimeoutSec
            );
            telemetry.addData("GM", "Start PARK_DRIVE");
            return;
        }

        if (drive.getState() == DriverManager.DriveState.DONE) {
            // Park move completed (or failed) → end auto
            forceDone("Park move finished", nowSec);
        }
    }

    // ------------------------------------------------------------
    // TRANSITION HELPERS
    // ------------------------------------------------------------

    /** Start drive cycle toward BALL_SPOT[ballIndex]. */
    private void startDriveToBallSpot(double nowSec) {
        if (ballSpots == null || ballIndex >= ballSpots.length) {
            goToParkState(nowSec);
            return;
        }

        // Stop intake during travel to intake start (matching FullAutoOperateTest behavior)
        // Disable passive power so intake stops when IDLE
        intake.enablePassivePower(false);
        intake.stopIntake();  // Always stops hardware, regardless of state

        FieldPose target = ballSpots[ballIndex];

        drive.resetCycle();
        drive.startCycle(
                DriverManager.DriveGoalKind.GOTO_BALL_SPOT,
                target,
                driveTimeoutSec
        );

        logStateTransition(GameState.AUTO_DRIVE_TO_BALL, String.format("Starting drive to ball spot[%d]", ballIndex));
        stateTimer.reset();
        telemetry.addData("GM", "Goto BALL_SPOT[%d]", ballIndex);
    }

    /** Start drive cycle toward defaultShootSpot. */
    private void startDriveToShootSpot(double nowSec) {
        // Validate defaultShootSpot is not null
        if (defaultShootSpot == null) {
            telemetry.addData("GM/ERROR", "defaultShootSpot is null - going to park");
            goToParkState(nowSec);
            return;
        }

        // FIXED: Enable passive power and set travel power when moving to shooting position
        // This applies power even when intake FSM is IDLE (matching FullAutoOperateTest)
        intake.enablePassivePower(true);
        intake.setIntakePower(0.5);  // INTAKE_TRAVEL_POWER (from FullAutoOperateTest)

        // Close gate for parallel preparation (matching FullAutoOperateTest behavior)
        // In FullAutoOperateTest, gate is closed in the preparation thread before driving
        shooter.closeGateForPrep();

        // Start parallel flywheel preparation (matches FullAutoOperateTest pattern)
        // This ramps flywheel while driving to shoot position, not during alignment
        if (cameraServo != null && flyWheel != null) {
            cameraServo.update();  // Update vision to get current distance
            double shootingVelocity = cameraServo.getFlywheelVelocity();
            flyWheel.startSpinUp(shootingVelocity);  // Start non-blocking spin-up (parallel with drive)
            flywheelSpinUpStarted = true;  // Mark that spin-up has been started
            telemetry.addData("GM", "Starting flywheel spin-up in parallel: %.0f RPM", shootingVelocity);
        }

        // Always use the single defaultShootSpot
        drive.resetCycle();
        drive.startCycle(
                DriverManager.DriveGoalKind.GOTO_SHOOT_SPOT,
                defaultShootSpot,
                driveTimeoutSec
        );

        logStateTransition(GameState.AUTO_DRIVE_TO_SHOOT, String.format("Starting drive to shoot spot (ballIndex=%d)", ballIndex));
        stateTimer.reset();
        telemetry.addData("GM", "Goto SHOOT_SPOT (ballIndex=%d)", ballIndex);
    }

    /**
     * After finishing intake/shoot, decide to:
     * - go next ball spot
     * - or go park if no more spots (or you later add time-based logic).
     */
    private void advanceBallIndexOrPark(double nowSec) {
        // FIXED: Stop flywheel when advancing to next ball or parking (safety and state consistency)
        // If shoot was skipped or aborted, flywheel may still be spinning from drive-to-shoot prep
        if (flyWheel != null && flywheelSpinUpStarted) {
            flyWheel.stopSpinUp();
            flywheelSpinUpStarted = false;  // Reset flag
        }
        
        ballIndex++;
        if (ballSpots == null || ballIndex >= ballSpots.length) {
            goToParkState(nowSec);
        } else {
            startDriveToBallSpot(nowSec);
        }
    }

    private void goToParkState(double nowSec) {
        // Abort any running cycles before parking (including drive)
        drive.abortCycle();
        intake.abortCycle();
        shooter.abortCycle();
        
        // FIXED: Completely stop intake when driving to park (safety requirement)
        // Disable passive power and explicitly stop intake to ensure it's completely off
        intake.enablePassivePower(false);
        intake.stopIntake();  // Always stops hardware, regardless of state
        
        // FIXED: Stop flywheel when aborting to park (safety and state consistency)
        // Flywheel may have been spinning up during drive-to-shoot, must stop on transition
        if (flyWheel != null && flywheelSpinUpStarted) {
            flyWheel.stopSpinUp();
            flywheelSpinUpStarted = false;  // Reset flag
        }
        
        // CRITICAL FIX: Actually set the state to PARK_DRIVE so handleParkDrive() can run
        // Without this, the state never changes and parking never starts
        logStateTransition(GameState.PARK_DRIVE, "Time remaining <= parkReserveSec");
        state = GameState.PARK_DRIVE;  // Set state so switch statement routes to handleParkDrive()
        stateTimer.reset();
        telemetry.addData("GM", "Transition → PARK_DRIVE");
    }

    private void forceDone(String reason, double nowSec) {
        drive.abortCycle();
        intake.abortCycle();
        shooter.abortCycle();
        
        // FIXED: Stop flywheel when forcing done (safety and state consistency)
        // Flywheel may have been spinning up, must stop on completion
        if (flyWheel != null && flywheelSpinUpStarted) {
            flyWheel.stopSpinUp();
            flywheelSpinUpStarted = false;  // Reset flag
        }

        logStateTransition(GameState.DONE, reason);
        telemetry.addData("GM", "DONE: %s", reason);
    }

    // ------------------------------------------------------------
    // TIME HELPERS
    // ------------------------------------------------------------

    private double timeElapsed(double nowSec) {
        return nowSec - autoStartTimeSec;
    }

    private double timeRemaining(double nowSec) {
        return autoTotalTimeSec - timeElapsed(nowSec);
    }

    // ------------------------------------------------------------
    // DISTANCE HELPERS
    // ------------------------------------------------------------

    // Note: Removed getShootDistance() - we now always use defaultShootDistanceInch
    // since there's only one shoot position
}
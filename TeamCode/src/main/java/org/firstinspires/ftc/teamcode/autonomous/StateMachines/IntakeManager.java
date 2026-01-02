package org.firstinspires.ftc.teamcode.autonomous.StateMachines;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.StateMachineLogger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.RobotUtil;


public class IntakeManager {

    // ------------- Local state enums -------------
    public enum IntakeState {
        IDLE,           // not doing anything
        RUNNING,
        DONE
    }

    public enum IntakeResult {
        NONE,           // no cycle yet
        GOT_BALLS,      // >=1 ball
        NO_BALL,        // 0 ball after cycle
        ABORTED         // e.g. time too short, or error
    }
    
    private  Intake intake;
    private  DistanceSensor channelSensor;
    private Telemetry telemetry;
    private boolean dryRun = false;
    private StateMachineLogger logger;  // Optional logger for detailed debugging

    private IntakeState state = IntakeState.IDLE;
    private IntakeState previousState = IntakeState.IDLE;  // Track previous state for logging
    private IntakeResult result = IntakeResult.NONE;

    private final ElapsedTime timer = new ElapsedTime();

    private long intakeTimeoutMs = 2500; //
    private int ballCount = 0;

    private int targetBalls;
    private int ballsCollected;
    private boolean previousDetected = false;  // Track previous sensor state for edge detection
    
    private double intakePower = 1.0;  // Default full power
    
    // Flag to disable timeout during intake drive (matches FullAutoOperateTest behavior)
    // When true, intake runs until drive completes or target balls reached, timeout is ignored
    private boolean ignoreTimeoutDuringDrive = false;
    
    // Flag to allow passive power application when intake FSM is IDLE
    // When true, setIntakePower() applies power to hardware even when not RUNNING
    // Used for travel power during drive to shoot (matching FullAutoOperateTest)
    private boolean allowPassivePower = false;

//    private long startTimeMs;

    public IntakeManager(Intake intake,
                         Telemetry telemetry) {
        this(intake, telemetry, null, false, null);
    }

    public IntakeManager(Intake intake,
                         Telemetry telemetry,
                         boolean dryRun) {
        this(intake, telemetry, null, dryRun, null);
    }
    
    public IntakeManager(Intake intake,
                         Telemetry telemetry,
                         boolean dryRun,
                         StateMachineLogger logger) {
        this(intake, telemetry, null, dryRun, logger);
    }
    
    public IntakeManager(Intake intake,
                         Telemetry telemetry,
                         DistanceSensor channelSensor,
                         boolean dryRun,
                         StateMachineLogger logger) {
        this.intake = intake;
        this.telemetry = telemetry;
        this.channelSensor = channelSensor;  // FIXED: Initialize sensor (can be null if not available)
        this.dryRun = dryRun;
        this.logger = logger;
    }

    // ---------- Public API for GameManager ----------

    public IntakeState getState()  { return state; }
    public IntakeResult getResult(){ return result; }

    public boolean isIdle() { return state == IntakeState.IDLE; }
    public boolean isDone() { return state == IntakeState.DONE; }

    public void resetCycle() {
        if (logger != null) {
            logger.logStateTransition(previousState, IntakeState.IDLE, null, "resetCycle()");
        }
        previousState = state;
        state = IntakeState.IDLE;
        result = IntakeResult.NONE;
        ballsCollected = 0;
        previousDetected = false;  // Reset edge detection state
        ignoreTimeoutDuringDrive = false;  // Reset timeout ignore flag
        // Note: allowPassivePower is NOT reset here - it's controlled by GameManager for travel power
    }
    
    /**
     * Enable/disable timeout during intake drive.
     * When enabled, timeout is ignored and intake runs until drive completes or target balls reached.
     * This matches FullAutoOperateTest behavior where intake runs through the whole finish move.
     * 
     * NOTE: Even when timeout is disabled, GameManager can still abort intake via abortCycle()
     * when time is running low (e.g., for parking). The timeout being disabled only prevents
     * premature stopping during normal operation.
     * 
     * @param ignore If true, timeout is ignored during intake drive (tied to drive completion)
     */
    public void setIgnoreTimeoutDuringDrive(boolean ignore) {
        this.ignoreTimeoutDuringDrive = ignore;
    }
    
    /**
     * Set intake power level.
     * If intake is currently running, update power immediately.
     * If power is 0.0, always stop the hardware (matching FullAutoOperateTest behavior).
     * If allowPassivePower is true, applies power even when IDLE (for travel power during drive).
     * 
     * @param power Power level (0.0 to 1.0)
     */
    public void setIntakePower(double power) {
        this.intakePower = power;
        if (!dryRun) {
            if (power == 0.0) {
                // Always stop hardware when power is 0.0 (matching intake.stopIntake() behavior)
                intake.stopIntake();
            } else if (state == IntakeState.RUNNING || (allowPassivePower && power > 0.0)) {
                // Update power if intake is running OR if passive power is allowed (for travel power)
                intake.setIntakePower(power);
            }
        }
    }
    
    /**
     * Enable/disable passive power application when intake FSM is IDLE.
     * When enabled, setIntakePower() applies power to hardware even when not RUNNING.
     * Used for travel power during drive to shoot (matching FullAutoOperateTest behavior).
     * 
     * @param allow If true, allows power application when IDLE (for travel power)
     */
    public void enablePassivePower(boolean allow) {
        this.allowPassivePower = allow;
    }
    
    /**
     * Stop intake motor immediately (always applies to hardware, regardless of state).
     * This matches FullAutoOperateTest's intake.stopIntake() behavior.
     */
    public void stopIntake() {
        this.intakePower = 0.0;
        if (!dryRun) {
            intake.stopIntake();
        }
    }

    public void startCycle(int targetBalls, long timeoutSec) {
        if (state != IntakeState.IDLE) return;   // ignore if busy

        this.targetBalls = targetBalls;
        this.intakeTimeoutMs = timeoutSec * 1000;  // Convert seconds to milliseconds
        this.ballCount = 0;

        timer.reset();
        
        if (dryRun) {
            // Testing mode: skip hardware, immediately complete
            finish(IntakeResult.GOT_BALLS);
        } else {
            // Normal mode: start intake motor with current power setting and begin running
            intake.setIntakePower(intakePower);
            if (logger != null) {
                logger.logStateTransition(previousState, IntakeState.RUNNING, IntakeState.DONE, 
                        String.format("startCycle(target=%d)", targetBalls));
            }
            previousState = state;
            state = IntakeState.RUNNING;
            result = IntakeResult.NONE;
        }

        telemetry.addData("Intake", "Start cycle: target=%d timeout=%.1fs",
                targetBalls, timeoutSec);
    }

    /** 
     * Allow GameManager to cancel mid-cycle (e.g., time is low, need to park).
     * This works even when ignoreTimeoutDuringDrive is true, ensuring intake can be stopped
     * when time is running out, regardless of timeout settings.
     */
    public void abortCycle() {
        if (state == IntakeState.RUNNING) {
            finish(IntakeResult.ABORTED);
        }
    }
    
    /**
     * Complete intake cycle with a specific result (called by GameManager when drive completes).
     * Used to tie intake completion to drive completion, matching FullAutoOperateTest behavior.
     * 
     * @param result The result to use when completing (typically GOT_BALLS when drive succeeds)
     */
    public void completeCycle(IntakeResult result) {
        if (state == IntakeState.RUNNING) {
            finish(result);
        }
    }

    // ---------- Internal FSM ----------

    public void update(double nowSec) {
        switch (state) {
            case IDLE:
            case DONE:
                // nothing to do
                return;

            case RUNNING:
                runUpdate();
                return;
        }
    }

    private void runUpdate() {
        // 1) Reached target balls?
        if (ballsCollected >= targetBalls) {
            finish(IntakeResult.GOT_BALLS);
            telemetry.addData("Intake", "Target reached, balls=%d", ballsCollected);
            return;
        }
        
        // 2) Timeout? (Only check if not ignoring timeout during drive)
        // FIXED: When ignoreTimeoutDuringDrive is true, timeout is disabled to match FullAutoOperateTest
        // Intake completion is tied to drive completion in GameManager, so timeout is only a safety
        // fallback when drive never completes
        if (!ignoreTimeoutDuringDrive && timer.seconds() * 1000.0 > intakeTimeoutMs) {
            if (ballsCollected == 0) {
                finish(IntakeResult.NO_BALL);
            } else {
                finish(IntakeResult.GOT_BALLS);
            }
            telemetry.addData("Intake", "Timeout; balls=%d result=%s",
                    ballsCollected, result);
            return;
        }

        // 3) Read sensor & detect new ball edge (if sensor available)
        boolean detected = false;
        if (channelSensor != null) {
            try {
                detected = RobotUtil.isObjectDetected(channelSensor, telemetry);
            } catch (Exception e) {
                finish(IntakeResult.ABORTED);
                telemetry.addData("Intake", "ERROR reading sensor: %s", e.getMessage());
                return;
            }
        } else {
            // No sensor available - rely on timeout only (sensor detection skipped)
            // This allows intake to work without a sensor, but won't detect balls mid-cycle
            detected = false;
        }

        // 3.5) Edge detection: increment ballsCollected on rising edge (false -> true)
        if (detected && !previousDetected) {
            ballsCollected++;
            telemetry.addData("Intake", "Ball detected! Total: %d/%d", ballsCollected, targetBalls);
        }
        previousDetected = detected;  // Update previous state for next cycle
    }

    /** Single place that:
     *  - stops motor
     *  - sets DONE
     *  - records result
     */
    private void finish(IntakeResult finalResult) {
        if (!dryRun) {
            intake.stopIntake();               // <- intake.stop() here, only once
        }
        if (logger != null) {
            logger.logStateTransition(previousState, state, IntakeState.DONE, 
                    String.format("finish(%s)", finalResult));
        }
        previousState = state;
        state = IntakeState.DONE;
        result = finalResult;
    }
}
